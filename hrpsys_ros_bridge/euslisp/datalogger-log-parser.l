(require :rtm-ros-robot-interface "package://hrpsys_ros_bridge/euslisp/rtm-ros-robot-interface.l")

(defclass datalogger-log-parser
  :super propertied-object
  :slots (file index count read-state time)
  )

(defmethod datalogger-log-parser
  (:init
   (fname)
   (setq file (open fname))
   (setq index 0 count 0)
   )
  (:reset
   (&key ((:count c) 0) ((:index i) 0))
   (let ((fname (send file :fname)))
     (close file)
     (setq file (open fname))
     (setq count c index i)
     ))
  (:fname () (send file :fname))
  (:read-state () read-state)
  (:time () time)
  (:proc
   ()
   (let (line)
     (when (< index count)
       (send self :reset :index index)
       )
     (while (>= index count)
       (unless (setq line (read-line file nil))
         (warn "hit EOF!~%")
         (send self :reset)
         (return-from :proc nil))
       (incf count)
       )
     (setq index count)

     (setq read-state (read-from-string (format nil "#f(~A)" (string-left-trim ".0123456789" line))))
     (let ((tm (data-string-split (car (data-string-split line " ")) ".")))
       (setq time (instance ros::time :init :sec (read-from-string (car tm)) :nsec (* 1000 (read-from-string (cadr tm))))))
     t
     ))
  )

(defun data-string-split ;; this function will be replaced by https://github.com/euslisp/EusLisp/issues/16
  (str separator)
  (let ((start-idx 0) (ret))
    (dotimes (i (length str))
      (when (= (elt str i) (elt separator 0))
        (push (subseq str start-idx i) ret)
        (setq start-idx (1+ i))))
    (if (/= start-idx (length str)) (push (subseq str start-idx) ret))
    (reverse ret)))

(defclass datalogger-log-parser-controller
  :super rtm-ros-robot-interface
  :slots (parser-list robothardware-name eof-p)
  )

(defmethod datalogger-log-parser-controller
  (:init
   (fname &key ((:robot r)))
   (setq robot (cond ((derivedp r metaclass) (instance r :init))
                     (t r)))
   ;;(send-super :init :robot r)
   (let* ((dir-str (format nil "/~A" (reduce #'(lambda (x y) (format nil "~A/~A" x y)) (cdr (pathname-directory fname)))))
          (fname-prefix (pathname-name fname)))
     ;; gather fname list with max lines
     (let* ((fname-candidate-list (remove-if-not #'(lambda (x) (substringp (format nil "~A." fname-prefix) x)) (directory dir-str)))
            (fname-liens-without-rh
             (mapcar #'(lambda (f)
                         (list (read-from-string (car (data-string-split (car (piped-fork-returns-list (format nil "LANG=C wc -l ~A/~A" dir-str f))) " "))) f))
                     (remove-if #'(lambda (x) (substringp "(" x)) fname-candidate-list)))
            (max-line (reduce #'(lambda (x y) (max x y)) (mapcar #'car fname-liens-without-rh)))
            (fname-candidate-list-with-valid-line-without-rh
             (mapcar #'cadr (remove-if-not #'(lambda (x) (= (car x) max-line)) fname-liens-without-rh))))
       (setq parser-list
             (mapcar #'(lambda (x) (instance datalogger-log-parser :init (format nil "~A/~A" dir-str x)))
                     (append (remove-if-not #'(lambda (x) (substringp "(" x)) fname-candidate-list)
                             fname-candidate-list-with-valid-line-without-rh)))
       (cond
        ((find-if #'(lambda (x) (substringp "(Robot)" x)) (send-all (send self :parser-list) :fname))
         (setq robothardware-name (car (data-string-split (pathname-type (find-if #'(lambda (x) (substringp "(Robot)" x)) (send-all (send self :parser-list) :fname))) "_"))))
        (t (setq robothardware-name "RobotHardware0")))
       )))
  (:parser-list
   (&optional (str))
   (if str
       (find-if #'(lambda (x) (string= str (pathname-type (send x :fname)))) parser-list)
     parser-list))
  (:eof-p () eof-p)
  (:update-robot-state
   ()
   (setq eof-p (some #'null (send-all parser-list :proc)))
   (send self :set-robot-state1
         :reference-vector
         (map float-vector #'rad2deg (send (send self :parser-list "sh_qOut") :read-state)))
   (send self :set-robot-state1
         :potentio-vector
         (map float-vector #'rad2deg
              (send (send self :parser-list (format nil "~A_q" robothardware-name)) :read-state)))
   (send robot :angle-vector (cdr (assoc :potentio-vector robot-state)))
   (send self :set-robot-state1
         :torque-vector
         (send (send self :parser-list (format nil "~A_tau" robothardware-name)) :read-state))
   ;; (send robot :torque-vector (cdr (assoc :torque-vector robot-state)))
   (send self :set-robot-state1
         :reference-torque-vector
         (send (send self :parser-list "sh_tqOut") :read-state))
   (send self :set-robot-state1
         :root-pos
         (scale 1e3 (send (send self :parser-list "sh_basePosOut") :read-state)))
   (send self :set-robot-state1
         :root-rpy
         (send (send self :parser-list "sh_baseRpyOut") :read-state))
   (send self :set-robot-state1
         :root-coords
         (make-coords :pos (cdr (assoc :root-pos robot-state))
                      :rpy (cdr (assoc :root-rpy robot-state))))
   (if (send self :parser-list "sh_zmpOut")
       (send self :set-robot-state1
             :zmp
             (send (send self :parser-list "sh_zmpOut") :read-state)))
   (send robot :move-coords (cdr (assoc :root-coords robot-state)) (car (send robot :links)))
   (dolist (f (send robot :force-sensors))
     (send self :set-robot-state1
           (send f :name)
           (send (send self :parser-list (format nil "~A_~A" robothardware-name (send f :name))) :read-state)
           ))
   ;; (dolist (i (send robot :imu-sensors))
   ;;   (send self :set-robot-state1
   ;;         (send i :name)
   ;;         (send (send self :parser-list (format nil "~A_~A" robothardware-name (send i :name))) :read-state)
   ;;         ))
   (if (send self :parser-list "kf_rpy")
       (send self :set-robot-state1
             :imu
             (let* ((rpy (send (send self :parser-list "kf_rpy") :read-state))
                    (qt (ros::rot->tf-quaternion (rpy-matrix (elt rpy 2) (elt rpy 1) (elt rpy 0)))))
               (instance* sensor_msgs::Imu :init
                          :orientation qt
                          (append
                           (if (send robot :imu-sensor "gsensor")
                               (list :linear_acceleration
                                     (let ((acc (send (send self :parser-list (format nil "~A_~A" robothardware-name "gsensor")) :read-state)))
                                       (instance geometry_msgs::Vector3 :init :x (elt acc 0) :y (elt acc 1) :z (elt acc 2)))))
                           (if (send robot :imu-sensor "gyrometer")
                               (list :angular_velocity
                                     (let ((gyro (send (send self :parser-list (format nil "~A_~A" robothardware-name "gyrometer")) :read-state)))
                                       (instance geometry_msgs::Vector3 :init :x (elt gyro 0) :y (elt gyro 1) :z (elt gyro 2))))))
                          ))))
   )
  ;; overwrite
  (:reference-vector () (cdr (assoc :reference-vector robot-state)))
  (:actual-vector () (send self :potentio-vector))
  (:error-vector () (v- (send self :potentio-vector) (send self :reference-vector)))
  (:time
   (&optional (str "sh_qOut"))
   (send (send self :parser-list str) :time)
   )
  )

;; (progn (setq *log* (instance datalogger-log-parser-controller :init "/tmp/testlog.sh_qOut")) (send *log* :state) (send *log* :potentio-vector))
