(require :rtm-ros-robot-interface "package://hrpsys_ros_bridge/euslisp/rtm-ros-robot-interface.l")

(defclass datalogger-log-parser
  :super propertied-object
  :slots (file index count read-state time)
  )

(defmethod datalogger-log-parser
  (:init
   (fname &key (name))
   (send-super :name name)
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
     (let* ((tm (read-from-string (format nil "(~A)" (substitute (elt " " 0) (elt "." 0) (subseq line 0 (position (elt " " 0) line)))))))
       (setq time (instance ros::time :init :sec (car tm) :nsec (* 1000 (cadr tm)))))
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
  :slots (parser-list eof-p limb-list)
  )

(defmethod datalogger-log-parser-controller
  (:init
   (fname &key ((:robot r)))
   (setq robot (cond ((derivedp r metaclass) (instance r :init))
                     (t r)))
   (setq limb-list (mapcar #'(lambda (x) (find-if #'(lambda (l) (member x (send robot l :force-sensors))) '(:rleg :lleg :rarm :larm)))
                           (send robot :force-sensors)))
   ;;(send-super :init :robot r)
   (let* ((dir-str (format nil "/~A" (reduce #'(lambda (x y) (format nil "~A/~A" x y)) (cdr (pathname-directory fname)))))
          (fname-prefix (pathname-name fname)))
     ;; gather fname list with max lines
     (let* ((fname-candidate-list (remove-if-not #'(lambda (x) (substringp (format nil "~A." fname-prefix) x)) (directory dir-str)))
            (fname-liens-without-rh
             (mapcar #'(lambda (f)
                         (list (read-from-string (car (data-string-split (car (piped-fork-returns-list (format nil "LANG=C wc -l ~A/~A" dir-str f))) " "))) f))
                     (remove-if #'(lambda (x) (substringp "(" x)) fname-candidate-list)))
            (max-line (car (find-if #'(lambda (x) (substringp "sh_qOut" (cadr x))) fname-liens-without-rh)))
            (fname-candidate-list-with-valid-line-without-rh
             (mapcar #'cadr (remove-if-not #'(lambda (x) (= (car x) max-line)) fname-liens-without-rh))))
       (unless fname-candidate-list-with-valid-line-without-rh
         (warning-message 1 "Failed to find candidate files, have you specified wrong path?: ~A~%" fname))
       (setq parser-list
             (mapcar #'(lambda (x)
                         (instance datalogger-log-parser :init (format nil "~A/~A" dir-str x)
                                   :name (let ((nm (pathname-type x)))
                                           (read-from-string
                                            (if (and (substringp "(" nm) (substringp ")" nm))
                                                (let* ((strlist (data-string-split nm "_")) ;; AA(Robot)0_BB_CC -> (list AA(Robot)0 BB CC)
                                                       (suffix-str-list (subseq strlist (1+ (position-if #'(lambda (x) (and (substringp "(" x) (substringp ")" x))) strlist)))) ;; (list AA(Robot)0 BB CC) -> (list BB CC)
                                                       (suffix-str (let ((str (car suffix-str-list))) (dolist (tstr (cdr suffix-str-list)) (setq str (format nil "~A_~A" str tstr))) str))) ;; (list BB CC) -> BB_CC
                                                  (format nil "RobotHardware0_~A" suffix-str))
                                              nm)))))
                     (append (remove-if-not #'(lambda (x) (substringp "(" x)) fname-candidate-list)
                             fname-candidate-list-with-valid-line-without-rh)))
       )))
  (:parser-list
   (&optional (str))
   (if str
       (let ((str-sym (read-from-string str)))
         (find-if #'(lambda (x) (eq str-sym (send x :name))) parser-list))
     parser-list))
  (:eof-p () eof-p)
  (:fix-unitsystem-of-angle-vector
   (read-state &key (joint-list (send robot :joint-list)))
   (let ((vec (instantiate float-vector (length joint-list))))
     (dotimes (i (length vec))
       (setf (elt vec i) (if (derivedp (elt joint-list i) rotational-joint)
                             (rad2deg (elt read-state i))
                           (* 1e3 (elt read-state i)))))
     vec))
  (:update-robot-state
   (&key (wait-until-update nil))
   (setq eof-p (some #'null (send-all parser-list :proc)))
   ;; AutoBalancer parameter
   (if (send self :parser-list "abc_q")
       (send self :set-robot-state1
             :auto-balancer-reference-vector
             (send self :fix-unitsystem-of-angle-vector (send (send self :parser-list "abc_q") :read-state))))
   (if (send self :parser-list "abc_baseTformOut")
       (send self :set-robot-state1
             :auto-balancer-reference-root-coords
             (send self :hrpsys-tform->coords (send (send self :parser-list "abc_baseTformOut") :read-state))))
   (let ((rsd (send self :parser-list "abc_controlSwingSupportTime")))
     (if rsd (send self :set-robot-state1 :auto-balancer-control-swing-support-time (send rsd :read-state))))
   (let ((rsd (send self :parser-list "abc_contactStates")))
     (if rsd (send self :set-robot-state1 :auto-balancer-contact-states (send rsd :read-state))))
   (let ((rsd (send self :parser-list "abc_zmpOut")))
     (if rsd (send self :set-robot-state1 :auto-balancer-zmp (scale 1e3 (send rsd :read-state)))))
   (let ((rsd (send self :parser-list "abc_cogOut")))
     (if rsd (send self :set-robot-state1 :auto-balancer-cog (scale 1e3 (send rsd :read-state)))))
   (dolist (f (send robot :force-sensors))
     (let ((rsd (send self :parser-list (format nil "abc_limbCOPOffset_~A" (send f :name)))))
       (if rsd (send self :set-robot-state1 (read-from-string (format nil ":auto-balancer-~A-COP-off"
                                                                      (find-if #'(lambda (x) (equal f (car (send robot x :force-sensors)))) '(:rleg :lleg :rarm :larm))))
                     (send rsd :read-state)))))
   (when (and (send self :auto-balancer-reference-vector) (send self :auto-balancer-reference-root-coords))
     ;; Set robot state from AutoBalancer reference
     (send robot :angle-vector (send self :auto-balancer-reference-vector))
     (send robot :move-coords (send self :auto-balancer-reference-root-coords) (car (send robot :links)))
     (send robot :update-descendants)
     (send self :set-robot-state1 :auto-balancer-abs-zmp (send (car (send robot :links)) :transform-vector (send self :auto-balancer-zmp)))
     (send self :set-robot-state1 :auto-balancer-end-coords-list
           (mapcar #'(lambda (x) (send robot x :end-coords :copy-worldcoords)) limb-list)))
   ;; Stabilizer parameter
   (let ((rsd (send self :parser-list "st_originRefCogVel")))
     (if rsd (send self :set-robot-state1 :stabilizer-reference-cogvel (scale 1e3 (send rsd :read-state)))))
   (let ((rsd (send self :parser-list "st_originActCogVel")))
     (if rsd (send self :set-robot-state1 :stabilizer-cogvel (scale 1e3 (send rsd :read-state)))))
   (let ((rsd (send self :parser-list "st_originRefZmp")))
     (if rsd (send self :set-robot-state1 :stabilizer-reference-zmp (scale 1e3 (send rsd :read-state)))))
   (let ((rsd (send self :parser-list "st_originActZmp")))
     (if rsd (send self :set-robot-state1 :stabilizer-zmp (scale 1e3 (send rsd :read-state)))))
   (let ((rsd (send self :parser-list "st_originNewZmp")))
     (if rsd (send self :set-robot-state1 :stabilizer-newzmp (scale 1e3 (send rsd :read-state)))))
   (let ((rsd (send self :parser-list "st_originRefCog")))
     (if rsd (send self :set-robot-state1 :stabilizer-reference-cog (scale 1e3 (send rsd :read-state)))))
   (let ((rsd (send self :parser-list "st_originActCog")))
     (if rsd (send self :set-robot-state1 :stabilizer-cog (scale 1e3 (send rsd :read-state)))))
   (let ((rsd (send self :parser-list "st_actBaseRpy")))
     (if rsd (send self :set-robot-state1 :stabilizer-act-base-rpy (map float-vector #'rad2deg (send rsd :read-state)))))
   (let ((rsd (send self :parser-list "st_currentBaseRpy")))
     (if rsd (send self :set-robot-state1 :stabilizer-current-base-rpy (map float-vector #'rad2deg (send rsd :read-state)))))
   (let ((rsd (send self :parser-list "st_debugData")))
     (if rsd (send self :set-robot-state1 :stabilizer-debug-data (send rsd :read-state))))
   ;; Set stable rtc data
   (send self :set-robot-state1
         :reference-vector
         (send self :fix-unitsystem-of-angle-vector (send (send self :parser-list "sh_qOut") :read-state)))
   (send self :set-robot-state1
         :potentio-vector
         (send self :fix-unitsystem-of-angle-vector (send (send self :parser-list "RobotHardware0_q") :read-state)))
   (send robot :angle-vector (cdr (assoc :potentio-vector robot-state)))
   (send self :set-robot-state1
         :torque-vector
         (send (send self :parser-list "RobotHardware0_tau") :read-state))
   ;; (send robot :torque-vector (cdr (assoc :torque-vector robot-state)))
   (if (send self :parser-list "RobotHardware0_servoState")
       (send self :set-robot-state1
             :servo-state
             (map float-vector #'(lambda (intval)
                                   (let ((str (make-string 4))) (sys::poke intval str 0 :integer) (sys::peek str 0 :float)))
                  (mapcar #'ceiling (coerce (send (send self :parser-list "RobotHardware0_servoState") :read-state) cons)))))
   (send self :set-robot-state1
         :reference-torque-vector
         (send (send self :parser-list "sh_tqOut") :read-state))
   (send self :set-robot-state1
         :reference-root-pos
         (scale 1e3 (send (send self :parser-list "sh_basePosOut") :read-state)))
   (send self :set-robot-state1
         :reference-root-rpy
         (send (send self :parser-list "sh_baseRpyOut") :read-state))
   (send self :set-robot-state1
         :reference-root-coords
         (make-coords :pos (cdr (assoc :reference-root-pos robot-state))
                      :rpy (cdr (assoc :reference-root-rpy robot-state))))
   (if (send self :parser-list "sh_zmpOut")
       (send self :set-robot-state1
             :zmp
             (send (send self :parser-list "sh_zmpOut") :read-state)))
   (dolist (f (send robot :force-sensors))
     (send self :set-robot-state1
           (send f :name)
           (send (send self :parser-list (format nil "RobotHardware0_~A" (send f :name))) :read-state))
     (send self :set-robot-state1
           (read-from-string (format nil ":off-~A" (string-downcase (send f :name))))
           (send (send self :parser-list (format nil "rmfo_off_~A" (send f :name))) :read-state))
     (send self :set-robot-state1
           (read-from-string (format nil ":reference-~A" (string-downcase (send f :name))))
           (send (send self :parser-list (format nil "sh_~AOut" (send f :name))) :read-state))
     )
   ;; (dolist (i (send robot :imu-sensors))
   ;;   (send self :set-robot-state1
   ;;         (send i :name)
   ;;         (send (send self :parser-list (format nil "~A_~A" robothardware-name (send i :name))) :read-state)
   ;;         ))
   (when (send self :parser-list "kf_rpy")
     (send self :set-robot-state1
             :imu
             (let* ((rpy (send (send self :parser-list "kf_rpy") :read-state))
                    (qt (ros::rot->tf-quaternion (rpy-matrix (elt rpy 2) (elt rpy 1) (elt rpy 0)))))
               (instance* sensor_msgs::Imu :init
                          :orientation qt
                          (append
                           (if (send robot :imu-sensor "gsensor")
                               (list :linear_acceleration
                                     (let ((acc (send (send self :parser-list "RobotHardware0_gsensor") :read-state)))
                                       (instance geometry_msgs::Vector3 :init :x (elt acc 0) :y (elt acc 1) :z (elt acc 2)))))
                           (if (send robot :imu-sensor "gyrometer")
                               (list :angular_velocity
                                     (let ((gyro (send (send self :parser-list "RobotHardware0_gyrometer") :read-state)))
                                       (instance geometry_msgs::Vector3 :init :x (elt gyro 0) :y (elt gyro 1) :z (elt gyro 2))))))
                          )))
     (let* ((imu-relative-to-body (send (send (car (send robot :links)) :worldcoords) :transformation (send (car (send robot :imu-sensors)) :worldcoords) :local))
            (body-relative-to-world (send (send self :auto-balancer-reference-root-coords) :copy-worldcoords))
            (imu-relative-to-world (send imu-relative-to-body :transform body-relative-to-world :world)))
       (send robot :move-coords
             (make-coords :pos (send imu-relative-to-world :worldpos)
                          :rot (ros::tf-quaternion->rot (send (cdr (assoc :imu robot-state)) :orientation)))
             (car (send robot :imu-sensors))))
     (send self :set-robot-state1
           :stabilizer-end-coords-list
           (mapcar #'(lambda (x) (send robot x :end-coords :copy-worldcoords)) limb-list)))
   ;; Actual root coords from Simulation's actual WAIST pos and rot, which cannot be used for real robots.
   (if (send self :parser-list "RobotHardware0_WAIST")
       (send self :set-robot-state1
             :actual-root-coords
             (let ((wpr (send (send self :parser-list "RobotHardware0_WAIST") :read-state)))
               (if (= (length wpr) 6) ;; If length = 6, rotation is rpy. Otherwise, 3x3 matrix.
                   (make-coords :pos (scale 1e3 (subseq wpr 0 3)) :rpy (list (elt wpr 5) (elt wpr 4) (elt wpr 3)))
                 (send self :hrpsys-tform->coords wpr)))))
   )
  (:hrpsys-tform->coords
   (tform-vector)
   (make-coords :pos (scale 1e3 (subseq tform-vector 0 3))
                :rot (make-matrix 3 3 (list (subseq tform-vector 3 6) (subseq tform-vector 6 9) (subseq tform-vector 9))))
   )
  (:reference-root-coords
   ()
   (cdr (assoc :reference-root-coords robot-state))
   )
  (:reference-worldcoords
   ()
   (send (send (send self :reference-root-coords) :copy-worldcoords)
         :transform
         (send (car (send robot :links)) :transformation robot))
   )
  (:auto-balancer-reference-vector () (cdr (assoc :auto-balancer-reference-vector robot-state)))
  (:auto-balancer-reference-root-coords () (cdr (assoc :auto-balancer-reference-root-coords robot-state)))
  (:auto-balancer-control-swing-support-time () (cdr (assoc :auto-balancer-control-swing-support-time robot-state)))
  (:auto-balancer-contact-states () (cdr (assoc :auto-balancer-contact-states robot-state)))
  (:auto-balancer-zmp () (cdr (assoc :auto-balancer-zmp robot-state)))
  (:auto-balancer-abs-zmp () (cdr (assoc :auto-balancer-abs-zmp robot-state)))
  (:auto-balancer-cog () (cdr (assoc :auto-balancer-cog robot-state)))
  (:auto-balancer-end-coords-list () (cdr (assoc :auto-balancer-end-coords-list robot-state)))
  (:stabilizer-reference-cog () (cdr (assoc :stabilizer-reference-cog robot-state)))
  (:stabilizer-cog () (cdr (assoc :stabilizer-cog robot-state)))
  (:stabilizer-reference-zmp () (cdr (assoc :stabilizer-reference-zmp robot-state)))
  (:stabilizer-zmp () (cdr (assoc :stabilizer-zmp robot-state)))
  (:stabilizer-newzmp () (cdr (assoc :stabilizer-newzmp robot-state)))
  (:stabilizer-cogvel () (cdr (assoc :stabilizer-cogvel robot-state)))
  (:stabilizer-reference-cogvel () (cdr (assoc :stabilizer-reference-cogvel robot-state)))
  (:stabilizer-act-base-rpy () (cdr (assoc :stabilizer-act-base-rpy robot-state)))
  (:stabilizer-current-base-rpy () (cdr (assoc :stabilizer-current-base-rpy robot-state)))
  (:stabilizer-debug-data () (cdr (assoc :stabilizer-debug-data robot-state)))
  (:stabilizer-end-coords-list () (cdr (assoc :stabilizer-end-coords-list robot-state)))
  (:servo-state () (cdr (assoc :servo-state robot-state)))
  (:actual-root-coords () (cdr (assoc :actual-root-coords robot-state)))
  ;; overwrite
  (:reference-vector () (cdr (assoc :reference-vector robot-state)))
  (:actual-vector () (send self :potentio-vector))
  (:error-vector () (v- (send self :potentio-vector) (send self :reference-vector)))
  (:time
   (&optional (str "sh_qOut"))
   (send (send self :parser-list str) :time)
   )
  )

(defun dump-seq-pattern-file-from-datalogger-log
  (fname &key (logger-instance *log*))
  "Dump SequencePlayer pattern file from data logger log.
   fname is basename for SequencePlayer pattern file."
  (let ((init-tm) (ret))
    (do-until-key-with-check
     (progn (null-output (send logger-instance :state)) (if (find-method logger-instance :eof-p) (not (send logger-instance :eof-p))))
     (unless init-tm
       (setq init-tm (send logger-instance :time)))
     (push (list :angle-vector (send logger-instance :reference-vector)
                 :time (+ (- (send (send logger-instance :time) :sec) (send init-tm :sec))
                          (/ (- (send (send logger-instance :time) :nsec) (send init-tm :nsec)) 1000000000.0)))
           ret))
    (dump-seq-pattern-file
     (reverse ret)
     fname)
    ))

;; (progn (setq *log* (instance datalogger-log-parser-controller :init "/tmp/testlog.sh_qOut")) (send *log* :state) (send *log* :potentio-vector))
