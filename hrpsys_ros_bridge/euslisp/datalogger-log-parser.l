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
  :super propertied-object
  :slots (parser-list robothardware-name)
  )

(defmethod datalogger-log-parser-controller
  (:init
   (fname)
   (let* ((dir-str (format nil "/~A" (reduce #'(lambda (x y) (format nil "~A/~A" x y)) (cdr (pathname-directory fname)))))
          (fname-prefix (pathname-name fname)))
     ;; gather fname list with max lines
     (let* ((fname-candidate-list (remove-if-not #'(lambda (x) (substringp (format nil "~A." fname-prefix) x)) (directory dir-str)))
            (fname-liens-without-rh
             (mapcar #'(lambda (f)
                         (let ((r (car (piped-fork-returns-list (format nil "LANG=C wc -l ~A/~A" dir-str f)))))
                           (list (read-from-string (string-right-trim (string-left-trim "0123456789 " r) r)) f)))
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
  (:state
   ()
   (send-all parser-list :proc)
   )
  ;;
  (:reference-vector
   ()
   (map float-vector #'rad2deg (send (send self :parser-list "sh_qOut") :read-state))
   )
  (:potentio-vector
   ()
   (map float-vector #'rad2deg
        (send (send self :parser-list (format nil "~A_q" robothardware-name)) :read-state))
   )
  (:reference-torque-vector
   ()
   (send (send self :parser-list "sh_tqOut") :read-state)
   )
  (:root-pos
   ()
   (scale 1e3 (send (send self :parser-list "sh_basePosOut") :read-state))
   )
  (:root-rpy
   ()
   (send (send self :parser-list "sh_baseRpyOut") :read-state)
   )
  (:root-coords
   ()
   (make-coords :pos (send self :root-pos) :rpy (send self :root-rpy))
   )
  (:time
   (&optional (str "sh_qOut"))
   (send (send self :parser-list str) :time)
   )
  )

;; (progn (setq *log* (instance datalogger-log-parser-controller :init "/tmp/testlog.sh_qOut")) (send *log* :state) (send *log* :potentio-vector))