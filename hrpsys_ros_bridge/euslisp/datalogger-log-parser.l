(defclass datalogger-log-parser
  :super propertied-object
  :slots (file index count read-state)
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
     t
     ))
  )

(defclass datalogger-log-parser-controller
  :super propertied-object
  :slots (parser-list robot-name)
  )

(defmethod datalogger-log-parser-controller
  (:init
   (fname)
   (let ((dir-str (format nil "/~A" (reduce #'(lambda (x y) (format nil "~A/~A" x y)) (cdr (pathname-directory fname)))))
         (fname-prefix (pathname-name fname)))
     (setq parser-list
           (mapcar #'(lambda (x) (instance datalogger-log-parser :init (format nil "~A/~A" dir-str x)))
                   (remove-if-not #'(lambda (x) (substringp (format nil "~A." fname-prefix) x)) (directory dir-str))))
     (setq robot-name
           (string-right-trim "(" (string-right-trim "Robot)0" (pathname-name (substitute #\. #\_ (pathname-type (find-if #'(lambda (x) (substringp "(Robot)" x)) (send-all (send self :parser-list) :fname))))))))
     ))
  (:parser-list
   (&optional (str))
   (if str
       (find-if #'(lambda (x) (string= str (pathname-type (send x :fname)))) parser-list)
     parser-list))
  (:robot-name () robot-name)
  (:state
   ()
   (send-all parser-list :proc)
   )
  ;;
  (:reference-vector
   ()
   (send (send self :parser-list "sh_qOut") :read-state)
   )
  (:potentio-vector
   ()
   (send (send self :parser-list (format nil "~A(Robot)0_q" robot-name)) :read-state)
   )
  )

;; (progn (setq *log* (instance datalogger-log-parser-controller :init "/tmp/testlog.sh_qOut")) (send *log* :state) (send *log* :potentio-vector))