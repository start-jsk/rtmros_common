(load "package://hrpsys_gazebo/euslisp/atlas-interface.l")
;; application for task1
;;(load "package://hrpsys_gazebo/euslisp/atlas-calc-position.l")

(defun wobj (obj) ;; write object to standard-output
  (dump-structure *standard-output* obj))

(defun simple-init ()
  (atlas-init :set-reset-pose nil
              :set-user-mode nil
              :unset-limit nil)
  t)

(defun init-task ()
  (my-init)
  (start-walk)
  t
  )

(defun calc-size (obj)
  (setq *prev-obj* obj)
  (let ((str-strm (make-string-output-stream)))
    (dump-structure str-strm obj)
    (setq *prev-strm* str-strm)
    (print (send str-strm :count))
    ))

(defun get-prev ()
  (when *prev-strm*
    (prog1
        (format *standard-output* "~A~%" (get-output-stream-string *prev-strm*))
      (setq *prev-strm* nil
            *prev-obj* nil))
    ))

(defun prev-size ()
  (when *prev-strm*
    (print (send *prev-strm* :count))))
