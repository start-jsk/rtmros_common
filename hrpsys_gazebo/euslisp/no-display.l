(defclass no-display-viewer
  :super propertied-object
  :slots ())
(labels ((find-all-methods
	  (x)
	  (append (mapcar #'car (send (eval x) :methods))
		  (if (send (eval x) :super)
		      (find-all-methods (send (eval x) :super))))))
  (let ((all-methods
	 (remove-duplicates
	  (apply #'append (mapcar #'(lambda (x) (find-all-methods x))
				  '(viewer x::irtviewer))))))
    (dolist (mth all-methods)
      (eval
       `(defmethod no-display-viewer
	  (,mth (&rest args))))
      )))
(defmethod no-display-viewer
  (:init (&rest args) self))
;; :draw-on overwrite
(dolist (s (mapcar #'car (documentation ':draw-on)))
  (eval `(defmethod ,(send s :name)
	   (:draw-on (&rest args)))))
;; functions
(defun x::objects (&rest args))
(defun make-irtviewer (&rest args)
  (setq *viewer* (instance no-display-viewer :init))
  (setq *irtviewer* (instance no-display-viewer :init)))
(defun x::window-main-one (&rest args))
