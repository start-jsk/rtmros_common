(require "../atlas-interface.l")
(require "../atlas-hihi.l") ;; for simple-fullbody and model2real

(ros::roseus "spacenav_controller")
(ros::roseus-add-msgs "sensor_msgs")

(defvar *spacenav-controller-centroid*
  (copy-seq (send *robot* :centroid)))
(defvar *spacenav-controller-rpy*
  (coerce (car (rpy-angle (send *robot* :rarm :end-coords :worldrot)))
	  float-vector))

(defvar *ik-target* :rarm)
(defvar *ik-mode* nil)
(defvar *ik-rotation-axis* nil)

(mapcar
  #'(lambda (j)
      (send j :min-angle -1000)
      (send j :max-angle 1000))
 (send *robot* :joint-list))

(defun spacenav-controller-callback
  (msg)
  (describe msg)
  (if (> (aref (send msg :buttons) 0) 0.1)
      ;; implement here
      (send *ri* :start-grasp)
    )
  (if (> (aref (send msg :buttons) 1) 0.1)
      ;; implement here
      (send *ri* :stop-grasp)
    )
  (let
      ((newcoords
	(let ((move
	       (scale
		30
		(coerce
		 (subseq (send msg :axes) 0 3)
		 float-vector)))
	      (rpy
	       (reverse
		(scale
		 0.15
		 (coerce
		  (subseq (send msg :axes) 3 6)
		  float-vector)))))
	  (make-coords
	   :pos
	   (v+
	    (send *robot* *ik-target* :end-coords :worldpos)
	    (if (> (norm move) 30)
		(scale 30 (normalize-vector move))
	      move))
	   :rpy
	   (setq
	    *spacenav-controller-rpy*
	    (v+
	     *spacenav-controller-rpy*
	     (if (> (norm rpy) 0.15)
		 (scale 0.15 (normalize-vector rpy))
	       rpy)))))))
    (if (eq *ik-target* :fullbody)
	(simple-fullbody
	 :robot *robot*
	 :target
	 (mapcar
	  #'(lambda (k)
	      (if (eq k *ik-target*)
		  (list (cons :target k)
			(cons :coords newcoords))
		(list (cons :target k))))
	  '(:rarm :larm :rleg :lleg))
	 :balence-leg nil
	 :debug-view :no-message
	 :stop 10
	 :revert-if-fail nil
	 :centroid-thre 30
	 :target-centroid-pos *spacenav-controller-centroid*)
      (send *robot* *ik-target* :inverse-kinematics
	    newcoords
	    :rotation-axis *ik-rotation-axis*
	    )))
  (if (and (boundp '*irtviewer*) *irtviewer*)
      (progn
	(send *irtviewer* :draw-objects)
	(x::window-main-one)))
  (send *ri* :angle-vector
	(send *robot* :angle-vector)
	50)
  (send *ri* :wait-interpolation)
  )

(ros::subscribe
 "/spacenav/joy"
 sensor_msgs::Joy
 #'spacenav-controller-callback)

(defun controller-start
  nil
;  (send *robot* :angle-vector
;	(send *ri* :potentio-vector))
  (if (find-method *ri* :hand-state)
      (progn
	(send *robot* :rarm :hand :angle-vector
	      (subseq (send *ri* :hand-state :potentio-vector) 0 6))
	(send *robot* :larm :hand :angle-vector
	      (subseq (send *ri* :hand-state :potentio-vector) 6 12))))
  (setq *spacenav-controller-centroid* (copy-seq (send *robot* :centroid)))
  (setq *spacenav-controller-rpy*
	(coerce
	 (car
	  (rpy-angle (send *robot* *ik-target* :end-coords :worldrot)))
	 float-vector))
  (model2real :sleep-time 1000)
  (send *ri* :wait-interpolation)
  (do-until-key
   (ros::spin-once)
   (ros::sleep))
  )

(format t
	(concatenate
	 string
	 "[usage]"
	 "  " "this requires two shell.~%"
	 "  " "first sh spacenav.sh~%"
	 "  " "second roseus spacenav-controller.l and call controller-start~%"))