
;; \file    atlas-vehicle-motion-utils.l
;; \author  Murooka Masaki
;; \date    2013/06/08
;; \brief   vrc_task_1関連関数 utils
;;


;; force

(defun get-axis-force
  (&optional (limb :larm) (axis (float-vector 0 0 1)))

  (when *lr-reverse*
    (setq axis (copy-seq axis))
    (setf (elt axis 1) (- (elt axis 1))))
  (let* ((idx (case limb (:larm 0) (:rarm 1) (:lleg 2) (:rleg 3)))
	 (sensor-coords (send (elt (send *atlas* :force-sensors) idx) :copy-worldcoords))
	 (sensor-pos (send sensor-coords :worldpos))
	 (sensor-rpy (car (rpy-angle (send sensor-coords :rot)))))
    (when *lr-reverse*
      (setq sensor-coords (make-coords :pos (float-vector (elt sensor-pos 0) (- (elt sensor-pos 1)) (elt sensor-pos 2))
				       :rpy (list (- (elt sensor-rpy 0)) (elt sensor-rpy 1) (- (elt sensor-rpy 2)))))
      (setq idx (case idx (0 1) (1 0) (2 3) (3 2))))
    (v. (send sensor-coords
	      :rotate-vector (elt (send *ri* :state :force-vector) idx)) (normalize-vector axis))))

(defun adapt-leg
  (&optional (limb :rleg))

  (let* ((fz (get-axis-force limb (float-vector 0 0 1))))
    (if (> fz 200)
	(set-middle-gain limb) (set-zero-gain limb))
    ;; (unix::usleep (* 5 1000 1000))
    (unix::usleep (* 3 1000 1000))
    (real2model)
    (model2real :wait t)
    (send *irtviewer* :draw-objects)
    (set-high-gain limb)
    ))

(defun reach-until-touch
  (&optional (limb :larm) (axis (float-vector 0 0 1)) ;; axis is moving direction
	     (fthre 100) (dx 5) (ik-time 200))

  (do-until-key
   (let* ((f (get-axis-force limb (v- axis))))
     (warning-message 6 "[reach-until-touch] f: ~a / ~a  limb: ~a  axis: ~a~%" f fthre limb axis)
     (if (> f fthre) (return))
     (unless
	 (send *atlas* limb :inverse-kinematics
	       (send (send (send *atlas* limb :end-coords) :copy-worldcoords) :translate (scale dx (normalize-vector axis)) :world)
               :revert-if-fail nil
	       :centroid-thre 500)
       (warning-message 1 "[reach-until-touch] ik failed~%")
       (return))
     (fix-leg-vehicle-side)
     (move-centroid)
     (send *irtviewer* :draw-objects)
     (model2real :time ik-time :wait t))))

;; (defun reach-until-touch
;;   (&optional (limb :larm) (axis (float-vector 0 0 1)) ;; axis is moving direction
;; 	     (fthre 100) (dx 5) (ik-time 200))

;;   (do-until-key
;;    (let* ((f (get-axis-force limb (v- axis))))
;;      (warning-message 6 "[reach-until-touch] f: ~a / ~a  limb: ~a  axis: ~a~%" f fthre limb axis)
;;      (if (> f fthre) (return))
;;      (unless
;; 	 (let* ((tc (list (send (send (send *atlas* limb :end-coords) :copy-worldcoords) :translate (scale dx (normalize-vector axis)) :world)
;; 			  nil
;; 			  nil
;; 			  nil)))
;; 	   (with-move-target-link-list
;; 	    (mt ll *atlas* (append (list limb) (remove limb '(:rleg :lleg :rarm :larm))))
;; 	    (send *atlas* :fullbody-inverse-kinematics
;; 		  (mapcar #'(lambda (c m) (if c c (send m :copy-worldcoords))) tc mt)
;; 		  :link-list ll :move-target mt
;; 		  :collision-avoidance-link-pair nil
;; 		  :translation-axis '(t t t t)
;; 		  :rotation-axis '(t t t t)
;; 		  :centroid-thre 500)))
;;        (warning-message 1 "[reach-until-touch] ik failed~%")
;;        (return))
;;      (send *irtviewer* :draw-objects)
;;      (model2real :time ik-time :wait t))))


;; torque

(defun init-torque-control
  ()
  (ros::roseus-add-msgs "atlas_msgs")
  (ros::advertise "/atlas/atlas_command" atlas_msgs::AtlasCommand 1))

(defun set-model-torque
  (&optional (scale-num 1))
  (let ((effort-vec (instantiate float-vector 28))
	(msg (instance atlas_msgs::AtlasCommand :init)))
    (mapcar #'(lambda (j e)
		(setf (elt effort-vec (eval (read-from-string (format nil "atlas_msgs::AtlasState::*~a*" j)))) e))
	    (send-all (send *atlas* :joint-list) :name) (concatenate cons (send *atlas* :torque-vector)))
    (send msg :effort (scale scale-num effort-vec))
    (ros::publish "/atlas/atlas_command" msg)))

(defun set-zero-torque
  ()
  (let ((effort-vec
	 (instantiate float-vector 28))
	(msg (instance atlas_msgs::AtlasCommand :init)))
    (send msg :effort effort-vec)
    (ros::publish "/atlas/atlas_command" msg)))
