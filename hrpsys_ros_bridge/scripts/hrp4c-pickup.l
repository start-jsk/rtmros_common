(load "package://hrpsys_ros_bridge/scripts/rtm-ros-robot-interface.l")
(require :hrp4c "package://hrpsys/scripts/hrp4c.l")

(defclass hrp4c-interface
  :super robot-interface
  :slots ())
(defmethod hrp4c-interface
  (:init (&rest args)
	  (send-super :init :robot hrp4-robot)))

(defun hrp4c-init ()
  (if (not (boundp '*ri*))
      (setq *ri* (instance hrp4c-interface :init)))
  (if (not (boundp '*hrp4c*))
      (setq *hrp4c* (instance hrp4-robot :init)))
  (send *hrp4c* :reset-pose)
  (objects (list *hrp4c*))
  ;; send reset pose to real robot
  (send *ri* :angle-vector (send *hrp4c* :angle-vector) 3000)
  (send *ri* :wait-interpolation)
  )

(defun hrp4c-lookdown ()
  ;; reset manip pose
  (send *hrp4c* :torso :waist-y :joint-angle -30)
  (send *hrp4c* :arms :shoulder-p :joint-angle 40)
  (send *hrp4c* :arms :elbow-p :joint-angle -90)
  (send *hrp4c* :head :neck-p :joint-angle -30)
  (objects (list *hrp4c*))
  (send *ri* :angle-vector (send *hrp4c* :angle-vector) 3000)
  (send *ri* :wait-interpolation)
  )

(defun hrp4c-pickup ()
  (send *hrp4c* :inverse-kinematics
	(make-coords :pos #f(400 -200 600))
	:link-list (send *hrp4c* :link-list (send *hrp4c* :rarm :end-coords :parent))
	:move-target (send *hrp4c* :rarm :end-coords)
	:rotation-axis nil
	:look-at-target t
	:debug-view t)
  (objects (list *hrp4c*))
  (send *ri* :angle-vector (send *hrp4c* :angle-vector) 3000)
  (send *ri* :wait-interpolation)
  )

(ros::roseus "hrp4c_pickup")
(hrp4c-init)
(hrp4c-lookdown)
(hrp4c-pickup)
(hrp4c-init)
