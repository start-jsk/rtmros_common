(ros::roseus "hrp4c_pickbox")
(load "package://hrpsys_ros_bridge/scripts/rtm-ros-robot-interface.l")
(require :hrp4c "package://hrpsys/scripts/hrp4c.l")

(defclass hrp4c-interface
  :super robot-interface
  :slots ())
(defmethod hrp4c-interface
  (:init (&rest args)
	  (send-super :init :robot hrp4-robot)))

(setq *ri* (instance hrp4c-interface :init))
(setq *sr* (instance hrp4-robot :init))
(send *sr* :reset-pose)
(objects (list *sr*))
(send *sr* :torso :waist-y :joint-angle -30)
(send *sr* :torso :waist-p :joint-angle 10)
(send *ri* :angle-vector (send *sr* :angle-vector) 3000)
(send *ri* :wait-interpolation)
(send *sr* :rarm :shoulder-p :joint-angle 40)
(send *sr* :rarm :elbow-p :joint-angle -130)
(send *ri* :angle-vector (send *sr* :angle-vector) 3000)
(send *ri* :wait-interpolation)
(send *sr* :rarm :shoulder-p :joint-angle -65)
(send *sr* :rarm :shoulder-y :joint-angle 60)
(send *sr* :rarm :elbow-p :joint-angle -40)
(send *sr* :rarm :wrist-y :joint-angle -60)
(send *sr* :rarm :wrist-r :joint-angle  40)
(send *ri* :angle-vector (send *sr* :angle-vector) 3000)
(send *ri* :wait-interpolation)

