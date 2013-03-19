(ros::roseus "samplerobot_pickbox")
(load "package://hrpsys_ros_bridge/scripts/rtm-ros-robot-interface.l")
(require :samplerobot "package://hrpsys_ros_bridge/models/samplerobot.l")

(defclass samplerobot-interface
  :super rtm-ros-robot-interface
  :slots ())
(defmethod samplerobot-interface
  (:init (&rest args)
	  (send-super :init :robot samplerobot-robot)))

(setq *ri* (instance samplerobot-interface :init))
(setq *sr* (instance samplerobot-robot :init))
(send *sr* :reset-pose)
(send *sr* :torso :waist-y :joint-angle -30)
(send *sr* :torso :waist-p :joint-angle 10)
;;(objects (list *sr*))
(send *ri* :angle-vector (send *sr* :angle-vector) 3000)
(send *ri* :wait-interpolation)
(send *sr* :rarm :shoulder-p :joint-angle 40)
(send *sr* :rarm :elbow-p :joint-angle -130)
;;(objects (list *sr*))
(send *ri* :angle-vector (send *sr* :angle-vector) 3000)
(send *ri* :wait-interpolation)
(send *sr* :rarm :shoulder-p :joint-angle -65)
(send *sr* :rarm :shoulder-y :joint-angle 60)
(send *sr* :rarm :elbow-p :joint-angle -40)
(send *sr* :rarm :wrist-y :joint-angle -60)
(send *sr* :rarm :wrist-r :joint-angle  40)
;;(objects (list *sr*))
(send *ri* :angle-vector (send *sr* :angle-vector) 3000)
(send *ri* :wait-interpolation)

