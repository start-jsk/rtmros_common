(ros::roseus "pickbox")
(load "package://pr2eus/pr2-interface.l")
(load "samplerobot.l")

(defclass samplerobot-interface
  :super ros-interface
  :slots ())
(defmethod samplerobot-interface
  (:init (&rest args)
	  (send-super :init :robot samplerobot-robot)))

(setq *ri* (instance samplerobot-interface :init))
(setq *sr* (instance samplerobot-robot :init))
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

