(ros::roseus "darwin_pickup")
(load "package://hrpsys_ros_bridge/scripts/rtm-ros-robot-interface.l")
(require :darwin "package://hrpsys_ros_bridge/models/darwin.l")

(defclass darwin-interface
  :super robot-interface
  :slots ())
(defmethod darwin-interface
  (:init (&rest args)
	  (send-super :init :robot darwin-robot)))

(setq *ri* (instance darwin-interface :init))
(setq *darwin* (instance darwin-robot :init))
(send *darwin* :reset-pose)
(make-irtviewer :width 320 :height 320)
(objects (list *darwin*))
(send *ri* :angle-vector (send *darwin* :angle-vector) 3000)
(send *ri* :wait-interpolation)
;;
(send *darwin* :rarm :shoulder-p :joint-angle 40)
(send *darwin* :rarm :elbow-p :joint-angle -130)
(objects (list *darwin*))
(send *ri* :angle-vector (send *darwin* :angle-vector) 3000)
(send *ri* :wait-interpolation)
;;
(send *darwin* :rarm :shoulder-p :joint-angle -65)
(send *darwin* :rarm :shoulder-r :joint-angle -60)
(send *darwin* :rarm :elbow-p :joint-angle -40)
(objects (list *darwin*))
(send *ri* :angle-vector (send *darwin* :angle-vector) 3000)
(send *ri* :wait-interpolation)

