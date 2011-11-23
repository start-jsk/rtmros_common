(ros::roseus "kawadahironx_pickbox")
(load "package://hrpsys_ros_bridge/scripts/rtm-ros-robot-interface.l")
(require :kawadahironx "package://hrpsys/scripts/kawada-hironx.l")

(defclass kawadahironx-interface
  :super robot-interface
  :slots ())
(defmethod kawadahironx-interface
  (:init (&rest args)
	  (send-super :init :robot hironx-robot)))

(setq *ri* (instance kawadahironx-interface :init))
(setq *hiro* (instance hironx-robot :init))
(send *hiro* :reset-pose)
(send *ri* :angle-vector (send *hiro* :angle-vector) 3000)
(objects (list *hiro*))
(send *ri* :wait-interpolation)
(send *hiro* :rarm :shoulder-p :joint-angle -40)
(send *hiro* :rarm :elbow-p :joint-angle -130)
(send *hiro* :head :neck-y :joint-angle -20)
(send *ri* :angle-vector (send *hiro* :angle-vector) 3000)
(objects (list *hiro*))
(send *ri* :wait-interpolation)
(warning-message 2 ";; type (exit) to finish program~%")
