(ros::roseus "darwin-pickup")
(load "package://roseus/euslisp/robot-interface.l")
(require :darwin "package://hrpsys/scripts/darwin.l")

(defclass darwin-interface
  :super robot-interface
  :slots ())
(defmethod darwin-interface
  (:init (&rest args)
	  (send-super :init :robot darwin-robot)))

(setq *ri* (instance darwin-interface :init))
(setq *sr* (instance darwin-robot :init))
(send *sr* :reset-pose)
(objects (list *sr*))
(send *sr* :rarm :shoulder-p :joint-angle 40)
(send *sr* :rarm :elbow-p :joint-angle -130)
(send *ri* :angle-vector (send *sr* :angle-vector) 3000)
(send *ri* :wait-interpolation)
(send *sr* :rarm :shoulder-p :joint-angle -65)
(send *sr* :rarm :shoulder-r :joint-angle -60)
(send *sr* :rarm :elbow-p :joint-angle -40)
(send *ri* :angle-vector (send *sr* :angle-vector) 3000)
(send *ri* :wait-interpolation)

