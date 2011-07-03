(ros::roseus "darwineus")
(load "package://pr2eus/pr2-interface.l")
(load "darwin.l")

(defclass darwin-interface
  :super ros-interface
  :slots ())
(defmethod darwin-interface
  (:init (&rest args)
	 (send-super :init :robot darwin-robot)))

(setq *ri* (instance darwin-interface :init))
(setq *dr* (instance darwin-robot :init))
(send *dr* :reset-pose)
(objects (list *dr*))
(send *dr* :head :neck-y :joint-angle 30)
(send *ri* :angle-vector (send *dr* :angle-vector) 3000)
(send *ri* :wait-interpolation)

(send *dr* :larm :shoulder-p :joint-angle 40)
(send *dr* :larm :elbow-p :joint-angle -130)
(send *ri* :angle-vector (send *dr* :angle-vector) 3000)
(send *ri* :wait-interpolation)

(send *dr* :legs :crotch-p :joint-angle 40)
(send *ri* :angle-vector (send *dr* :angle-vector) 3000)
(send *ri* :wait-interpolation)

(send *dr* :legs :crotch-p :joint-angle -40)
(send *ri* :angle-vector (send *dr* :angle-vector) 1000)
(send *ri* :wait-interpolation)
