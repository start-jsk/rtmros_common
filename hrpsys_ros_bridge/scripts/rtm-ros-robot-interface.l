(load "package://roseus/euslisp/robot-interface.l")

(ros::load-ros-manifest "dynamic_reconfigure")
(defmethod robot-interface
  (:sendmsg
   (strs)
   (ros::service-call
    "sendmsg"
    (let ((req (instance dynamic_reconfigure::ReconfigureRequest :init)))
      (send (send req :config) :strs
	    (mapcar #'(lambda (str)
			(instance dynamic_reconfigure::StrParameter :init :name "sendmsg" :value str))
		    strs))
      req)))
  (:set-interpolation-mode (mode) (send self :sendmsg (list "setInterpolationMode" mode)))
  )
