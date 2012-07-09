(load "package://pr2eus/robot-interface.l")

(ros::load-ros-manifest "dynamic_reconfigure")
(ros::roseus-add-msgs "hrpsys_ros_bridge")
(ros::roseus-add-srvs "hrpsys")

(defclass rtm-ros-robot-interface
  :super robot-interface
  :slots ()
  )

(defmethod rtm-ros-robot-interface
  (:init
   (&rest args)
   (prog1
       (send-super* :init args)
     (ros::subscribe "/motor_states" hrpsys_ros_bridge::MotorStates
       #'send self :ros-state-callback-rtmros :groupname groupname)
     ))
  (:ros-state-callback-rtmros
   (msg)
   (dolist (key '(:temperature))
     (send self :set-robot-state1 key (send msg key))))
  (:temperature-vector () (cdr (assoc :temperature robot-state)))
  (:sendmsg
   (strs &optional (service-name "sendmsg"))
   (ros::service-call
    service-name
    (let ((req (instance dynamic_reconfigure::ReconfigureRequest :init)))
      (send (send req :config) :strs
	    (mapcar #'(lambda (str)
			(instance dynamic_reconfigure::StrParameter :init :name service-name :value str))
		    strs))
      req)))
  (:set-interpolation-mode (mode) (send self :sendmsg (list "setInterpolationMode" (format nil "~A" mode))))
  ;; just for seq debug
  (:wait-interpolation-for-seq-debug () (send self :sendmsg (list "waitInterpolation" "")))
  (:angle-vector-for-seq-debug
   (v tm)
   (send self :sendmsg
	 (list "setJointAngles"
	       (let ((str""))
		 (dotimes (i (length v)) (setq str (format nil "~A ~A" str (deg2rad (elt v i)))))
		 (format nil "~A ~A" str (* 1e-3 tm))))))
  ;; for logger
  (:save-log
   (fname)
   (ros::service-call
    "DataLoggerServiceROSBridge/save"
    (instance hrpsys::Openhrp_Dataloggerservice_saveRequest :init :basename fname))
   )
  )
