(require :robot-interface "package://pr2eus/robot-interface.l")

(ros::load-ros-manifest "dynamic_reconfigure")
(ros::roseus-add-msgs "hrpsys_ros_bridge")
(ros::load-ros-manifest "hrpsys")

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
                     #'send self :rtmros-temperature-callback :groupname groupname)
     (mapcar #'(lambda (x)
                 (ros::subscribe (format nil "/~A" (string-downcase x)) geometry_msgs::WrenchStamped
                                 #'send self :rtmros-force-sensor-callback x :groupname groupname))
             (send-all (send robot :force-sensors) :name))
     ))
  (:rtmros-temperature-callback
   (msg)
   (send self :set-robot-state1 :temperature (send msg :temperature)))
  (:temperature-vector () (cdr (assoc :temperature robot-state)))
  (:rtmros-force-sensor-callback
   (fsensor-name msg)
   (let ((wrc (send msg :wrench)))
     (send self :set-robot-state1 fsensor-name
           (float-vector (send (send wrc :force) :x)
                         (send (send wrc :force) :y)
                         (send (send wrc :force) :z)
                         (send (send wrc :torque) :x)
                         (send (send wrc :torque) :y)
                         (send (send wrc :torque) :z)))))
  (:tmp-force-moment-vector
   (f/m &optional (limb))
   (if limb
       (subseq (cdr (assoc limb robot-state))
               (if (eq f/m :force) 0 3)
               (if (eq f/m :force) 3 6))
     (mapcar #'(lambda (x)
                 (subseq (cdr (assoc x robot-state))
                         (if (eq f/m :force) 0 3)
                         (if (eq f/m :force) 3 6)))
             (send-all (send robot :force-sensors) :name))))
  (:force-vector
   (&optional (limb))
   (send self :tmp-force-moment-vector :force limb))
  (:moment-vector
   (&optional (limb))
   (send self :tmp-force-moment-vector :moment limb))
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
  ;;
  ;; CollisionDetectorService.idl
  (:set-tolerance
   (&key (tolerance 0.1) (link_pair_name "all"))
   (let* ((req (instance hrpsys::OpenHRP_CollisionDetectorService_setToleranceRequest :init
			 :link_pair_name link_pair_name
			 :tolerance tolerance)))
     (ros::service-call "CollisionDetectorServiceROSBridge/setTolerance" req)))
  ;; for logger
  (:save-log
   (fname)
   (ros::service-call
    "DataLoggerServiceROSBridge/save"
    (instance hrpsys::Openhrp_Dataloggerservice_saveRequest :init :basename fname))
   )
  (:get-force-sensor-from-limb-name
   (limb)
   (find-if #'(lambda (x) (member (send x :parent) (send robot limb)))
            (send robot :force-sensors)))
  ;; impedance controller
  (:start-impedance
   (limb &rest args)
   (cond
    ((send self :get-force-sensor-from-limb-name limb)
     (let ((fs (send self :get-force-sensor-from-limb-name limb)))
       (send* self :setImpedanceControllerParam
              :name (string-downcase (send fs :name))
              :base-name (send (send (send (send robot limb :root-link) :parent) :joint) :name)
              :target-name (send (send (send fs :parent) :joint) :name) args)))
    ((eq limb :legs) (mapcar #'(lambda (l) (send* self :start-impedance l args)) '(:rleg :lleg)))
    ((eq limb :arms) (mapcar #'(lambda (l) (send* self :start-impedance l args)) '(:rarm :larm)))
    (t ) ;; virtual force sensor is not supported in this file
    ))
  (:stop-impedance
   (limb)
   (cond
    ((send self :get-force-sensor-from-limb-name limb)
     (let ((fs (send self :get-force-sensor-from-limb-name limb)))
       (send self :deleteImpedanceController
             :name (string-downcase (send fs :name)))))
    ((eq limb :legs) (mapcar #'(lambda (l) (send self :stop-impedance l)) '(:rleg :lleg)))
    ((eq limb :arms) (mapcar #'(lambda (l) (send self :stop-impedance l)) '(:rarm :larm)))
    (t ) ;; virtual force sensor is not supported in this file
    ))
  )

;; SequencePlayerService
(defmethod rtm-ros-robot-interface
  (:setBasePose
   (&optional base-coords (tm 0.1))
   (unless base-coords
     (setq base-coords (send robot :worldcoords)))
   (let ((pos_req (instance hrpsys::OpenHRP_SequencePlayerService_setBasePosRequest :init))
         (rpy_req (instance hrpsys::OpenHRP_SequencePlayerService_setBaseRpyRequest :init)))
     (send pos_req :pos (scale 0.001 (send base-coords :pos)))
     (send pos_req :tm tm)
     (let ((rpy (coerce (car (send base-coords :rpy-angle)) float-vector)))
       (send rpy_req :rpy rpy)
       (send rpy_req :tm tm))
     (ros::service-call
      "SequencePlayerServiceROSBridge/setBasePos" pos_req)
     (ros::service-call
      "SequencePlayerServiceROSBridge/setBaseRpy" rpy_req)
     ))
  (:waitInterpolationOfGroup (groupname)
   (let ((req (instance hrpsys::OpenHRP_SequencePlayerService_waitInterpolationOfGroupRequest :init)))
     (send req :gname groupname)
     (ros::service-call
      "SequencePlayerServiceROSBridge/waitInterpolationOfGroup" req)))
  ;; just testing code
  (:addJointGroup (groupname jnames)
   (let ((req (instance hrpsys::OpenHRP_SequencePlayerService_addJointGroupRequest :init)))
     (send req :gname groupname)
     (send req :jnames jnames)
     (ros::service-call
      "SequencePlayerServiceROSBridge/addJointGroup" req)))
  (:setJointAnglesOfGroup (groupname av tm)
   (let ((req (instance hrpsys::OpenHRP_SequencePlayerService_setJointAnglesOfGroupRequest :init)))
     (send req :gname groupname)
     (send req :jvs av)
     (send req :tm tm)
     (ros::service-call
      "SequencePlayerServiceROSBridge/setJointAnglesOfGroup" req)))
#| ;; angle group sample
  (send *ri* :addJointgroup "larm" (send-all (send *robot* :larm :joint-list) :name))
  (send *ri* :setJointAnglesOfGroup "larm" (scale (/ pi 180.0) (send *robot* :larm :angle-vector)) 4.0)
  (send *ri* :waitinterpolationofgroup "larm")
|#
  )

;; CollisionDetectorService
(defmethod rtm-ros-robot-interface
  (:enableCollisionDetection ()
   (let ((req (instance hrpsys::OpenHRP_CollisionDetectorService_enableCollisionDetectionRequest :init)))
     (ros::service-call
      "CollisionDetectorServiceROSBridge/enableCollisionDetection" req)))
  (:disableCollisionDetection ()
   (let ((req (instance hrpsys::OpenHRP_CollisionDetectorService_disableCollisionDetectionRequest :init)))
     (ros::service-call
      "CollisionDetectorServiceROSBridge/disableCollisionDetection" req)))
  )

;; RobotHardwareService
(defmethod rtm-ros-robot-interface
  (:setServoGainPercentage (name percentage)
   (let ((req (instance hrpsys::OpenHRP_RobotHardwareService_setServoGainPercentageRequest :init)))
     (send req :name name)
     (send req :percentage percentage)
     (ros::service-call
      "RobotHardwareServiceROSBridge/setServoGainPercentage" req)))
  (:removeForceSensorOffset
   ()
   (ros::service-call
    "RobotHardwareServiceROSBridge/removeForceSensorOffset"
    (instance hrpsys::Openhrp_RobotHardwareservice_removeForceSensorOffsetRequest :init)))
  )

;; ImpedanceControllerService
(defmethod rtm-ros-robot-interface
  (:getImpedanceControllerParam
   (name)
   (let ((req (instance hrpsys::OpenHRP_ImpedanceControllerService_getImpedanceControllerParamRequest :init :name name)))
     (send (ros::service-call "/ImpedanceControllerServiceROSBridge/getImpedanceControllerParam" req) :i_param)))
  ;; set impedance controller param
  ;;   params other than user-defined params are used in current parameter in ImpedanceController RTC by using getImpedanceControllerParam
  ;;   original default parameters are defined in hrpsys-base/rtc/ImpedanceController/ImpedancControl.h
  (:setImpedanceControllerParam
   (&key name base-name target-name
         M-p D-p K-p
         M-r D-r K-r
         ref-force ref-moment
         force-gain moment-gain
         sr-gain avoid-gain reference-gain manipulability-limit)
   (let* ((current-imp-param
           (send self :getImpedanceControllerParam name))
          (param (instance hrpsys::OpenHRP_ImpedanceControllerService_impedanceParam :init
                            :name name :base_name base-name :target_name target-name
                            :M_p (if M-p M-p (send current-imp-param :M_p))
                            :D_p (if D-p D-p (send current-imp-param :D_p))
                            :K_p (if K-p K-p (send current-imp-param :K_p))
                            :M_r (if M-r M-r (send current-imp-param :M_r))
                            :D_r (if D-r D-r (send current-imp-param :D_r))
                            :K_r (if K-r K-r (send current-imp-param :K_r))
                            :ref_force (if ref-force ref-force (send current-imp-param :ref_force))
                            :ref_moment (if ref-moment ref-moment (send current-imp-param :ref_moment))
                            :force_gain (if force-gain force-gain (send current-imp-param :force_gain))
                            :moment_gain (if moment-gain moment-gain (send current-imp-param :moment_gain))
                            :sr_gain (if sr-gain sr-gain (send current-imp-param :sr_gain))
                            :avoid_gain (if avoid-gain avoid-gain (send current-imp-param :avoid_gain))
                            :reference_gain (if reference-gain reference-gain (send current-imp-param :reference_gain))
                            :manipulability_limit (if manipulability-limit manipulability-limit (send current-imp-param :manipulability_limit))))
          (req (instance hrpsys::OpenHRP_ImpedanceControllerService_setImpedanceControllerParamRequest :init :i_param param)))
     (ros::service-call "/ImpedanceControllerServiceROSBridge/setImpedanceControllerParam" req)))
  (:deleteImpedanceController
   (&key name)
   (let ((req (instance hrpsys::OpenHRP_ImpedanceControllerService_deleteImpedanceControllerRequest :init :name name)))
     (ros::service-call "/ImpedanceControllerServiceROSBridge/deleteImpedanceController" req)))
  )


