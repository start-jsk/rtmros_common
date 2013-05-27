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
     (send self :define-all-ROSBridge-srv-methods)
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
       (subseq (cdr (assoc (send (send self :get-force-sensor-from-limb-name limb) :name) robot-state))
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
  ;; automatically define methods for ROSBridge services
  (:define-all-ROSBridge-srv-methods
   (&key (debug-view nil) (ros-pkg-name "hrpsys"))
   (let ((srv-fnames (send self :get-ROSBridge-srv-fnames ros-pkg-name)))
     (dolist (idl (send self :get-ROSBridge-idl-fnames ros-pkg-name))
       (let ((rtc-name (pathname-name idl)))
         (dolist (srv-name (mapcar #'pathname-name (remove-if-not #'(lambda (x) (substringp rtc-name x)) srv-fnames)))
           (let ((method-def (send self :get-ROSBridge-method-def-macro rtc-name srv-name)))
             (when method-def
               (if debug-view (pprint (macroexpand method-def)))
               (eval method-def)
               )))))))
  (:get-ROSBridge-fnames-from-type
   (type-name &optional (ros-pkg-name "hrpsys"))
   (let ((path (ros::resolve-ros-path (format nil "package://~A" ros-pkg-name))))
     (remove-if-not #'(lambda (x) (substringp (format nil ".~A" type-name) x)) (directory (format nil "~A/~A" path type-name)))
     ))
  (:get-ROSBridge-idl-fnames (&optional (ros-pkg-name "hrpsys")) (send self :get-ROSBridge-fnames-from-type "idl" ros-pkg-name))
  (:get-ROSBridge-srv-fnames (&optional (ros-pkg-name "hrpsys")) (send self :get-ROSBridge-fnames-from-type "srv" ros-pkg-name))
  (:get-ROSBridge-method-def-macro
   (rtc-name srv-name)
   (let* ((meth-name (string-left-trim "_" (string-left-trim rtc-name (string-left-trim "_" (string-left-trim "OpenHRP" srv-name)))))
          (srv-request (read-from-string (format nil "hrpsys::~ARequest" srv-name)))
          (init-method (find-if #'(lambda (x) (eq (car x) :init)) (send (eval srv-request) :methods)))
          ;;(new-method-name (read-from-string (format nil ":~A" (string-left-trim "_" (string-left-trim rtc-name (string-left-trim "_" (string-left-trim "OpenHRP" srv-name))))))))
          (new-method-name (read-from-string (format nil ":~A" (string-left-trim "_" (string-left-trim "OpenHRP" srv-name))))))
     (if (find-method self new-method-name)
         (progn
           (warn ";; Method conflict in ROSBridge defmethod!! ;; ~A~%" srv-name)
           nil)
       `(defmethod rtm-ros-robot-interface
          (,new-method-name
           ,(cadr init-method)
           (ros::service-call
            ,(format nil "/~AROSBridge/~A" rtc-name meth-name)
            (instance ,(eval srv-request) :init ,@(mapcan #'(lambda (x) (list (caar x) (cadar x))) (cdadr init-method))))
           )
          )
       )))
  ;;
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
  ;; dump pattern file for SequencePlayer
  ;;  rs-list : list of (list :time time0 :angle-vector av ...)
  ;;  output-basename : output file (output-basename.pos, ...)
  (:dump-seq-pattern-file
   (rs-list output-basename &key (initial-sync-tile 3.0)) ;; [s]
   (with-open-file
    (f (format nil "~A.pos" output-basename) :direction :output)
    (dolist (l rs-list)
      (format f "~A " (+ initial-sync-tile (cadr (memq :time l))))
      (let ((av (cadr (memq :angle-vector l))))
        (dotimes (i (length av))
          (format f "~A " (deg2rad (elt av i)))
          ))
      (format f "~%")
      )))
  ;;
  (:get-force-sensor-from-limb-name
   (limb)
   (if (find-method robot limb)
       (find-if #'(lambda (x) (member (send x :parent) (send robot limb)))
                (send robot :force-sensors))))
  (:state
    (&rest args)
    (case (car args)
      (:imucoords
        (send *tfl* :lookup-transform "/imu_floor" "/BODY" (ros::time)))
      (t
        (send-super* :state args))))
  )

;; SequencePlayerService
(defmethod rtm-ros-robot-interface
  (:set-base-pose
   (&optional base-coords (tm 0.1))
   (unless base-coords
     (setq base-coords (send robot :worldcoords)))
   (send self :sequenceplayerservice_setbasepos :pos (scale 0.001 (send base-coords :pos)) :tm tm)
   (send self :sequenceplayerservice_setbaserpy :rpy (coerce (car (send base-coords :rpy-angle)) float-vector) :tm tm))
  (:wait-interpolation-of-group
   (groupname)
   (send self :sequenceplayerservice_waitinterpolationofgroup :gname groupname))
  ;; just testing code
  (:add-joint-group
   (groupname jnames)
   (send self :sequenceplayerservice_addjointgroup :gname groupname :jnames jnames))
  (:set-joint-angles-of-group
   (groupname av tm)
   (send self :sequenceplayerservice_setjointangle :gname groupname :jvs av :tm tm))
  (:load-pattern (basename) (send self :sequenceplayerservice_loadpattern :basename basename))
  ;; deprecated
  (:setBasePose
   (&optional base-coords (tm 0.1))
   (warning-message 1 ";; :setBasePose is deprecated!! Please use :set-base-pose !!~%")
   (send self :set-base-pose base-coords tm))
  (:waitInterpolationOfGroup
   (groupname)
   (warning-message 1 ";; :waitInterpolationOfGroup is deprecated!! Please use :wait-interpolation-of-group!!~%")
   (send self :wait-interpolation-of-group groupname))
  (:addJointGroup
   (groupname jnames)
   (warning-message 1 ";; :addJointGroup is deprecated!! Please use :add-joint-group!!~%")
   (send self :add-joint-group groupname jnames))
  (:setJointAnglesOfGroup
   (groupname av tm)
   (warning-message 1 ";; :setJointAnglesOfGroup is deprecated!! Please use :set-joint-angles-of-group !!~%")
   (send self :set-joint-angles-of-group groupname av tm))
#| ;; angle group sample
  (send *ri* :add-joint-group "larm" (send-all (send *robot* :larm :joint-list) :name))
  (send *ri* :set-jointangles-of-group "larm" (scale (/ pi 180.0) (send *robot* :larm :angle-vector)) 4.0)
  (send *ri* :waitinterpolation-of-group "larm")
|#
  )

;; CollisionDetectorService
(defmethod rtm-ros-robot-interface
  (:set-tolerance
   (&key (tolerance 0.1) (link-pair-name "all"))
   (send self :collisiondetectorservice_settolerance
         :link_pair_name link-pair-name :tolerance tolerance))
  (:start-collision-detection
   ()
   (send self :collisiondetectorservice_enablecollisiondetection))
  (:stop-collision-detection
   ()
   (send self :collisiondetectorservice_disablecollisiondetection))
  ;; deprecated
  (:enableCollisionDetection
   ()
   (warning-message 1 ";; :enableCollisionDetection is deprecated!! Please use :start-collision-detection!!~%")
   (send self :start-collision-detection))
  (:disableCollisionDetection ()
   (warning-message 1 ";; :disableCollisionDetection is deprecated!! Please use :stop-collision-detection!!~%")
   (send self :stop-collision-detection))
  )

;; DataLoggerService
(defmethod rtm-ros-robot-interface
  (:save-log (fname) (send self :dataloggerservice_save :basename fname))
  ;; start log by clearing log
  (:start-log () (send self :dataloggerservice_clear))
  (:set-log-maxlength
   (&optional (maxlength 4000))
   (send self :dataloggerservice_maxlength :len maxlength))
  )

;; RobotHardwareService
(defmethod rtm-ros-robot-interface
  (:set-servo-gain-percentage
   (name percentage)
   (send self :robothardwareservice_setservogainpercentage :name name :percentage percentage))
  (:remove-force-sensor-offset () (send self :robothardwareservice_removeforcesensoroffset))
  (:set-servo-error-limit
    (name limit)
    (send self :robothardwareservice_setservoerrorlimit :name name :limit limit)
    )
  ;; deprecated
  (:setServoGainPercentage
   (name percentage)
   (warning-message 1 ";; :setServoGainPercentage is deprecated!! Please use :set-servo-gain-percentage!!~%")
   (send self :set-servo-gain-percentage name percentage))
  )

;; ImpedanceControllerService
(defmethod rtm-ros-robot-interface
  (:get-impedance-controller-param (name) (send (send self :impedancecontrollerservice_getimpedancecontrollerparam :name name) :i_param))
  ;;
  ;; usage:
  ;;   (send *ri* :start-impedance :rarm ..), (send *ri* :stop-impedance :rarm ..), (send *ri* :set-impedance-controller-param :rarm :M-p 200) ...
  ;;
  ;; set impedance controller param
  ;;   params other than user-defined params are used in current parameter in ImpedanceController RTC by using getImpedanceControllerParam
  ;;   original default parameters are defined in hrpsys-base/rtc/ImpedanceController/ImpedancControl.h
  (:_set-impedance-controller-param
   (&key name base-name target-name
         M-p D-p K-p
         M-r D-r K-r
         ref-force ref-moment
         force-gain moment-gain
         sr-gain avoid-gain reference-gain manipulability-limit)
   (let* ((current-imp-param (send self :get-impedance-controller-param name))
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
                            :manipulability_limit (if manipulability-limit manipulability-limit (send current-imp-param :manipulability_limit)))))
     (send self :impedancecontrollerservice_setimpedancecontrollerparam :i_param param)))
  (:set-impedance-controller-param
   (limb &rest args)
   (cond
    ((send self :get-force-sensor-from-limb-name limb)
     (let* ((fs (send self :get-force-sensor-from-limb-name limb))
            (base-name (send (send (send (send robot limb :root-link) :parent) :joint) :name)))
       (send* self :_set-impedance-controller-param
              :name (string-downcase (send fs :name))
              :base-name (if base-name base-name "WAIST")
              :target-name (send (send (send fs :parent) :joint) :name) args)))
    ((and (substringp ":v" (format nil "~A" limb))
          (find-method robot (read-from-string (format nil ":~A" (string-left-trim ":v" (format nil "~A" limb))))))
     (let ((rlimb (read-from-string (format nil ":~A" (string-left-trim ":v" (format nil "~A" limb))))))
       (send* self :_set-impedance-controller-param
              :name (format nil "v~C~A" (elt (string-left-trim ":v" (format nil "~A" limb)) 0) (case rlimb ((:rarm :larm) "hsensor") (t "fsensor")))
              :base-name (send (send (send (send robot rlimb :root-link) :parent) :joint) :name)
              :target-name (send (send (send (send robot rlimb :end-coords) :parent) :joint) :name)
              args)))
    ((eq limb :legs) (mapcar #'(lambda (l) (send* self :start-impedance l args)) '(:rleg :lleg)))
    ((eq limb :arms) (mapcar #'(lambda (l) (send* self :start-impedance l args)) '(:rarm :larm)))
    (t ) ;; virtual force sensor is not supported in this file
    ))
  (:start-impedance
   (limb &rest args)
   (send* self :set-impedance-controller-param limb args))
  (:stop-impedance
   (limb)
   (cond
    ((send self :get-force-sensor-from-limb-name limb)
     (let ((fs (send self :get-force-sensor-from-limb-name limb)))
       (send self :impedancecontrollerservice_deleteimpedancecontroller
             :name (string-downcase (send fs :name)))))
    ((and (substringp ":v" (format nil "~A" limb))
          (find-method robot (read-from-string (format nil ":~A" (string-left-trim ":v" (format nil "~A" limb))))))
     (let ((rlimb (read-from-string (format nil ":~A" (string-left-trim ":v" (format nil "~A" limb))))))
       (send self :impedancecontrollerservice_deleteimpedancecontroller
	     :name (format nil "v~C~A" (elt (string-left-trim ":v" (format nil "~A" limb)) 0) (case rlimb ((:rarm :larm) "hsensor") (t "fsensor"))))))
    ((eq limb :legs) (mapcar #'(lambda (l) (send self :stop-impedance l)) '(:rleg :lleg)))
    ((eq limb :arms) (mapcar #'(lambda (l) (send self :stop-impedance l)) '(:rarm :larm)))
    (t ) ;; virtual force sensor is not supported in this file
    ))
  (:_set-forcemoment-offset-param
   (name &key force-offset moment-offset link-offset-centroid link-offset-mass)
   (let* ((current-fm-param (send self :get-forcemoment-offset-param name))
          (param (instance hrpsys::OpenHRP_ImpedanceControllerService_ForceMomentOffsetParam :init
                           :force_offset (if force-offset force-offset (send current-fm-param :force_offset))
                           :moment_offset (if moment-offset moment-offset (send current-fm-param :moment_offset))
                           :link_offset_centroid (if link-offset-centroid link-offset-centroid (send current-fm-param :link_offset_centroid))
                           :link_offset_mass (if link-offset-mass link-offset-mass (send current-fm-param :link_offset_mass)))))
     (send self :impedancecontrollerservice_setforcemomentoffsetparam :name name :i_param param)))
  (:set-forcemoment-offset-param
   (limb &rest args)
   (cond
    ((send self :get-force-sensor-from-limb-name limb)
     (let* ((fs (send self :get-force-sensor-from-limb-name limb))
            (base-name (send (send (send (send robot limb :root-link) :parent) :joint) :name)))
       (send* self :_set-forcemoment-offset-param
              (string-downcase (send fs :name)) args)))))
  (:get-forcemoment-offset-param (name) (send (send self :impedancecontrollerservice_getforcemomentoffsetparam :name name) :i_param))
  )

;; AutoBalancerService
(defmethod rtm-ros-robot-interface
  (:start-auto-balancer
   (&key (limbs '(:rleg :lleg)))
   (send self :autobalancerservice_startABC
         :alp (mapcar #'(lambda (limb)
                          (let ((target2foot (send (send (send robot limb :end-coords) :parent) :transformation (send robot limb :end-coords))))
                            (instance hrpsys::openhrp_autobalancerservice_autobalancerlimbparam
                                      :init :name (format nil "~A" limb)
                                      :target_name (send (send (send robot limb :end-coords) :parent) :joint :name)
                                      :base_name (if (send (send (send robot limb :root-link) :parent) :joint) (send (send (send robot limb :root-link) :parent) :joint :name) "WAIST")
                                      :target2foot_offset_pos (scale 1e-3 (send target2foot :worldpos))
                                      :target2foot_offset_rot (matrix2quaternion (send target2foot :worldrot)))))
                      limbs)))
  (:stop-auto-balancer () (send self :autobalancerservice_stopABC))
  (:go-pos-no-wait
   (xx yy th)
   (send self :autobalancerservice_goPos :x xx :y yy :th th))
  (:go-pos
   (xx yy th)
   (send self :start-auto-balancer)
   (send self :go-pos-no-wait xx yy th)
   (send self :wait-foot-steps)
   (send self :stop-auto-balancer))
  (:set-foot-steps-no-wait (fs) (send self :setFootSteps fs))
  (:set-foot-steps
   (fs)
   (send self :start-auto-balancer)
   (send self :set-foot-steps-no-wait fs)
   (send self :wait-foot-steps)
   (send self :stop-auto-balancer))
  (:go-velocity
   (vx vy vth)
   (send self :start-auto-balancer)
   (send self :autobalancerservice_goVelocity :vx vx :vy vy :vth vth))
  (:go-stop
   ()
   (send self :autobalancerservice_goStop)
   (send self :stop-auto-balancer))
  (:wait-foot-steps
   ()
   (send self :autobalancerservice_waitFootSteps))
  (:get-auto-balancer-param
   ()
   (send (send self :autobalancerservice_getautobalancerparam) :i_param))
  (:get-gait-generator-param
   ()
   (send (send self :autobalancerservice_getgaitgeneratorparam) :i_param))
  (:set-gait-generator-param
   (&key default-step-time default-step-height stride-parameter)
   (let* ((current-param (send self :get-gait-generator-param))
          (param (instance hrpsys::Openhrp_AutoBalancerService_GaitGeneratorParam
                           :init
                           :default_step_time (if default-step-time default-step-time (send current-param :default_step_time))
                           :default_step_height (if default-step-height default-step-height (send current-param :default_step_height))
                           :stride_parameter (if stride-parameter stride-parameter (send current-param :stride_parameter)))))
     (send self :autobalancerservice_setgaitgeneratorparam :i_param param)))
  (:set-auto-balancer-param
   (&key default-zmp-offsets move-base-gain)
   (let* ((current-param (send self :get-auto-balancer-param))
          (param (instance hrpsys::Openhrp_AutoBalancerService_AutoBalancerParam
                           :init
                           :default_zmp_offsets (if default-zmp-offsets default-zmp-offsets (send current-param :default_zmp_offsets))
                           :move_base_gain (if move-base-gain move-base-gain (send current-param :move_base_gain)))))
     (send self :autobalancerservice_setautobalancerparam :i_param param)))
  (:eus-footstep->abc-footstep
   (f)
   (instance hrpsys::openhrp_autobalancerservice_footstep :init
             :pos (scale 1e-3 (send f :worldpos))
             :rot (matrix2quaternion (send f :worldrot))
             :leg (print (format nil "~A" (if (find-method f :l/r) (send f :l/r) (send f :get :l/r)))))
   )
  )

;; SoftErrorLimiterService
(defmethod rtm-ros-robot-interface
  (:set-soft-error-limit
    (name limit)
    (send self :softerrorlimiterservice_setservoerrorlimit :name name :limit limit)
    )
  )

