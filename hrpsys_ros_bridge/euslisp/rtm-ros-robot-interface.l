(require :robot-interface "package://pr2eus/robot-interface.l")

(ros::load-ros-manifest "dynamic_reconfigure")
(ros::roseus-add-msgs "hrpsys_ros_bridge")
(ros::load-ros-manifest "hrpsys_ros_bridge")

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
                     #'send self :rtmros-motor-states-callback :groupname groupname)
     (mapcar #'(lambda (x)
                 (ros::subscribe (format nil "/~A" (string-downcase x)) geometry_msgs::WrenchStamped
                                 #'send self :rtmros-force-sensor-callback x :groupname groupname)
                 (ros::subscribe (format nil "/off_~A" (string-downcase x)) geometry_msgs::WrenchStamped
                                 #'send self :rtmros-force-sensor-callback (read-from-string (format nil ":off-~A" (string-downcase x))) :groupname groupname)
                 (ros::subscribe (format nil "/ref_~A" (string-downcase x)) geometry_msgs::WrenchStamped
                                 #'send self :rtmros-force-sensor-callback (read-from-string (format nil ":reference-~A" (string-downcase x))) :groupname groupname))
             (send-all (send robot :force-sensors) :name))
     (ros::subscribe "/zmp" geometry_msgs::PointStamped
                     #'send self :rtmros-zmp-callback :groupname groupname)
     (ros::subscribe "/imu" sensor_msgs::Imu
                     #'send self :rtmros-imu-callback :groupname groupname)
     ))
  (:rtmros-motor-states-callback
   (msg)
   (send self :set-robot-state1 :motor-extra-data (send msg :extra_data))
   (send self :set-robot-state1 :temperature (send msg :temperature)))
  (:rtmros-zmp-callback
   (msg)
   (let ((p (send msg :point)))
     (send self :set-robot-state1 :zmp (float-vector (send p :x) (send p :y) (send p :z)))))
  (:rtmros-imu-callback
   (msg)
   (send self :set-robot-state1 :imu msg)
   (let ((imucoords (make-coords :rot (ros::tf-quaternion->rot (send (cdr (assoc :imu robot-state)) :orientation)))))
     (send robot :move-coords imucoords (car (send robot :imu-sensors)))))
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
  (:tmp-force-moment-vector-for-limb
   (f/m fsensor-name &optional (topic-name-prefix nil)) ;; topic-name-prefix is "off" or "reference"
   (let ((key-name (if topic-name-prefix
                       (read-from-string (format nil ":~A-~A" topic-name-prefix (string-downcase fsensor-name)))
                     fsensor-name)))
     (subseq (cdr (assoc key-name robot-state))
             (if (eq f/m :force) 0 3)
             (if (eq f/m :force) 3 6))))
  (:tmp-force-moment-vector
   (f/m &optional (limb) (topic-name-prefix nil)) ;; topic-name-prefix is "off" or "reference"
   (if limb
       (send self :tmp-force-moment-vector-for-limb f/m (send (car (send robot limb :force-sensors)) :name) topic-name-prefix)
     (mapcar #'(lambda (x)
                 (send self :tmp-force-moment-vector-for-limb f/m x topic-name-prefix))
             (send-all (send robot :force-sensors) :name))))
  (:force-vector
   (&optional (limb))
   "Returns :force-vector [N] list for all limbs obtained by :state.
    If a limb argument is specified, returns a vector for the limb."
   (send self :tmp-force-moment-vector :force limb))
  (:moment-vector
   (&optional (limb))
   "Returns :moment-vector [Nm] list for all limbs obtained by :state.
    If a limb argument is specified, returns a vector for the limb."
   (send self :tmp-force-moment-vector :moment limb))
  (:off-force-vector
   (&optional (limb))
   "Returns offset-removed :force-vector [N] list for all limbs obtained by :state.
    This value corresponds to RemoveForceSensorLinkOffset RTC.
    If a limb argument is specified, returns a vector for the limb."
   (send self :tmp-force-moment-vector :force limb "off"))
  (:off-moment-vector
   (&optional (limb))
   "Returns offset-removed :moment-vector [Nm] list for all limbs obtained by :state.
    This value corresponds to RemoveForceSensorLinkOffset RTC.
    If a limb argument is specified, returns a vector for the limb."
   (send self :tmp-force-moment-vector :moment limb "off"))
  (:reference-force-vector
   (&optional (limb))
   "Returns reference force-vector [N] list for all limbs obtained by :state.
    This value corresponds to StateHolder and SequencePlayer RTC.
    If a limb argument is specified, returns a vector for the limb."
   (send self :tmp-force-moment-vector :force limb "reference"))
  (:reference-moment-vector
   (&optional (limb))
   "Returns reference moment-vector [Nm] list for all limbs obtained by :state.
    This value corresponds to StateHolder and SequencePlayer RTC.
    If a limb argument is specified, returns a vector for the limb."
   (send self :tmp-force-moment-vector :moment limb "reference"))
  (:absolute-force-vector
   (&optional (limb))
   "Returns offset-removed :force-vector [N] list for all limbs in world frame obtained by :state.
    This value corresponds to RemoveForceSensorLinkOffset RTC.
    If a limb argument is specified, returns a vector for the limb."
   (if limb
       (send (car (send robot limb :force-sensors)) :rotate-vector (send self :off-force-vector limb))
     (mapcar #'(lambda (fs force)
                 (send fs :rotate-vector force))
             (send robot :force-sensors) (send self :off-force-vector))))
  (:absolute-moment-vector
   (&optional (limb))
   "Returns offset-removed :moment-vector [Nm] list for all limbs in world frame obtained by :state.
    This value corresponds to RemoveForceSensorLinkOffset RTC.
    If a limb argument is specified, returns a vector for the limb."
   (if limb
       (send (car (send robot limb :force-sensors)) :rotate-vector (send self :off-moment-vector limb))
     (mapcar #'(lambda (fs moment)
                 (send fs :rotate-vector moment))
             (send robot :force-sensors) (send self :off-moment-vector))))
  (:zmp-vector
   (&optional (wrt :local))
   "Returns zmp vector [mm].
    If wrt is :local, returns zmp in the base-link frame. If wrt is :world, returns zmp in the world frame."
   (let ((zmp (scale 1e3 (cdr (assoc :zmp robot-state))))) ;; [m] -> [mm]
     (case wrt
       (:local zmp)
       (:world (send (car (send robot :links)) :transform-vector zmp)))))
  (:temperature-vector
   ()
   "Returns temperature vector."
   (cdr (assoc :temperature robot-state)))
  (:motor-extra-data
   ()
   "Returns motor extra data. Please see iob definition for each system."
   (let* ((d (cdr (assoc :motor-extra-data robot-state)))
          (dims (mapcar #'(lambda (x) (send x :size)) (send (send d :layout) :dim)))
          (ret))
     (dotimes (j (car dims))
       (push (subseq (send d :data) (* j (cadr dims)) (* (1+ j) (cadr dims))) ret))
     (reverse ret)))
  (:imucoords
   ()
   "Returns robot's coords based on imu measurement."
   (send robot :copy-worldcoords))
  (:accel-vector
   ()
   "Returns acceleration [m/s2] of the acceleration sensor."
   (let ((acc (send (cdr (assoc :imu robot-state)) :linear_acceleration)))
     (float-vector (send acc :x) (send acc :y) (send acc :z))))
  (:gyro-vector
   ()
   "Returns angular velocity [rad/s] of the gyro sensor."
   (let ((gyro (send (cdr (assoc :imu robot-state)) :angular_velocity)))
     (float-vector (send gyro :x) (send gyro :y) (send gyro :z))))
  (:state
    (&rest args)
    "Obtains sensor and robot command topics using spin-once."
    (case (car args)
      (:imucoords
       (send-super* :state args)
       (send self :imucoords))
      (t
       (send-super* :state args))))
  ;; automatically define methods for ROSBridge services
  (:define-all-ROSBridge-srv-methods
   (&key (debug-view nil) (ros-pkg-name "hrpsys_ros_bridge"))
   (let ((srv-fnames (send self :get-ROSBridge-srv-fnames ros-pkg-name)))
     (dolist (idl (send self :get-ROSBridge-idl-fnames ros-pkg-name))
       (let ((rtc-name (pathname-name idl)))
         (dolist (srv-name (mapcar #'pathname-name (remove-if-not #'(lambda (x) (and (substringp rtc-name x) (not (= (char x 0) (char "." 0))))) srv-fnames)))
           (let ((method-def (send self :get-ROSBridge-method-def-macro rtc-name srv-name ros-pkg-name)))
             (when method-def
               (if debug-view (pprint (macroexpand method-def)))
               (eval method-def)
               )))))))
  (:get-ROSBridge-fnames-from-type
   (type-name &optional (ros-pkg-name "hrpsys_ros_bridge"))
   (let ((path (ros::resolve-ros-path (format nil "package://~A" ros-pkg-name))))
     (remove-if-not #'(lambda (x) (substringp (format nil ".~A" type-name) x)) (directory (format nil "~A/~A" path type-name)))
     ))
  (:get-ROSBridge-idl-fnames (&optional (ros-pkg-name "hrpsys_ros_bridge")) (send self :get-ROSBridge-fnames-from-type "idl" ros-pkg-name))
  (:get-ROSBridge-srv-fnames (&optional (ros-pkg-name "hrpsys_ros_bridge")) (send self :get-ROSBridge-fnames-from-type "srv" ros-pkg-name))
  (:get-ROSBridge-method-def-macro
   (rtc-name srv-name &optional (ros-pkg-name "hrpsys_ros_bridge"))
   (let* ((meth-name (string-left-trim "_" (string-left-trim rtc-name (string-left-trim "_" (string-left-trim "OpenHRP" srv-name)))))
          (srv-request (read-from-string (format nil "~A::~ARequest" ros-pkg-name srv-name)))
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
           (or
           (ros::service-call
            ,(format nil "/~AROSBridge/~A" rtc-name meth-name)
            (instance ,(eval srv-request) :init ,@(mapcan #'(lambda (x) (list (caar x) (cadar x))) (cdadr init-method))))
           (error ";; service call failed (~A)" ,new-method-name)
           )
           )
          )
       )))
  (:get-idl-enum-values
   (value ;; value is current enum value obtained from :get-xxx-param.
    enum-type-string) ;; enum-type-string is HRPSYS_ROS_BRIDGE::OPENHRP_[RTC SERVICE NAME]_[ENUM TYPE NAME]. Please see idl files.
   (let* ((str (find-if #'(lambda (x) (= (eval x) value)) (constants "" enum-type-string))))
     (read-from-string (format nil ":~A" (string-right-trim "*" (string-left-trim "*" str))))
     ))
  (:set-interpolation-mode
   (interpolation-mode)
   "Set interpolation mode for SequencePlayer."
   (if (integerp interpolation-mode)
       interpolation-mode
     (let ((im (read-from-string (format nil "HRPSYS_ROS_BRIDGE::OPENHRP_SEQUENCEPLAYERSERVICE_INTERPOLATIONMODE::*~A*" (string-downcase interpolation-mode)))))
       (if (boundp im)
           (send self :sequenceplayerservice_setinterpolationmode :i_mode_ (eval im))
         (error ";; no such interpolation-mode ~A in :set-interpolation-mode~%" interpolation-mode)))
     ))
  ;;
  ;; Calculate zmp from state [mm]
  ;; example ;; (progn (send *ri* :go-velocity 0 0 0) (objects (list (*ri* . robot))) (do-until-key (let ((zmp (send *ri* :calc-zmp-from-state))) (send *irtviewer* :draw-objects :flush nil) (send zmp :draw-on :flush t :size 300))))
  (:calc-zmp-from-state
   (&key (wrt :world))
   ;; :wrt is :local => calc local zmp for (*ri* . robot)'s root-link coords
   ;; :wrt is :world => calc world zmp for (*ri* . robot)
   (send self :state)
   (send robot :calc-zmp-from-forces-moments
         (mapcar #'(lambda (x) (send self :force-vector x)) '(:rleg :lleg))
         (mapcar #'(lambda (x) (send self :moment-vector x)) '(:rleg :lleg))
         :wrt wrt)
   )
  (:get-robot-date-string
   ()
   (let* ((dt (unix:localtime)))
     (format nil "~A_~A~A~A~A~A~A"
             (send (send self :robot) :name)
             (digits-string (+ 1900 (aref dt 5)) 4)
             (digits-string (1+ (aref dt 4)) 2)
             (digits-string (aref dt 3) 2)
             (digits-string (aref dt 2) 2)
             (digits-string (aref dt 1) 2)
             (digits-string (aref dt 0) 2))))
  )

;; define Euslisp setter and getter method
(defun def-set-get-param-method
  (param-class ;; parameter class
   set-param-method-name get-param-method-name ;; Euslisp setter and getter method which user want to define
   set-param-idl-name get-param-idl-name ;; raw setter and getter method converted from idl2srv files
   &key (optional-args) ;; arguments for raw setter and getter method
        (debug nil))
  (let* ((param-slots-list ;; get slots list for param-class
          (remove-if #'(lambda (x) (string= "plist" x))
                     (mapcar #'(lambda (x) (string-left-trim "::_" (string-left-trim "ros" (format nil "~A" x))))
                             (concatenate cons (send (eval param-class) :slots)))))
         (getter-defmethod-macro
           `(defmethod rtm-ros-robot-interface
              (,get-param-method-name
               ,(if optional-args (list (cadr optional-args)) (list ))
               (send (send self ,get-param-idl-name ,@optional-args) :i_param))))
         ;; generate defmethod like
         ;;  (:set-xx-param (&rest args &key yy-zz)
         ;;   (let ((current-param (send self :get-xx-param))
         ;;         (param (instance ww :init (if (memq :yy-zz args) (cadr (memq :yy-zz args)) (send current-param :yy_zz))))
         ;;      (send self :aaService_setParameter :i_param param)))
         (setter-defmethod-macro
           `(defmethod rtm-ros-robot-interface
              (,set-param-method-name
               ,(append (if optional-args (list (cadr optional-args))) (list '&rest 'args '&key) (mapcar #'(lambda (x) (read-from-string (substitute (elt "-" 0) (elt "_" 0) x))) param-slots-list)) ;; replace _ => - for Euslisp friendly argument
               (let* ((current-param ,(append (list 'send 'self get-param-method-name) (if optional-args (list (cadr optional-args)))))
                      (param (instance ,param-class
                                       :init
                                       ,@(apply #'append
                                                (mapcar #'(lambda (x)
                                                            (let* ((eus-sym (read-from-string (substitute (elt "-" 0) (elt "_" 0) x)))
                                                                   (eus-keyword (read-from-string (format nil ":~A" eus-sym)))
                                                                   (param-sym (read-from-string (format nil ":~A" x))))
                                                              (list param-sym (list 'if
                                                                                    (list 'memq eus-keyword 'args)
                                                                                    (list 'cadr (list 'memq eus-keyword 'args))
                                                                                    (list 'send 'current-param param-sym)))))
                                                        param-slots-list))
                                       )))
                 (send self ,set-param-idl-name :i_param param ,@optional-args)
                 )))))
    (when debug
      (pprint (macroexpand getter-defmethod-macro))
      (pprint (macroexpand setter-defmethod-macro)))
    (eval getter-defmethod-macro)
    (eval setter-defmethod-macro)
    t))

;; SequencePlayerService
(defmethod rtm-ros-robot-interface
  (:set-base-pose
   (&optional base-coords (tm 0.1))
   (warn ";; :set-base-pose is deprecated. Use :set-base-coords~%")
   (unless base-coords
     (setq base-coords (send robot :worldcoords)))
   (send self :sequenceplayerservice_setbasepos :pos (scale 0.001 (send base-coords :pos)) :tm tm)
   (send self :sequenceplayerservice_setbaserpy :rpy (coerce (reverse (car (send base-coords :rpy-angle))) float-vector) :tm tm))
  (:set-base-coords
   (base-coords tm)
   "Set base coordinates in the world frame.
    base-coords is Euslisp coords and tm is [ms]."
   (send self :set-base-pos (send base-coords :worldpos) tm)
   (send self :set-base-rpy (coerce (reverse (car (rpy-angle (send base-coords :worldrot)))) float-vector) tm)
   )
  (:set-base-pos
   (base-pos tm)
   "Set base pos in the world frame.
    base-pos is [mm] and tm is [ms]."
   (send self :sequenceplayerservice_setbasepos :pos (scale 0.001 base-pos) :tm (* 0.001 tm)) ;; [mm]->[m], [ms]->[s]
   )
  (:set-base-rpy
   (base-rpy tm)
   "Set base rpy in the world frame.
    base-rpy is [rad] and tm is [ms]."
   (send self :sequenceplayerservice_setbaserpy :rpy base-rpy :tm (* 0.001 tm)) ;; [ms]->[s]
   )
  (:wait-interpolation-of-group
   (groupname)
   (send self :sequenceplayerservice_waitinterpolationofgroup :gname groupname))
  (:add-joint-group
   (groupname jnames)
   (send self :sequenceplayerservice_addjointgroup :gname groupname :jnames jnames))
  (:remove-joint-group
   (groupname)
   (send self :sequenceplayerservice_removejointgroup :gname groupname))
  (:set-joint-angles-of-group
   (groupname av tm)
   (send self :sequenceplayerservice_setjointanglesofgroup :gname groupname :jvs av :tm tm))
  (:load-pattern (basename &optional (tm 0.0)) (send self :sequenceplayerservice_loadpattern :basename basename :tm tm))
  (:wait-interpolation-seq () (send self :sequenceplayerservice_waitinterpolation))
  (:sync-controller
   (controller &optional (interpolation-time 1000) (blockp t))
   (let* ((controller-info (car (send self controller)))
          (groupname (cdr (assoc :group-name controller-info)))
          (jointnames (cdr (assoc :joint-names controller-info)))
          (current-reference (send self :state :reference-vector)))
     (unless current-reference
       (error ";; cannot get reference-vector in :sync-controller~%")
       )
     (warn "sync controller ~A~%" controller)
     (send self :angle-vector current-reference interpolation-time controller)
     (when blockp
       (send self :wait-interpolation)
       )
     (send self :angle-vector current-reference interpolation-time :default-controller)
     (when blockp
       (send self :wait-interpolation)
       )
     (send self :remove-joint-group groupname)
     (send self :wait-interpolation) ;; wait until removing
     (send self :add-joint-group groupname jointnames)
     )
   )
#| ;; angle group sample
  (send *ri* :add-joint-group "larm" (send-all (send *robot* :larm :joint-list) :name))
  (send *ri* :set-jointangles-of-group "larm" (scale (/ pi 180.0) (send *robot* :larm :angle-vector)) 4.0)
  (send *ri* :waitinterpolation-of-group "larm")
|#
  (:set-ref-forces-moments
   (force-list moment-list tm)
   "Set reference wrenches. wrench-list is list of wrench ([N],[Nm]) for all end-effectors. tm is interpolation time [ms]."
   (send self :sequenceplayerservice_setwrenches :wrenches (apply #'concatenate float-vector (mapcan #'(lambda (f m) (list f m)) force-list moment-list)) :tm (* 1e-3 tm)) ;; [ms]->[s]
   )
  (:set-ref-forces
   (force-list tm &key (update-robot-state t))
   "Set reference forces. force-list is list of force ([N]) for all end-effectors. tm is interpolation time [ms]."
   (if update-robot-state (send self :state))
   (send self :set-ref-forces-moments force-list (send self :reference-moment-vector) tm)
   )
  (:set-ref-moments
   (moment-list tm &key (update-robot-state t))
   "Set reference moments. moment-list is list of moment ([Nm]) for all end-effectors. tm is interpolation time [ms]."
   (if update-robot-state (send self :state))
   (send self :set-ref-forces-moments (send self :reference-force-vector) moment-list tm)
   )
  (:set-ref-force
   (force tm &optional (limb :arms) &key (update-robot-state t))
   "Set reference force [N]. tm is interpolation time [ms].
    limb should be limb symbol name such as :rarm, :larm, :rleg, :lleg, :arms, or :legs."
   (if update-robot-state (send self :state))
   (let ((limbs (case limb
                      (:arms (list :rarm :larm))
                      (:legs (list :rleg :lleg))
                      (t (list limb)))))
     (send self :set-ref-forces
           (mapcar #'(lambda (fs rfv)
                       (if (find-if #'(lambda (l) (equal fs (car (send robot l :force-sensors)))) limbs)
                           force rfv))
                   (send robot :force-sensors) (send self :reference-force-vector))
           tm)
     ))
  (:set-ref-moment
   (moment tm &optional (limb :arms) &key (update-robot-state t))
   "Set reference moment [Nm]. tm is interpolation time [ms].
    limb should be limb symbol name such as :rarm, :larm, :rleg, :lleg, :arms, or :legs."
   (if update-robot-state (send self :state))
   (let ((limbs (case limb
                      (:arms (list :rarm :larm))
                      (:legs (list :rleg :lleg))
                      (t (list limb)))))
     (send self :set-ref-moments
           (mapcar #'(lambda (fs rfv)
                       (if (find-if #'(lambda (l) (equal fs (car (send robot l :force-sensors)))) limbs)
                           moment rfv))
                   (send robot :force-sensors) (send self :reference-moment-vector))
           tm)
     ))
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
  (:get-collision-status
   ()
   (send (send self :collisiondetectorservice_getcollisionstatus) :cs)
   )
  )

;; DataLoggerService
(defmethod rtm-ros-robot-interface
  (:save-log
   (fname &key (set-robot-date-string t))
   "Save log files as [fname].[component_name]_[dataport_name].
    This method corresponds to DataLogger save().
    If set-robot-date-string is t, filename includes date string and robot name. By default, set-robot-date-string is t."
   (send self :dataloggerservice_save :basename (format nil "~A~A" fname (if set-robot-date-string (format nil "_~A" (send self :get-robot-date-string)) ""))))
  ;; start log by clearing log
  (:start-log
   ()
   "Start logging.
    This method corresponds to DataLogger clear()."
   (send self :dataloggerservice_clear))
  (:set-log-maxlength
   (&optional (maxlength 4000))
   "Set max log length.
    This method corresponds to DataLogger maxLength()."
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
  (:calibrate-inertia-sensor
   ()
   (send self :robothardwareservice_calibrateInertiaSensor)
   )
  )

(def-set-get-param-method 'hrpsys_ros_bridge::OpenHRP_ImpedanceControllerService_impedanceParam
  :raw-set-impedance-controller-param :raw-get-impedance-controller-param
  :impedancecontrollerservice_setimpedancecontrollerparam :impedancecontrollerservice_getimpedancecontrollerparam
  :optional-args (list :name 'name))

;; ImpedanceControllerService
(defmethod rtm-ros-robot-interface
  (:start-impedance
   (limb &rest args)
   "Start impedance controller mode.
    limb should be limb symbol name such as :rarm, :larm, :rleg, :lleg, :arms, or :legs."
   (if args
       (progn
         (send* self :set-impedance-controller-param limb args)
         (send self :raw-start-impedance limb))
     (send self :raw-start-impedance limb)))
  (:raw-start-impedance
   (limb)
   (send self :force-sensor-method
         limb
         #'(lambda (name &rest _args)
             (send self :impedancecontrollerservice_startimpedancecontroller :name (string-downcase name)))
         :raw-start-impedance))
  (:start-impedance-no-wait
   (limb)
   (send self :force-sensor-method
         limb
         #'(lambda (name &rest _args)
             (send self :impedancecontrollerservice_startimpedancecontrollernowait :name (string-downcase name)))
         :start-impedance-no-wait))
  (:stop-impedance
   (limb)
   "Stop impedance controller mode.
    limb should be limb symbol name such as :rarm, :larm, :rleg, :lleg, :arms, or :legs."
   (send self :force-sensor-method
         limb
         #'(lambda (name &rest _args)
             (send self :impedancecontrollerservice_stopimpedancecontroller :name (string-downcase name)))
         :stop-impedance))
  (:stop-impedance-no-wait
   (limb)
   (send self :force-sensor-method
         limb
         #'(lambda (name &rest _args)
             (send self :impedancecontrollerservice_stopimpedancecontrollernowait :name (string-downcase name)))
         :stop-impedance-no-wait))
  (:wait-impedance-controller-transition
   (limb)
   (send self :force-sensor-method
         limb
         #'(lambda (name &rest _args)
             (send self :impedancecontrollerservice_waitImpedanceControllerTransition :name (string-downcase name)))
         :wait-impedance-controller-transition))
  (:set-impedance-controller-param
   (limb &rest args)
   "Set impedance controller parameter like (send *ri* :set-impedance-controller-param :rarm :K-p 400).
    limb should be limb symbol name such as :rarm, :larm, :rleg, :lleg, :arms, or :legs."
   (send* self :force-sensor-method
          limb
          #'(lambda (name &rest _args)
              (send* self :raw-set-impedance-controller-param (string-downcase name) args))
          :set-impedance-controller-param
          args))
  (:get-impedance-controller-param
   (limb)
   "Get impedance controller parameter.
    limb should be limb symbol name such as :rarm, :larm, :rleg, :lleg, :arms, or :legs."
   (send self :force-sensor-method
         limb
         #'(lambda (name &rest _args)
             (send self :raw-get-impedance-controller-param (string-downcase name)))
         :get-impedance-controller-param))
  (:get-impedance-controller-controller-mode
   (name)
   "Get ImpedanceController ControllerMode as Euslisp symbol."
   (let ((param (send self :get-impedance-controller-param name)))
     (labels ((get-euslisp-prm
               (prm)
               (send self :get-idl-enum-values (send prm :controller_mode)
                     "HRPSYS_ROS_BRIDGE::OPENHRP_IMPEDANCECONTROLLERSERVICE_CONTROLLERMODE")))
       (if (atom param)
           (get-euslisp-prm param)
         (mapcar #'get-euslisp-prm param))
       )))
  (:force-sensor-method
   (limb method-func method-name &rest args)
   (cond
    ((eq limb :legs) (mapcar #'(lambda (l) (send* self method-name l args)) '(:rleg :lleg)))
    ((eq limb :arms) (mapcar #'(lambda (l) (send* self method-name l args)) '(:rarm :larm)))
    ((and (find-method robot limb) (car (send robot limb :force-sensors)))
     (apply method-func limb args))
    (t (error ";; No such limb or force sensor is defined for ~A~%." limb))
    ))
  )

;; RemoveForceSensorLinkOffset
(def-set-get-param-method 'hrpsys_ros_bridge::OpenHRP_RemoveForceSensorLinkOffsetService_ForceMomentOffsetParam
  :raw-set-forcemoment-offset-param :raw-get-forcemoment-offset-param
  :removeforcesensorlinkoffsetservice_setforcemomentoffsetparam :removeforcesensorlinkoffsetservice_getforcemomentoffsetparam
  :optional-args (list :name 'name))

(defmethod rtm-ros-robot-interface
  (:zero-set-forcemoment-offset-param
   (limb)
   (send self :set-forcemoment-offset-param limb :force-offset #f(0 0 0) :moment-offset #f(0 0 0) :link-offset-centroid #f(0 0 0) :link-offset-mass 0)
   )
  (:set-forcemoment-offset-param
   (limb &rest args)
   (send* self :force-sensor-method
         limb
         #'(lambda (name &rest _args)
             (send* self :raw-set-forcemoment-offset-param (send (car (send robot name :force-sensors)) :name) _args))
         :set-forcemoment-offset-param
         args))
  (:get-forcemoment-offset-param
   (limb)
   (send self :force-sensor-method
         limb
         #'(lambda (name &rest _args)
             (send self :raw-get-forcemoment-offset-param (send (car (send robot name :force-sensors)) :name)))
         :get-forcemoment-offset-param))
  (:load-forcemoment-offset-param
   (fname &key (set-offset t))
   (mapcar #'(lambda (x)
               (send* self :set-forcemoment-offset-param (car x)
                      (if set-offset
                          (cdr x)
                        (list :link-offset-mass (cadr (memq :link-offset-mass (cdr x)))
                              :link-offset-centroid (cadr (memq :link-offset-centroid (cdr x)))))))
           (with-open-file
            (f fname :direction :input)
            (read f nil nil)))
   )
  (:load-forcemoment-offset-params
   (filename)
   "Load RMFO offset parameters from parameter file.
    This method corresponds to RemoveForceSensorLinkOffset loadForceMomentOffsetParams()."
   (send self :removeforcesensorlinkoffsetservice_loadforcemomentoffsetparams :filename filename)
   )
  (:dump-forcemoment-offset-params
   (filename &key (set-robot-date-string t))
   "Save all RMFO offset parameters.
    This method corresponds to RemoveForceSensorLinkOffset dumpForceMomentOffsetParams().
    If set-robot-date-string is t, filename includes date string and robot name. By default, set-robot-date-string is t."
   (send self :removeforcesensorlinkoffsetservice_dumpforcemomentoffsetparams :filename (format nil "~A~A" filename (if set-robot-date-string (format nil "_~A" (send self :get-robot-date-string)) "")))
   )
  (:reset-force-moment-offset-arms
   ()
   "Remove force and moment offset for :rarm and :larm"
   (send self :reset-force-moment-offset '(:rarm :larm)))
  (:reset-force-moment-offset
   (limbs)
   "Remove force and moment offsets. limbs should be list of limb symbol name."
   (send self :_reset-force-moment-offset limbs :force)
   (send self :_reset-force-moment-offset limbs :moment)
   )
  (:_reset-force-moment-offset
   (limbs f/m &key (itr 10))
   (let* ((params (mapcar #'(lambda (alimb) (send self :get-forcemoment-offset-param alimb)) limbs))
          (nominal-mgs (mapcar #'(lambda (param) (scale (* 1e-3 (send param :link_offset_mass)) *g-vec*)) params)))
     (labels ((calc-off
               (alimb param nominal-mg)
               (v+ (send self (if (eq f/m :force) :force-vector :moment-vector) alimb)
                   (if (eq f/m :force)
                       (send (car (send robot alimb :force-sensors)) :inverse-rotate-vector nominal-mg)
                     (v* (send param :link_offset_centroid) (send (car (send robot alimb :force-sensors)) :inverse-rotate-vector nominal-mg)))))
              (get-avg-fm
               ()
               (let ((fm (mapcar #'(lambda (i)
                                     (send self :state)
                                     (mapcar #'(lambda (alimb) (send self (if (eq f/m :force) :off-force-vector :off-moment-vector) alimb)) limbs))
                                 (make-list itr))))
                 (mapcar #'(lambda (alimb)
                             (let ((idx (position alimb limbs)))
                               (vector-mean (mapcar #'(lambda (d) (elt d idx)) fm))))
                         limbs))))
       ;; estimate offsets
       (let* ((tmp-fm-offsets (mapcar #'(lambda (i)
                                          (send self :state)
                                          (mapcar #'calc-off limbs params nominal-mgs))
                                      (make-list itr)))
              (new-fm-offsets (mapcar #'(lambda (alimb)
                                          (let ((idx (position alimb limbs)))
                                            (vector-mean (mapcar #'(lambda (d) (elt d idx)) tmp-fm-offsets))))
                                      limbs))
              (org-fm-list (get-avg-fm)))
         ;; set offsets
         (mapcar #'(lambda (alimb new-fm-offset)
                     (send self :set-forcemoment-offset-param alimb
                           (if (eq f/m :force) :force-offset :moment-offset)
                           new-fm-offset))
                 limbs new-fm-offsets)
         (unix:usleep 10000)
         ;; check ;; compare sensor value before & after resetting
         (mapcar #'(lambda (alimb org-fm new-fm)
                     (format t ";; ~A error of ~A ;; ~A[~A] -> ~A[~A]~%"
                             (string-downcase f/m) alimb
                             (norm org-fm) (if (eq f/m :force) "N" "Nm")
                             (norm new-fm) (if (eq f/m :force) "N" "Nm")))
                 limbs org-fm-list (get-avg-fm))
         ))))
  )

;; AutoBalancerService
(def-set-get-param-method
  'hrpsys_ros_bridge::Openhrp_AutoBalancerService_GaitGeneratorParam
  :raw-set-gait-generator-param :get-gait-generator-param
  :autobalancerservice_setgaitgeneratorparam :autobalancerservice_getgaitgeneratorparam)

(defmethod rtm-ros-robot-interface
  (:start-auto-balancer
   (&key (limbs '(:rleg :lleg)))
   (send self :autobalancerservice_startAutoBalancer
         :limbs (mapcar #'(lambda (x) (format nil "~A" (string-downcase x))) limbs)))
  (:stop-auto-balancer () (send self :autobalancerservice_stopAutoBalancer))
  (:go-pos-no-wait
   (xx yy th)
   (send self :autobalancerservice_goPos :x xx :y yy :th th))
  (:go-pos
   (xx yy th)
   (send self :go-pos-no-wait xx yy th)
   (send self :wait-foot-steps))
  (:raw-get-foot-step-param
   ()
   (send (send self :autobalancerservice_getfootstepparam) :i_param))
  (:get-foot-step-params
   ()
   "Get AutoBalancer foot step params."
   (let ((param (send self :raw-get-foot-step-param)))
     (append
      (mapcan #'(lambda (meth param-name) (list param-name (send self :abc-footstep->eus-footstep (send param meth))))
              '(:rleg_coords :lleg_coords :support_leg_coords :swing_leg_coords :swing_leg_src_coords :swing_leg_dst_coords :dst_foot_midcoords)
              '(:rleg-coords :lleg-coords :support-leg-coords :swing-leg-coords :swing-leg-src-coords :swing-leg-dst-coords :dst-foot-midcoords))
      (mapcan #'(lambda (meth param-name)
                  (list param-name (send self :get-idl-enum-values
                                         (send param meth)
                                         "HRPSYS_ROS_BRIDGE::OPENHRP_AUTOBALANCERSERVICE_SUPPORTLEGSTATE")))
              '(:support_leg :support_leg_with_both)
              '(:support-leg :support-leg-with-both)))
     ))
  (:get-foot-step-param
   (param-name)
   "Get AutoBalancer foot step param by given name.
    param-name is key word for parameters defined in IDL.
    param-name should be :rleg-coords :lleg-coords :support-leg-coords :swing-leg-coords :swing-leg-src-coords :swing-leg-dst-coords :dst-foot-midcoords :support-leg :support-leg-with-both."
   (if (memq param-name '(:rleg-coords :lleg-coords :support-leg-coords :swing-leg-coords :swing-leg-src-coords :swing-leg-dst-coords :dst-foot-midcoords :support-leg :support-leg-with-both))
       (cadr (memq param-name (send self :get-foot-step-params)))
     (error ";; no such abc footstep param ~A~%" param-name))
   )
  (:set-foot-steps-no-wait
   (foot-step-list)
   "Set foot step by default parameters and do not wait for step finish.
    foot-step-list is list of footsteps."
   (send self :autobalancerservice_setfootsteps
         :fs
         (mapcar #'(lambda (f)
                     (send self :eus-footstep->abc-footstep f))
                 foot-step-list)))
  (:set-foot-steps
   (foot-step-list)
   "Set foot step by default parameters and wait for step finish.
    foot-step-list is list of footsteps."
   (send self :set-foot-steps-no-wait foot-step-list)
   (send self :wait-foot-steps))
  (:set-foot-steps-with-param-no-wait
   (foot-step-list step-height-list step-time-list toe-angle-list heel-angle-list)
   "Set foot step with step parameter and do not wait for step finish.
    foot-step-list is list of footsteps.
    step-height-list is list of step height.
    step-time-list is list of step time.
    toe-angle-list is list of toe angle.
    heel-angle-list is list of heel angle."
   (send self :autobalancerservice_setfootstepswithparam
         :fs
         (mapcar #'(lambda (f)
                     (send self :eus-footstep->abc-footstep f))
                 foot-step-list)
         :sps
         (mapcar #'(lambda (sh st ta ha)
                     (instance hrpsys_ros_bridge::openhrp_autobalancerservice_stepparam :init :step_height (* sh 1e-3) :step_time st :toe_angle ta :heel_angle ha))
                 step-height-list step-time-list toe-angle-list heel-angle-list)
         ))
  (:set-foot-steps-with-param
   (foot-step-list step-height-list step-time-list toe-angle-list heel-angle-list)
   "Set foot step with step parameter and wait for step finish.
    For arguments, please see :set-foot-steps-with-param-no-wait documentation."
   (send self :set-foot-steps-with-param-no-wait foot-step-list step-height-list step-time-list toe-angle-list heel-angle-list)
   (send self :wait-foot-steps))
  (:set-foot-steps-roll-pitch
   (angle &key (axis :x))
   "Set foot steps with roll or pitch orientation.
    angle is roll or pitch angle [deg].
    axis is :x (roll) or :y (pitch)."
   (let ((off (cadr (memq :default-half-offset (send robot :footstep-parameter))))
         (fmcoords (midcoords 0.5 (send self :get-foot-step-param :rleg-coords) (send self :get-foot-step-param :lleg-coords))))
     (send self :set-foot-steps (list (make-coords :coords (send self :get-foot-step-param :rleg-coords) :name :rleg)
                                      (make-coords :coords (send (send (make-coords :pos (copy-object (send fmcoords :worldpos))) :rotate (deg2rad angle) axis)
                                                                 :translate off)
                                                   :name :lleg)
                                      (make-coords :coords (send (send (make-coords :pos (copy-object (send fmcoords :worldpos))) :rotate (deg2rad angle) axis)
                                                                 :translate (scale -1 off))
                                                   :name :rleg)))
     ))
  (:set-foot-steps-with-base-height
   (fs av-list time-list)
   (send self :set-foot-steps-no-wait fs)
   (send self :angle-vector-sequence av-list time-list)
   (send self :wait-interpolation)
   (send self :wait-foot-steps)
   )
  (:adjust-foot-steps
   (rfoot-coords lfoot-coords)
   "Adjust current footsteps during autobalancer mode and not walking.
    rfoot-coords and lfoot-coords are end-coords for new foot steps."
   (send self :autobalancerservice_adjustfootsteps
         :rfootstep (send self :eus-footstep->abc-footstep rfoot-coords)
         :lfootstep (send self :eus-footstep->abc-footstep lfoot-coords)))
  (:adjust-foot-steps-roll-pitch
   (angle &key (axis :x))
   "Adjust foot steps with roll or pitch orientation.
    angle is roll or pitch angle [deg].
    axis is :x (roll) or :y (pitch)."
   (let ((off (cadr (memq :default-half-offset (send robot :footstep-parameter))))
         (fmcoords (midcoords 0.5 (send self :get-foot-step-param :rleg-coords) (send self :get-foot-step-param :lleg-coords))))
     (send self :adjust-foot-steps
           (make-coords :coords (send (send (make-coords :pos (copy-object (send fmcoords :worldpos))) :rotate (deg2rad angle) axis)
                                      :translate (scale -1 off))
                        :name :rleg)
           (make-coords :coords (send (send (make-coords :pos (copy-object (send fmcoords :worldpos))) :rotate (deg2rad angle) axis)
                                      :translate off)
                        :name :lleg))
     ))
  (:get-remaining-foot-step-sequence
   ()
   "Get remaining foot steps from GaitGenerator.
   Return is (list current-support-foot-coords remaining-swing-dst-coords-0 ... )."
   (mapcar #'(lambda (fs)
               (send self :abc-footstep->eus-footstep fs))
           (send (send self :autobalancerservice_getRemainingFootstepSequence) :o_footstep))
   )
  (:draw-remaining-foot-step-sequence
   (vwer
    &key (flush) (rleg-color #f(1 0 0)) (lleg-color #f(0 1 0))
         (change-support-leg-color t) (support-leg-color #f(1 1 1)))
   "Draw remaining foot steps."
   (let ((fs (send self :get-remaining-foot-step-sequence)))
     (mapcar #'(lambda (f)
                 (let* ((leg (send f :name))
                        (fc (instance face :init
                                    :vertices
                                    (mapcar #'(lambda (x) (send f :transform-vector (send robot leg :end-coords :inverse-transform-vector x))) (butlast (send (send robot :support-polygon leg) :vertices))))))
                 (if (and change-support-leg-color (= (position f fs) 0))
                     (send fc :draw-on :color support-leg-color)
                   (send fc :draw-on :color (case (send f :name) (:rleg rleg-color) (:lleg lleg-color))))))
             fs)
     (if flush (send vwer :viewsurface :flush))
     ))
  (:go-velocity
   (vx vy vth)
   (send self :autobalancerservice_goVelocity :vx vx :vy vy :vth vth))
  (:go-stop
   ()
   "Stop stepping."
   (send self :autobalancerservice_goStop))
  (:emergency-walking-stop
   ()
   "Stop stepping immediately."
   (send self :autobalancerservice_emergencyStop))
  (:calc-go-velocity-param-from-velocity-center-offset
   (ang velocity-center-offset)
   "Calculate go-velocity velocities from rotation center and rotation angle.
    ang is rotation angle [rad]. velocity-center-offset is velocity center offset [mm] from foot mid coords."
   (let* ((default-step-time (send (send self :get-gait-generator-param) :default_step_time)) ;; [s]
          (cen (make-cascoords :pos velocity-center-offset))
          (cc (make-cascoords)))
     (send cen :assoc cc)
     (send cen :rotate (deg2rad ang) :z)
     (let ((tf (send (make-coords) :transformation cc)))
       (list
        (/ (* 1e-3 (elt (send tf :worldpos) 0)) default-step-time) ;; velx [m/s]
        (/ (* 1e-3 (elt (send tf :worldpos) 1)) default-step-time) ;; vely [m/s]
        (/ ang default-step-time) ;; velth [rad/s]
        ))))
  (:wait-foot-steps
   ()
   "Wait for whole footsteps are executed."
   (send self :autobalancerservice_waitFootSteps))
  ;; wrap :set-gait-generator-param to use symbol for default-orbit-type
  (:set-gait-generator-param
   (&rest args &key default-orbit-type &allow-other-keys)
   (send* self :raw-set-gait-generator-param
          (append
           (if (memq :default-orbit-type args)
               (list :default-orbit-type
                     (if (or (null default-orbit-type) (integerp default-orbit-type))
                         default-orbit-type
                       (let ((ot (read-from-string (format nil "HRPSYS_ROS_BRIDGE::OPENHRP_AUTOBALANCERSERVICE_ORBITTYPE::*~A*" (string-downcase default-orbit-type)))))
                         (if (boundp ot)
                             (eval ot)
                           (error ";; no such :default-orbit-type ~A in :set-gait-generator-param~%" default-orbit-type))
                         ))))
           args)))
  (:print-gait-generator-orbit-type
   ()
   (let ((cs (constants "" "HRPSYS_ROS_BRIDGE::OPENHRP_AUTOBALANCERSERVICE_ORBITTYPE")))
     (mapcar #'(lambda (x) (format t ";; ~A => ~A~%" x (eval x))) cs)
     t))
  (:get-gait-generator-orbit-type
   ()
   "Get GaitGenerator Orbit Type as Euslisp symbol."
   (send self :get-idl-enum-values
         (send (send self :get-gait-generator-param) :default_orbit_type)
         "HRPSYS_ROS_BRIDGE::OPENHRP_AUTOBALANCERSERVICE_ORBITTYPE"))
  (:get-auto-balancer-controller-mode
   ()
   "Get AutoBalancer ControllerMode as Euslisp symbol."
   (send self :get-idl-enum-values
         (send (send self :get-auto-balancer-param) :controller_mode)
         "HRPSYS_ROS_BRIDGE::OPENHRP_AUTOBALANCERSERVICE_CONTROLLERMODE"))
  ;; :get-auto-balancer-param and :set-auto-balancer-param is not defined by def-set-get-param-method yet.
  (:get-auto-balancer-param
   ()
   (send (send self :autobalancerservice_getautobalancerparam) :i_param))
  (:set-auto-balancer-param
   (&key default-zmp-offsets move-base-gain
         graspless-manip-arm graspless-manip-mode
         graspless-manip-p-gain
         graspless-manip-reference-trans-pos graspless-manip-reference-trans-rot
         transition-time
         zmp-transition-time
         adjust-footstep-transition-time)
   (let* ((current-param (send self :get-auto-balancer-param))
          (param (instance hrpsys_ros_bridge::Openhrp_AutoBalancerService_AutoBalancerParam
                           :init
                           :default_zmp_offsets
                           (if default-zmp-offsets
                               (let ((dzo (copy-object (send current-param :default_zmp_offsets))))
                                 (setq (dzo . ros::_data)
                                       (apply #'concatenate float-vector (mapcar #'(lambda (x) (scale 1e-3 x))
                                                                                 default-zmp-offsets)))
                                 dzo)
                             (send current-param :default_zmp_offsets))
                           :move_base_gain (if move-base-gain move-base-gain (send current-param :move_base_gain))
                           :graspless_manip_mode (if graspless-manip-mode graspless-manip-mode (send current-param :graspless_manip_mode))
                           :graspless_manip_arm (if graspless-manip-arm graspless-manip-arm (send current-param :graspless_manip_arm))
                           :graspless_manip_p_gain (if graspless-manip-p-gain graspless-manip-p-gain (send current-param :graspless_manip_p_gain))
                           :graspless_manip_reference_trans_pos
                           (if graspless-manip-reference-trans-pos
                               (scale 1e-3 graspless-manip-reference-trans-pos)
                             (send current-param :graspless_manip_reference_trans_pos))
                           :graspless_manip_reference_trans_rot
                           (if graspless-manip-reference-trans-rot
                               (matrix2quaternion graspless-manip-reference-trans-rot)
                             (send current-param :graspless_manip_reference_trans_rot))
                           :transition_time
                           (if transition-time
                               transition-time
                             (send current-param :transition_time))
                           :zmp_transition_time
                           (if zmp-transition-time
                               zmp-transition-time
                             (send current-param :zmp_transition_time))
                           :adjust_footstep_transition_time
                           (if adjust-footstep-transition-time
                               adjust-footstep-transition-time
                             (send current-param :adjust_footstep_transition_time))
                           )))
     (send self :autobalancerservice_setautobalancerparam :i_param param)))
  (:abc-footstep->eus-footstep
   (f)
   (make-coords :pos (scale 1e3 (send f :pos))
                :rot (quaternion2matrix (send f :rot))
                :name (read-from-string (format nil ":~A" (send f :leg))))
   )
  (:eus-footstep->abc-footstep
   (f)
   (instance hrpsys_ros_bridge::openhrp_autobalancerservice_footstep :init
             :pos (scale 1e-3 (send f :worldpos))
             :rot (matrix2quaternion (send f :worldrot))
             :leg (string-downcase (if (find-method f :l/r) (send f :l/r) (send f :name))))
   )
  (:cmd-vel-cb
   (msg &key (vel-x-ratio 1.0) (vel-y-ratio 1.0) (vel-th-ratio 1.0))
   (send self :go-velocity
	 (* vel-x-ratio (send (send msg :linear) :x))
         (* vel-y-ratio (send (send msg :linear) :y))
         (* vel-th-ratio (send (send msg :angular) :z)))
   )
  (:cmd-vel-mode
   ()
   (send self :start-cmd-vel-mode)
   (do-until-key
    (send self :go-velocity 0 0 0)
    (ros::spin-once)
    (ros::sleep)
    )
   (send self :stop-cmd-vel-mode)
   )
  (:start-cmd-vel-mode
   ()
   (print ";; start cmd-vel mode")
   (ros::subscribe "cmd_vel" geometry_msgs::Twist #'send self :cmd-vel-cb)
   (send self :go-velocity 0 0 0)
   (print ";; start cmd-vel mode done.")
   )
  (:stop-cmd-vel-mode
   ()
   (print ";; stop cmd-vel mode")
   (send self :go-stop)
   (ros::unsubscribe "cmd_vel")
   (print ";; stop cmd-vel mode done.")
   )
  )

;; SoftErrorLimiterService
(defmethod rtm-ros-robot-interface
  (:set-soft-error-limit
    (name limit)
    (send self :softerrorlimiterservice_setservoerrorlimit :name name :limit limit)
    )
  )

;; StabilizerService
(def-set-get-param-method
  'hrpsys_ros_bridge::Openhrp_StabilizerService_stParam
  :raw-set-st-param :get-st-param
  :stabilizerservice_setparameter :stabilizerservice_getparameter)

(defmethod rtm-ros-robot-interface
  (:set-st-param
   (&rest args &key st-algorithm &allow-other-keys)
   "Set Stabilizer parameters.
    To learn more about arguments, please see (pprint (find-method *ri* :raw-set-st-param))."
   (unless (send self :get :default-st-param)
     (send self :put :default-st-param (send self :get-st-param)))
   (send* self :raw-set-st-param
          (append
           (if (memq :st-algorithm args)
               (list :st-algorithm
                     (if (or (null st-algorithm) (integerp st-algorithm))
                         st-algorithm
                       (let ((sa (read-from-string (format nil "HRPSYS_ROS_BRIDGE::OPENHRP_STABILIZERSERVICE_STALGORITHM::*~A*" (string-downcase st-algorithm)))))
                         (if (boundp sa)
                             (eval sa)
                           (error ";; no such :st-algorithm ~A in :set-st-param~%" st-algorithm))))))
           args)))
  (:set-st-param-for-non-feedback-LIP-mode
   ()
   "Set Stabilizer parameters to make robot Linear Inverted Pendulum mode without state feedback.
    By using this mode, robot feets adapt to ground surface."
   (send self :set-st-param
         :eefm-body-attitude-control-gain #F(0 0)
         :eefm-wrench-alpha-blending 1.0
         :eefm-k1 #f(0 0)
         :eefm-k2 #f(0 0)
         :eefm-k3 #f(0 0))
   )
  (:set-default-st-param
   ()
   "Set Stabilzier parameter by default parameters."
   (unless (send self :get :default-st-param)
     (send self :put :default-st-param (send self :get-st-param)))
   (send self :stabilizerservice_setparameter :i_param (send self :get :default-st-param))
   )
  (:set-st-param-by-ratio
   (state-feedback-gain-ratio
    damping-control-gain-ratio
    attitude-control-gain-ratio)
   "Set Stabilzier parameter by ratio from default parameters.
    state-feedback-gain-ratio is ratio for state feedback gain.
    damping-control-gain-ratio is ratio for damping control gain.
    attitude-control-gain-ratio is ratio for attitude control gain.
    When ratio = 1, parameters are same as default parameters.
    For state-feedback-gain-ratio and attitude-control-gain-ratio,
    when ratio < 1 Stabilzier works less feedback control and less oscillation and when ratio > 1 more feedback and more oscillation.
    For dampnig-control-gain-ratio,
    when ratio > 1 feet behavior becomes hard and safe and when ratio < 1 soft and dangerous."
   (unless (send self :get :default-st-param)
     (send self :put :default-st-param (send self :get-st-param)))
   (let ((stp (send self :get :default-st-param)))
     (send self :set-st-param
           ;; state feedback gain
           :eefm-k1 (scale state-feedback-gain-ratio (send stp :eefm_k1))
           :eefm-k2 (scale state-feedback-gain-ratio (send stp :eefm_k2))
           :eefm-k3 (scale state-feedback-gain-ratio (send stp :eefm_k3))
           ;; damping control gain
           :eefm-pos-damping-gain (* damping-control-gain-ratio (send stp :eefm_pos_damping_gain))
           :eefm-rot-damping-gain (* damping-control-gain-ratio (send stp :eefm_rot_damping_gain))
           :eefm-pos-time-const-support (/ (send stp :eefm_pos_time_const_support) damping-control-gain-ratio)
           :eefm-rot-time-const (/ (send stp :eefm_rot_time_const) damping-control-gain-ratio)
           ;; body attitude control gain
           :eefm-body-attitude-control-gain (scale attitude-control-gain-ratio (send stp :eefm_body_attitude_control_gain)))
     ))
  (:get-st-controller-mode
   ()
   "Get Stabilizer ControllerMode as Euslisp symbol."
   (send self :get-idl-enum-values
         (send (send self :get-st-param) :controller_mode)
         "HRPSYS_ROS_BRIDGE::OPENHRP_STABILIZERSERVICE_CONTROLLERMODE"))
  (:get-st-algorithm
   ()
   "Get Stabilizer Algorithm as Euslisp symbol."
   (send self :get-idl-enum-values
         (send (send self :get-st-param) :st_algorithm)
         "HRPSYS_ROS_BRIDGE::OPENHRP_STABILIZERSERVICE_STALGORITHM"))
  (:start-st
   ()
   "Start Stabilizer Mode."
   (send self :stabilizerservice_startstabilizer)
   )
  (:stop-st
   ()
   "Stop Stabilizer Mode."
   (send self :stabilizerservice_stopstabilizer)
   )
  )

;; KalmanFilterService
(def-set-get-param-method
  'hrpsys_ros_bridge::Openhrp_KalmanFilterService_KalmanFilterParam
  :set-kalman-filter-param :get-kalman-filter-param
  :kalmanfilterservice_setkalmanfilterparam :kalmanfilterservice_getkalmanfilterparam)

(defmethod rtm-ros-robot-interface
  (:get-kalman-filter-algorithm
   ()
   "Get KalmanFilter Algorithm as Euslisp symbol."
   (send self :get-idl-enum-values
         (send (send self :get-kalman-filter-param) :kf_algorithm)
         "HRPSYS_ROS_BRIDGE::OPENHRP_KALMANFILTERSERVICE_KFALGORITHM"))
)

(defmethod rtm-ros-robot-interface
  (:start-default-unstable-controllers
   (&key (ic-limbs '(:rarm :larm)) (abc-limbs '(:rleg :lleg)))
   "Start default unstable RTCs controller mode.
    Currently Stabilzier, AutoBalancer, and ImpedanceController are started."
   (send self :start-st)
   (dolist (limb ic-limbs)
     (send self :start-impedance-no-wait limb))
   (send self :start-auto-balancer :limbs abc-limbs)
   (dolist (limb ic-limbs)
     (send self :wait-impedance-controller-transition limb))
   )
  (:stop-default-unstable-controllers
   (&key (ic-limbs '(:rarm :larm)))
   "Stop default unstable RTCs controller mode.
    Currently Stabilzier, AutoBalancer, and ImpedanceController are stopped."
   (send self :stop-st)
   (dolist (limb ic-limbs)
     (send self :stop-impedance-no-wait limb))
   (send self :stop-auto-balancer)
   (dolist (limb ic-limbs)
     (send self :wait-impedance-controller-transition limb))
   )
)

(defun print-end-effector-parameter-conf-from-robot
  (rb)
  "Print end effector setting for hrpsys conf file."
  (format t "end_effectors:")
  (let ((limb-list (mapcar #'(lambda (x) (find-if #'(lambda (l) (member x (send rb l :force-sensors))) '(:rleg :lleg :rarm :larm)))
                           (send rb :force-sensors))))
    (dolist (l limb-list)
      (let ((rl (send (send (send rb l :root-link) :parent) :joint)))
        (format t " ~A,~A,~A," (string-downcase (string l)) (send (send (send rb l :end-coords :parent) :joint) :name) (if rl (send rl :name) "WAIST"))
        (let* ((dif (send (send rb l :end-coords :parent) :transformation (send rb l :end-coords)))
               (wp (scale 1e-3 (send dif :worldpos))) ;; [mm] -> [m]
               (wr (normalize-vector (matrix-log (send dif :worldrot))))
               (wrn (norm (matrix-log (send dif :worldrot)))))
          (format t "~A,~A,~A," (elt wp 0) (elt wp 1) (elt wp 2))
          (format t "~A,~A,~A,~A," (elt wr 0) (elt wr 1) (elt wr 2) wrn)
          )))
    (format t "~%")
    ))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; utility functions for seq
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun dump-seq-pattern-file
  (rs-list output-basename &key (initial-sync-time 3.0)) ;; [s]
  "Dump pattern file for SequencePlayer.
     rs-list : list of (list :time time0 :angle-vector av :root-coords rc ...).
               Fields other than :time and :angle-vector are optional.
     output-basename : output file (output-basename.pos, ...).
     root-coords : worldcoords for root link.
     zmp : world zmp[mm].
     wrench-list : world (list force-list moment-list) at end effector."
  (with-open-file
   (pf (format nil "~A.pos" output-basename) :direction :output) ;; pos file -> joint angles
   (dolist (l rs-list)
     ;; write pos file
     (format pf "~A " (+ initial-sync-time (cadr (memq :time l))))
     (let ((av (cadr (memq :angle-vector l))))
       (dotimes (i (length av))
         (format pf "~A " (deg2rad (elt av i)))
         ))
     (format pf "~%"))) ;; with-open-file for .pos file
  (when (cadr (memq :torque-vector (car rs-list)))
    (with-open-file
     (tf (format nil "~A.torque" output-basename) :direction :output) ;; torque file -> joint torques
     (dolist (l rs-list)
       ;; write torque file
       (format tf "~A " (+ initial-sync-time (cadr (memq :time l))))
       (let ((av (cadr (memq :torque-vector l))))
         (dotimes (i (length av))
           (format tf "~A " (elt av i))
           ))
       (format tf "~%")))) ;; with-open-file for .torque file
  (when (cadr (memq :root-coords (car rs-list)))
    (with-open-file
     (wf (format nil "~A.waist" output-basename) :direction :output) ;; waist file -> base position and orientation
     (dolist (l rs-list)
       ;; write waist file
       (format wf "~A " (+ initial-sync-time (cadr (memq :time l))))
       (let ((wp (scale 1e-3 (send (cadr (memq :root-coords l)) :worldpos))) ;; [mm] -> [m]
             (wr (car (send (send (cadr (memq :root-coords l)) :copy-worldcoords) :rpy-angle))))
         (format wf "~A ~A ~A " (elt wp 0) (elt wp 1) (elt wp 2))
         (format wf "~A ~A ~A " (elt wr 2) (elt wr 1) (elt wr 0))
         (format wf "~%")
         ))) ;; with-open-file for .waist file
    )
  (when (or (cadr (memq :root-local-zmp (car rs-list)))
            (and (cadr (memq :root-coords (car rs-list))) (cadr (memq :zmp (car rs-list)))))
    (with-open-file
     (zf (format nil "~A.zmp" output-basename) :direction :output) ;; zmp file -> root-relative zmp[m]
     (dolist (l rs-list)
       ;; write zmp file
       (format zf "~A " (+ initial-sync-time (cadr (memq :time l))))
       (let ((zp (scale 1e-3
                        (or (cadr (memq :root-local-zmp l))
                            (send (cadr (memq :root-coords l)) :inverse-transform-vector (cadr (memq :zmp l))))
                        ))) ;; [mm] -> [m]
         (format zf "~A ~A ~A " (elt zp 0) (elt zp 1) (elt zp 2))
         (format zf "~%")
         ))) ;; with-open-file for .zmp file
    )
  (when (or (cadr (memq :ee-local-wrench-list (car rs-list)))
            (and (cadr (memq :end-coords-list (car rs-list))) (cadr (memq :wrench-list (car rs-list)))))
    (with-open-file
     (wrf (format nil "~A.wrenches" output-basename) :direction :output) ;; wrench file -> force and moment ;; 6 x force-sensor-num
     (dolist (l rs-list)
       ;; write wrenches file
       (format wrf "~A " (+ initial-sync-time (cadr (memq :time l))))
       (let ((wrl
              (or (cadr (memq :ee-local-wrench-list l))
                  (list
                   (mapcar #'(lambda (ee ff)
                               (send ee :inverse-rotate-vector ff))
                           (cadr (memq :end-coords-list l)) (car (cadr (memq :wrench-list l))))
                   (mapcar #'(lambda (ee mm)
                               (send ee :inverse-rotate-vector mm))
                           (cadr (memq :end-coords-list l)) (cadr (cadr (memq :wrench-list l))))))))
         (dotimes (j (length (car wrl))) ;; j is limb index
           (let ((f (elt (car wrl) j)) (m (elt (cadr wrl) j)))
             (dotimes (i 3) (format wrf "~A " (elt f i))) ;; force
             (dotimes (i 3) (format wrf "~A " (elt m i))) ;; moment
             )))
       (format wrf "~%")
       )) ;; with-open-file for .wrenches file
    )
  (when (cadr (memq :optional-data (car rs-list)))
    (with-open-file
     (opf (format nil "~A.optionaldata" output-basename) :direction :output) ;; optionaldata file
     (let ((len (length (cadr (memq :optional-data (car rs-list))))))
       (dolist (rs rs-list)
         (let ((dd (cadr (memq :optional-data rs))))
           (format opf "~A " (+ initial-sync-time (cadr (memq :time rs))))
           (dotimes (i len) (format opf "~A " (elt dd i))))
         (format opf "~%")
         )
       )) ;; with-open-file for .optionaldata
    )
  )

(defun load-from-seq-pattern-file (input-basename)
  "Load from seq pattern file and generate robot state list."
  (labels ((data-string-split ;; this function will be replaced by https://github.com/euslisp/EusLisp/issues/16
            (str separator)
            (let ((start-idx 0) (ret))
              (dotimes (i (length str))
                (when (= (elt str i) (elt separator 0))
                  (push (subseq str start-idx i) ret)
                  (setq start-idx (1+ i))))
              (if (/= start-idx (length str)) (push (subseq str start-idx) ret))
              (reverse ret)))
           (data-str->data-list
            (data-str)
            (mapcar #'(lambda (x) (read-from-string x))
                    (remove-if #'(lambda (x) (string= "" x)) (data-string-split data-str " ")))))
    (let (rs-list)
      ;; pos file
      (let ((pln) (posfile (concatenate string input-basename ".pos")))
        (with-open-file
         (pf posfile :direction :input)
         (while (setq pln (read-line pf nil))
           (let ((dd (data-str->data-list pln)))
             (push (list :angle-vector (map float-vector #'rad2deg (concatenate float-vector (cdr dd)))
                         :time (car dd))
                   rs-list)
             ))))
      (setq rs-list (reverse rs-list))
      ;; waist file
      (let ((wln) (waistfile (concatenate string input-basename ".waist")) (idx 0))
        (if (probe-file waistfile)
            (with-open-file
             (wf waistfile :direction :input)
             (while (setq wln (read-line wf nil))
               (let ((dd (data-str->data-list wln)))
                 (nconc (elt rs-list idx)
                        (list :root-coords (make-coords :pos (float-vector (* 1e3 (elt dd 1)) (* 1e3 (elt dd 2)) (* 1e3 (elt dd 3))) ;; [m]->[mm]
                                                        :rpy (list (elt dd 6) (elt dd 5) (elt dd 4))))) ;; rpy->ypr
                 (incf idx)
                 )))))
      ;; zmp file
      (let ((zln) (zmpfile (concatenate string input-basename ".zmp")) (idx 0))
        (if (probe-file zmpfile)
            (with-open-file
             (zf zmpfile :direction :input)
             (while (setq zln (read-line zf nil))
               (let* ((dd (data-str->data-list zln))
                      (root-local-zmp (float-vector (* 1e3 (elt dd 1)) (* 1e3 (elt dd 2)) (* 1e3 (elt dd 3))))) ;; [m]->[mm]
                 (nconc (elt rs-list idx)
                        (if (cadr (memq :root-coords (elt rs-list idx)))
                            (list :zmp (send (cadr (memq :root-coords (elt rs-list idx))) :transform-vector root-local-zmp))
                          (list :root-local-zmp root-local-zmp)))
                 (incf idx)
                 )))))
      rs-list)))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; utility functions for st
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun calculate-eefm-st-state-feedback-gain
  (default-cog-height
   &key (alpha -13.0) (beta -4.0)
        ((:time-constant Tp) 0.04)
        ((:gravitational-acceleration ga) (* 1e-3 (elt *g-vec* 2)))
        ((:print-mode pm) :euslisp))
  "Calculate EEFMe st state feedback gain (k1, k2, k3) and print them.
   default-cog-height is default COG height[mm].
   alpha and beta are poles. -13.0 and -4.0 by default.
   time-constant is time constant of ZMP tracking delay [s]. 0.04[s] by default.
   gravitational-acceleration is gravitational acceleration [m/s^2].
   When print-mode is :euslisp, print k1, k2, and k3 in Euslisp manner.
   When print-mode is :python, print k1, k2, and k3 in python hrpsys_config manner."
  (let* ((omega (sqrt (/ ga (* 1e-3 default-cog-height))))
         (gamma (- omega))
         (omega2 (* omega omega))
         (d1 (+ alpha beta gamma))
         (d2 (+ (* alpha beta) (* beta gamma) (* gamma alpha)))
         (d3 (* alpha beta gamma))
         (k3 (- (* -1 Tp d1) 1))
         (k2 (* Tp (+ -1 (* -1 (/ d2 omega2)))))
         (k1 (- (* Tp (/ d3 omega2)) 1 k3)))
    (case
     pm
     (:euslisp
      (format t "(send *ri* :set-st-param :eefm-k1 #f(~A ~A))~%" k1 k1)
      (format t "(send *ri* :set-st-param :eefm-k2 #f(~A ~A))~%" k2 k2)
      (format t "(send *ri* :set-st-param :eefm-k3 #f(~A ~A))~%" k3 k3)
      )
     (:python
      (format t "stp=hcf.st_svc.getParameter()~%")
      (format t "stp.eefm_k1=[~A,~A]~%" k1 k1)
      (format t "stp.eefm_k2=[~A,~A]~%" k2 k2)
      (format t "stp.eefm_k3=[~A,~A]~%" k3 k3)
      (format t "hcf.st_svc.setParameter(stp)~%")
      )
     (t))
    t))

(defun calculate-eefm-st-state-feedback-default-gain-from-robot
  (robot
   &key (alpha -13.0) (beta -4.0)
        ((:time-constant Tp) 0.04)
        ((:gravitational-acceleration ga) (* 1e-3 (elt *g-vec* 2)))
        ((:print-mode pm) :euslisp)
        (exec-reset-pose-p t))
  "Calculate EEFMe st state feedback gain (k1, k2, k3) from robot model and print them.
   robot is robot model and calculate gains from reset-pose by default."
  (if exec-reset-pose-p (send robot :reset-pose))
  (calculate-eefm-st-state-feedback-gain
   (elt (send (send robot :foot-midcoords) :inverse-transform-vector (send robot :centroid)) 2)
   :alpha alpha :beta beta :time-constant Tp :print-mode pm :gravitational-acceleration ga)
  )

(defun calculate-toe-heel-offsets
  (robot &key ((:print-mode pm) :euslisp) (pos-or-zmp :pos))
  (let* ((sp-vertices (mapcar #'(lambda (x) (send robot :rleg :end-coords :inverse-transform-vector x)) (send (send robot :support-polygon :rleg) :vertices)))
         (toe-pos (elt (find-extream sp-vertices #'(lambda (x) (elt x 0)) #'>) 0))
         (heel-pos (elt (find-extream sp-vertices #'(lambda (x) (elt x 0)) #'<) 0)))
    (case
        pm
      (:euslisp
       (format t "(send *ri* :set-gait-generator-param :toe-~A-offset-x (* 1e-3 ~A) :heel-~A-offset-x (* 1e-3 ~A))~%"
               (string-downcase pos-or-zmp) toe-pos (string-downcase pos-or-zmp) heel-pos))
      (:python
       (format t "ggp=hcf.abc_svc.getGaitGeneratorParam()[1];~%")
       (format t "ggp.toe_~A_offset_x = 1e-3*~A;ggp.heel_~A_offset_x = 1e-3*~A;~%"
               (string-downcase pos-or-zmp) toe-pos (string-downcase pos-or-zmp) heel-pos)
       (format t "hcf.abc_svc.setGaitGeneratorParam(ggp);~%")))
    (list toe-pos heel-pos)
    ))

(defun calculate-toe-heel-pos-offsets
  (robot &key ((:print-mode pm) :euslisp))
  "Calculate toe and heel position offset in ee frame for toe heel contact used in GaitGenerator in AutoBalancer.
   robot is robot model.
   When print-mode is :euslisp, print parameters in Euslisp style.
   When print-mode is :python, print parameters in Python hrpsys_config style."
  (calculate-toe-heel-offsets robot :print-mode pm))

(defun calculate-toe-heel-zmp-offsets
  (robot &key ((:print-mode pm) :euslisp))
  "Calculate toe and heel zmp offset in ee frame for toe heel contact used in GaitGenerator in AutoBalancer.
   robot is robot model.
   When print-mode is :euslisp, print parameters in Euslisp style.
   When print-mode is :python, print parameters in Python hrpsys_config style."
  (calculate-toe-heel-offsets robot :print-mode pm :pos-or-zmp :zmp))


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; utility functions for project generator generation
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;; clone euslisp robot and objects to OpenHRP3 project file
;;   add euslisp model + locate euslisp model in OpenHRP3 world
(defun dump-project-file-by-cloning-euslisp-models
  (robot robot-file-path ;; robot euslisp model, robot VRML(or Collada) file path
   &key (object-models) (object-models-file-path) ;; list of object euslisp model, list of object VRML(or Collada) file path
        (nosim) (timestep 0.005) (dt 0.005) ;; [s]
        (output-fname (format nil "/tmp/~A" (send robot :name)))) ;; output file name "output-fname.xml"
  ;; TODO : Is longfloor.wrl necessary??
  (let ((str
         (format nil "rosrun hrpsys_ros_bridge rtmtest -t hrpsys_tools _gen_project.launch \\\
               INPUT:=~A,~A \\\
               OBJECT_MODELS:='`rospack find openhrp3`/share/OpenHRP-3.1/sample/model/longfloor.wrl,0,0,0,1,0,0,0 ~A' \\\
               OUTPUT:=~A.xml \\\
               INTEGRATE:=~A CORBA_PORT:=15005 CONF_DT_OPTION:='--dt ~A' \\\
               SIMULATION_TIMESTEP_OPTION:='--timeStep ~A' \\\
               SIMULATION_JOINT_PROPERTIES_OPTION:='--joint-properties ~A'"
                 robot-file-path
                 (gen-ProjectGenerator-model-root-coords-string robot)
                 (let ((obj-path-list
                        (mapcar #'(lambda (fpath obj) (format nil "~A,~A" fpath (gen-ProjectGenerator-model-root-coords-string obj)))
                                object-models-file-path object-models)))
                   (if obj-path-list
                       (reduce #'(lambda (x y) (format nil "~A ~A" x y)) obj-path-list)
                     ""))
                 (format nil "~A~A" output-fname (if nosim "_nosim" ""))
                 (if nosim "false" "true")
                 dt
                 timestep
                 (gen-ProjectGenerator-joint-properties-string robot)
                 )))
    (unix:system (format nil "bash -c -i \"~A;exit 0\";exit 0" str))
    ))

(defun gen-ProjectGenerator-joint-properties-string
  (robot)
  (let ((str))
    (dolist (j (send robot :joint-list))
      (if str
          (setq str (format nil "~A,~A.angle,~A" str (send j :name) (deg2rad (send j :joint-angle))))
        (setq str (format nil "~A.angle,~A" (send j :name) (deg2rad (send j :joint-angle)))))
      )
    str))

(defun gen-ProjectGenerator-model-root-coords-string
  (obj)
  (let* ((dr (matrix-log (send (car (send obj :links)) :worldrot)))
         (ndr (normalize-vector dr))
         (rpos (scale 1e-3 (send (car (send obj :links)) :worldpos))))
      (format nil "~A,~A,~A,~A,~A,~A,~A"
              (elt rpos 0) (elt rpos 1) (elt rpos 2)
              (elt ndr 0) (elt ndr 1) (elt ndr 2) (/ (elt dr 1) (elt ndr 1)) ;; openhrp3 axis angle orientation representation
              )))

