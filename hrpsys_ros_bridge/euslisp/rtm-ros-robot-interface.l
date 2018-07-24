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
   ":init method. This method should be overriden in subclass."
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
     (ros::subscribe "/emergency_mode" std_msgs::Int32
                     #'send self :rtmros-emergency-mode-callback :groupname groupname)
     (mapcar #'(lambda (x)
                 (ros::subscribe (format nil "/~A_capture_point" x) geometry_msgs::PointStamped
                                 #'send self :rtmros-capture-point-callback (read-from-string (format nil ":~A-capture-point" x)) :groupname groupname))
             '(ref act))
     (mapcar #'(lambda (x)
                 (ros::subscribe (format nil "/~A_contact_states" x) hrpsys_ros_bridge::ContactStatesStamped
                                 #'send self :rtmros-contact-states-callback (read-from-string (format nil ":~A-contact-states" x)) :groupname groupname))
             '(ref act))
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
  (:rtmros-emergency-mode-callback
   (msg)
   (let ((m (send msg :data)))
     (send self :set-robot-state1 :emergency-mode m))
   )
  (:rtmros-capture-point-callback
   (ref-act msg)
   (let ((p (send msg :point)))
     (send self :set-robot-state1 ref-act (float-vector (send p :x) (send p :y) (send p :z)))))
  (:rtmros-contact-states-callback
   (ref-act msg)
   (let ((ss (send-all (send msg :states) :state :state)))
     (send self :set-robot-state1 ref-act ss)))
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
  (:ref-capture-point-vector
   (&optional (wrt :local))
   "Returns ref-capture-point vector [mm].
    If wrt is :local, returns ref-capture-point in the base-link frame. If wrt is :world, returns ref-capture-point in the world frame."
   (let ((cp (scale 1e3 (cdr (assoc :ref-capture-point robot-state))))) ;; [m] -> [mm]
     (case wrt
       (:local cp)
       (:world (send (car (send robot :links)) :transform-vector cp)))))
  (:act-capture-point-vector
   (&optional (wrt :local))
   "Returns act-capture-point vector [mm].
    If wrt is :local, returns act-capture-point in the base-link frame. If wrt is :world, returns act-capture-point in the world frame."
   (let ((cp (scale 1e3 (cdr (assoc :act-capture-point robot-state))))) ;; [m] -> [mm]
     (case wrt
       (:local cp)
       (:world (send (car (send robot :links)) :transform-vector cp)))))
  (:ref-contact-states
   ()
   "Returns contact states from AutoBalancer."
   (cdr (assoc :ref-contact-states robot-state)))
  (:act-contact-states
   ()
   "Returns contact states from Stabilizer."
   (cdr (assoc :act-contact-states robot-state)))
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
           ,(append (cadr init-method) (list (list 'rosbridge-name (format nil "~AROSBridge" rtc-name))))
           (or
           (ros::service-call
            (format nil "/~A/~A" rosbridge-name ,meth-name)
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
  (:calc-zmp-from-state
   (&key (wrt :world))
   "Calculate zmp from state [mm].
    For example ;; (progn (send *ri* :go-velocity 0 0 0) (objects (list (*ri* . robot))) (do-until-key (let ((zmp (send *ri* :calc-zmp-from-state))) (send *irtviewer* :draw-objects :flush nil) (send zmp :draw-on :flush t :size 300))))
    :wrt is :local => calc local zmp for (*ri* . robot)'s root-link coords
    :wrt is :world => calc world zmp for (*ri* . robot)"
   (send self :state)
   (send robot :calc-zmp-from-forces-moments
         (mapcar #'(lambda (x) (send self :force-vector x)) '(:rleg :lleg))
         (mapcar #'(lambda (x) (send self :moment-vector x)) '(:rleg :lleg))
         :wrt wrt)
   )
  (:get-robot-date-string
   ()
   "Get string including robot name and date.
    For example, \"SampleRobot_20160412163151\"."
   (let* ((dt (unix:localtime)))
     (format nil "~A_~A~A~A~A~A~A"
             (send (send self :robot) :name)
             (digits-string (+ 1900 (aref dt 5)) 4)
             (digits-string (1+ (aref dt 4)) 2)
             (digits-string (aref dt 3) 2)
             (digits-string (aref dt 2) 2)
             (digits-string (aref dt 1) 2)
             (digits-string (aref dt 0) 2))))
  (:def-limb-controller-method
    (limb &key (debugp nil))
    "Method to add limb controller action by default setting.
     Currently, FollowJointTrajectoryAction is used.
     This method calls defmethod. If :debugp t, we can see defmethod s-expressions."
    (let ((sexp `(defmethod ,(send (class self):name)
                   (,(read-from-string (format nil "~A-controller" limb))
                    ()
                    (list
                     (list
                      (cons :group-name ,(string-downcase limb))
                      (cons :controller-action ,(format nil "~A_controller/follow_joint_trajectory_action" (string-downcase limb)))
                      (cons :controller-state ,(format nil "~A_controller/state" (string-downcase limb)))
                      (cons :action-type control_msgs::FollowJointTrajectoryAction)
                      (cons :joint-names (list ,@(send-all (send robot limb :joint-list) :name))))
                     )))))
      (if debugp
          (pprint (macroexpand sexp)))
      (eval sexp)
      ))
  )

;; define Euslisp setter and getter method
(defun def-set-get-param-method
  (param-class ;; parameter class
   set-param-method-name get-param-method-name getarg-method-name ;; Euslisp setter and getter method which user want to define
   set-param-idl-name get-param-idl-name ;; raw setter and getter method converted from idl2srv files
   &key (optional-args) ;; arguments for raw setter and getter method
        (debug nil))
  (when (boundp param-class)
    (let* ((param-slots-list ;; get slots list for param-class
            (remove-if #'(lambda (x) (string= "plist" x))
                       (mapcar #'(lambda (x) (string-left-trim "::_" (string-left-trim "ros" (format nil "~A" x))))
                               (concatenate cons (send (eval param-class) :slots)))))
           (param-name
            ;; Extract param class, e.g., extract zz from hrpsys_ros_bridge::xx_yy_zz
            (car (last (read-from-string (substitute (elt " " 0) (elt ":" 0) (substitute (elt " " 0) (elt "_" 0) (format nil "(~A)" param-class)))))))
           (getter-defmethod-macro
            `(defmethod rtm-ros-robot-interface
               (,get-param-method-name
                ;; Arguments
                ,(if optional-args (list (cadr optional-args)) (list ))
                ;; Documentation string
                ,(unless (substringp "raw-" (string-downcase get-param-method-name))
                   (format nil "Get ~A." param-name))
                ;; Implementation
                (send (send self ,get-param-idl-name ,@optional-args) :i_param))))
           ;; generate defmethod like
           ;;  (:set-xx-param (&rest args &key yy-zz)
           ;;   (let ((current-param (send self :get-xx-param))
           ;;         (param (instance ww :init (if (memq :yy-zz args) (cadr (memq :yy-zz args)) (send current-param :yy_zz))))
           ;;      (send self :aaService_setParameter :i_param param)))
           (setter-defmethod-macro
            `(defmethod rtm-ros-robot-interface
               (,set-param-method-name
                ;; Arguments
                ,(append (if optional-args (list (cadr optional-args))) (list '&rest 'args '&key) (mapcar #'(lambda (x) (read-from-string (substitute (elt "-" 0) (elt "_" 0) x))) param-slots-list)) ;; replace _ => - for Euslisp friendly argument
                ;; Documentation string
                ,(unless (substringp "raw-" (string-downcase set-param-method-name))
                   (format nil "Set ~A. For arguments, please see (send *ri* ~A)." param-name getarg-method-name))
                ;; Implementation
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
                  ))))
           (getarg-defmethod-macro
            `(defmethod rtm-ros-robot-interface
               (,getarg-method-name
                ()
                ,(format nil "Get arguments of ~A" set-param-method-name)
                ,(append '(list) (mapcar #'(lambda (x) (read-from-string (format nil ":~A" (substitute (elt "-" 0) (elt "_" 0) x)))) param-slots-list)))))
           )
      (when debug
        (pprint (macroexpand getter-defmethod-macro))
        (pprint (macroexpand setter-defmethod-macro))
        (pprint (macroexpand getarg-defmethod-macro))
        )
      (eval getter-defmethod-macro)
      (eval setter-defmethod-macro)
      (eval getarg-defmethod-macro)
      t)))

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
   "Wait interpolation of group.
    !!This method is not recommended, please use :wait-interpolation method like (send *ri* :wait-interpolation :head-controller).!!"
   (send self :sequenceplayerservice_waitinterpolationofgroup :gname groupname))
  (:add-joint-group
   (groupname &optional (jnames
                         (if (find-method self (read-from-string (format nil ":~A-controller" (string-downcase groupname))))
                             (cdr (assoc :joint-names (car (send self (read-from-string (format nil ":~A-controller" (string-downcase groupname))))))))))
   "Add joint group for SequencePlayer.
    groupname is joint group name such as rarm or lleg.
    jnames is list of joint name."
   (unless jnames
     (error ";; jnames argument is require in :add-joint-group~%"))
   (send self :sequenceplayerservice_addjointgroup :gname groupname :jnames jnames))
  (:remove-joint-group
   (groupname)
   "Remove joint group for SequencePlayer.
    groupname is joint group name such as rarm or lleg."
   (send self :sequenceplayerservice_removejointgroup :gname groupname))
  (:set-joint-angles-of-group
   (groupname av tm)
   "Set joint angles of group.
    !!This method is not recommended, please use :angle-vector method like (send *ri* :angle-vector (send *robot* :angle-vector) 2000 :head-controller).!!"
   (send self :sequenceplayerservice_setjointanglesofgroup :gname groupname :jvs av :tm tm))
  (:load-pattern
   (basename &optional (tm 0.0))
   "Load pattern files, such as xx.pos and xx.waist.
    For pattern file definitions, please see loadPattern in SequencePlayer documentation in hrpsys-base API Doc."
   (send self :sequenceplayerservice_loadpattern :basename basename :tm tm))
  (:wait-interpolation-seq
   ()
   "Directly call SequencePlayer waitInterpolation.
    This can be used for force/moment interpolation."
   (send self :sequenceplayerservice_waitinterpolation))
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
  (:angle-vector-sequence-full
   (jpos tm
         &key
         (sequence-length (length jpos))
         (joint-length (length (car jpos)))
         (fsensor-length (length (send robot :force-sensors))) ;; "fsensor length = ee length" is assumed
         (vel (make-list sequence-length :initial-element (instantiate float-vector joint-length)))
         (torque (make-list sequence-length :initial-element (instantiate float-vector joint-length)))
         (root-coords (make-list sequence-length :initial-element (make-coords)))
         (acc (make-list sequence-length :initial-element (instantiate float-vector 3)))
         (zmp (make-list sequence-length :initial-element (instantiate float-vector 3)))
         (wrench (make-list sequence-length :initial-element (instantiate float-vector (* 6 fsensor-length)))) ;; wrench = 6dim
         (optional (make-list sequence-length :initial-element (instantiate float-vector (* 2 fsensor-length)))) ;; 2 is for contactState and controlSwingSupportTime
         (pos (send-all root-coords :worldpos))
         (rpy (mapcar #'(lambda (x) (reverse (car (rpy-angle (send x :worldrot))))) root-coords))
         (root-local-zmp (mapcar #'(lambda (zz cc) (send cc :inverse-transform-vector zz)) zmp root-coords))
         )
   "Call service for setJointAnglesSequenceFull. Definition of each sequence is similar to sequence file of loadPattern.
    Arguments type:
     Required
      jpos: sequence of joint angles(float-vector) [deg],  (list av0 av1 ... avn)
      tm: sequence of duration(float) [ms],  (list tm0 tm1 ... tmn)
     Key
      vel: sequence of joint angular velocities(float-vector) [deg/s],  (list vel0 vel1 ... veln)
      torque: sequence of torques(float-vector) [Nm],  (list torque0 torque1 ... torquen)
      root-coords: sequence of waist(root-link) coords in the world frame. (list rc0 rc1 ... rcn). Origin coords by default.
      acc: sequence of waist acc(float-vector) [m/s^2],  (list acc0 acc1 ... accn)
      zmp: sequence of zmp in the world frame (float-vector) [mm],  (list zmp0 zmp1 ... zmpn). Zero by default.
      wrench: sequence of wrench(float-vector) [N, Nm] for all fsensors,  (list wrench0 wrench1 ... wrenchn)
      optional: sequence of optional(float-vector) [],  (list optional0 optional1 ... optionaln)
     Not required (calculated from other arguments by default), therefore users need not to use these arguments.
      pos: sequence of waist pos(float-vector) [mm] in the world frame,  (list pos0 pos1 ... posn). If root-coords is specified, calculated from root-coords and do not set pos.
      rpy: sequence of waist rpy(float-vector) [rad] in the world frame,  (list rpy0 rpy1 ... rpyn). If root-coords is specified, calculated from root-coords and do not set rpy.
      root-local-zmp: sequence of zmp in the waist(root-link) frame (float-vector) [mm],  (list zmp0 zmp1 ... zmpn). If root-coords and zmp are specified, calculated from root-coords and zmp and do not set root-local-zmp."
   (let* ((jvss (instance std_msgs::float64multiarray :init))
          (vels (instance std_msgs::float64multiarray :init))
          (torques (instance std_msgs::float64multiarray :init))
          (poss (instance std_msgs::float64multiarray :init))
          (rpys (instance std_msgs::float64multiarray :init))
          (accs (instance std_msgs::float64multiarray :init))
          (zmps (instance std_msgs::float64multiarray :init))
          (wrenchs (instance std_msgs::float64multiarray :init))
          (optionals (instance std_msgs::float64multiarray :init))
          (tms (coerce (mapcar #'(lambda (ttm) (* 1e-3 ttm)) tm) float-vector))) ;; [ms]->[s], list -> float-vector
     ;; jvss
     (let ((jvs (apply #'matrix (mapcar #'(lambda (x) (map float-vector #'deg2rad x)) jpos)))) ;; [deg] -> [rad]
       (send jvss :layout :dim (list
                                (instance std_msgs::multiArraydimension :init :label "sequence length" :size (nth 0 (array-dimensions jvs)) :stride (* (nth 0 (array-dimensions jvs)) (nth 1 (array-dimensions jvs))))
                                (instance std_msgs::multiArraydimension :init :label "jvs length" :size (nth 1 (array-dimensions jvs)) :stride (nth 1 (array-dimensions jvs)))))
       (send jvss :data (array-entity jvs)))
     ;; vels
     (setq vel (apply #'matrix (mapcar #'(lambda (x) (map float-vector #'deg2rad x)) vel))) ;; [deg/s] -> [rad/s]
     (send vels :layout :dim (list
                              (instance std_msgs::multiArraydimension :init :label "sequence length" :size (nth 0 (array-dimensions vel)) :stride (* (nth 0 (array-dimensions vel)) (nth 1 (array-dimensions vel))))
                              (instance std_msgs::multiArraydimension :init :label "vel length" :size (nth 1 (array-dimensions vel)) :stride (nth 1 (array-dimensions vel)))))
     (send vels :data (array-entity vel))
     ;; torques
     (setq torque (apply #'matrix torque))
     (send torques :layout :dim (list
                                 (instance std_msgs::multiArraydimension :init :label "sequence length" :size (nth 0 (array-dimensions torque)) :stride (* (nth 0 (array-dimensions torque)) (nth 1 (array-dimensions torque))))
                                 (instance std_msgs::multiArraydimension :init :label "torque length" :size (nth 1 (array-dimensions torque)) :stride (nth 1 (array-dimensions torque)))))
     (send torques :data (array-entity torque))
     ;; poss
     (setq pos (apply #'matrix (mapcar #'(lambda (x) (scale 1e-3 x)) pos))) ;; [mm] -> [m]
     (send poss :layout :dim (list
                              (instance std_msgs::multiArraydimension :init :label "sequence length" :size (nth 0 (array-dimensions pos)) :stride (* (nth 0 (array-dimensions pos)) (nth 1 (array-dimensions pos))))
                              (instance std_msgs::multiArraydimension :init :label "pos length" :size (nth 1 (array-dimensions pos)) :stride (nth 1 (array-dimensions pos)))))
     (send poss :data (array-entity pos))
     ;; rpys
     (setq rpy (apply #'matrix rpy))
     (send rpys :layout :dim (list
                              (instance std_msgs::multiArraydimension :init :label "sequence length" :size (nth 0 (array-dimensions rpy)) :stride (* (nth 0 (array-dimensions rpy)) (nth 1 (array-dimensions rpy))))
                              (instance std_msgs::multiArraydimension :init :label "rpy length" :size (nth 1 (array-dimensions rpy)) :stride (nth 1 (array-dimensions rpy)))))
     (send rpys :data (array-entity rpy))
     ;; accs
     (setq acc (apply #'matrix acc))
     (send accs :layout :dim (list
                              (instance std_msgs::multiArraydimension :init :label "sequence length" :size (nth 0 (array-dimensions acc)) :stride (* (nth 0 (array-dimensions acc)) (nth 1 (array-dimensions acc))))
                              (instance std_msgs::multiArraydimension :init :label "acc length" :size (nth 1 (array-dimensions acc)) :stride (nth 1 (array-dimensions acc)))))
     (send accs :data (array-entity acc))
     ;; zmps
     (setq zmp (apply #'matrix (mapcar #'(lambda (x) (scale 1e-3 x)) root-local-zmp))) ;; [mm] -> [m]
     (send zmps :layout :dim (list
                              (instance std_msgs::multiArraydimension :init :label "sequence length" :size (nth 0 (array-dimensions zmp)) :stride (* (nth 0 (array-dimensions zmp)) (nth 1 (array-dimensions zmp))))
                              (instance std_msgs::multiArraydimension :init :label "zmp length" :size (nth 1 (array-dimensions zmp)) :stride (nth 1 (array-dimensions zmp)))))
     (send zmps :data (array-entity zmp))
     ;; wrenchs
     (setq wrench (apply #'matrix wrench))
     (send wrenchs :layout :dim (list
                                 (instance std_msgs::multiArraydimension :init :label "sequence length" :size (nth 0 (array-dimensions wrench)) :stride (* (nth 0 (array-dimensions wrench)) (nth 1 (array-dimensions wrench))))
                                 (instance std_msgs::multiArraydimension :init :label "wrench length" :size (nth 1 (array-dimensions wrench)) :stride (nth 1 (array-dimensions wrench)))))
     (send wrenchs :data (array-entity wrench))
     ;; optionals
     (setq optional (apply #'matrix optional))
     (send optionals :layout :dim (list
                                   (instance std_msgs::multiArraydimension :init :label "sequence length" :size (nth 0 (array-dimensions optional)) :stride (* (nth 0 (array-dimensions optional)) (nth 1 (array-dimensions optional))))
                                   (instance std_msgs::multiArraydimension :init :label "optional length" :size (nth 1 (array-dimensions optional)) :stride (nth 1 (array-dimensions optional)))))
     (send optionals :data (array-entity optional))

     (send self :sequenceplayerservice_setjointanglessequencefull :jvss jvss :vels vels :torques torques :poss poss :rpys rpys :accs accs :zmps zmps :wrenchs wrenchs :optionals optionals :tms tms)
     )
   )
  )

;; CollisionDetectorService
(defmethod rtm-ros-robot-interface
  (:set-tolerance
   (&key (tolerance 0.1) (link-pair-name "all"))
   "Set tolerance [m] of collision detection with given link-pair-name (all by default)."
   (send self :collisiondetectorservice_settolerance
         :link_pair_name link-pair-name :tolerance tolerance))
  (:start-collision-detection
   ()
   "Enable collision detection."
   (send self :collisiondetectorservice_enablecollisiondetection))
  (:stop-collision-detection
   ()
   "Disable collision detection."
   (send self :collisiondetectorservice_disablecollisiondetection))
  (:get-collision-status
   ()
   "Get collision status."
   (send (send self :collisiondetectorservice_getcollisionstatus) :cs)
   )
  )

;; DataLoggerService
(defmethod rtm-ros-robot-interface
  (:save-log
   (fname &key (set-robot-date-string t) (make-directory nil))
   "Save log files as [fname].[component_name]_[dataport_name].
    This method corresponds to DataLogger save().
    If set-robot-date-string is t, filename includes date string and robot name. By default, set-robot-date-string is t.
    If make-directory is t and fname is /foo/bar/basename, make directory of /foo/bar/basename and log files will be saved with /foo/bar/basename/basename"
   (let ((basename
          (format nil "~A~A" fname (if set-robot-date-string (format nil "_~A" (send self :get-robot-date-string)) ""))))
     (when make-directory
       (let* ((pname (pathname basename))
              dirname
              (fname
               (if (send pname :type)
                   (concatenate string (send pname :name) "." (send pname :type))
                   (send pname :name))))
         (send pname :add-directory (list fname))
         (setq dirname (send pname :directory-string))
         (when (and dirname
                    (not (lisp::probe-file dirname)))
           (unix::mkdir dirname))
         (setq basename (send pname :namestring))
         ))
     (warn ";; save log to ~A~%" basename)
     (send self :dataloggerservice_save :basename basename)
     ))
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
   "Set servo gain percentage [0-100] with given name."
   (send self :robothardwareservice_setservogainpercentage :name name :percentage percentage))
  (:remove-force-sensor-offset
   ()
   "Remove force sensor offset.
    This function takes 10[s]. Please keep the robot static and make sure that robot's sensors do not contact with any objects."
   (warning-message 1 ";; !!!!!!!!~%")
   (warning-message 1 ";; !! Warning, :remove-force-sensor-offset by RobotHardware is deprecated. Please use :remove-force-sensor-offset-rmfo~%")
   (warning-message 1 ";; !!!!!!!!~%")
   (send self :robothardwareservice_removeforcesensoroffset))
  (:set-servo-error-limit
    (name limit)
    "Set RobotHardware servo error limit [rad] with given name."
    (send self :robothardwareservice_setservoerrorlimit :name name :limit limit)
    )
  (:calibrate-inertia-sensor
   ()
   "Calibrate inetria sensor.
    This function takes 10[s]. Please keep the robot static."
   (send self :robothardwareservice_calibrateInertiaSensor)
   )
  )

(def-set-get-param-method 'hrpsys_ros_bridge::OpenHRP_ImpedanceControllerService_impedanceParam
  :raw-set-impedance-controller-param :raw-get-impedance-controller-param :get-impedance-controller-param-arguments
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
    limb should be limb symbol name such as :rarm, :larm, :rleg, :lleg, :arms, or :legs.
    For arguments, please see (send *ri* :get-impedance-controller-param-arguments)."
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

;; ObjectContactTurnaroundDetector
(def-set-get-param-method 'hrpsys_ros_bridge::OpenHRP_ObjectContactTurnaroundDetectorService_objectContactTurnaroundDetectorParam
  :raw-set-object-turnaround-detector-param :get-object-turnaround-detector-param :get-object-turnaround-detector-param-arguments
  :ObjectContactTurnaroundDetectorService_setObjectContactTurnaroundDetectorParam
  :ObjectContactTurnaroundDetectorService_getObjectContactTurnaroundDetectorParam)
(defmethod rtm-ros-robot-interface
  (:start-object-turnaround-detection
   (&key (ref-diff-wrench) (max-time) (limbs))
   "Start ObjectContactTurnaroundDetection mode.
    ref-diff-wrench is final reference wrench (scalar).
    max-time is max time [msec].
    limbs is limb list to be used."
   (send self :ObjectContactTurnaroundDetectorService_startObjectContactTurnaroundDetection
         :i_ref_diff_wrench ref-diff-wrench :i_max_time max-time :i_ee_names (mapcar #'string-downcase limbs)))
  (:check-object-turnaround-detection
   ()
   "Check object contact turnaround detection.
   If t, detected. Otherwise, not detected."
   (send self :get-idl-enum-values
         (send (send self :ObjectContactTurnaroundDetectorService_checkObjectContactTurnaroundDetection) :operation_return)
         "HRPSYS_ROS_BRIDGE::OPENHRP_OBJECTCONTACTTURNAROUNDDETECTORSERVICE_DETECTORMODE"))
  (:get-otd-object-forces-moments
   ()
   "Get ObjectContactTurnaroundDetector's curernt forces and moments for used limbs.
    Return value is (list force-list moment-list)."
   (labels ((split-vector
             (vec &optional (dim 3))
             (let ((vret))
               (dotimes (i (/ (length vec) dim))
                 (push (subseq vec (* i 3) (+ (* i 3) 3)) vret))
               (reverse vret))))
     (let ((ret (send self :ObjectContactTurnaroundDetectorService_getObjectForcesMoments)))
       (list (split-vector (send (send ret :o_forces) :data))
             (split-vector (send (send ret :o_moments) :data))
             (send ret :o_3dofwrench)
             (send ret :o_fric_coeff_wrench))
       ))
   )
  (:set-object-turnaround-ref-force
   (&key (limbs '(:rarm :larm))
         (axis (float-vector 0 0 -1))
         (max-time 4000.0)
         (max-ref-force)
         (detect-time-offset 2000.0)
         (set-ref-force-time 2000.0)
         (set-ref-force-p t)
         (periodic-time 200) ;; [msec]
         (detector-total-wrench :total-force)
         (return-value-mode :forces)
         (set-ref-force-linear-p)
         )
   "Set object turnaround reference force.
    axis is resultant force direction.
    max-time is max time [msec] and max-ref-force is max reference resultant force.
    max-ref-force is max resultant reference force [N]. max-ref-force is added to original ref forces.
    detect-time-offset is additional checking time [msec].
    set-ref-force-time is time to set reference force [msec].
    If set-ref-force-p is t, set estimated reference force. Otherwise, set reference force to zero.
    periodic-time is loop wait time[msec].
    return-value-mode is used to select return value type.
    If :all, returns the results of :get-otd-object-forces-moments itself. If :forces, returns forces results.
    If set-ref-force-linear-p t, set linear interpolation first and set hoffarbib interpolation in return."
   ;; Step.0 Initialize
   (format t ";; Start object turnaround detection~%")
   (send self :set-object-turnaround-detector-param :axis axis :detector-total-wrench detector-total-wrench)
   (send self :state)
   (let* ((all-limbs (sort '(:rleg :lleg :rarm :larm) #'< #'(lambda (x) (position (car (send robot x :force-sensors)) (send robot :force-sensors)))))
          (org-ref-forces (mapcar #'(lambda (limb) (send self :reference-force-vector limb)) all-limbs))
          (diff-force (scale (/ max-ref-force (length limbs)) axis)))
     ;; Step.1 Set ref force and start detection
     (if set-ref-force-linear-p (send self :set-interpolation-mode :linear))
     (send self :set-ref-forces
           (mapcar #'(lambda (org-f limb)
                       (if (memq limb limbs)
                           (v+ org-f diff-force)
                         org-f))
                   org-ref-forces all-limbs)
           max-time
           :update-robot-state nil)
     (send self :start-object-turnaround-detection
           :ref-diff-wrench max-ref-force
           :max-time (* 1e-3 (+ detect-time-offset max-time))
           :limbs limbs)
     (let ((detector-mode :mode_detector_idle) (fm-ret))
       ;; Step.2 Detection loop
       (do-until-key-with-check
        (not (not (memq detector-mode '(:mode_started :mode_detector_idle))))
        (unix:usleep (* 1000 periodic-time))
        (setq detector-mode (send self :check-object-turnaround-detection))
        (setq fm-ret (send self :get-otd-object-forces-moments))
        (format t ";;   Detecting... (detector-mode = ~A, limbs = ~A, forces = ~A[N], moments = ~A[Nm], res force = ~A[N], fric force = ~A[N])~%"
                detector-mode limbs
                (car fm-ret) (cadr fm-ret) (caddr fm-ret) (cadddr fm-ret))
        )
       ;; Step.3 If detected, update ref force
       (if (eq detector-mode :mode_detected)
           (progn
             (format t ";; Detected (limbs = ~A, forces = ~A[N], moments = ~A[Nm], res force = ~A[N], fric force = ~A[N])~%"
                     limbs
                     (car fm-ret) (cadr fm-ret) (caddr fm-ret) (cadddr fm-ret))
             (if set-ref-force-p
                 (send self :set-ref-forces
                       (mapcar #'(lambda (org-f limb)
                                   ;; Revert org force orthogonal to axis direction
                                   (let ((org-f2 (v- org-f (scale (v. axis org-f) axis))))
                                     (if (memq limb limbs)
                                         (v+ org-f2 (elt (car fm-ret) (position limb limbs)))
                                       org-f)))
                               org-ref-forces all-limbs)
                       set-ref-force-time)
               (send self :set-ref-forces
                     org-ref-forces set-ref-force-time)))
         (progn
           (format t ";; Not detected (~A)~%" detector-mode)
           (send self :set-ref-forces
                 org-ref-forces set-ref-force-time)
           )
         )
       (send self :wait-interpolation-seq)
       (if set-ref-force-linear-p (send self :set-interpolation-mode :hoffarbib))
       (if (eq detector-mode :mode_detected)
           (case return-value-mode
                 (:all fm-ret)
                 (:forces (car fm-ret))
                 (t nil)))
       )))
  (:set-object-turnaround-ref-moment
   (&key (limbs '(:rarm :larm))
         (axis (float-vector 0 0 1))
         (max-ref-moment 10)
         (func)
         (moment-center)
         (max-time 4000.0)
         (detect-time-offset 2000.0)
         (set-ref-force-time 2000.0)
         (set-ref-force-p t)
         (periodic-time 200) ;; [msec]
         (detector-total-wrench :total-moment)
         (return-value-mode :all)
         (set-ref-force-linear-p)
         )
   "Set object turnaround reference force.
    axis is axis which resultant moment is around.
    max-time is max time [msec] and max-ref-moment is max reference resultant moment.
    func is function to distribute estimated resultant moment to hands' forces/moments.
    moment-center is moment center [mm].
    detect-time-offset is additional checking time [msec].
    set-ref-force-time is time to set reference force [msec].
    If set-ref-force-p is t, set estimated reference force. Otherwise, set reference force to zero.
    periodic-time is loop wait time[msec].
    return-value-mode is used to select return value type.
    If :all, returns the results of :get-otd-object-forces-moments itself. If :forces, returns forces results.
    If set-ref-force-linear-p t, set linear interpolation first and set hoffarbib interpolation in return."
   (format t ";; Start object turnaround detection~%")
   (let ((all-limbs (sort '(:rleg :lleg :rarm :larm) #'< #'(lambda (x) (position (car (send robot x :force-sensors)) (send robot :force-sensors)))))
         (fs-list (funcall func max-ref-moment)))
     (send self :set-object-turnaround-detector-param
           :axis axis :moment-center (scale 1e-3 moment-center) :detector-total-wrench detector-total-wrench)
     (if set-ref-force-linear-p (send self :set-interpolation-mode :linear))
     (send self :set-ref-forces
           (mapcar #'(lambda (limb)
                       (if (memq limb '(:rleg :lleg))
                           (float-vector 0 0 0)
                         (elt fs-list (position limb limbs))))
                   all-limbs)
           max-time)
     (send self :start-object-turnaround-detection
           :ref-diff-wrench max-ref-moment
           :max-time (* 1e-3 (+ detect-time-offset max-time))
           :limbs limbs)
     (let ((detector-mode :mode_detector_idle) (fm-ret) (forces-from-total-moment))
       (do-until-key-with-check
        (not (not (memq detector-mode '(:mode_started :mode_detector_idle))))
        (unix:usleep (* 1000 periodic-time))
        (setq detector-mode (send self :check-object-turnaround-detection))
        (Setq fm-ret (send self :get-otd-object-forces-moments))
        (format t ";;   Detecting... (detector-mode = ~A, limbs = ~A, forces = ~A[N], moments = ~A[Nm], res moment = ~A[Nm], fric force = ~A[N])~%"
                detector-mode limbs
                (car fm-ret) (cadr fm-ret) (caddr fm-ret) (cadddr fm-ret))
        )
       (if (eq detector-mode :mode_detected)
           (let ((total-moment (caddr fm-ret)))
             (setq forces-from-total-moment (funcall func (v. axis total-moment)))
             (format t ";; Detected (limbs = ~A, forces from total moment = ~A[N], forces = ~A[N], moments = ~A[Nm], res moment = ~A[Nm], fric force = ~A[N])~%"
                     limbs forces-from-total-moment
                     (car fm-ret) (cadr fm-ret) (caddr fm-ret) (cadddr fm-ret))
             (if set-ref-force-p
                 (send self :set-ref-forces
                       (mapcar #'(lambda (limb)
                                   (if (memq limb '(:rleg :lleg))
                                       (float-vector 0 0 0)
                                     (elt forces-from-total-moment (position limb limbs))))
                               all-limbs)
                       set-ref-force-time)
               (send self :set-ref-force
                     (float-vector 0 0 0)
                     set-ref-force-time
                     (if (= (length limbs) 1) (car limbs) :arms))))
         (progn
           (format t ";; Not detected (~A)~%" detector-mode)
           (send self :set-ref-force
                 (float-vector 0 0 0)
                 set-ref-force-time
                 (if (= (length limbs) 1) (car limbs) :arms))
           )
         )
       (send self :wait-interpolation-seq)
       (if set-ref-force-linear-p (send self :set-interpolation-mode :hoffarbib))
       (if (eq detector-mode :mode_detected)
           (case return-value-mode
                 (:all fm-ret)
                 (:forces (car fm-ret))
                 (:forces-from-total-moment forces-from-total-moment)
                 (t nil)))
       )))
  (:set-object-turnaround-detector-param
   (&rest args
    &key detector-total-wrench
    &allow-other-keys)
   "Set Object Turnaround Detector.
    For arguments, please see (send *ri* :get-object-turnaround-detector-param-arguments)."
   (let ((prm (send self :get-object-turnaround-detector-param)))
     (send* self :raw-set-object-turnaround-detector-param
            (append
             (if (memq :detector-total-wrench args)
                 (list :detector-total-wrench
                       (if (or (null detector-total-wrench) (integerp detector-total-wrench))
                           detector-total-wrench
                         (let ((dtw (read-from-string (format nil "HRPSYS_ROS_BRIDGE::OPENHRP_OBJECTCONTACTTURNAROUNDDETECTORSERVICE_DETECTORTOTALWRENCH::*~A*" (substitute (elt "_" 0) (elt "-" 0) (string-downcase detector-total-wrench))))))
                           (if (boundp dtw)
                               (eval dtw)
                             (error ";; no such :detector-total-wrench ~A in :set-object-turnaround-detector-param~%" detector-total-wrench))))))
             args))))
  (:get-object-turnaround-detector-detector-total-wrench
   ()
   "Get Object Turnadound Detector as Euslisp symbol."
   (send self :get-idl-enum-values
         (send (send self :get-object-turnaround-detector-param) :detector_total_wrench)
         "HRPSYS_ROS_BRIDGE::OPENHRP_OBJECTCONTACTTURNAROUNDDETECTORSERVICE_DETECTORTOTALWRENCH"))
  )

;; RemoveForceSensorLinkOffset
(def-set-get-param-method 'hrpsys_ros_bridge::OpenHRP_RemoveForceSensorLinkOffsetService_ForceMomentOffsetParam
  :raw-set-forcemoment-offset-param :raw-get-forcemoment-offset-param :get-forcemoment-offset-param-arguments
  :removeforcesensorlinkoffsetservice_setforcemomentoffsetparam :removeforcesensorlinkoffsetservice_getforcemomentoffsetparam
  :optional-args (list :name 'name))

(defmethod rtm-ros-robot-interface
  (:zero-set-forcemoment-offset-param
   (limb)
   "Set RemoveForceSensorLinkOffset's params offset to zero."
   (send self :set-forcemoment-offset-param limb :force-offset #f(0 0 0) :moment-offset #f(0 0 0) :link-offset-centroid #f(0 0 0) :link-offset-mass 0)
   )
  (:set-forcemoment-offset-param
   (limb &rest args)
   "Set RemoveForceSensorLinkOffset params for given limb.
    For arguments, please see (send *ri* :get-forcemoment-offset-param-arguments)."
   (send* self :force-sensor-method
         limb
         #'(lambda (name &rest _args)
             (send* self :raw-set-forcemoment-offset-param (send (car (send robot name :force-sensors)) :name) _args))
         :set-forcemoment-offset-param
         args))
  (:get-forcemoment-offset-param
   (limb)
   "Get RemoveForceSensorLinkOffset params for given limb."
   (send self :force-sensor-method
         limb
         #'(lambda (name &rest _args)
             (send self :raw-get-forcemoment-offset-param (send (car (send robot name :force-sensors)) :name)))
         :get-forcemoment-offset-param))
  (:load-forcemoment-offset-param
   (fname &key (set-offset t))
   "Load RemoveForceSensorLinkOffset params from fname (file path)."
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
  (:remove-force-sensor-offset-rmfo
   (&key (limbs) ((:time tm) 8.0))
   "remove offsets on sensor outputs form force/torque sensors.
    Sensor offsets (force_offset and moment_offset in ForceMomentOffsetParam) are calibrated.
    Please keep the robot static and make sure that robot's sensors do not contact with any objects.
    Argument:
      limbs is list of sensor names to be calibrated.
      If not specified, all sensors are calibrated by default.
      time is duration of calibration[s]. 8.0[s] by default.
    Return:
      t if set successfully, nil otherwise"
   (send self :removeforcesensorlinkoffsetservice_removeforcesensoroffset
         :names
         (mapcar #'(lambda (limb) (send (car (send robot limb :force-sensors)) :name)) limbs)
         :tm tm))
  (:remove-force-sensor-offset-rmfo-arms
   (&key ((:time tm) 8.0))
   "Remove force and moment offset for :rarm and :larm.
    time is duration of calibration[s]. 8.0[s] by default."
   (send self :remove-force-sensor-offset-rmfo :limbs '(:rarm :larm) :time tm))
  (:remove-force-sensor-offset-rmfo-legs
   (&key ((:time tm) 8.0))
   "Remove force and moment offset for :rleg and :lleg.
    time is duration of calibration[s]. 8.0[s] by default."
   (send self :remove-force-sensor-offset-rmfo :limbs '(:rleg :lleg) :time tm))
  ;; Deprecated
  (:reset-force-moment-offset-arms
   (&key ((:time tm) 0.1))
   "time[s]"
   (warning-message 1 ";; !!!!!!!!~%")
   (warning-message 1 ";; !! Warning, :reset-force-moment-offset-arms is deprecated. Please use (send *ri* :remove-force-sensor-offset-rmfo-arms :time 0.1)~%")
   (warning-message 1 ";; !!!!!!!!~%")
   (send self :remove-force-sensor-offset-rmfo-arms :time tm))
  (:reset-force-moment-offset-legs
   (&key ((:time tm) 0.1))
   "time[s]"
   (warning-message 1 ";; !!!!!!!!~%")
   (warning-message 1 ";; !! Warning, :reset-force-moment-offset-legs is deprecated. Please use (send *ri* :remove-force-sensor-offset-rmfo-legs :time 0.1)~%")
   (warning-message 1 ";; !!!!!!!!~%")
   (send self :remove-force-sensor-offset-rmfo-legs :time tm))
  (:reset-force-moment-offset
   (limbs)
   "Remove force and moment offsets. limbs should be list of limb symbol name."
   (send self :_reset-force-moment-offset limbs :force)
   (send self :_reset-force-moment-offset limbs :moment)
   )
  (:_reset-force-moment-offset
   (limbs f/m &key (itr 10))
   (let* ((params (mapcar #'(lambda (alimb) (send self :get-forcemoment-offset-param alimb)) limbs)))
     (labels ((calc-off
               (alimb)
               (send self (if (eq f/m :force) :off-force-vector :off-moment-vector) alimb))
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
                                          (mapcar #'calc-off limbs))
                                      (make-list itr)))
              (new-fm-offsets (mapcar #'(lambda (alimb)
                                          (let ((idx (position alimb limbs)))
                                            (vector-mean (mapcar #'(lambda (d) (elt d idx)) tmp-fm-offsets))))
                                      limbs))
              (org-fm-list (get-avg-fm)))
         ;; set offsets
         (mapcar #'(lambda (alimb new-fm-offset param)
                     (send self :set-forcemoment-offset-param alimb
                           (if (eq f/m :force) :force-offset :moment-offset)
                           (v+ (if (eq f/m :force)
                                   (send param :force_offset)
                                 (send param :moment_offset))
                               new-fm-offset)))
                 limbs new-fm-offsets params)
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
  :raw-set-gait-generator-param :get-gait-generator-param :get-gait-generator-param-arguments
  :autobalancerservice_setgaitgeneratorparam :autobalancerservice_getgaitgeneratorparam)
(def-set-get-param-method
  'hrpsys_ros_bridge::Openhrp_AutoBalancerService_AutoBalancerParam
  :raw-set-auto-balancer-param :get-auto-balancer-param :get-auto-balancer-param-arguments
  :autobalancerservice_setAutoBalancerparam :autobalancerservice_getAutoBalancerparam)

(defmethod rtm-ros-robot-interface
  (:start-auto-balancer
   (&key (limbs (if (not (every #'null (send robot :arms)))
                    '(:rleg :lleg :rarm :larm)
                  '(:rleg :lleg))))
   "startAutoBalancer.
    If robot with arms, start auto balancer with legs and arms ik by default.
    Otherwise, start auto balancer with legs ik by default."
   (send self :autobalancerservice_startAutoBalancer
         :limbs (mapcar #'(lambda (x) (format nil "~A" (string-downcase x))) limbs)))
  (:stop-auto-balancer
   ()
   "Stop auto balancer mode"
   (send self :autobalancerservice_stopAutoBalancer))
  (:go-pos-no-wait
   (xx yy th)
   "Call goPos without wait."
   (send self :autobalancerservice_goPos :x xx :y yy :th th))
  (:go-pos
   (xx yy th)
   "Call goPos with wait."
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
              '(:support_leg_coords :swing_leg_coords :swing_leg_src_coords :swing_leg_dst_coords :dst_foot_midcoords)
              '(:support-leg-coords :swing-leg-coords :swing-leg-src-coords :swing-leg-dst-coords :dst-foot-midcoords))
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
   (if (memq param-name '(:support-leg-coords :swing-leg-coords :swing-leg-src-coords :swing-leg-dst-coords :dst-foot-midcoords :support-leg :support-leg-with-both))
       (cadr (memq param-name (send self :get-foot-step-params)))
     (error ";; no such abc footstep param ~A~%" param-name))
   )
  (:set-foot-steps-no-wait
   (foot-step-list &key (overwrite-footstep-index 0))
   "Set foot step by default parameters and do not wait for step finish.
    foot-step-list is list of footstep (only biped) or list of list of footstep.
    overwrite-footstep-index is index to be overwritten. overwrite_fs_idx is used only in walking."
   (unless (listp (car foot-step-list))
     (setq foot-step-list (mapcar #'(lambda (fs) (list fs)) foot-step-list)))
   (send self :autobalancerservice_setfootsteps
         :fss
         (mapcar #'(lambda (fs)
                     (instance hrpsys_ros_bridge::openhrp_autobalancerservice_footsteps :init
                               :fs
                               (mapcar #'(lambda (f)
                                           (send self :eus-footstep->abc-footstep f))
                                       fs)))
                 foot-step-list)
         :overwrite_fs_idx overwrite-footstep-index))
  (:set-foot-steps
   (foot-step-list &key (overwrite-footstep-index 0))
   "Set foot step by default parameters and wait for step finish.
    foot-step-list is list of footstep (only biped) or list of list of footstep.
    overwrite-footstep-index is index to be overwritten. overwrite_fs_idx is used only in walking."
   (send self :set-foot-steps-no-wait foot-step-list :overwrite-footstep-index overwrite-footstep-index)
   (send self :wait-foot-steps))
  (:set-foot-steps-with-param-no-wait
   (foot-step-list step-height-list step-time-list toe-angle-list heel-angle-list &key (overwrite-footstep-index 0))
   "Set foot step with step parameter and do not wait for step finish.
    foot-step-list is list of footstep (only biped) or list of list of footstep.
    step-height-list is list of step height (only biped) or list of list of step height.
    step-time-list is list of step time (only biped) or list of list of step time.
    toe-angle-list is list of toe angle (only biped) or list of list of toe angle.
    heel-angle-list is list of heel angle (only biped) or list of list of heel angle."
   (unless (listp (car foot-step-list))
     (setq foot-step-list (mapcar #'(lambda (fs) (list fs)) foot-step-list)
           step-height-list (mapcar #'(lambda (sh) (list sh)) step-height-list)
           step-time-list (mapcar #'(lambda (st) (list st)) step-time-list)
           toe-angle-list (mapcar #'(lambda (ta) (list ta)) toe-angle-list)
           heel-angle-list (mapcar #'(lambda (ha) (list ha)) heel-angle-list)))
   (send self :autobalancerservice_setfootstepswithparam
         :fss
         (mapcar #'(lambda (fs)
                     (instance hrpsys_ros_bridge::openhrp_autobalancerservice_footsteps :init
                               :fs
                               (mapcar #'(lambda (f)
                                           (send self :eus-footstep->abc-footstep f))
                                       fs)))
                 foot-step-list)
         :spss
         (mapcar #'(lambda (shs sts tas has)
                     (instance hrpsys_ros_bridge::openhrp_autobalancerservice_stepparams :init
                               :sps
                               (mapcar #'(lambda (sh st ta ha)
                                           (instance hrpsys_ros_bridge::openhrp_autobalancerservice_stepparam :init :step_height (* sh 1e-3) :step_time st :toe_angle ta :heel_angle ha))
                                       shs sts tas has)))
                 step-height-list step-time-list toe-angle-list heel-angle-list)
         :overwrite_fs_idx overwrite-footstep-index
         ))
  (:set-foot-steps-with-param
   (foot-step-list step-height-list step-time-list toe-angle-list heel-angle-list &key (overwrite-footstep-index 0))
   "Set foot step with step parameter and wait for step finish.
    For arguments, please see :set-foot-steps-with-param-no-wait documentation."
   (send self :set-foot-steps-with-param-no-wait foot-step-list step-height-list step-time-list toe-angle-list heel-angle-list :overwrite-footstep-index overwrite-footstep-index)
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
   "Set foot steps with sending angle-vector."
   (send self :set-foot-steps-no-wait fs)
   (send self :angle-vector-sequence av-list time-list)
   (send self :wait-interpolation)
   (send self :wait-foot-steps)
   )
  (:set-foot-steps-with-param-and-base-height
   (fs-params av-list time-list)
   "Set foot steps and params with sending angle-vector."
   (send self :set-foot-steps-with-param-no-wait fs-params)
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
  (:get-remaining-foot-step-sequence-current-index
   ()
   "Get remaining foot steps from GaitGenerator and current index.
   Return is (list (list current-support-foot-coords remaining-swing-dst-coords-0 ... ) current-index)."
   (let ((ret (send self :autobalancerservice_getRemainingFootstepSequence)))
     (list (mapcar #'(lambda (fs)
                       (send self :abc-footstep->eus-footstep fs))
                   (send ret :o_footstep))
           (send ret :o_current_fs_idx))
     ))
  (:get-current-footstep-index
   ()
   "Get current footstep index."
   (cadr (send self :get-remaining-foot-step-sequence-current-index)))
  (:get-remaining-foot-step-sequence
   ()
   "Get remaining foot steps from GaitGenerator.
   Return is (list current-support-foot-coords remaining-swing-dst-coords-0 ... )."
   (car (send self :get-remaining-foot-step-sequence-current-index)))
  (:get-go-pos-footsteps-sequence
   (xx yy th)
   "Get foot steps of go-pos without executing them.
   Return is list of list of footstep."
   (let ((ret (send self :autobalancerservice_getGoPosFootstepsSequence :x xx :y yy :th th)))
     (mapcar #'(lambda (fsl)
                 (mapcar #'(lambda (fs)
                             (send self :abc-footstep->eus-footstep fs))
                         (send fsl :fs)))
             (send ret :o_footstep))
     ))
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
   "Call goVelocity."
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
   (&rest args
    &key default-orbit-type
         leg-default-translate-pos ;; [mm]
         stride-parameter
    &allow-other-keys)
   "Set gait generator param.
    For arguments, please see (send *ri* :get-gait-generator-param-arguments)."
   (let ((gg-prm (if (or (memq :leg-default-translate-pos args) (memq :stride-parameter args)) (send self :get-gait-generator-param))))
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
             (if (memq :leg-default-translate-pos args)
                 (let ((lo (copy-object (send gg-prm :leg_default_translate_pos))))
                   (setq (lo . ros::_data) (scale 1e-3 (apply #'concatenate float-vector leg-default-translate-pos)))
                   (list :leg-default-translate-pos lo)))
             (if (memq :stride-parameter args)
                 (let ((sprm (send gg-prm :stride_parameter)))
                   (cond
                    ((= (length sprm) (length stride-parameter)) ;; If same length
                     (list :stride-parameter stride-parameter))
                    ((< (length sprm) (length stride-parameter)) ;; If rtcd's length is shorter than argument's length
                     (list :stride-parameter (subseq stride-parameter 0 (length sprm))))
                    (t ;; If rtcd's length is longer than argument's length
                     (list :stride-parameter (concatenate float-vector stride-parameter (subseq sprm (length stride-parameter))))))
                   ))
             args))))
  (:print-gait-generator-orbit-type
   ()
   "Print GaitGenerator orbit types."
   (let ((cs (constants "" "HRPSYS_ROS_BRIDGE::OPENHRP_AUTOBALANCERSERVICE_ORBITTYPE")))
     (mapcar #'(lambda (x) (format t ";; ~A => ~A~%" x (eval x))) cs)
     t))
  (:get-gait-generator-orbit-type
   ()
   "Get GaitGenerator Orbit Type as Euslisp symbol."
   (send self :get-idl-enum-values
         (send (send self :get-gait-generator-param) :default_orbit_type)
         "HRPSYS_ROS_BRIDGE::OPENHRP_AUTOBALANCERSERVICE_ORBITTYPE"))
  (:calc-toe-heel-phase-ratio
   (toe-angle heel-angle double-support-ratio-half)
   "Calculate vector for toe heel phase ratio.
    Arguments:
      toe-angle, heel-angle : max toe/heel angle [deg].
      double-support-ratio-half : half of double support ratio in terms of toe heel phase ratio."
   (let* ((remain-ratio (- 1 (* 2 double-support-ratio-half)))
          (total-angle-diff (+ (* toe-angle 2.0) (* heel-angle 2.0)))
          (ret
           (list double-support-ratio-half
                 (* remain-ratio (/ toe-angle total-angle-diff))
                 (* remain-ratio (/ toe-angle total-angle-diff))
                 0
                 (* remain-ratio (/ heel-angle total-angle-diff))
                 (* remain-ratio (/ heel-angle total-angle-diff))
                 double-support-ratio-half)))
     (concatenate float-vector ret)
     ))
  (:set-gait-generator-toe-heel-angles
   (toe-angle heel-angle double-support-ratio-half)
   "Set toe-angle, heel-angle, and toe-heel-phase-ratio.
    toe-heel-phase-ratio is automatically calculated.
    Arguments:
      toe-angle, heel-angle : max toe/heel angle [deg].
      double-support-ratio-half : half of double support ratio in terms of toe heel phase ratio."
   (send self :set-gait-generator-param
         :toe-angle toe-angle :heel-angle heel-angle
         :toe-heel-phase-ratio (send self :calc-toe-heel-phase-ratio toe-angle heel-angle double-support-ratio-half)))
  (:get-auto-balancer-controller-mode
   ()
   "Get AutoBalancer ControllerMode as Euslisp symbol."
   (send self :get-idl-enum-values
         (send (send self :get-auto-balancer-param) :controller_mode)
         "HRPSYS_ROS_BRIDGE::OPENHRP_AUTOBALANCERSERVICE_CONTROLLERMODE"))
  ;; :get-auto-balancer-param and :set-auto-balancer-param is not defined by def-set-get-param-method yet.
  (:get-auto-balancer-param
   ()
   "Get AutoBalancer param."
   (send (send self :autobalancerservice_getautobalancerparam) :i_param))
  (:set-auto-balancer-param
   (&rest args
    &key leg-names
         default-zmp-offsets ;; [mm]
         graspless-manip-arm
         graspless-manip-reference-trans-pos ;; [mm]
         graspless-manip-reference-trans-rot ;; rotation-matrix
         use-force-mode
    &allow-other-keys)
   "Set AutoBalancer param.
    For arguments, please see (send *ri* :get-auto-balancer-param-arguments)."
   (send* self :raw-set-auto-balancer-param
          (append
           (if (and (memq :leg-names args) leg-names)
               (list :leg-names (mapcar #'(lambda (x) (format nil "~A" (string-downcase x))) leg-names)))
           (if (and (memq :default-zmp-offsets args) default-zmp-offsets)
               (let ((dzo (copy-object (send (send self :get-auto-balancer-param) :default_zmp_offsets)))
                     (lnum (length (remove-if-not #'(lambda (x) (send robot x)) '(:rarm :larm :rleg :lleg)))))
                 (setq (dzo . ros::_data)
                       (apply #'concatenate float-vector (mapcar #'(lambda (x) (scale 1e-3 x))
                                                                 (append default-zmp-offsets
                                                                         (make-list (- lnum (length default-zmp-offsets))
                                                                                    :initial-element (float-vector 0 0 0))))))
                 (list :default-zmp-offsets dzo)))
           (if (and (memq :graspless-manip-arm args) graspless-manip-arm)
               (list :graspless-manip-arm (string-downcase graspless-manip-arm)))
           (if (and (memq :graspless-manip-reference-trans-pos args) graspless-manip-reference-trans-pos)
               (list :graspless-manip-reference-trans-pos (scale 1e-3 graspless-manip-reference-trans-pos)))
           (if (and (memq :graspless-manip-reference-trans-rot args) graspless-manip-reference-trans-rot)
               (list :graspless-manip-reference-trans-rot (matrix2quaternion graspless-manip-reference-trans-rot)))
           (if (memq :use-force-mode args)
               (list :use-force-mode
                     (if (or (null use-force-mode) (integerp use-force-mode))
                         use-force-mode
                       (let ((ot (read-from-string (format nil "HRPSYS_ROS_BRIDGE::OPENHRP_AUTOBALANCERSERVICE_USEFORCEMODE::*~A*" (substitute (elt "_" 0) (elt "-" 0) (string-downcase use-force-mode))))))
                         (if (boundp ot)
                             (eval ot)
                           (error ";; no such :use-force-mode ~A in :set-auto-balancer-param~%" use-force-mode))
                         ))))
           args)))
  (:print-auto-balancer-use-force-mode
   ()
   "Print AutoBalancer UseForceMode."
   (let ((cs (constants "" "HRPSYS_ROS_BRIDGE::OPENHRP_AUTOBALANCERSERVICE_USEFORCEMODE")))
     (mapcar #'(lambda (x) (format t ";; ~A => ~A~%" x (eval x))) cs)
     t))
  (:get-auto-balancer-use-force-mode
   ()
   "Get AutoBalancer UseForceMode as Euslisp symbol."
   (send self :get-idl-enum-values
         (send (send self :get-auto-balancer-param) :use_force_mode)
         "HRPSYS_ROS_BRIDGE::OPENHRP_AUTOBALANCERSERVICE_USEFORCEMODE"))
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
   "Walk with subscribing /cmd_vel topic."
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
  (:calc-dvel-with-velocity-center-offset
   (ang velocity-center-offset)
   "Calculate velocity params for rotating with given velocity center offset.
    Ang : [deg], offset vector [mm]"
   (let* ((default-step-time (send (send self :get-gait-generator-param) :default_step_time))
          (cen (make-cascoords :pos velocity-center-offset))
          (cc (make-cascoords)))
     (send cen :assoc cc)
     (send cen :rotate (deg2rad ang) :z)
     (let ((tf (send (make-coords) :transformation cc)))
       (list
        (/ (* 1e-3 (elt (send tf :worldpos) 0)) default-step-time)
        (/ (* 1e-3 (elt (send tf :worldpos) 1)) default-step-time)
        (/ ang default-step-time)
        ))))
  (:set-default-step-time-with-the-same-swing-time
   (default-step-time)
   "Set default step time with the same swing time."
   (let* ((ggp (send self :get-gait-generator-param))
          (prev-default-step-time (send ggp :default_step_time))
          (prev-default-double-support-ratio (send ggp :default_double_support_ratio))
          (swing-time (* (- 1.0 prev-default-double-support-ratio) prev-default-step-time))
          (new-default-double-support-ratio (- 1.0 (/ swing-time default-step-time))))
     (if (>= swing-time default-step-time)
         (error ";; default step time(~A[s]) should be greater than swing time(~A[s])~%" default-step-time swing-time))
     (send self :set-gait-generator-param
           :default-step-time default-step-time
           :default-double-support-ratio new-default-double-support-ratio)
     )
   )
  (:start-graspless-manip-mode
   (robot arm)
   "Start graspless manip mode while walking.
    robot is robot instance which angle-vector is for graspless manip.
    arm is used arm (:rarm, :larm, :arms)."
   (let ((crds (if (eq arm :arms)
                   (send self :calc-hand-trans-coords-dual-arms robot)
                 (send self :calc-hand-trans-coords-single-arm robot arm)
                 )))
     (send self :set-auto-balancer-param
           :graspless-manip-arm arm
           :graspless-manip-mode t :is-hand-fix-mode t
           :graspless-manip-reference-trans-rot (send crds :worldrot)
           :graspless-manip-reference-trans-pos (send crds :worldpos)
           )))
  (:stop-graspless-manip-mode
   ()
   "Stop graspless manip mode while walking."
   (send self :set-auto-balancer-param :graspless-manip-mode nil))
  (:calc-hand-trans-coords-dual-arms
   (robot)
   "Calculate foot->hand coords transformation for dual-arm graspless manipulation."
   (let* ((fc (send robot :foot-midcoords))
          (arm-pos (mapcar #'(lambda (x) (send fc :inverse-transform-vector (send robot x :end-coords :worldpos))) '(:larm :rarm)))
          (yy (apply #'v- arm-pos)))
     (setf (elt yy 2) 0)
     (setq yy (normalize-vector yy))
     (make-coords :pos (apply #'midpoint 0.5 arm-pos)
                  :rot (transpose (make-matrix 3 3 (list (v* yy (float-vector 0 0 1))
                                                         yy
                                                         (float-vector 0 0 1)))))))
  (:calc-hand-trans-coords-single-arm
   (robot arm)
   "Calculate foot->hand coords transformation for single-arm graspless manipulation."
   (send (send robot :foot-midcoords) :transformation (send robot arm :end-coords)))
  )

;; SoftErrorLimiterService
(defmethod rtm-ros-robot-interface
  (:set-soft-error-limit
    (name limit)
    "Set SoftErrorLimiter servo error limit [rad] with given name."
    (send self :softerrorlimiterservice_setservoerrorlimit :name name :limit limit)
    )
  )

;; StabilizerService
(def-set-get-param-method
  'hrpsys_ros_bridge::Openhrp_StabilizerService_stParam
  :raw-set-st-param :get-st-param :get-st-param-arguments
  :stabilizerservice_setparameter :stabilizerservice_getparameter)

(defmethod rtm-ros-robot-interface
  (:set-st-param
   (&rest args
    &key st-algorithm
         eefm-pos-damping-gain eefm-rot-damping-gain eefm-pos-time-const-support eefm-rot-time-const eefm-swing-pos-spring-gain eefm-swing-pos-time-const eefm-swing-rot-spring-gain eefm-swing-rot-time-const eefm-ee-forcemoment-distribution-weight
    &allow-other-keys)
   "Set Stabilizer parameters.
    For arguments, please see (send *ri* :get-st-param-arguments)."
   (unless (send self :get :default-st-param)
     (send self :put :default-st-param (send self :get-st-param)))
   (let ((prm (send self :get-st-param)))
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
             (if (and (memq :eefm-pos-damping-gain args) eefm-pos-damping-gain)
                 (let ((tmp (send prm :eefm_pos_damping_gain)))
                   (setq (tmp . ros::_data) (apply #'concatenate float-vector eefm-pos-damping-gain))
                   (list :eefm-pos-damping-gain tmp)))
             (if (and (memq :eefm-rot-damping-gain args) eefm-rot-damping-gain)
                 (let ((tmp (send prm :eefm_rot_damping_gain)))
                   (setq (tmp . ros::_data) (apply #'concatenate float-vector eefm-rot-damping-gain))
                   (list :eefm-rot-damping-gain tmp)))
             (if (and (memq :eefm-rot-time-const args) eefm-rot-time-const)
                 (let ((tmp (send prm :eefm_rot_time_const)))
                   (setq (tmp . ros::_data) (apply #'concatenate float-vector eefm-rot-time-const))
                   (list :eefm-rot-time-const tmp)))
             (if (and (memq :eefm-pos-time-const-support args) eefm-pos-time-const-support)
                 (let ((tmp (send prm :eefm_pos_time_const_support)))
                   (setq (tmp . ros::_data) (apply #'concatenate float-vector eefm-pos-time-const-support))
                   (list :eefm-pos-time-const-support tmp)))
             (if (and (memq :eefm-swing-pos-spring-gain args) eefm-swing-pos-spring-gain)
                 (let ((tmp (send prm :eefm_swing_pos_spring_gain)))
                   (setq (tmp . ros::_data) (apply #'concatenate float-vector eefm-swing-pos-spring-gain))
                   (list :eefm-swing-pos-spring-gain tmp)))
             (if (and (memq :eefm-swing-pos-time-const args) eefm-swing-pos-time-const)
                 (let ((tmp (send prm :eefm_swing_pos_time_const)))
                   (setq (tmp . ros::_data) (apply #'concatenate float-vector eefm-swing-pos-time-const))
                   (list :eefm-swing-pos-time-const tmp)))
             (if (and (memq :eefm-swing-rot-spring-gain args) eefm-swing-rot-spring-gain)
                 (let ((tmp (send prm :eefm_swing_rot_spring_gain)))
                   (setq (tmp . ros::_data) (apply #'concatenate float-vector eefm-swing-rot-spring-gain))
                   (list :eefm-swing-rot-spring-gain tmp)))
             (if (and (memq :eefm-swing-rot-time-const args) eefm-swing-rot-time-const)
                 (let ((tmp (send prm :eefm_swing_rot_time_const)))
                   (setq (tmp . ros::_data) (apply #'concatenate float-vector eefm-swing-rot-time-const))
                   (list :eefm-swing-rot-time-const tmp)))
             (if (and (memq :eefm-ee-forcemoment-distribution-weight args) eefm-ee-forcemoment-distribution-weight)
                 (let ((tmp (send prm :eefm_ee_forcemoment_distribution_weight)))
                   (setq (tmp . ros::_data) (apply #'concatenate float-vector eefm-ee-forcemoment-distribution-weight))
                   (list :eefm-ee-forcemoment-distribution-weight tmp)))
             args))))
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
  :set-kalman-filter-param :get-kalman-filter-param :get-kalman-filter-param-arguments
  :kalmanfilterservice_setkalmanfilterparam :kalmanfilterservice_getkalmanfilterparam)

(defmethod rtm-ros-robot-interface
  (:get-kalman-filter-algorithm
   ()
   "Get KalmanFilter Algorithm as Euslisp symbol."
   (send self :get-idl-enum-values
         (send (send self :get-kalman-filter-param) :kf_algorithm)
         "HRPSYS_ROS_BRIDGE::OPENHRP_KALMANFILTERSERVICE_KFALGORITHM"))
)

;; EmergencyStopperService
(def-set-get-param-method
  'hrpsys_ros_bridge::Openhrp_EmergencyStopperService_EmergencyStopperParam
  :set-emergency-stopper-param :get-emergency-stopper-param :get-emergency-stopper-param-arguments
  :emergencystopperservice_setemergencystopperparam :emergencystopperservice_getemergencystopperparam)

(defmethod rtm-ros-robot-interface
  (:emergency-stop-motion
   ()
   "Stop motion emergently in EmergencyStopper."
   (send self :emergencystopperservice_stopmotion))
  (:hard-emergency-stop-motion
   ()
   "Stop motion emergently in hard EmergencyStopper."
   (send self :emergencystopperservice_stopmotion :rosbridge-name "/HardEmergencyStopperServiceROSBridge"))
  (:emergency-release-motion
   ()
   "Release emergency motion stopping in EmergencyStopper."
   (send self :emergencystopperservice_releasemotion))
  (:hard-emergency-release-motion
   ()
   "Release emergency motion stopping in hard EmergencyStopper."
   (send self :emergencystopperservice_releasemotion :rosbridge-name "/HardEmergencyStopperServiceROSBridge"))
  (:emergency-mode
   ()
   "Returns emergency mode."
   (cdr (assoc :emergency-mode robot-state))
   )
  )

;; GraspControllserService
(defmethod rtm-ros-robot-interface
  ;; To use GraspControllserService, please define
  ;;  1. :get-hand-config-list
  ;;  2. :start-grasp's target-error and gain-percentage
  (:get-hand-config-list
   ()
   (error ";; :get-hand-config-list should be defined in subclass of (~A)~%" (class self))
   )
  (:start-grasp
   (limb &key (target-error) (gain-percentage))
   (unless (and target-error gain-percentage)
     (error ";; :start-grasp target-error and gain-percentage should be defined in subclass of (~A)~%" (class self)))
   (case limb
     (:larm
      (send self :set-servo-gain-percentage
            (caddr (assoc :larm (send self :get-hand-config-list))) gain-percentage)
      (send self :graspcontrollerservice_startgrasp
            :name (cadr (assoc :larm (send self :get-hand-config-list))) :target_error target-error))
     (:rarm
      (send self :set-servo-gain-percentage
            (caddr (assoc :rarm (send self :get-hand-config-list))) gain-percentage)
      (send self :graspcontrollerservice_startgrasp
            :name (cadr (assoc :rarm (send self :get-hand-config-list))) :target_error target-error))
     (:arms
      (send self :start-grasp :rarm :target-error target-error :gain-percentage gain-percentage)
      (send self :start-grasp :larm :target-error target-error :gain-percentage gain-percentage))
     )
   )
  (:stop-grasp
   (limb)
   (case limb
     (:larm
      (send self :set-servo-gain-percentage (caddr (assoc :larm (send self :get-hand-config-list))) 100)
      (send self :graspcontrollerservice_stopgrasp :name (cadr (assoc :larm (send self :get-hand-config-list)))))
     (:rarm
      (send self :set-servo-gain-percentage (caddr (assoc :rarm (send self :get-hand-config-list))) 100)
      (send self :graspcontrollerservice_stopgrasp :name (cadr (assoc :rarm (send self :get-hand-config-list)))))
     (:arms
      (send self :stop-grasp :rarm)
      (send self :stop-grasp :larm))
     )
   )
  )

(defmethod rtm-ros-robot-interface
  (:start-default-unstable-controllers
   (&key (ic-limbs '(:rarm :larm))
         (abc-limbs (if (not (every #'null (send robot :arms)))
                        '(:rleg :lleg :rarm :larm)
                      '(:rleg :lleg))))
   "Start default unstable RTCs controller mode.
    Currently Stabilzier, AutoBalancer, and ImpedanceController are started."
   (send self :start-auto-balancer :limbs abc-limbs)
   (dolist (limb ic-limbs)
     (send self :start-impedance-no-wait limb))
   (send self :start-st)
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

;; ReferenceForceUpdater
(def-set-get-param-method
  'hrpsys_ros_bridge::Openhrp_ReferenceForceUpdaterService_ReferenceForceUpdaterParam
  :raw-set-reference-force-updater-param :raw-get-reference-force-updater-param :get-reference-force-updater-param-arguments
  :ReferenceForceUpdaterService_setReferenceForceUpdaterParam :ReferenceForceUpdaterService_getReferenceForceUpdaterParam
  :optional-args (list :name 'name))
(defmethod rtm-ros-robot-interface
  (:get-supported-reference-force-updater-name-list
   ()
   "Get supported reference force updater names (:rarm, :larm, ...)"
   (unless (send self :get :supported-reference-force-updater-name-list)
     (send self :put :supported-reference-force-updater-name-list
           (mapcar #'(lambda (x) (read-from-string (format nil ":~A" x))) (send (send self :referenceforceupdaterservice_getSupportedReferenceForceUpdaterNameSequence) :o_names))))
   (send self :get :supported-reference-force-updater-name-list))
  (:start-reference-force-updater
   (limb)
   "Start reference force updater mode.
    limb should be limb symbol name such as :rarm, :larm, :arms, or supported names (see :get-supported-reference-force-updater-name-list)."
   (cond
    ((memq limb (send self :get-supported-reference-force-updater-name-list))
     (send self :referenceforceupdaterservice_startReferenceforceupdater :name (string-downcase limb)))
    (t
     (send self :force-sensor-method
           limb
           #'(lambda (name)
               (send self :referenceforceupdaterservice_startReferenceforceupdater :name (string-downcase name)))
           :start-reference-force-updater))))
  (:stop-reference-force-updater
   (limb)
   "Stop reference force updater mode.
    limb should be limb symbol name such as :rarm, :larm, :arms, or supported names (see :get-supported-reference-force-updater-name-list)."
   (cond
    ((memq limb (send self :get-supported-reference-force-updater-name-list))
     (send self :referenceforceupdaterservice_stopReferenceforceupdater :name (string-downcase limb)))
    (t
     (send self :force-sensor-method
           limb
           #'(lambda (name)
               (send self :referenceforceupdaterservice_stopReferenceforceupdater :name (string-downcase name)))
           :stop-reference-force-updater))))
  (:start-reference-force-updater-no-wait
   (limb)
   "Start reference force updater mode without wait.
    limb should be limb symbol name such as :rarm, :larm, :arms, or supported names (see :get-supported-reference-force-updater-name-list)."
   (cond
    ((memq limb (send self :get-supported-reference-force-updater-name-list))
     (send self :referenceforceupdaterservice_startReferenceforceupdaterNoWait :name (string-downcase limb)))
    (t
     (send self :force-sensor-method
           limb
           #'(lambda (name)
               (send self :referenceforceupdaterservice_startReferenceforceupdaterNoWait :name (string-downcase name)))
           :start-reference-force-updater))))
  (:stop-reference-force-updater-no-wait
   (limb)
   "Start reference force updater mode without wait.
    limb should be limb symbol name such as :rarm, :larm, :arms, or supported names (see :get-supported-reference-force-updater-name-list)."
   (cond
    ((memq limb (send self :get-supported-reference-force-updater-name-list))
     (send self :referenceforceupdaterservice_stopReferenceforceupdaterNoWait :name (string-downcase limb)))
    (t
     (send self :force-sensor-method
           limb
           #'(lambda (name)
               (send self :referenceforceupdaterservice_stopReferenceforceupdaterNoWait :name (string-downcase name)))
           :stop-reference-force-updater))))
  (:set-reference-force-updater-param
   (limb &rest args)
   "Set reference force updater parameter like (send *ri* :set-reference-force-updater-param :rarm :p-gain 0.02).
    limb should be limb symbol name such as :rarm, :larm, :arms, or supported names (see :get-supported-reference-force-updater-name-list).
    For arguments, please see (send *ri* :get-reference-force-updater-param-arguments)"
   (cond
    ((memq limb (send self :get-supported-reference-force-updater-name-list))
     (send* self :raw-set-reference-force-updater-param (string-downcase limb) args))
    (t
     (send* self :force-sensor-method
            limb
            #'(lambda (name &rest _args)
                (send* self :raw-set-reference-force-updater-param (string-downcase name) args))
            :set-reference-force-updater-param
            args))))
  (:get-reference-force-updater-param
   (limb)
   "Get reference force updater parameter.
    limb should be limb symbol name such as :rarm, :larm, :arms, or supported names (see :get-supported-reference-force-updater-name-list)."
   (cond
    ((memq limb (send self :get-supported-reference-force-updater-name-list))
     (send self :raw-get-reference-force-updater-param (string-downcase limb)))
    (t
     (send self :force-sensor-method
           limb
           #'(lambda (name)
               (send self :raw-get-reference-force-updater-param (string-downcase name)))
           :get-reference-force-updater-param))))
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

;; TorqueControllerService
(def-set-get-param-method 'hrpsys_ros_bridge::OpenHRP_TorqueControllerService_TorqueControllerParam
  :set-torque-controller-param :get-torque-controller-param :get-torque-controller-param-arguments
  :torquecontrollerservice_settorquecontrollerparam :torquecontrollerservice_gettorquecontrollerparam
  :optional-args (list :jname 'jname))
(defmethod rtm-ros-robot-interface
  (:enable-torque-control
   (jname)
   "enable torque controller for specified joint (only start torque control when tau exceeds tauMax)
    jname should be string of jointname (for one joint) or list of jointname (for multiple joints)"
   (cond
    ((listp jname)
     (send self :torquecontrollerservice_enablemultipletorquecontrollers :jnames jname))
    (t
     (send self :torquecontrollerservice_enabletorquecontroller :jname jname))
    ))
  (:disable-torque-control
   (jname)
   "disable torque control in specified joint (torque controller do nothing and pass qRefIn when disabeld)
    jname should be string of jointname (for one joint) or list of jointname (for multiple joints)"
   (cond
    ((listp jname)
     (send self :torquecontrollerservice_disablemultipletorquecontrollers :jnames jname)
     )
    (t
     (send self :torquecontrollerservice_disabletorquecontroller :jname jname))
    ))
  (:start-torque-control
   (jname &optional (tauref nil))
   "start torque control in specified joint. set reference torque before start if tauref is given.
    jname should be string of jointname (for one joint) or list of jointname (for multiple joints)
    tauref is reference torque [N/m], should be a number (for one joint) or list of numbers (for multiple joints)"
   (cond
    ((listp jname)
     (when tauref
       (send self :set-multiple-reference-torques jname tauref))
     (send self :torquecontrollerservice_startmultipletorquecontrols :jnames jname)
     )
    (t
     (when tauref
       (send self :set-reference-torque jname tauref))
     (send self :torquecontrollerservice_starttorquecontrol :jname jname))
    ))
  (:stop-torque-control
   (jname)
   "stop torque controller for specified joint (this method does not disable torque control)
    jname should be string of jointname (for one joint) or list of jointname (for multiple joints)"
   (cond
    ((listp jname)
     (send self :torquecontrollerservice_stopmultipletorquecontrols :jnames jname)
     )
    (t
     (send self :torquecontrollerservice_stoptorquecontrol :jname jname))
    ))
  (:set-reference-torque
   (jname tauref)
   "set reference torque in specified joint
    jname should be string of jointname (for one joint) or list of jointname (for multiple joints)
    tauref (reference torque [N/m]) should be a number (for one joint) or list of numbers (for multiple joints)"
   (cond
    ((listp jname)
     (unless (listp tauref)
       (warn ";; tauref ~A should be a list~%")
       (return-from :set-reference-torque))
     (send self :torquecontrollerservice_setmultiplereferencetorques :jnames jname :tauRefs tauref)
     )
    (t
     (when (listp tauref)
       (warn ";; tauref ~A should be a number~%")
       (return-from :set-reference-torque))
     (send self :torquecontrollerservice_setreferencetorque
           :jname jname :tauRef tauref))
    ))
  )

(defmethod rtm-ros-robot-interface
  (:print-rtc-param-all
   ()
   "Print RTC parameter struct probided by :get-xx method in more readable style for all recommended RTCs.
    Check all possible RTCs by checking existence of ROSBridge nodes."
   (let ((rosbridge-nodes (ros::get-nodes)))
     (when (find "/AutoBalancerServiceROSBridge" rosbridge-nodes :test #'string=)
       (format t ";; :get-gait-generator-param~%")
       (print-ros-msg (send self :get-gait-generator-param))
       (format t ";; :get-auto-balancer-param~%")
       (print-ros-msg (send self :get-auto-balancer-param)))
     (when (find "/StabilizerServiceROSBridge" rosbridge-nodes :test #'string=)
       (format t ";; :get-st-param~%")
       (print-ros-msg (send self :get-st-param)))
     (when (find "/ObjectContactTurnaroundDetectorServiceROSBridge" rosbridge-nodes :test #'string=)
       (format t ";; :get-object-turnaround-detector-param~%")
       (print-ros-msg (send self :get-object-turnaround-detector-param)))
     (when (find "/KalmanFilterServiceROSBridge" rosbridge-nodes :test #'string=)
       (format t ";; :get-kalman-filter-param~%")
       (print-ros-msg (send self :get-kalman-filter-param)))
     (when (find "/ReferenceForceUpdaterServiceROSBridge" rosbridge-nodes :test #'string=)
       (dolist (limb (sort '(:rleg :lleg :rarm :larm) #'< #'(lambda (x) (position (car (send robot x :force-sensors)) (send robot :force-sensors)))))
         (format t ";; :get-reference-force-updater-param ~A~%" limb)
         (print-ros-msg (send self :get-reference-force-updater-param limb))))
     (when (find "/ImpedanceControllerServiceROSBridge" rosbridge-nodes :test #'string=)
       (dolist (limb (sort '(:rleg :lleg :rarm :larm) #'< #'(lambda (x) (position (car (send robot x :force-sensors)) (send robot :force-sensors)))))
         (format t ";; :get-impedance-controller-param ~A~%" limb)
         (print-ros-msg (send self :get-impedance-controller-param limb))))
     (when (find "/RemoveForceSensorLinkOffsetServiceROSBridge" rosbridge-nodes :test #'string=)
       (dolist (limb (sort '(:rleg :lleg :rarm :larm) #'< #'(lambda (x) (position (car (send robot x :force-sensors)) (send robot :force-sensors)))))
         (format t ";; :get-forcemoment-offset-param ~A~%" limb)
         (print-ros-msg (send self :get-forcemoment-offset-param limb))))
     (when (find "/TorqueControllerServiceROSBridge" rosbridge-nodes :test #'string=)
       (dolist (j (send robot :joint-list))
         (format t ";; :get-torque-controller-param ~A~%" (read-from-string (format nil ":~A" (send j :name))))
         (print-ros-msg (send self :get-torque-controller-param (read-from-string (format nil ":~A" (send j :name)))))))
     t))
  )


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; utility functions for seq
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun dump-seq-pattern-file
  (rs-list output-basename &key (initial-sync-time 3.0) (robot)) ;; [s]
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
  (when (cadr (memq :wrench-list (car rs-list)))
    (with-open-file
     (wrf (format nil "~A.wrenches" output-basename) :direction :output) ;; wrench file -> force and moment ;; 6 x force-sensor-num
     (dolist (l rs-list)
       ;; write wrenches file
       (format wrf "~A " (+ initial-sync-time (cadr (memq :time l))))
       (let ((wrl (cadr (memq :wrench-list l))))
         (dotimes (j (length (car wrl))) ;; j is limb index
           (let ((f (elt (car wrl) j)) (m (elt (cadr wrl) j)))
             (dotimes (i 3) (format wrf "~A " (elt f i))) ;; force
             (dotimes (i 3) (format wrf "~A " (elt m i))) ;; moment
             )))
       (format wrf "~%")
       )) ;; with-open-file for .wrenches file
    )
  (if (and (cadr (memq :contact-state (car rs-list))) robot)
      (setq rs-list (add-optional-data-from-rs-list rs-list robot)))
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

(defun add-optional-data-from-rs-list
  (rs-list robot
  &key (all-limbs
        (mapcar #'(lambda (fs)
                    (find-if #'(lambda (l) (equal fs (car (send robot l :force-sensors)))) '(:rleg :lleg :rarm :larm)))
                (send robot :force-sensors)))
       (limbs
        (remove nil
                (mapcar #'(lambda (fs)
                    (find-if #'(lambda (l) (equal fs (car (send robot l :force-sensors))))
                             (case (length (cadr (memq :contact-state (car rs-list))))
                                   (2 '(:rleg :lleg))
                                   (4 '(:rleg :lleg :rarm :larm)))))
                (send robot :force-sensors))))
       (add-optional-data-p t))
  "Add optionalData from rs-list.
   all-limbs is all limbs with ForceSensor in VRML.
   limbs is all limbs included in rs-list.
   If add-optional-data-p is nil, return contact state and swing support time list.
   :contact-state is required in rs-list.
   :support = 1.0, :swing = 0.0"
  ;; Get contact-state list
  (let ((cs-list
         (mapcar #'(lambda (rs)
                     (mapcar #'(lambda (l)
                                 (cond
                                  ((not (memq l limbs)) 0.0)
                                  ((eq (elt (cadr (memq :contact-state rs)) (position l limbs)) :support) 1.0)
                                  (t 0.0)))
                             all-limbs))
                 rs-list)))
    ;; Add remain time for contact-states
    (let* ((dt (- (cadr (memq :time (cadr rs-list))) (cadr (memq :time (car rs-list)))))
           (prev-cs (cadr (memq :contact-state (car rs-list))))
           (contact-change-time-list (make-list (length limbs)))
           (count-time-list (make-list (length limbs) :initial-element 0))
           (limb-idx-list (let ((cnt -1)) (mapcar #'(lambda (x) (incf cnt)) (make-list (length limbs)))))
           (all-limb-idx-list (let ((cnt -1)) (mapcar #'(lambda (x) (incf cnt)) (make-list (length all-limbs)))))
           (swing-support-time-list)
           (default-swing-support-time 1.0)) ;; [s]
      ;; Get time count when contac state changes
      (dolist (rs rs-list)
        (let ((cs (cadr (memq :contact-state rs))))
          (mapcar #'(lambda (idx)
                      (incf (elt count-time-list idx))
                      (when (not (eq (elt cs idx) (elt prev-cs idx)))
                        (push (elt count-time-list idx) (elt contact-change-time-list idx))
                        (setf (elt count-time-list idx) 0)))
                  limb-idx-list)
          (setq prev-cs cs)
          ))
      (setq contact-change-time-list (mapcar #'(lambda (x) (reverse x)) contact-change-time-list))
      (setq contact-change-time-list (mapcar #'(lambda (l)
                                                 (if (memq l limbs)
                                                     (elt contact-change-time-list (position l limbs))))
                                             all-limbs))
      ;; Calc remain time
      (setq count-time-list
            (mapcar #'(lambda (idx)
                        (if (elt contact-change-time-list idx)
                            (car (elt contact-change-time-list idx))
                          (round (/ default-swing-support-time dt))))
                    all-limb-idx-list))
      (dolist (rs rs-list)
        (push (mapcar #'(lambda (idx)
                          (if (elt contact-change-time-list idx)
                              (progn
                                (if (= (elt count-time-list idx) 1) ;; Contact change
                                    (progn
                                      (pop (elt contact-change-time-list idx))
                                      (setf (elt count-time-list idx) (car (elt contact-change-time-list idx)))) ;; Reset count
                                  (setf (elt count-time-list idx) (- (elt count-time-list idx) 1)))
                                ;; Return count => time [s]
                                (if (elt count-time-list idx)
                                    (* dt (elt count-time-list idx))
                                  default-swing-support-time))
                            default-swing-support-time))
                      all-limb-idx-list)
              swing-support-time-list)
        )
      ;; Add contact state and swing support time to rs-list as optionalData
      (mapcar #'(lambda (rs cs sstime)
                  (append rs (list :optional-data
                                   (concatenate float-vector cs sstime))))
              rs-list cs-list (reverse swing-support-time-list))
      )))

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
      ;; optional data
      (let ((oln) (optfile (concatenate string input-basename ".optionaldata")) (idx 0))
        (if (probe-file optfile)
            (with-open-file
             (of optfile :direction :input)
             (while (setq oln (read-line of nil))
               (let* ((dd (subseq (data-str->data-list oln) 1)))
                 (nconc (elt rs-list idx)
                        (list :optional-data dd
                              :contact-state (mapcar #'(lambda (x) (if (eps= x 1.0) :support :swing)) (concatenate cons (subseq dd 0 (/ (length dd) 2))))))
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

(defun calculate-sole-margin-params
  (robot &key ((:print-mode pm) :euslisp) (margin-ratio 1.0))
  "Calculate sole edge margin in Stabilizer.
   robot is robot model.
   margin-ratio is multiplied by values (1.0 by default).
   When print-mode is :euslisp, print parameters in Euslisp style.
   When print-mode is :python, print parameters in Python hrpsys_config style."
  (let* ((sp-vertices (mapcar #'(lambda (x) (send robot :rleg :end-coords :inverse-transform-vector x)) (send (send robot :support-polygon :rleg) :vertices)))
         (front-margin (* margin-ratio (elt (find-extream sp-vertices #'(lambda (x) (elt x 0)) #'>) 0)))
         (rear-margin (* margin-ratio (abs (elt (find-extream sp-vertices #'(lambda (x) (elt x 0)) #'<) 0))))
         (inside-margin (* margin-ratio (elt (find-extream sp-vertices #'(lambda (x) (elt x 1)) #'>) 1)))
         (outside-margin (* margin-ratio (abs (elt (find-extream sp-vertices #'(lambda (x) (elt x 1)) #'<) 1)))))
    (case
        pm
      (:euslisp
       (format t "(send *ri* :set-st-param :eefm-leg-front-margin (* 1e-3 ~A) :eefm-leg-rear-margin (* 1e-3 ~A) :eefm-leg-inside-margin (* 1e-3 ~A) :eefm-leg-outside-margin (* 1e-3 ~A))~%"
               front-margin rear-margin inside-margin outside-margin))
      (:python
       (format t "stp=hcf.st_svc.getParameter()[1];~%")
       (format t "stp.eefm_leg_front_margin=1e-3*~A;stp.eefm_leg_rear_margin=1e-3*~A;stp.eefm_leg_inside_margin=1e-3*~A;stp.eefm_leg_outside_margin=1e-3*~A;~%"
               front-margin rear-margin inside-margin outside-margin)
       (format t "hcf.st_svc.setParameter(stp);~%")))
    ))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; utility functions for project generator generation
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun dump-project-file-by-cloning-euslisp-models
  (robot robot-file-path
   &key (object-models) (object-models-file-path)
        (timestep 0.005) (dt 0.005)
        (use-highgain-mode t) (integrate t) (method :euler)
        (output-fname (format nil "/tmp/~A" (send robot :name)))
        (debug)
        )
  "Clone euslisp robot and objects to OpenHRP3 project file.
   Add euslisp model + locate euslisp model in OpenHRP3 world.
   Arguments (required)
      robot  : Robot model
      robot-file-path : Path to the robot euslisp model, robot VRML(or Collada) file path.
   Arguments (key word)
      object-models : list of object euslisp model
      object-models-file-path) ;; list of object VRML(or Collada) file path
      debug : print openhrp3
   Arguments (key word + openhrp-project-generator's argument)
      Please see https://github.com/fkanehiro/openhrp3/blob/master/server/ModelLoader/README.md
      timestep, dt : [s]
      output-fname : output file name '[output-fname].xml'. By default, '[robot'sname].xml'
      integrate, use-highgain-mode, method : See above README
  "
  ;; TODO : Is longfloor.wrl necessary??
  (let ((str
         (format nil "openhrp-project-generator \\\
               ~A,~A \\\
               `rospack find openhrp3`/share/OpenHRP-3.1/sample/model/longfloor.wrl,0,0,0,1,0,0,0 ~A \\\
               --output ~A.xml \\\
               --integrate ~A --dt ~A --timeStep ~A --use-highgain-mode ~A --method ~A\\\
               --joint-properties ~A"
                 robot-file-path
                 (gen-ProjectGenerator-model-root-coords-string robot)
                 (let ((obj-path-list
                        (mapcar #'(lambda (fpath obj) (format nil "~A,~A" fpath (gen-ProjectGenerator-model-root-coords-string obj)))
                                object-models-file-path object-models)))
                   (if obj-path-list
                       (reduce #'(lambda (x y) (format nil "~A ~A" x y)) obj-path-list)
                     ""))
                 output-fname
                 (if integrate "true" "false")
                 dt
                 timestep
                 (if use-highgain-mode "true" "false")
                 (case method
                       (:runge-kutta "RUNGE_KUTTA")
                       (:euler "EULER")
                       (t "EULER"))
                 (gen-ProjectGenerator-joint-properties-string robot)
                 )))
    (if debug (warn ";; execute : ~A~%" str))
    (unix:system (format nil "bash -c -i \"~A;exit 0\";exit 0" str))
    ))

(defun gen-ProjectGenerator-joint-properties-string
  (robot)
  "Generate joint properties setting for openhrp-project-generator.
   Object (cascaded-link) is required as an argument."
  (let ((str))
    (dolist (j (send robot :joint-list))
      (if str
          (setq str (format nil "~A,~A.angle,~A" str (send j :name) (deg2rad (send j :joint-angle))))
        (setq str (format nil "~A.angle,~A" (send j :name) (deg2rad (send j :joint-angle)))))
      )
    str))

(defun gen-ProjectGenerator-model-root-coords-string
  (obj)
  "Generate root-coords offset setting for openhrp-project-generator.
   Object (cascaded-link) is required as an argument."
  (let* ((epsilon 1.0e-20) ;; same as normalize-vector threshold
         ;; Rot
         (dr (matrix-log (send (car (send obj :links)) :worldrot)))
         (ndr (if (< (norm dr) epsilon) (float-vector 1 0 0) (normalize-vector dr)))
         (ang (if (< (norm dr) epsilon) 0 (norm dr)))
         ;; Removing offset
         ;;   Initial rootLink pos offset should be removed https://github.com/fkanehiro/openhrp3/blob/master/server/ModelLoader/projectGenerator.cpp#L501-L503 (rootLink->p addition)
         (off-root-pos (send obj :inverse-transform-vector (send (car (send obj :links)) :worldpos)))
         ;; Pos
         (rpos (scale 1e-3 (v- (send (car (send obj :links)) :worldpos) off-root-pos))))
      (format nil "~A,~A,~A,~A,~A,~A,~A"
              (elt rpos 0) (elt rpos 1) (elt rpos 2)
              (elt ndr 0) (elt ndr 1) (elt ndr 2) ang ;; openhrp3 axis angle orientation representation
              )))


