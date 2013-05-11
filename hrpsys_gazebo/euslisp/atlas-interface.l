(require :rtm-ros-robotics-interface "package://hrpsys_ros_bridge/euslisp/rtm-ros-robot-interface.l")
(require :atlas "package://hrpsys_gazebo/models/atlas.l")
(load "package://hrpsys_gazebo/euslisp/atlas-applications.l")

(ros::load-ros-manifest "hrpsys_gazebo")

;; load compiled messages
(setq ros::*compile-message* t)
(ros::roseus-add-msgs "visualization_msgs")
(ros::roseus-add-msgs "sensor_msgs")
(setq ros::*compile-message* nil)

(ros::roseus "atlas_interface")

(defclass atlas-interface
  :super rtm-ros-robot-interface
  :slots (l_hand_grasp_pub r_hand_grasp_pub bdi-action walk-seq))
(defmethod atlas-interface
  (:init (&optional (qualify nil))
   (prog1 (send-super :init :robot atlas-robot)
     (setq l_hand_grasp_pub (ros::advertise "/sandia_hands/l_hand/simple_grasp"
                                            sandia_hand_msgs::SimpleGrasp 1)
           r_hand_grasp_pub (ros::advertise "/sandia_hands/r_hand/simple_grasp"
                                            sandia_hand_msgs::SimpleGrasp 1))
     (ros::advertise "atlas/control_mode" std_msgs::String 1)
     (ros::advertise "atlas/atlas_sim_interface_command"
                     atlas_msgs::AtlasSimInterfaceCommand 1)
     (ros::subscribe "atlas/atlas_sim_interface_state"
		     atlas_msgs::AtlasSimInterfaceState #'send self :sim-interface-state-cb)
     ;; do not use in Qualification
     (unless qualify
       (ros::advertise "atlas/mode" std_msgs::String 1)
       (ros::advertise "atlas/set_pose" geometry_msgs::Pose 1))
     ;; action lib / needs node of atlas_msgs/actionlib_server
     (setq bdi-action
           (instance ros::simple-action-client :init
                     "atlas/bdi_control" atlas_msgs::WalkDemoAction))
     (send bdi-action :wait-for-server 3)
     (setf (get robot :footstep-parameter)
           '(:default-half-offset #f(0.0 100.0 0.0) :forward-offset-length 180.0 :outside-offset-length 85.0 :rotate-rad 40.0))
     (setq walk-seq 0)
     ))
  (:sim-interface-state-cb
   (msg)
   (let* ((pos-est (send msg :pos_est))
	  (pos (send pos-est :position))
	  (foot-pos-est1 (car (send msg :foot_pos_est)))
	  (foot-pos-est2 (cadr (send msg :foot_pos_est)))
	  (yaw (+ (acos (send (send foot-pos-est1 :orientation) :w))
		  (acos (send (send foot-pos-est2 :orientation) :w))))
	  (cur-bhv (send msg :current_behavior)))
     (send self :put :pose-on-gazebo (list (send pos :x) (send pos :y) (send pos :z) (rad2deg yaw)))
     (send self :put :current-behavior cur-bhv)
     ))
  (:wait-for-start-bdi-walk
   ()
   (ros::spin-once)
   (while (not (equal (send self :get :current-behavior) 4))
     (ros::spin-once)
     ))
  (:wait-for-bdi-walk
   ()
   (ros::spin-once)
   (while (equal (send self :get :current-behavior) 4)
     (ros::spin-once)
     ))
  (:grasp
   (value &optional (arm :arms))
   (let ((msg (instance sandia_hand_msgs::SimpleGrasp :init :name "cylindrical" :closed_amount value)))
     (case arm
       (:arms
        (send self :grasp value :larm)
        (send self :grasp value :rarm))
       (:larm
        (ros::publish "/sandia_hands/l_hand/simple_grasp" msg))
       (:rarm
        (ros::publish "/sandia_hands/r_hand/simple_grasp" msg))
       )))
  (:start-grasp (&optional (arm :arms)) (send self :grasp 1 arm))
  (:stop-grasp  (&optional (arm :arms)) (send self :grasp 0 arm))
  (:control-mode (&optional (mode "Stand")) (send_atlas_control_mode mode))
  (:atlas-mode   (&optional (mode "nominal")) (send_atlas_mode mode))
  (:set-simulation-mode  (&key (effort 255) (sleep 0.5) (mode :user))
   (set-simulation-mode :effort effort :sleep sleep :mode mode))
  (:bdi_footstep
   (footstep-list &key (step-duration 0.63) (swing-height 0.3) &allow-other-keys)
   ;;(send_atlas_control_mode "Walk")
   (let* ((fstep (car footstep-list))
          (offset (copy-object (send fstep :worldpos))))
     (setf (elt offset 1) 0.0)
     (scale -1.0 offset offset)
     (let ((walk-goal (send bdi-action :make-goal-instance))
           (goal (instance atlas_msgs::WalkDemoGoal :init))
           ;;(step-params  (instance atlas_msgs::AtlasBehaviorStepParams :init))
           ;;(stand-params (instance atlas_msgs::AtlasBehaviorStandParams :init))
           ;;(manip-params (instance atlas_msgs::AtlasBehaviorManipulateParams :init))
           ;;(k-effort (instantiate integer-vector 28))
           (k-effort (instantiate string 28))
           steps (step-idx 0))
       (fill k-effort 0)
       ;; add first step ???
       (let* ((ffstep (copy-object (car footstep-list)))
              (step (instance atlas_msgs::AtlasBehaviorStepData
                              :init :step_index step-idx
                              :foot_index (if (eq (get ffstep :l/r) :rleg) 1 0) ;; :lleg -> 0 / :rleg -> 1
                              :duration 0
                              :swing_height 0)))
         (send ffstep :translate offset :world)
         (send step :pose (ros::coords->tf-pose ffstep))
         (push step steps)
         (incf step-idx))
       ;;
       (dolist (fstep footstep-list)
         (let ((step (instance atlas_msgs::AtlasBehaviorStepData
                               :init :step_index step-idx
                               :foot_index (if (eq (get fstep :l/r) :rleg) 1 0) ;; :lleg -> 0 / :rleg -> 1
                               :duration step-duration
                               :swing_height swing-height)))
           (send fstep :translate offset :world)
           (send step :pose (ros::coords->tf-pose fstep))
           (push step steps)
           (incf step-idx)
           ))
       ;;(send goal :header (instancd std_msgs::header :init))
       (send goal :behavior atlas_msgs::WalkDemoGoal::*WALK*)
       (send goal :k_effort k-effort)
       (send goal :steps (nreverse steps))
       (send walk-goal :header :seq walk-seq)
       (incf walk-seq)
       (send walk-goal :header :stamp (ros::time-now))
       (send goal :header :stamp (send walk-goal :header :stamp))
       (send walk-goal :goal_id :stamp (send walk-goal :header :stamp))
       (send walk-goal :goal goal)
       (send bdi-action :send-goal walk-goal)
       )))
  (:bdi-go-pos (x y &optional (theta 0) &key (r robot) (step-duration 0.63) (swing-height 0.3))
               ;; x [m],  y[m], theta [deg]
   (send self :bdi_footstep
         (send robot :go-pos-params->footstep-list (* x 1000) (* y 1000) theta)))
  (:teleport-to (coords)
   (unix::usleep (* 100 1000))
   (send self :atlas-mode "harnessed")
   (send self :control-mode "Freeze")
   (send self :control-mode "StandPrep")
   (unix::usleep (* 2 1000 1000))
   (send self :atlas-mode "nominal")
   (unix::usleep (* 10 1000))
   (ros::publish "atlas/set_pose" (ros::coords->tf-pose coords))
   (unix::usleep (* 200 1000))
   (send self :control-mode "Stand"))
  )

(defun set-simulation-mode (&key (effort 255) (sleep 0.5) (mode :user))
  (let ((msg (instance atlas_msgs::AtlasSimInterfaceCommand :init))
        (k-effort (instantiate string 28)))
    (case mode
      (:stand
       (setq mode atlas_msgs::AtlasSimInterfaceCommand::*STAND*))
      (:freeze
       (setq mode atlas_msgs::AtlasSimInterfaceCommand::*FREEZE*))
      (:stand-prep
       (setq mode atlas_msgs::AtlasSimInterfaceCommand::*STAND_PREP*))
      (:walk
       (setq mode atlas_msgs::AtlasSimInterfaceCommand::*WALK*))
      (:step
       (setq mode atlas_msgs::AtlasSimInterfaceCommand::*STEP*))
      (:manipulate
       (setq mode atlas_msgs::AtlasSimInterfaceCommand::*MANIPULATE*))
      ((:user t)
       (setq mode atlas_msgs::AtlasSimInterfaceCommand::*USER*)))
    (send msg :behavior mode)
    (fill k-effort effort)
    (send msg :k_effort k-effort)
    (ros::publish "atlas/atlas_sim_interface_command" msg)
    (when sleep (unix::usleep (round (* sleep 1000 1000))))
    ))

(defun send_atlas_control_mode
  (&optional (mode "Stand"))
  (let ((msg (instance std_msgs::String :init :data mode)))
    (ros::publish "atlas/control_mode" msg)
    (warn ";; send_atlas_control_mode is deprecated! use (send *ri* :set-simulation-mode :mode :hogehoge) instead~%")))

;; do not use in Qualification
(defun send_atlas_mode
  (&optional (mode "nominal"))
  (let ((msg (instance std_msgs::String :init :data mode)))
    (ros::publish "atlas/mode" msg)))

(defun start-tilt-laser (&optional (speed 1.2))
  (one-shot-publish "/multisense_sl/set_spindle_speed" (instance std_msgs::Float64 :init :data speed)))

(defun get-tilt-laser (&key (remove-nan))
  (let ((msg (one-shot-subscribe "/full_cloud2" sensor_msgs::PointCloud2 :timeout 30000)))
    (when msg
      (make-eus-pointcloud-from-ros-msg msg :remove-nan remove-nan))))

;;
;; footstep
;;
(defun generate-atlas-walk-pattern
  (&key (no-gen nil) (fname nil))
  (let (pattern-list
        (fname (if fname fname
		 (format nil "~A/atlas-walk-pattern"
			 (ros::resolve-ros-path "package://hrpsys_gazebo/build")))))
    (unless no-gen
      (format t ";; generate pattern~%")
      (setq pattern-list
	    (send *atlas* :calc-walk-pattern-from-footstep-list
		  (send *atlas* :go-pos-params->footstep-list
			500 150 45) ;; x[mm] y[mm] th[rad]
		  :debug-view :no-message :dt 0.01))
      (format t ";; dump pattern file~%")
      (send *ri* :dump-seq-pattern-file pattern-list fname))
    (format t ";; load pattern file~%")
    (send *ri* :loadPattern fname)
    ))

(defun atlas-walk-with-walk-pattern-gen ()
  (send_atlas_control_mode "Stand")
  (atlas-init :set-reset-pose nil)
  (objects (list *atlas*))
  (send *atlas* :angle-vector (send *ri* :state :potentio-vector))
  (send *ri* :angle-vector (send *atlas* :angle-vector) 2000)
  (send_atlas_control_mode "User")
  (generate-atlas-walk-pattern)
  )

(defvar *footstep-planning-service* "/plan_footsteps")

(ros::load-ros-manifest "footstep_planner")

(defun make-ros-footstep (start goal)
  (let ((req (instance humanoid_nav_msgs::PlanFootstepsRequest :init)))
    ;;
    (let ((pos (send start :pos))
          (theta (caar (send start :rpy-angle))))
      (send req :start :x (* 1e-3 (elt pos 0)))
      (send req :start :y (* 1e-3 (elt pos 1)))
      (send req :start :theta theta))
    ;;
    (let ((pos (send goal :pos))
          (theta (caar (send goal :rpy-angle))))
      (send req :goal :x (* 1e-3 (elt pos 0)))
      (send req :goal :y (* 1e-3 (elt pos 1)))
      (send req :goal :theta theta))
    ;;
    (ros::service-call *footstep-planning-service* req)
    ))

(defun footstep-planning-armin
  (start goal)
  (let* ((res
          (make-ros-footstep start goal))
         (path
          (mapcar
           #'(lambda (msg)
	       (let (coords)
		 (setq coords (make-coords :pos (float-vector
						 (* 1000 (send msg :pose :x))
						 (* 1000 (send msg :pose :y)) 0)))
		 (send coords :rotate (send msg :pose :theta) :z)
		 (setq coords (send start :transformation coords))
		 (send coords :put :l/r
		       (cond
			((= (send msg :leg) humanoid_nav_msgs::StepTarget::*left*) :lleg)
			((= (send msg :leg) humanoid_nav_msgs::StepTarget::*right*) :rleg)))
		 coords
		 ))
           (send res :footsteps))))
    ;; append first 1 or 2 step(s)
    path))
;; (send *ri* :bdi_footstep (footstep-planning-armin (make-coords :pos (scale 1000 (float-vector 0.5 1.55 0)) :rpy (list 0 0 0)) (make-coords :pos (scale 1000 (float-vector 6.2 1.55 0)) :rpy (list 0 0 0))))
;; (send *ri* :bdi_footstep (footstep-planning-armin (make-coords :pos (scale 1000 (float-vector 1 1 0)) :rpy (list 0 0 0)) (make-coords :pos (scale 1000 (float-vector 17.3 9 0)) :rpy (list 0 0 0))))

(defun atlas-init
  (&key (set-reset-pose nil) (set-user-mode t))
  (if (not (boundp '*ri*)) (setq *ri* (instance atlas-interface :init)))
  (if (not (boundp '*atlas*)) (setq *atlas* (instance atlas-robot :init)))
  ;; parameter for bdi_control
  (setf (get *atlas* :footstep-parameter)
       '(:default-half-offset #f(0.0 100.0 0.0) :forward-offset-length 180.0 :outside-offset-length 85.0 :rotate-rad 40.0))
  (send *ri* :set-soft-error-limit "all" 100) ;; disable SoftErrorLimiter
  (send *ri* :set-servo-error-limit "all" 0) ;; disable RobotHardware error limit
  (when set-user-mode
    (send *ri* :set-simulation-mode))
  (when set-reset-pose
    (cond
     ((keywordp set-reset-pose)
      (send *atlas* set-reset-pose))
     (t (send *atlas* :reset-pose)))
    ;; send reset pose to real robot
    (send *ri* :angle-vector (send *atlas* :angle-vector) 3000)
    (send *ri* :wait-interpolation))
  (setq *tfb* (instance ros::transform-broadcaster :init))
  )

#| ;; step is not supported in qualification
(defun publish-step (cds)
  (let ((msg (instance atlas_msgs::AtlasSimInterfaceCommand :init))
        (k-effort (instantiate string 28)))
    (send msg :behavior atlas_msgs::AtlasSimInterfaceCommand::*STEP*)
    ;;(send msg :step_params :use_demo_walk t)
    (send msg :step_params :desired_step :step_index 0)
    (send msg :step_params :desired_step :foot_index 0)  ;; :lleg -> 0 / :rleg -> 1
    (send msg :step_params :desired_step :duration 0.63)
    (send msg :step_params :desired_step :swing_height 0.3)
    (send msg :step_params :desired_step :pose (ros::coords->tf-pose cds))
    (fill k-effort 0)
    (send msg :k_effort k-effort)
    (send msg :header :stamp (ros::time+ (ros::time-now) (ros::time 1.0)))
    (ros::publish "atlas/atlas_sim_interface_command" msg)
    ))
(defun publish-step (cds &optional (baction bdi-action))
  (let ((walk-goal (send bdi-action :make-goal-instance))
        (goal (instance atlas_msgs::WalkDemoGoal :init))
        (k-effort (instantiate string 28)))

    (send goal :behavior atlas_msgs::WalkDemoGoal::*STEP*)
    ;;(send goal :step_params :use_demo_walk t)
    (send goal :step_params :desired_step :step_index 1)
    (send goal :step_params :desired_step :foot_index 0)  ;; :lleg -> 0 / :rleg -> 1
    (send goal :step_params :desired_step :duration 0.63)
    (send goal :step_params :desired_step :swing_height 0.3)
    (send goal :step_params :desired_step :pose (ros::coords->tf-pose cds))

    (fill k-effort 0)
    (send goal :k_effort k-effort)
    (send walk-goal :header :seq 1)
    (send walk-goal :header :stamp (ros::time-now))
    (send goal :header :stamp (send walk-goal :header :stamp))
    (send walk-goal :goal_id :stamp (send walk-goal :header :stamp))
    (send walk-goal :goal goal)
    (send bdi-action :send-goal walk-goal)
    ))
|#
