(require :rtm-ros-robotics-interface "package://hrpsys_ros_bridge/euslisp/rtm-ros-robot-interface.l")
(require :atlas "package://hrpsys_gazebo/models/atlas.l")
(ros::load-ros-manifest "hrpsys_gazebo")

(ros::roseus "atlas_interface")

(defmethod atlas-robot
  (:init-ending ()
   (prog1
       (send-super :init-ending)
     ;; (setq larm-root-link (send self :larm :shoulder-y :child-link))
     ;; (setq rarm-root-link (send self :rarm :shoulder-y :child-link))
     )
   ))

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
     ;; do not use in Qualification
     (unless qualify
       (ros::advertise "atlas/mode" std_msgs::String 1)
       (ros::advertise "atlas/set_pose" geometry_msgs::Pose 1))
     ;; action lib / needs node of atlas_msgs/actionlib_server
     (setq bdi-action
           (instance ros::simple-action-client :init
                     "atlas/bdi_control" atlas_msgs::WalkDemoAction))
     (send bdi-action :wait-for-server 3)
;     (send self :add-controller :larm-controller)
;     (send self :add-controller :rarm-controller)
;     (send self :add-controller :head-controller)
;     (send self :add-controller :torso-controller)
     (setf (get robot :footstep-parameter)
           '(:default-half-offset #f(0.0 100.0 0.0) :forward-offset-length 180.0 :outside-offset-length 85.0 :rotate-rad 40.0))
     (setq walk-seq 0)
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
  (:bdi_footstep
   (footstep-list &key (step-duration 0.63) (swing-height 0.3) &allow-other-keys)
   ;;(send_atlas_control_mode "Walk")
   (let* ((fstep (car footstep-list))
          (offset (copy-object (send fstep :worldpos))))
     (setf (elt offset 1) 0.0)
     (scale -1.0 offset offset)
     (instance atlas_msgs::WalkDemoActionGoal :init)
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
  (:bdi-go-pos (x y &optional (theta 0) &key (r robot)) ;; x [m],  y[m], theta [rad]
   (send self :bdi_footstep
         (send robot :go-pos-params->footstep-list (* x 1000) (* y 1000) theta)))
  (:teleport-to (coords)
   (unix::usleep (* 100 1000))
   (send self :atlas-mode "harnessed")
   (send self :control-mode "Freeze")
   (send self :control-mode "StandPrep")
   (unix::usleep (* 2 1000 1000))
   (send self :atlas-mode "nominal")
   (unix::usleep (* 100 1000))
   (ros::publish "atlas/set_pose" (ros::coords->tf-pose coords))
   (unix::usleep (* 200 1000))
   (send self :control-mode "Stand"))
  )

(defun atlas-init
  (&key (set-reset-pose t))
  (if (not (boundp '*ri*)) (setq *ri* (instance atlas-interface :init)))
  (if (not (boundp '*atlas*)) (setq *atlas* (instance atlas-robot :init)))
  ;; parameter for bdi_control
  (setf (get *atlas* :footstep-parameter)
        '(:default-half-offset #f(0.0 100.0 0.0) :forward-offset-length 180.0 :outside-offset-length 85.0 :rotate-rad 40.0))
  (send_atlas_control_mode "User")
  (send *ri* :set-soft-error-limit "all" 100) ;; disable SoftErrorLimiter
  (send *ri* :set-servo-error-limit "all" 0) ;; disable RobotHardware error limit
  (when set-reset-pose
    (send *atlas* :reset-pose)
    ;; send reset pose to real robot
    (send *ri* :angle-vector (send *atlas* :angle-vector) 3000)
    (send *ri* :wait-interpolation))
  )

(defun atlas-tool-look ()
  (send *atlas* :head :neck-p :joint-angle 50)
  (send *ri* :angle-vector (send *atlas* :angle-vector) 3000)
  (send *ri* :wait-interpolation)
  )

(defun atlas-tool-grasp ()
  (send *atlas* :larm :shoulder-r :joint-angle -50)
  (send *atlas* :larm :shoulder-y :joint-angle  40)
  (send *atlas* :larm :elbow-p    :joint-angle  90)
  (send *atlas* :larm :elbow-y    :joint-angle  90)
  (send *atlas* :larm :wrist-y    :joint-angle   0)
  (send *atlas* :larm :wrist-p    :joint-angle -20)

  (send *atlas* :rarm :shoulder-r :joint-angle  50)
  (send *atlas* :rarm :shoulder-y :joint-angle  40)
  (send *atlas* :rarm :elbow-p    :joint-angle -90)
  (send *atlas* :rarm :elbow-y    :joint-angle  90)
  (send *atlas* :rarm :wrist-y    :joint-angle   0)
  (send *atlas* :rarm :wrist-p    :joint-angle -20)

  ;; (send *atlas* :legs :knee-p :joint-angle 30)
  ;; (send *atlas* :legs :crotch-p :joint-angle -15)
  ;; (send *atlas* :legs :ankle-p :joint-angle -15)
  ;; (send *atlas* :move-centroid-on-foot :both '(:rleg :lleg :rarm :larm))

  (send *ri* :angle-vector (send *atlas* :angle-vector) 20000)
  (send *ri* :wait-interpolation)

  (send *atlas* :rarm :inverse-kinematics (make-coords :pos #f(370 -350 200)) :rotation-axis :z)
;;  (send *atlas* :move-centroid-on-foot :both '(:rleg :lleg :rarm :larm))
  (send *ri* :angle-vector (send *atlas* :angle-vector) 15000)
  (send *ri* :wait-interpolation)

  (send *ri* :start-grasp :rarm)
  (unix:usleep 3)

  (send *atlas* :rarm :inverse-kinematics (make-coords :pos #f(550 0 400)) :rotation-axis :z :use-torso t)
;;  (send *atlas* :move-centroid-on-foot :both '(:rleg :lleg :rarm :larm))
  (send *ri* :angle-vector (send *atlas* :angle-vector) 15000)
  (send *ri* :wait-interpolation)

  (send *ri* :stop-grasp :rarm)
  (unix:usleep 3)
  )

(defun atlas-walk-pattern-gen
  (&key (no-gen nil) (fname nil))
  ;; limit crotch-y range which are originally inf and -inf
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

(defun atlas-walk ()
  (send_atlas_control_mode "Stand")
  (atlas-init :set-reset-pose nil)
  (objects (list *atlas*))
  (send *atlas* :angle-vector (send *ri* :state :potentio-vector))
  (send *ri* :angle-vector (send *atlas* :angle-vector) 2000)
  (send_atlas_control_mode "User")
  (atlas-walk-pattern-gen)
  )

(defun send_atlas_control_mode
  (&optional (mode "Stand"))
  (let ((msg (instance std_msgs::String :init :data mode)))
    (ros::publish "atlas/atlas_sim_interface_command" msg)))

;; do not use in Qualification
(defun send_atlas_mode
  (&optional (mode "nominal"))
  (let ((msg (instance std_msgs::String :init :data mode)))
    (ros::publish "atlas/mode" msg)))
