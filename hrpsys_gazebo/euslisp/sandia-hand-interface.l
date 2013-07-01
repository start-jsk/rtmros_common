(require :robot-interface "package://pr2eus/robot-interface.l")
(require :sandia_hand_left  "package://hrpsys_gazebo/models/sandia_hand_left.l")
(require :sandia_hand_right "package://hrpsys_gazebo/models/sandia_hand_right.l")

(setq ros::*compile-message* t)
(ros::roseus-add-msgs "osrf_msgs")
(ros::roseus-add-srvs "roseus")
(setq ros::*compile-message* nil)

(defclass sandia-hand-interface
  :super robot-interface
  :slots (l/r service-name command-topic))

(defmethod sandia-hand-interface
  (:init (&optional (l/r_ :larm) (g "hand_group"))
   (let (gname lrobot topic ns)
     (setq l/r l/r_)
     (case l/r
       (:larm
        (setq service-name "/sandia_hand_left/controller_manager/query")
        (if g (setq gname (format nil "left_~A" g)))
        (setq lrobot (sandia_hand_left))
        (setq ns "sandia_hand_left")
        (setq command-topic "/sandia_hands/l_hand/joint_commands"))
       (:rarm
        (setq service-name "/sandia_hand_right/controller_manager/query")
        (if g (setq gname (format nil "right_~A" g)))
        (setq lrobot (sandia_hand_right))
        (setq ns "sandia_hand_right")
        (setq command-topic "/sandia_hands/r_hand/joint_commands"))
        )
     (ros::advertise command-topic osrf_msgs::JointCOmmands)
     (cond
      (gname (send-super :init :use-tf2 t :namespace ns
                         :robot lrobot :groupname gname))
      (t (send-super :init :use-tf2 t :namespace ns
                     :robot lrobot)))
     self))
  (:left-default-controller ()
   (list
    (list
     (cons :controller-action "sandia_hand_left_controller/joint_trajectory_action")
     (cons :controller-state "sandia_hand_left_controller/state")
     (cons :action-type pr2_controllers_msgs::JointTrajectoryAction)
     (cons :joint-names (mapcar #'(lambda (n) (if (symbolp n) (symbol-name n) n)) (send-all (send robot :joint-list) :name))))))
  (:right-default-controller ()
   (list
    (list
     (cons :controller-action "sandia_hand_right_controller/joint_trajectory_action")
     (cons :controller-state "sandia_hand_right_controller/state")
     (cons :action-type pr2_controllers_msgs::JointTrajectoryAction)
     (cons :joint-names (mapcar #'(lambda (n) (if (symbolp n) (symbol-name n) n)) (send-all (send robot :joint-list) :name))))))
  (:default-controller ()
   (case l/r
     (:larm
      (send self :left-default-controller))
     (:rarm
      (send self :right-default-controller))))
  (:wait-interpolation
   (&rest args)
   (let ((req (instance roseus::stringstringrequest :init :str "stop"))
         ret)
     (prog1
         (send-super* :wait-interpolation args)
       (ros::service-call service-name req))
     ))
  (:angle-vector
   (&rest args)
   (let ((req (instance roseus::stringstringrequest :init :str "start"))
         ret)
     (setq ret (ros::service-call service-name req))
     (unix::usleep (* 100 1000)) ;; 100 ms
     (send-super* :angle-vector args)
     ))
  (:send_query (&optional (str "stop"))
   (let ((req (instance roseus::stringstringrequest :init :str str))
         ret)
     (setq ret (ros::service-call service-name req))
     ret))
  (:send-command (&key (pgain) (dgain) (angle-vector))
   (let ((msg (instance osrf_msgs::JointCommands :init))
         (jlst (send robot :joint-list)))
     ;;(send msg :header :stamp (ros::time-now))
     (send msg :name (mapcar #'(lambda (j) (send j :name)) jlst))
     (when angle-vector
       (send msg :position angle-vector))
     (when pgain
       (send msg :kp_position (instantiate float-vector (length jlst)))
       (fill (send msg :kp_position) pgain))
     (when dgain
       (send msg :kd_position (instantiate float-vector (length jlst)))
       (fill (send msg :kd_position) dgain))
     (ros::publish command-topic msg)
     ))
  )
