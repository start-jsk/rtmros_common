(load "package://hrpsys_ros_bridge/euslisp/rtm-ros-robot-interface.l")
(require :hrp4c "package://hrpsys_ros_bridge_tutorials/models/hrp4c.l")

(defclass hrp4c-interface
  :super rtm-ros-robot-interface
  :slots ())
(defmethod hrp4c-interface
  (:init (&rest args)
   (prog1 (send-super :init :robot hrp4-robot)
     (send self :add-controller :larm-controller)
     (send self :add-controller :rarm-controller)
     (send self :add-controller :head-controller)
     (send self :add-controller :torso-controller)
     ))
  (:larm-controller ()
   (list
    (list
     (cons :controller-action "larm_controller/joint_trajectory_action")
     (cons :controller-state "larm_controller/state")
     (cons :action-type pr2_controllers_msgs::JointTrajectoryAction)
     (cons :joint-names (list "L_SHOULDER_P" "L_SHOULDER_R" "L_SHOULDER_Y" "L_ELBOW_P"
                              "L_WRIST_Y" "L_WRIST_R" "L_HAND_J0" "L_HAND_J1")))))
  (:rarm-controller ()
   (list
    (list
     (cons :controller-action "rarm_controller/joint_trajectory_action")
     (cons :controller-state "rarm_controller/state")
     (cons :action-type pr2_controllers_msgs::JointTrajectoryAction)
     (cons :joint-names (list "R_SHOULDER_P" "R_SHOULDER_R" "R_SHOULDER_Y" "R_ELBOW_P"
                              "R_WRIST_Y" "R_WRIST_R" "R_HAND_J0" "R_HAND_J1")))))
  (:head-controller ()
   (list
    (list
     (cons :controller-action "head_controller/joint_trajectory_action")
     (cons :controller-state "head_controller/state")
     (cons :action-type pr2_controllers_msgs::JointTrajectoryAction)
     (cons :joint-names (list "NECK_Y" "NECK_R" "NECK_P")))))
  (:torso-controller ()
   (list
    (list
     (cons :controller-action "torso_controller/joint_trajectory_action")
     (cons :controller-state "torso_controller/state")
     (cons :action-type pr2_controllers_msgs::JointTrajectoryAction)
     (cons :joint-names (list "CHEST_P" "CHEST_R" "CHEST_Y")))))
  )

(defun hrp4c-init ()
  (if (not (boundp '*ri*))
      (setq *ri* (instance hrp4c-interface :init)))
  (if (not (boundp '*hrp4c*))
      (setq *hrp4c* (instance hrp4-robot :init)))
  )
