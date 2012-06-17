(ros::roseus "kawadahironx_pickbox")
(load "package://hrpsys_ros_bridge/scripts/rtm-ros-robot-interface.l")
(require :kawadahironx "package://hrpsys_ros_bridge/models/kawada-hironx.l")

(defclass kawadahironx-interface
  :super rtm-ros-robot-interface
  :slots (r-gripper-action l-gripper-action))
(defmethod kawadahironx-interface
  (:init (&rest args)
   (setq r-gripper-action (instance ros::simple-action-client :init
                                    "/r_gripper_controller/gripper_action"
                    pr2_controllers_msgs::Pr2GripperCommandAction)
	 l-gripper-action (instance ros::simple-action-client :init
                                    "/l_gripper_controller/gripper_action"
                    pr2_controllers_msgs::Pr2GripperCommandAction))
   (send r-gripper-action :wait-for-server 3)
   (send l-gripper-action :wait-for-server 3)
   (send-super :init :robot hironx-robot))
  (:move-gripper
   (arm pos &key (effort 25) (wait t))
   (let* (goal
          (clients (case arm
                     (:rarm (list r-gripper-action))
                     (:larm (list l-gripper-action))
                     (:arms (list r-gripper-action l-gripper-action))
                     (t (ros::ros-warn "arm[~a] in :move-gripper is invalid type" arm)))))
     (dolist (client clients)
       (setq goal (instance pr2_controllers_msgs::Pr2GripperCommandActionGoal :init))
       (send goal :goal :command :position pos)
       (send goal :goal :command :max_effort effort)
       (send client :send-goal goal))
     (if wait (send-all clients :wait-for-result))
     ))
  )

(setq *ri* (instance kawadahironx-interface :init))
(setq *hiro* (instance hironx-robot :init))

(defmethod hironx-robot
  (:off-pose ()
   (send self :angle-vector (float-vector 0.0 -140.0 -158.0 0.0 0.0 0.0 0.0 -140.0 -158.0 0.0 0.0 0.0 0.0 0.0 0.0))))

(dotimes (i 10)
  (send *hiro* :reset-pose)
  ;;(objects (list *hiro*))
  (send *ri* :angle-vector (send *hiro* :angle-vector) 3000)
  (send *ri* :wait-interpolation)
  (send *ri* :move-gripper :rarm 0.07 :wait t)
  (send *hiro* :rarm :shoulder-p :joint-angle -40)
  (send *hiro* :rarm :elbow-p :joint-angle -130)
  (send *hiro* :head :neck-y :joint-angle -20)
  ;;(objects (list *hiro*))
  (send *ri* :angle-vector (send *hiro* :angle-vector) 3000)
  (send *ri* :wait-interpolation)
  (send *ri* :move-gripper :rarm 0.02 :wait t)
  )
(warning-message 2 ";; type (exit) to finish program~%")
