(require :robot-interface "package://pr2eus/euslisp/robot-interface.l")
(require :sandia_hand_left "package://hrpsys_gazebo/models/sandia_hand_left.l")

(defclass sandia-hand-interface
  :super robot-interface
  :slots (l/r))
(defmethod sandia-hand-interface
  (:init (&optional (l/r_ :larm) (g "hand_group"))
   (setq l/r l/r_)
   (let ((gname))
     (when g
       (case l/r
         (:larm
          (setq gname (format nil "left_~A" g)))
         (:rarm
          (setq gname (format nil "right_~A" g)))))
     (cond
      (gname (send-super :init :robot  :groupname gname))
      (t (send-super :init :robot r))))
   self)
  (:left-default-controller ()
   (list
    (list
     (cons :controller-action "/sandia_hand_left/sandia_hand_left_controller/joint_trajectory_action")
     (cons :controller-state "/sandia_hand_left/sandia_hand_left_controller/state")
     (cons :action-type pr2_controllers_msgs::JointTrajectoryAction)
     (cons :joint-names (mapcar #'(lambda (n) (if (symbolp n) (symbol-name n) n)) (send-all (send robot :joint-list) :name))))))
  (:right-ult-controller ()
   (list
    (list
     (cons :controller-action "/sandia_hand_left/sandia_hand_left_controller/joint_trajectory_action")
     (cons :controller-state "/sandia_hand_left/sandia_hand_left_controller/state")
     (cons :action-type pr2_controllers_msgs::JointTrajectoryAction)
     (cons :joint-names (mapcar #'(lambda (n) (if (symbolp n) (symbol-name n) n)) (send-all (send robot :joint-list) :name))))))
  (:default-controller ()
   (case l/r
     (:larm
      (send self :left-defualt-controller))
     (:rarm
      (send self :right-defualt-controller))))
  )
