(load "package://hrpsys_ros_bridge/euslisp/rtm-ros-robot-interface.l")
(require :samplerobot "package://hrpsys_ros_bridge_tutorials/models/samplerobot.l")

(defclass samplerobot-interface
  :super rtm-ros-robot-interface
  :slots ())
(defmethod samplerobot-interface
  (:init (&rest args)
         (send-super :init :robot samplerobot-robot)))

(defun samplerobot-init ()
  (if (not (boundp '*ri*))
      (setq *ri* (instance samplerobot-interface :init)))
  (if (not (boundp '*sr*))
      (setq *sr* (instance samplerobot-robot :init)))
  )
