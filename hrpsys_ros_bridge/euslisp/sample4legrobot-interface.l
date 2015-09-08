(load "package://hrpsys_ros_bridge/euslisp/rtm-ros-robot-interface.l")
(require :sample4legrobot "package://hrpsys_ros_bridge/models/sample4legrobot.l")
(when (probe-file (ros::resolve-ros-path "package://hrpsys_ros_bridge/euslisp/sample4legrobot-utils.l"))
  (require :sample4legrobot-utils "package://hrpsys_ros_bridge/euslisp/sample4legrobot-utils.l"))

(defclass sample4legrobot-interface
  :super rtm-ros-robot-interface
  :slots ())
(defmethod sample4legrobot-interface
  (:init (&rest args)
         (send-super* :init :robot sample4legrobot-robot args)))

(defun sample4legrobot-init (&rest args)
  (if (not (boundp '*ri*))
      (setq *ri* (instance* sample4legrobot-interface :init args)))
  (if (not (boundp '*sample4legrobot*))
      (setq *sample4legrobot* (instance sample4legrobot-robot :init))))
