(load "package://hrpsys_ros_bridge/euslisp/rtm-ros-robot-interface.l")
(require :kawadahironx "package://hrpsys_ros_bridge_tutorials/models/kawada-hironx.l")

(defclass kawadahironx-interface
  :super rtm-ros-robot-interface
  :slots ())
(defmethod kawadahironx-interface
  (:init (&rest args)
	  (send-super :init :robot hironx-robot)))

(defun kawada-hironx-init ()
  (if (not (boundp '*ri*))
      (setq *ri* (instance kawadahironx-interface :init)))
  (if (not (boundp '*hiro*))
      (setq *hiro* (instance hironx-robot :init))))
