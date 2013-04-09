(load "package://hrpsys_ros_bridge/euslisp/rtm-ros-robot-interface.l")
(require :hrp4c "package://hrpsys_gazebo/models/atlas.l")

(defclass atlas-interface
  :super rtm-ros-robot-interface
  :slots ())
(defmethod atlas-interface
  (:init ()
   (prog1 (send-super :init :robot atlas-robot)
;     (send self :add-controller :larm-controller)
;     (send self :add-controller :rarm-controller)
;     (send self :add-controller :head-controller)
;     (send self :add-controller :torso-controller)
     ))
  )

(defun atlas-init ()
  (if (not (boundp '*ri*)) (setq *ri* (instance atlas-interface :init)))
  (if (not (boundp '*atlas*)) (setq *atlas* (instance atlas-robot :init)))
  (send *atlas* :reset-pose)
  ;; send reset pose to real robot
  (send *ri* :angle-vector (send *atlas* :angle-vector) 3000)
  (send *ri* :wait-interpolation)
  )






