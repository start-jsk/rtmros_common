#!/usr/bin/env roseus

;; $ rtmlaunch hrpsys_ros_bridge samplerobot.launch
;; $ roseus samplerobot-sequence-player.l "(samplerobot-sequence-player-demo)"

(load "package://hrpsys_ros_bridge/euslisp/samplerobot-interface.l")

(defun samplerobot-sequence-player-init ()
  (samplerobot-init)
  t)

(defun samplerobot-sequence-player-demo0 ()
  "1. :angle-vector (setJointAngles) and :wait-interpolation (waitInterpolation)"
  (send *ri* :angle-vector (send *sr* :angle-vector) 700)
  (send *ri* :wait-interpolation)
  (eps= (distance (send *ri* :state :potentio-vector) (send *sr* :angle-vector)) 0.0)
  )

(defun samplerobot-sequence-player-demo1 ()
  "2. :set-interpolation-mode"
  (send *ri* :set-interpolation-mode :linear)
  (send *sr* :reset-manip-pose)
  (send *ri* :angle-vector (send *sr* :angle-vector) 500)
  (send *ri* :wait-interpolation)
  (send *ri* :set-interpolation-mode :hoffarbib)
  (send *sr* :reset-pose)
  (send *ri* :angle-vector (send *sr* :angle-vector) 500)
  (send *ri* :wait-interpolation)
  t)

(defun samplerobot-sequence-player-demo ()
  (samplerobot-sequence-player-init)
  (samplerobot-sequence-player-demo0)
  (samplerobot-sequence-player-demo1)
  )

(warn ";; (samplerobot-sequence-player-demo)~%")