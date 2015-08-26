#!/usr/bin/env roseus

;; $ rtmlaunch hrpsys_ros_bridge samplerobot.launch
;; $ roseus samplerobot-collision-detector.l "(samplerobot-collision-detector-demo)"

(load "package://hrpsys_ros_bridge/euslisp/samplerobot-interface.l")

(defun samplerobot-collision-detector-init ()
  (samplerobot-init)
  (send *sr* :reset-pose)
  (send *ri* :angle-vector (send *sr* :angle-vector) 2000)
  (send *ri* :wait-interpolation)
  t)

(defun samplerobot-collision-detector-demo0 ()
  "1. CollisionCheck in safe pose"
  (send *sr* :reset-pose)
  (send *ri* :angle-vector (send *sr* :angle-vector) 2000)
  (send *ri* :wait-interpolation)
  (if (send (send *ri* :get-collision-status) :safe_posture)
      (print "Safe pose"))
  t)

(defun samplerobot-collision-detector-demo1 ()
  "2. CollisionCheck in fail pose"
  (send *sr* :reset-pose)
  (send *sr* :rarm :move-end-pos (float-vector -50 150 50) :world :rotation-axis nil)
  (send *ri* :angle-vector (send *sr* :angle-vector) 1000)
  (send *ri* :wait-interpolation)
  (unless (send (send *ri* :get-collision-status) :safe_posture)
    (print "Successfully stop fail pose"))
  (send *sr* :reset-pose)
  (send *ri* :angle-vector (send *sr* :angle-vector) 3000)
  (send *ri* :wait-interpolation)
  (if (send (send *ri* :get-collision-status) :safe_posture)
    (print "Successfully return to safe pose"))
  t)

(defun samplerobot-collision-detector-demo2 ()
  "3. CollisionCheck in fail pose with 0.1[m] tolerance"
  (send *ri* :set-tolerance :tolerance 0.1)
  (send *sr* :reset-pose)
  (send *sr* :rarm :move-end-pos (float-vector -50 150 50) :world :rotation-axis nil)
  (send *ri* :angle-vector (send *sr* :angle-vector) 1000)
  (send *ri* :wait-interpolation)
  (unless (send (send *ri* :get-collision-status) :safe_posture)
    (print "Successfully stop fail pose (0.1[m] tolerance)"))
  (send *sr* :reset-pose)
  (send *ri* :angle-vector (send *sr* :angle-vector) 3000)
  (send *ri* :wait-interpolation)
  (if (send (send *ri* :get-collision-status) :safe_posture)
    (print "Successfully return to safe pose"))
  (send *ri* :set-tolerance :tolerance 0.0)
  t)

(defun samplerobot-collision-detector-demo ()
  (samplerobot-collision-detector-init)
  (samplerobot-collision-detector-demo0)
  (samplerobot-collision-detector-demo1)
  (samplerobot-collision-detector-demo2)
  )

(warn ";; (samplerobot-collision-detector-demo)~%")