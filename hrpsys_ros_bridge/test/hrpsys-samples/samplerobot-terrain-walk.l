#!/usr/bin/env roseus

;; for SlopeUpDown
;; $ rtmlaunch hrpsys_ros_bridge samplerobot.launch PROJECT_FILE:=`rospack find hrpsys`/share/hrpsys/samples/SampleRobot/SampleRobot.TerrainFloor.SlopeUpDown.xml
;; $ roseus samplerobot-terrain-walk.l "(demo-slope-updown)"
;; for StairUp
;; $ rtmlaunch hrpsys_ros_bridge samplerobot.launch PROJECT_FILE:=`rospack find hrpsys`/share/hrpsys/samples/SampleRobot/SampleRobot.TerrainFloor.StairUp.xml
;; $ roseus samplerobot-terrain-walk.l "(demo-stair-up)"
;; for StairDown
;; $ rtmlaunch hrpsys_ros_bridge samplerobot.launch PROJECT_FILE:=`rospack find hrpsys`/share/hrpsys/samples/SampleRobot/SampleRobot.TerrainFloor.StairDown.xml
;; $ roseus samplerobot-terrain-walk.l "(demo-stair-down)"


(load "package://hrpsys_ros_bridge/euslisp/samplerobot-interface.l")

(defun init ()
  (samplerobot-init)
  )

(defun init-pose ()
  (send *sr* :reset-pose)
  (send *sr* :legs :move-end-pos #f(0 0 70))
  (send *ri* :angle-vector (send *sr* :angle-vector) 1000)
  (send *ri* :wait-interpolation)
  )
(init)
(init-pose)

(defun setup-gait-generator-param (&optional (use-rectangle t))
  (send* *ri* :set-gait-generator-param
         :default-double-support-ratio 0.3
         :default-step-time 1.2
         :swing-trajectory-delay-time-offset 0.2
         (if use-rectangle
             (list :default-orbit-type :RECTANGLE)
           (list
            :stair-trajectory-way-point-offset #f(0.04 0 0)
            :default-orbit-type :STAIR)))
  )

(defun stair-walk (&optional (stair-height (* 1e3 0.1524)))
  (let ((stair-stride-x (* 1e3 0.25))
        (floor-stride-x (* 1e3 0.157))
        (init-step-x 0)
        (init-step-z 0)
        (fs))
    (dolist (step-idx (list 1 2 3 4))
      (setup-gait-generator-param (= (mod step-idx 2) 1))
      (setq fs (list (make-coords :pos (float-vector init-step-x -90 init-step-z) :name :rleg)
                     (make-coords :pos (float-vector (+ init-step-x stair-stride-x) 90 (+ init-step-z stair-height)) :name :lleg)))
      (send *ri* :set-foot-steps fs)
      (send *ri* :wait-foot-steps)
      (setq fs (list (make-coords :pos (float-vector (+ init-step-x stair-stride-x) 90 (+ init-step-z stair-height)) :name :lleg)
                     (make-coords :pos (float-vector (+ init-step-x stair-stride-x) -90 (+ init-step-z stair-height)) :name :rleg)))
      (send *ri* :set-foot-steps fs)
      (send *ri* :wait-foot-steps)
      (setq fs (list (make-coords :pos (float-vector (+ init-step-x stair-stride-x) 90 (+ init-step-z stair-height)) :name :lleg)
                     (make-coords :pos (float-vector (+ init-step-x stair-stride-x floor-stride-x) -90 (+ init-step-z stair-height)) :name :rleg)
                     (make-coords :pos (float-vector (+ init-step-x stair-stride-x floor-stride-x) 90 (+ init-step-z stair-height)) :name :lleg)))
      (send *ri* :set-foot-steps fs)
      (send *ri* :wait-foot-steps)
      )
    ))

(defun demo-slope-updown ()
  (print "Start stlop up down")
  (let ((fs-list (list (make-coords :pos (scale 1e3 #f(0.8 -0.09 0.0)) :rot (quaternion2matrix (float-vector 1.0 0.0 2.775558e-17 0.0)) :name :rleg)
                       (make-coords :pos (scale 1e3 #f(1.0953 0.09 0.030712)) :rot (quaternion2matrix (float-vector 0.991445 0.0 -0.130526 0.0)) :name :lleg)
                       (make-coords :pos (scale 1e3 #f(1.28848 -0.09 0.082475)) :rot (quaternion2matrix (float-vector 0.991445 0.0 -0.130526 0.0)) :name :rleg)
                       (make-coords :pos (scale 1e3 #f(1.38508 0.09 0.108357)) :rot (quaternion2matrix (float-vector 0.991445 0.0 -0.130526 0.0)) :name :lleg)
                       (make-coords :pos (scale 1e3 #f(1.38508 -0.09 0.108357)) :rot (quaternion2matrix (float-vector 0.991445 0.0 -0.130526 0.0)) :name :rleg)
                       (make-coords :pos (scale 1e3 #f(1.54959 0.09 0.125863)) :rot (quaternion2matrix (float-vector 0.991445 0.0 0.130526 0.0)) :name :lleg)
                       (make-coords :pos (scale 1e3 #f(1.74277 -0.09 0.074099)) :rot (quaternion2matrix (float-vector 0.991445 0.0 0.130526 0.0)) :name :rleg)
                       (make-coords :pos (scale 1e3 #f(1.79107 0.09 0.061158)) :rot (quaternion2matrix (float-vector 0.991445 0.0 0.130526 0.0)) :name :lleg)
                       (make-coords :pos (scale 1e3 #f(2.05 -0.09 0.0)) :rot (quaternion2matrix (float-vector 1.0 0.0 0.0 0.0)) :name :rleg)
                       (make-coords :pos (scale 1e3 #f(2.05 0.09 0.0)) :rot (quaternion2matrix (float-vector 1.0 0.0 0.0 0.0)) :name :lleg))))
    (setup-gait-generator-param)
    (send *ri* :start-auto-balancer)
    (dotimes (fs-idx (- (length fs-list) 1))
      (send *ri* :set-foot-steps (list (elt fs-list fs-idx) (elt fs-list (+ 1 fs-idx))))
      (send *ri* :wait-foot-steps)
      )
    (send *ri* :stop-auto-balancer)
    ))

(defun demo-Stair-Up ()
  (print "Start stair up")
  (setup-gait-generator-param)
  (send *ri* :start-auto-balancer)
  (stair-walk)
  (send *ri* :stop-auto-balancer)
  )

(defun demo-Stair-Down ()
  (print "Start stair down")
  (setup-gait-generator-param)
  (send *ri* :start-auto-balancer)
  (stair-walk (* 1e3 -0.1524))
  (send *ri* :stop-auto-balancer)
  )
