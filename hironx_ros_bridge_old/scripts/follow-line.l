#!/usr/bin/env roseus

(load "package://hrpsys_ros_bridge/scripts/rtm-ros-robot-interface.l")
(require :kawadahironx "package://hrpsys_ros_bridge/models/kawada-hironx.l")

(defclass kawadahironx-interface
  :super robot-interface
  :slots ())
(defmethod kawadahironx-interface
  (:init (&rest args)
	  (send-super :init :robot hironx-robot)))

(load "models/arrow-object.l")

(ros::roseus "follow_line")

(if (not (boundp '*ri*))
    (setq *ri* (instance kawadahironx-interface :init)))
(setq *hiro* (instance hironx-robot :init))
(setq *arrow* (arrow))
(objects (list *hiro* *arrow*))

(defun test1 nil
  (dolist (c (list (make-coords :pos #f(350 200 0))
                   (make-coords :pos #f(450 200 0))
                   (make-coords :pos #f(450 0 0))
                   (make-coords :pos #f(350 0 0))
                   ))
    (send *arrow* :newcoords c)
    (send *irtviewer* :draw-objects) ;; update viewer 
    (unix::usleep 500000)
    (send *hiro* :larm :inverse-kinematics *arrow* :move-target (send *hiro* :larm :end-coords) :rotation-axis :z)
    (send *irtviewer* :draw-objects) ;; update viewer 
    ;; send to robot
    (send *ri* :angle-vector (send *hiro* :angle-vector) 3000)
    (send *ri* :wait-interpolation)
    (unix::usleep 500000)
    (x::window-main-one)
    )
  )
;; (test1)

(defun test2 nil
  (setq c1 (send *hiro* :larm :end-coords :copy-worldcoords))
  (dolist (c2 (list (make-coords :pos #f(350 200 0))
                    (make-coords :pos #f(450 200 0))
                    (make-coords :pos #f(450 0 0))
                    (make-coords :pos #f(350 0 0))
                    (make-coords :pos #f(350 200 0))
                    ))
    (setq avs nil tms nil)
    (dotimes (i 10)
      (send *arrow* :newcoords (midcoords (/ i 10.0) c1 c2))
      (send *irtviewer* :draw-objects) ;; update viewer 
      (send *hiro* :larm :inverse-kinematics *arrow* :move-target (send *hiro* :larm :end-coords) :rotation-axis :z)
      (send *irtviewer* :draw-objects) ;; update viewer 
      (push (send *hiro* :angle-vector) avs)
      (push 300 tms))
    (send *ri* :angle-vector-sequence (reverse avs) (reverse tms))
    (send *ri* :wait-interpolation)
    (setq c1 c2)
    ))
(test2)


