#!/usr/bin/env roseus

;; $ rtmlaunch hrpsys_ros_bridge_tutorials samplerobot.launch
;; $ roseus samplerobot-remove-force-offset.l "(demo)"

(load "package://hrpsys_ros_bridge_tutorials/euslisp/samplerobot-interface.l")

(defun init ()
  (samplerobot-init)
  )

(defun demo ()
  (init)
  (send *sr* :reset-pose)
  (send *ri* :angle-vector (send *sr* :angle-vector) 1000)
  (send *ri* :wait-interpolation)

  ;; 1. force and moment are large because of link offsets
  (let ((fm (norm (send *ri* :state :off-force-vector :rarm))))
    (format t "no-offset-removed force moment (rhsensor) ~A => ~A~%" fm (> fm 1e-2)))
  (let ((fm (norm (send *ri* :state :off-force-vector :larm))))
    (format t "no-offset-removed force moment (lhsensor) ~A => ~A~%" fm (> fm 1e-2)))
  ;; 2. Set link offsets
  ;;    link_offset_centroid and link_offset_mass are identified value.
  (send *ri* :set-forcemoment-offset-param :rarm :force-offset #f(0 0 0) :moment-offset #f(0 0 0) :link-offset-centroid #f(0 0.0368 -0.076271) :link-offset-mass 0.800011)
  (send *ri* :set-forcemoment-offset-param :larm :force-offset #f(0 0 0) :moment-offset #f(0 0 0) :link-offset-centroid #f(0 -0.0368 -0.076271) :link-offset-mass 0.800011)
  (if (= (send (send *ri* :get-forcemoment-offset-param :rarm) :link_offset_mass) 0.800011)
      (print "getForceMomentOffsetParam(\"rhsensor\") => OK"))
  (if (= (send (send *ri* :get-forcemoment-offset-param :larm) :link_offset_mass) 0.800011)
      (print "getForceMomentOffsetParam(\"lhsensor\") => OK"))
  ;; 3. force and moment are reduced
  (let ((fm (norm (send *ri* :state :off-force-vector :rarm))))
    (format t "no-offset-removed force moment (rhsensor) ~A => ~A~%" fm (< fm 1e-2)))
  (let ((fm (norm (send *ri* :state :off-force-vector :larm))))
    (format t "no-offset-removed force moment (lhsensor) ~A => ~A~%" fm (< fm 1e-2)))
  )

;;(demo)