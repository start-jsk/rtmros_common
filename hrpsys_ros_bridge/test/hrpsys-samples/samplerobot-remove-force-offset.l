#!/usr/bin/env roseus

;; $ rtmlaunch hrpsys_ros_bridge samplerobot.launch
;; $ roseus samplerobot-remove-force-offset.l "(samplerobot-remove-force-offset-demo)"

(load "package://hrpsys_ros_bridge/euslisp/samplerobot-interface.l")

(defun samplerobot-remove-force-offset-init ()
  (samplerobot-init)
  ;; initial pose
  (send *sr* :reset-pose)
  (send *ri* :angle-vector (send *sr* :angle-vector) 1000)
  (send *ri* :wait-interpolation)
  t)

(defun samplerobot-remove-force-offset-demo0 ()
  "1. force and moment are large because of link offsets"
  ;; check force moment values
  (let ((rfm (norm (send *ri* :state :off-force-vector :rarm)))
        (lfm (norm (send *ri* :state :off-force-vector :larm))))
    (format t "no-offset-removed force moment (rhsensor) ~A => ~A~%" rfm (> rfm 1e-2))
    (format t "no-offset-removed force moment (lhsensor) ~A => ~A~%" lfm (> lfm 1e-2))
    (and (> rfm 1e-2) (> lfm 1e-2))
    ))

(defun samplerobot-remove-force-offset-demo1 ()
  "2. Set link offsets
      link_offset_centroid and link_offset_mass are identified value."
  (send *ri* :set-forcemoment-offset-param :rarm :force-offset #f(0 0 0) :moment-offset #f(0 0 0) :link-offset-centroid #f(0 0.0368 -0.076271) :link-offset-mass 0.800011)
  (send *ri* :set-forcemoment-offset-param :larm :force-offset #f(0 0 0) :moment-offset #f(0 0 0) :link-offset-centroid #f(0 -0.0368 -0.076271) :link-offset-mass 0.800011)
  (let ((rarm-ok (= (send (send *ri* :get-forcemoment-offset-param :rarm) :link_offset_mass) 0.800011))
        (larm-ok (= (send (send *ri* :get-forcemoment-offset-param :larm) :link_offset_mass) 0.800011)))
    (if rarm-ok (print "getForceMomentOffsetParam(\"rhsensor\") => OK"))
    (if larm-ok (print "getForceMomentOffsetParam(\"lhsensor\") => OK"))
    (and rarm-ok larm-ok)))

(defun samplerobot-remove-force-offset-demo2 ()
  "3. force and moment are reduced"
  ;; wait for 200ms instead of sleep
  (send *ri* :angle-vector (send *sr* :angle-vector) 200)
  (send *ri* :wait-interpolation)
  ;; check force moment values
  (let ((rfm (norm (send *ri* :state :off-force-vector :rarm)))
        (lfm (norm (send *ri* :state :off-force-vector :larm))))
    (format t "offset-removed force moment (rhsensor) ~A => ~A~%" rfm (< rfm 1e-2))
    (format t "offset-removed force moment (lhsensor) ~A => ~A~%" lfm (< lfm 1e-2))
    (and (< rfm 1e-2) (< lfm 1e-2))))

(defun samplerobot-remove-force-offset-demo3 ()
  "4. dump and load parameter file"
  (send *ri* :dump-forcemoment-offset-params "/tmp/test-rmfo" :set-robot-date-string nil)
  (send *ri* :load-forcemoment-offset-params "/tmp/test-rmfo")
  )

(defun samplerobot-remove-force-offset-demo ()
  (samplerobot-remove-force-offset-init)
  (samplerobot-remove-force-offset-demo0)
  (samplerobot-remove-force-offset-demo1)
  (samplerobot-remove-force-offset-demo2)
  (samplerobot-remove-force-offset-demo3)
  )

(warn ";; (samplerobot-remove-force-offset-demo)~%")
