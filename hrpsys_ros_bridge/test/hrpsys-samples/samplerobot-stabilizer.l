#!/usr/bin/env roseus

;; $ rtmlaunch hrpsys_ros_bridge samplerobot.launch
;; $ roseus samplerobot-stabilizer.l "(samplerobot-stabilizer-demo)"

(load "package://hrpsys_ros_bridge/euslisp/samplerobot-interface.l")

(defun samplerobot-stabilizer-init ()
  (samplerobot-init)
  (setq *robot* *sr*)
  (send *robot* :reset-pose)
  (send *ri* :start-auto-balancer)
  (send *ri* :angle-vector (send *robot* :angle-vector) 1000)
  (send *ri* :wait-interpolation)
  t)

(defun change-st-algorithm
  (new-alg)
  (unless (eq (send *ri* :get-st-algorithm) new-alg)
    (send *ri* :stop-st)
    (send *ri* :set-st-param :st-algorithm new-alg)
    (send *ri* :start-st)
    ;; Wait for osscilation being samall
    (send *ri* :angle-vector (send *robot* :angle-vector) 2000)
    (send *ri* :wait-interpolation)
    )
  )

(defun samplerobot-stabilizer-demo0 ()
  "1. getParameter"
  (pprint (send (send *ri* :get-st-param) :slots))
  (print "getParameter() => OK")
  t)

(defun samplerobot-stabilizer-demo1 ()
  "2. setParameter"
  (let ((k-tpcc-p (float-vector 0.2 0.2))
        (k-tpcc-x (float-vector 4.0 4.0))
        (k-brot-p (float-vector 0.0 0.0)))
    (send *ri* :set-st-param
          :k-tpcc-p k-tpcc-p
          :k-tpcc-x k-tpcc-x
          :k-brot-p k-brot-p
          :eefm-pos-damping-gain (mapcar #'(lambda (x) (float-vector (* 3500 50) (* 3500 50) (* 3500 1.0 1.5))) (make-list 4))
          :eefm-rot-damping-gain (mapcar #'(lambda (x) (float-vector (* 20 1.6 1.5) (* 20 1.6 1.5) 1e5)) (make-list 4))
          :eefm-swing-rot-damping-gain (subseq (send (send (send *ri* :get-st-param) :eefm_rot_damping_gain) :data) 0 3)
          :eefm-swing-pos-damping-gain (subseq (send (send (send *ri* :get-st-param) :eefm_pos_damping_gain) :data) 0 3)
          :eefm-use-swing-damping t)
    (if (and (eps-v= (send (send *ri* :get-st-param) :k_tpcc_p) k-tpcc-p)
             (eps-v= (send (send *ri* :get-st-param) :k_tpcc_x) k-tpcc-x)
             (eps-v= (send (send *ri* :get-st-param) :k_brot_p) k-brot-p))
        (print "setParameter() => OK")))
  t)

(defun samplerobot-stabilizer-demo2 ()
  "3. St + SequencePlayer loadPattern"
  (send *robot* :reset-pose)
  (send *robot* :fix-leg-to-coords (make-coords))
  (warn ";;   Generating walking pattern~%")
  (let* ((rs-list ;; Including :angle-vector, :root-coords, :zmp, :contact-states
          (send *robot* :calc-walk-pattern-from-footstep-list
                (list (make-coords :coords (send *robot* :rleg :end-coords :copy-worldcoords) :name :rleg)
                      (make-coords :coords (send (send *robot* :lleg :end-coords :copy-worldcoords) :translate (float-vector 150 0 0)) :name :lleg)
                      (make-coords :coords (send (send *robot* :rleg :end-coords :copy-worldcoords) :translate (float-vector 150 0 0))  :name :rleg))
                :dt 0.002)))
    (warn ";;   Set initial pose~%")
    (send *ri* :stop-auto-balancer)
    (dump-seq-pattern-file (subseq rs-list 0 2) "/tmp/samplerobot-initpose" :initial-sync-time 0.0 :robot *robot*)
    (send *ri* :load-pattern "/tmp/samplerobot-initpose" 2.0)
    (send *ri* :wait-interpolation-seq)
    (change-st-algorithm :eefmqp)
    (send *ri* :set-st-param :emergency-check-mode 0) ;; 0 is nocheck
    (warn ";;   Walk~%")
    (dump-seq-pattern-file rs-list "/tmp/samplerobot-gopos000" :initial-sync-time 0.0 :robot *robot*)
    (send *ri* :start-st)
    (send *ri* :load-pattern "/tmp/samplerobot-gopos000" 0.0)
    (send *ri* :wait-interpolation-seq)
    (send *ri* :stop-st)
    (print "Start and Stop Stabilizer => OK")
    t))

(defun samplerobot-stabilizer-demo3 ()
  "4. start and stop st"
  (change-st-algorithm :tpcc)
  ;;(send *ri* :go-pos 0.5 0.1 10)
  (print "Start and Stop Stabilizer => OK")
  t)

(defun samplerobot-stabilizer-demo4 ()
  "5. start and stop st"
  (send *ri* :start-auto-balancer)
  (change-st-algorithm :eefmqp)
  (send *ri* :go-pos 0.3 0 0)
  (print "Start and Stop Stabilizer => OK")
  t)

(defun samplerobot-stabilizer-demo5 ()
  "6. EEFMQPCOP + stair"
  (change-st-algorithm :eefmqpcop)
  (send *ri* :start-st)
  (send *ri* :start-auto-balancer)
  ;; half sitting pose
  (send *robot* :reset-pose)
  (send *robot* :fix-leg-to-coords (make-coords) '(:rleg :lleg))
  (send *robot* :legs :move-end-pos #f(0 0 70) :world)
  (send *ri* :angle-vector (send *robot* :angle-vector) 1000)
  (send *ri* :wait-interpolation)
  ;; Get gg parameter just for reverting parameters
  (let* ((prm (send *ri* :get-gait-generator-param))
         (org-param
          (list :default-orbit-type (send prm :default_orbit_type)
                :swing-trajectory-time-offset-xy2z (send prm :swing_trajectory_time_offset_xy2z)
                :swing-trajectory-delay-time-offset (send prm :swing_trajectory_delay_time_offset)
                :toe-heel-phase-ratio (send prm :toe_heel_phase_ratio)
                :toe-pos-offset-x (send prm :toe_pos_offset_x)
                :heel-pos-offset-x (send prm :heel_pos_offset_x)
                :toe-zmp-offset-x (send prm :toe_zmp_offset_x)
                :heel-zmp-offset-x (send prm :heel_zmp_offset_x)
                :use-toe-heel-transition (send prm :use_toe_heel_transition)
                :use-toe-heel-auto-set (send prm :use_toe_heel_auto_set)
                :toe-angle (send prm :toe_angle)
                :heel-angle (send prm :heel_angle)
                )))
    ;; set gg param
    (send *ri* :set-gait-generator-param
          :default-orbit-type :stair
          :swing-trajectory-time-offset-xy2z 0.1
          :swing-trajectory-delay-time-offset 0.2
          :toe-heel-phase-ratio #f(0.05 0.25 0.20 0.0 0.18 0.23 0.09)
          :toe-pos-offset-x (* 182.0 1e-3)
          :heel-pos-offset-x (* -72.0 1e-3)
          :toe-zmp-offset-x (* 182.0 1e-3)
          :heel-zmp-offset-x (* -72.0 1e-3)
          :use-toe-heel-transition t
          :use-toe-heel-auto-set t
          :toe-angle 20
          :heel-angle 10)
    (send *ri* :set-foot-steps
          (list (make-coords :coords (send *robot* :rleg :end-coords :copy-worldcoords) :name :rleg)
                (make-coords :coords (send (send *robot* :lleg :end-coords :copy-worldcoords) :translate (float-vector 270 0 100)) :name :lleg)
                (make-coords :coords (send (send *robot* :rleg :end-coords :copy-worldcoords) :translate (float-vector 270 0 100)) :name :rleg)
                (make-coords :coords (send (send *robot* :lleg :end-coords :copy-worldcoords) :translate (float-vector 520 0 0)) :name :lleg)
                (make-coords :coords (send (send *robot* :rleg :end-coords :copy-worldcoords) :translate (float-vector 520 0 0)) :name :rleg)
                ))
    ;; Revert gg parameter
    (send* *ri* :set-gait-generator-param org-param)
    (print "  ST + Stair => OK")
    t))

(defun samplerobot-stabilizer-demo6 ()
  "7. EEFMQPCOP + toeheel"
  (change-st-algorithm :eefmqpcop)
  (send *ri* :start-st)
  (send *ri* :start-auto-balancer)
  ;; half sitting pose
  (send *robot* :reset-pose)
  (send *robot* :fix-leg-to-coords (make-coords) '(:rleg :lleg))
  (send *ri* :angle-vector (send *robot* :angle-vector) 1000)
  (send *ri* :wait-interpolation)
  ;; Get gg parameter just for reverting parameters
  (let* ((prm (send *ri* :get-gait-generator-param))
         (org-param
          (list :default-orbit-type (send prm :default_orbit_type)
                :swing-trajectory-time-offset-xy2z (send prm :swing_trajectory_time_offset_xy2z)
                :swing-trajectory-delay-time-offset (send prm :swing_trajectory_delay_time_offset)
                :toe-heel-phase-ratio (send prm :toe_heel_phase_ratio)
                :toe-pos-offset-x (send prm :toe_pos_offset_x)
                :heel-pos-offset-x (send prm :heel_pos_offset_x)
                :toe-zmp-offset-x (send prm :toe_zmp_offset_x)
                :heel-zmp-offset-x (send prm :heel_zmp_offset_x)
                :use-toe-heel-transition (send prm :use_toe_heel_transition)
                :use-toe-heel-auto-set (send prm :use_toe_heel_auto_set)
                :default-double-support-ratio (send prm :default_double_support_ratio)
                :toe-angle (send prm :toe_angle)
                :heel-angle (send prm :heel_angle)
                :stride-parameter (send prm :stride_parameter)
                )))
    ;; set gg param
    (send *ri* :set-gait-generator-param
          :default-orbit-type :stair
          :swing-trajectory-time-offset-xy2z 0.1
          :swing-trajectory-delay-time-offset 0.2
          :toe-heel-phase-ratio #f(0.05 0.35 0.20 0.0 0.13 0.13 0.14)
          :toe-pos-offset-x (* 182.0 1e-3)
          :heel-pos-offset-x (* -72.0 1e-3)
          :toe-zmp-offset-x (* 182.0 1e-3)
          :heel-zmp-offset-x (* -72.0 1e-3)
          :use-toe-heel-transition t
          :use-toe-heel-auto-set t
          :default-double-support-ratio 0.7)
    (dolist (sgn '(1 -1))
      (send *ri* :set-foot-steps-with-param
            ;; foot-step-list
            (list (make-coords :coords (send *robot* :rleg :end-coords :copy-worldcoords) :name :rleg)
                  (make-coords :coords (send (send *robot* :lleg :end-coords :copy-worldcoords) :translate (float-vector (* sgn 220) 0 0)) :name :lleg)
                  (make-coords :coords (send (send *robot* :rleg :end-coords :copy-worldcoords) :translate (float-vector (* sgn 440) 0 0)) :name :rleg)
                  (make-coords :coords (send (send *robot* :lleg :end-coords :copy-worldcoords) :translate (float-vector (* sgn 440) 0 0)) :name :lleg)
                  )
            ;; step-height-list
            (list 50 50 50 50)
            ;; step-time-list
            (list 1.0 4.0 4.0 4.0)
            ;; toe-angle-list
            (list 0.0 20.0 20.0 20.0)
            ;; heel-angle-list
            (list 0.0 10.0 10.0 10.0)
            ))
    (send *ri* :set-gait-generator-param
          :default-double-support-ratio 0.2
          :stride-parameter #f(0.22 0.1 20 0.22)
          :toe-angle 20.0 :heel-angle 10.0)
    (dolist (sgn '(1 -1))
      (send *ri* :go-pos (* sgn 0.66) (* sgn 0.2) (* sgn 40.0))
      )
    ;; Revert gg parameter
    (send* *ri* :set-gait-generator-param org-param)
    (print "  ST + ToeHeel => OK")
    t))

(defun samplerobot-stabilizer-demo7 ()
  "8. EEFMQPCOP st + Turn walk"
  (change-st-algorithm :eefmqpcop)
  (send *ri* :start-st)
  (send *ri* :start-auto-balancer)
  ;; half sitting pose
  (send *robot* :reset-pose)
  (send *robot* :fix-leg-to-coords (make-coords) '(:rleg :lleg))
  (send *ri* :angle-vector (send *robot* :angle-vector) 1000)
  (send *ri* :wait-interpolation)
  (send *ri* :stop-collision-detection)
  ;; Get gg parameter just for reverting parameters
  (let ((org-stride-param (send (send *ri* :get-gait-generator-param) :stride_parameter)))
    ;; set gg param
    (send *ri* :set-gait-generator-param :stride-parameter #f(0.15 0.15 90 0.05))
    (send *ri* :go-pos 0.0 -0.2 0)
    (send *ri* :go-pos 0.0 0.0 175)
    (send *ri* :go-pos 0.4 0.15 40)
    (send *ri* :start-collision-detection)
    ;; Revert gg parameter
    (send *ri* :set-gait-generator-param :stride-parameter org-stride-param)
    (print "  ST + Turnwalk => OK")
    t))

(defun gen-root-rot-pose
  (y p r)
  (send *robot* :reset-pose)
  (send *robot* :fix-leg-to-coords (make-coords))
  (send *robot* :legs :move-end-pos #f(0 0 70))
  (send *robot* :fix-leg-to-coords (make-coords))
  (let ((lc (send *robot* :legs :end-coords :copy-worldcoords)))
    (send *robot* :move-coords
          (make-coords :pos (send (send (car (send *robot* :links)) :copy-worldcoords) :worldpos)
                       :rpy (list y p r))
          (car (send *robot* :links)))
    (mapcar #'(lambda (l c) (send *robot* l :inverse-kinematics c)) '(:lleg :rleg) lc)
    (send *robot* :move-centroid-on-foot :both '(:rleg :lleg))
    (send *robot* :angle-vector)))

(defun samplerobot-stabilizer-demo8 ()
  "9. ST root rot change"
  (change-st-algorithm :eefmqpcop)
  (send *ri* :start-st)
  (send *ri* :start-auto-balancer)
  (gen-root-rot-pose 0 0 (deg2rad 10))
  (send *ri* :angle-vector (send *robot* :angle-vector) 1000)
  (send *ri* :wait-interpolation)
  (send *ri* :angle-vector (send *robot* :angle-vector) 1000)
  (send *ri* :wait-interpolation)
  (gen-root-rot-pose 0 (deg2rad 35) 0)
  (send *ri* :angle-vector (send *robot* :angle-vector) 1000)
  (send *ri* :wait-interpolation)
  (send *ri* :angle-vector (send *robot* :angle-vector) 1000)
  (send *ri* :wait-interpolation)
  (gen-root-rot-pose (deg2rad 25) 0 0)
  (send *ri* :angle-vector (send *robot* :angle-vector) 1000)
  (send *ri* :wait-interpolation)
  (send *ri* :angle-vector (send *robot* :angle-vector) 1000)
  (send *ri* :wait-interpolation)
  (gen-root-rot-pose (deg2rad 10) (deg2rad 10) (deg2rad 10))
  (send *ri* :angle-vector (send *robot* :angle-vector) 1000)
  (send *ri* :wait-interpolation)
  (send *ri* :angle-vector (send *robot* :angle-vector) 1000)
  (send *ri* :wait-interpolation)
  (send *robot* :reset-pose)
  (send *robot* :fix-leg-to-coords (make-coords))
  (send *ri* :angle-vector (send *robot* :angle-vector) 1000)
  (send *ri* :wait-interpolation)
  (print "  ST root rot change")
  t)

(defun samplerobot-stabilizer-demo9 ()
  "10. ST mimic rough terrain walk"
  (change-st-algorithm :eefmqpcop)
  (send *ri* :set-foot-steps
        (list (make-coords :coords (send *robot* :rleg :end-coords :copy-worldcoords) :name :rleg)
              (make-coords :coords (send (send *robot* :lleg :end-coords :copy-worldcoords) :translate (float-vector 220 0 40)) :name :lleg)
              (make-coords :coords (send (send *robot* :rleg :end-coords :copy-worldcoords) :translate (float-vector 440 0 0)) :name :rleg)
              (make-coords :coords (send (send *robot* :lleg :end-coords :copy-worldcoords) :translate (float-vector 440 0 0)) :name :lleg)
              ))
  (send *ri* :wait-foot-steps)
  (send *ri* :set-foot-steps
        (list (make-coords :coords (send *robot* :rleg :end-coords :copy-worldcoords) :name :rleg)
              (make-coords :coords (send (send *robot* :lleg :end-coords :copy-worldcoords) :translate (float-vector 220 0 -40)) :name :lleg)
              (make-coords :coords (send (send *robot* :rleg :end-coords :copy-worldcoords) :translate (float-vector 440 0 0)) :name :rleg)
              (make-coords :coords (send (send *robot* :lleg :end-coords :copy-worldcoords) :translate (float-vector 440 0 0)) :name :lleg)
              ))
  (send *ri* :wait-foot-steps)
  (print "  ST mimic rough terrain walk")
  t)

(defun samplerobot-stabilizer-demo ()
  (samplerobot-stabilizer-init)
  (samplerobot-stabilizer-demo0)
  (samplerobot-stabilizer-demo1)
  (samplerobot-stabilizer-demo2)
  (samplerobot-stabilizer-demo3)
  (samplerobot-stabilizer-demo4)
  (samplerobot-stabilizer-demo5)
  (samplerobot-stabilizer-demo6)
  (samplerobot-stabilizer-demo7)
  (samplerobot-stabilizer-demo8)
  (samplerobot-stabilizer-demo9)
  )

(warn ";; (samplerobot-stabilizer-demo)~%")
