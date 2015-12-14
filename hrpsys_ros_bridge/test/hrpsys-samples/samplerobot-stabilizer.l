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
          :k-brot-p k-brot-p)
    (if (and (eps-v= (send (send *ri* :get-st-param) :k_tpcc_p) k-tpcc-p)
             (eps-v= (send (send *ri* :get-st-param) :k_tpcc_x) k-tpcc-x)
             (eps-v= (send (send *ri* :get-st-param) :k_brot_p) k-brot-p))
        (print "setParameter() => OK")))
  t)

(defun samplerobot-stabilizer-demo2 ()
  "3. start and stop st"
  (send *ri* :set-st-param :st-algorithm :tpcc)
  (send *ri* :start-st)
  ;;(send *ri* :go-pos 0.5 0.1 10)
  (send *ri* :stop-st)
  (print "Start and Stop Stabilizer => OK")
  t)

(defun samplerobot-stabilizer-demo3 ()
  "4. start and stop st"
  (send *ri* :set-st-param :st-algorithm :eefmqp)
  (send *ri* :start-st)
  (send *ri* :go-pos 0.3 0 0)
  (send *ri* :stop-st)
  (print "Start and Stop Stabilizer => OK")
  t)

(defun samplerobot-stabilizer-demo4 ()
  "5. St + SequencePlayer loadPattern"
  (send *ri* :set-st-param :st-algorithm :eefmqp :emergency-check-mode 0) ;; 0 is nocheck
  (send *robot* :reset-pose)
  (send *robot* :fix-leg-to-coords (make-coords))
  (warn ";;   Generating walking pattern~%")
  (let* ((rs-list ;; Including :angle-vector, :root-coords, :zmp, :contact-states
          (send *robot* :calc-walk-pattern-from-footstep-list
                (list (make-coords :coords (send *robot* :rleg :end-coords :copy-worldcoords) :name :rleg)
                      (make-coords :coords (send (send *robot* :lleg :end-coords :copy-worldcoords) :translate (float-vector 150 0 0)) :name :lleg)
                      (make-coords :coords (send (send *robot* :rleg :end-coords :copy-worldcoords) :translate (float-vector 150 0 0))  :name :rleg))
                :dt 0.002))
         (rs-list2 ;; Add optionalData from rs-list
          (add-optional-data-from-rs-list rs-list *robot*)))
    (warn ";;   Set initial pose~%")
    (send *ri* :stop-auto-balancer)
    (dump-seq-pattern-file (subseq rs-list2 0 1) "/tmp/samplerobot-initpose" :initial-sync-time 0.0)
    (send *ri* :load-pattern "/tmp/samplerobot-initpose" 2.0)
    (send *ri* :wait-interpolation-seq)
    (warn ";;   Walk~%")
    (dump-seq-pattern-file rs-list2 "/tmp/samplerobot-gopos000" :initial-sync-time 0.0)
    (send *ri* :start-st)
    (send *ri* :load-pattern "/tmp/samplerobot-gopos000" 0.0)
    (send *ri* :wait-interpolation-seq)
    (send *ri* :stop-st)
    (print "Start and Stop Stabilizer => OK")
    t))

(defun samplerobot-stabilizer-demo ()
  (samplerobot-stabilizer-init)
  (samplerobot-stabilizer-demo0)
  (samplerobot-stabilizer-demo1)
  (samplerobot-stabilizer-demo2)
  (samplerobot-stabilizer-demo3)
  (samplerobot-stabilizer-demo4)
  )

(warn ";; (samplerobot-stabilizer-demo)~%")
