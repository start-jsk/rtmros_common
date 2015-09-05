#!/usr/bin/env roseus

;; $ rtmlaunch hrpsys_ros_bridge samplerobot.launch
;; $ roseus samplerobot-stabilizer.l "(samplerobot-stabilizer-demo)"

(load "package://hrpsys_ros_bridge/euslisp/samplerobot-interface.l")

(defun samplerobot-stabilizer-init ()
  (samplerobot-init)
  (send *sr* :reset-pose)
  (send *ri* :angle-vector (send *sr* :angle-vector) 1000)
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
  (send *ri* :start-st)
  (send *ri* :go-pos 0.5 0.1 10)
  (send *ri* :stop-st)
  (print "Start and Stop Stabilizer => OK")
  t)

(defun samplerobot-stabilizer-demo ()
  (samplerobot-stabilizer-init)
  (samplerobot-stabilizer-demo0)
  (samplerobot-stabilizer-demo1)
  (samplerobot-stabilizer-demo2)
  )

(warn ";; (samplerobot-stabilizer-demo)~%")