#!/usr/bin/env roseus

;; $ rtmlaunch hrpsys_ros_bridge samplerobot.launch
;; $ roseus samplerobot-data-logger.l "(samplerobot-data-logger-demo)"

(load "package://hrpsys_ros_bridge/euslisp/samplerobot-interface.l")

(defun samplerobot-data-logger-init ()
  (samplerobot-init)
  t)

(defun samplerobot-data-logger-demo0 ()
  "1. Set max ring-buffer length : 200 [loop] * 0.002 [s] = 0.4 [s] data"
  (send *ri* :set-log-maxlength 200)
  (print "maxLength() =>OK")
  t)

(defun samplerobot-data-logger-demo1 ()
  "2. Clear buffer"
  (send *ri* :start-log)
  (print "clear() =>OK")
  t)

(defun samplerobot-data-logger-demo2 ()
  "3. Save log"
  (send *sr* :reset-pose)
  (send *ri* :angle-vector (send *sr* :angle-vector) 2000)
  (send *ri* :wait-interpolation)
  ;; Save log files for each ports as /tmp/test-samplerobot-log.*
  ;;   file names are /tmp/test-samplerobot-log.[RTCName]_[PortName], c.f.,  /tmp/test-samplerobot-log.sh_qOut ... etc
  (send *ri* :save-log "/tmp/test-samplerobot-log")
  (find-if #'(lambda (x) (substringp "test-samplerobot-log" x)) (directory "/tmp"))
  )

(defun samplerobot-data-logger-demo ()
  (samplerobot-data-logger-init)
  (samplerobot-data-logger-demo0)
  (samplerobot-data-logger-demo1)
  (samplerobot-data-logger-demo2)
  )

(warn ";; (samplerobot-data-logger-demo)~%")