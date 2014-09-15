#!/usr/bin/env roseus

;; $ rtmlaunch hrpsys_ros_bridge_tutorials samplerobot.launch
;; $ roseus samplerobot-data-logger.l "(demo)"

(load "package://hrpsys_ros_bridge_tutorials/euslisp/samplerobot-interface.l")

(defun init ()
  (samplerobot-init)
  )

(defun demo ()
  (init)

  ;; 1. Set max ring-buffer length : 200 [loop] * 0.002 [s] = 0.4 [s] data
  (send *ri* :set-log-maxlength 200)
  (print "maxLength() =>OK")
  ;; Clear buffer
  (send *ri* :start-log)
  (print "clear() =>OK")
  (send *sr* :reset-pose)
  (send *ri* :angle-vector (send *sr* :angle-vector) 2000)
  (send *ri* :wait-interpolation)
  ;; Save log files for each ports as /tmp/test-samplerobot-log.*
  ;;   file names are /tmp/test-samplerobot-log.[RTCName]_[PortName], c.f.,  /tmp/test-samplerobot-log.sh_qOut ... etc
  (send *ri* :save-log "/tmp/test-samplerobot-log")
  (print "save() =>OK")
  )

;;(demo)