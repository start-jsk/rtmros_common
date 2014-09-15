#!/usr/bin/env roseus

;; $ rtmlaunch hrpsys_ros_bridge_tutorials samplerobot.launch
;; $ roseus samplerobot-impedance-controller.l "(demo)"

(load "package://hrpsys_ros_bridge_tutorials/euslisp/samplerobot-interface.l")

(defun init ()
  (samplerobot-init)
  )

(defun demo ()
  (init)
  (send *sr* :reset-pose)
  (send *ri* :angle-vector (send *sr* :angle-vector) 1000)
  (send *ri* :wait-interpolation)

  ;; 1. get
  (pprint (send (send *ri* :get-impedance-controller-param :rarm) :slots))
  (print "getImpedanceControllerParam => OK")
  ;; 2. set and start
  (send *ri* :start-impedance :rarm :K-r 1.0 :D-r 2.0 :M-r 0.2)
  (pprint (send (send *ri* :get-impedance-controller-param :rarm) :slots))
  (print "setImpedanceControllerParam => OK")
  ;; 3. set ref force and moment
  (send *ri* :set-impedance-controller-param :rarm :ref-force #f(10 10 -10))
  (unix:sleep 2)
  (send *ri* :set-impedance-controller-param :rarm :ref-force #f(0 0 0))
  (unix:sleep 2)
  (send *ri* :set-impedance-controller-param :rarm :ref-moment #f(0.2 0.2 -0.5))
  (unix:sleep 2)
  (send *ri* :set-impedance-controller-param :rarm :ref-moment #f(0 0 0))
  (unix:sleep 2)
  ;; 4. stop
  (send *ri* :stop-impedance :rarm)
  (print "deleteImpedanceController => OK")
  )

;;(demo)