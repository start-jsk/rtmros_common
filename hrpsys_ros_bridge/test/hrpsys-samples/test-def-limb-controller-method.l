(require :unittest "lib/llib/unittest.l")
(load "package://hrpsys_ros_bridge/euslisp/samplerobot-interface.l")

(init-unit-test)

;; Wait until Ros bridges are prepared.
(ros::roseus "unittest")
(unless (boundp '*tfl*)
  (defvar *tfl* (instance ros::transform-listener :init)))
(send *tfl* :wait-for-transform "WAIST_LINK0" "odom" (ros::time) 20.0)

(deftest test-def-limb-controller-method
  ;; Test :def-limb-controller-method before *ri* initialization
  (assert (instance samplerobot-interface :def-limb-controller-method :rleg))
  (samplerobot-init)
  ;; Test :def-limb-controller-method after *ri* initialization
  (assert (send *ri* :def-limb-controller-method :rleg))
  )

(run-all-tests)
(exit)
