#!/usr/bin/env roseus

(require :unittest "lib/llib/unittest.l")
(load "package://hrpsys_ros_bridge/euslisp/samplerobot-interface.l")

(init-unit-test)

(samplerobot-init)

(deftest test-motor-extra-data
  (assert (not (send *ri* :motor-extra-data))
          ":motor-extra-data should return nil when no msg is got from /motor_states")
  )

(run-all-tests)
(exit 0)
