#!/usr/bin/env roseus

(load "unittest.l")
(ros::roseus-add-srvs "openrtm_ros_bridge")

(ros::roseus "test-myservice-bridge")

(init-unit-test)

(deftest test-myservice-bridge-echo ()
  (let ((srvname "/bridge/echo") req res (testdata "this is test data 123"))
    (ros::wait-for-service srvname)
    (setq req (instance openrtm_ros_bridge::SimpleService_MyService_echoRequest :init))
    (send req :msg testdata)
    (setq res (ros::service-call srvname req))
    (assert (string= (send res :operation_return) testdata)
	    "SimpleService.echo returns incorrect string")
  ))

(run-all-tests)

(exit)
