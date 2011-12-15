#!/usr/bin/env roseus
(load "model/beego.l")
(ros::roseus-add-msgs "visualization_msgs")
(ros::roseus "beego_model_publisher")
(ros::advertise "beego_robot" visualization_msgs::MarkerArray 10)


(defun exec-main()
  (let* ((beego-robot (beego))
	 (msg (instance visualization_msgs::MarkerArray :init))
	 (header (instance std_msgs::header :init
			   :stamp (ros::time-now) :frame_id "base_link"))
	 (ob-msg (object->marker-msg beego-robot header :color #f(0.1 0.1 0.1) :ns "test")))
    (send msg :markers (list ob-msg))
    (ros::publish "beego_robot" msg)))
(while (ros::ok)
  (exec-main)
  (unix::sleep 10))
(exit)