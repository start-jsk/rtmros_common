(ros::load-ros-manifest "hrpsys_gazebo")
(ros::roseus "check_score")
(defun vrc-score-callback (msg)
  (unless (equal (send msg :message) "")
    (format t "falls ~A~%" (send msg :falls))
    (format t "score ~A~%" (send msg :completion_score))
  )
)


(ros::subscribe "/vrc_score"
		atlas_msgs::vrcscore
		#'vrc-score-callback)

(print "print falls and score when they are changed")
(while (ros::ok)
  (ros::spin-once)
)