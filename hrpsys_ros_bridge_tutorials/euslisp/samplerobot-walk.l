(ros::roseus "samplerobot_walk")
(load "package://hrpsys_ros_bridge_tutorials/euslisp/samplerobot-interface.l")

(defun cmd-vel-cb (msg)
  (send *ri* :go-velocity
        (send (send msg :linear) :x)
        (send (send msg :linear) :y)
        (send (send msg :angular) :z))
  )

(samplerobot-init)
(objects (list *sr*))
(ros::subscribe
  "/cmd_vel"
  geometry_msgs::Twist
  #'cmd-vel-cb)

(defun samplerobot-walk
  ()
  (send *sr* :reset-pose)
  (send *ri* :angle-vector (send *sr* :angle-vector) 3000)
  (send *ri* :wait-interpolation)
  (send *ri* :start-auto-balancer)
  (send *ri* :go-pos 0.5 0.15 45) ;; x[m] y[m] th[rad]
  (send *ri* :stop-auto-balancer)
  )
(warn ";; (samplerobot-walk) ;; example to specify target x, y, theta~%")

(defun samplerobot-walk2
  ()
  (send *sr* :reset-pose)
  (send *ri* :angle-vector (send *sr* :angle-vector) 3000)
  (send *ri* :wait-interpolation)
  (send *ri* :start-auto-balancer)
  (send *ri* :go-velocity 0 0 0)
  (print ";; Press Enter to Stop")
  (ros::rate 10)
  (do-until-key
   (send *ri* :go-velocity 0 0 0)
   (ros::spin-once)
   (ros::sleep)
   )
  (send *ri* :go-stop)
  (send *ri* :stop-auto-balancer)
  )
(warn ";; (samplerobot-walk2) ;; example to specify target velocity ;; please use \"teleop_pr2_keyboard\" terminal~%")

(defun samplerobot-walk-eus
  ()
  ;; limit crotch-y range which are originally inf and -inf
  (send *sr* :legs :crotch-y :min-angle -90)
  (send *sr* :legs :crotch-y :max-angle 90)
  (format t ";; generate pattern~%")
  (let ((pattern-list
         (send *sr* :calc-walk-pattern-from-footstep-list
               (send *sr* :go-pos-params->footstep-list
                     500 150 45) ;; x[mm] y[mm] th[rad]
               :debug-view :no-message :dt 0.01))
        (fname (format nil "~A/samplerobot-walk-pattern"
                       (ros::resolve-ros-path "package://hrpsys_ros_bridge_tutorials/build"))))
    (format t ";; dump pattern file~%")
    (send *ri* :dump-seq-pattern-file pattern-list fname)
    (format t ";; load pattern file~%")
    (send *ri* :loadPattern fname)
    ))

