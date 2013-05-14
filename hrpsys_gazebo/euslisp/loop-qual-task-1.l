;; roseus loop-qual-task-1.l

(load "package://hrpsys_gazebo/euslisp/atlas-interface.l")

(defun atlas-sim-interface-state-cb
  (msg)
  )

(let* ((try-num 1000)
       (save-dir-name "~/Desktop/qual_task_1"))

  (unix:system (format nil "mkdir -p ~a" save-dir-name))
  (ros::subscribe "atlas/atlas_sim_interface_state" atlas_msgs::AtlasSimInterfaceState #'atlas-sim-interface-state-cb)

  (while t
    (let* ((trans-x-max 0.15)
	   (trans-y-max 0.05)
	   (trans-yaw-max 0.025)
	   (trans-x-step 0.05)
	   (trans-y-step 0.02)
	   (trans-yaw-step 0.025)
	   (trans-x (- trans-x-max))
	   (trans-y (- trans-y-max))
	   (trans-yaw (- trans-yaw-max))
	   field-footstep
	   call-field-footstep-flag
	   tmp-pose)

      (while t

	(incf try-num)
	(setq call-field-footstep-flag nil)

	(unix:system "pkill roslaunch")
	(unix:system "pkill python")
	(unix:system "pkill gzserver")
	(unix:system "pkill rviz")
	(unix:usleep (* 10 1000 1000))

	(warn "[my-msg] start qual_task_1~%")

	(warn "[my-msg] launch qual_task_1.launch~%")
	(unix:system "roslaunch atlas_utils qual_task_1.launch &")
	(unix:usleep (* 15 1000 1000))

	(warn "[my-msg] launch atlas_hrpsys.launch~%")
	(unix:system "roslaunch hrpsys_gazebo atlas_hrpsys.launch &")
	(unix:usleep (* 10 1000 1000))

	(unless field-footstep
	  (warn "[my-msg] launch footstep_planner.launch~%")
	  (setq call-field-footstep-flag t)
	  (unix:system "roslaunch hrpsys_gazebo footstep_planner.launch MAP_FILE\:=$(rospack find hrpsys_gazebo)/maps/field_map.yaml &")
	  (unix:usleep (* 10 1000 1000)))

	(warn "[my-msg] call (atlas-init)~%")
	(atlas-init :set-user-mode nil)
	(unix:usleep (* 1000 1000))

	(when call-field-footstep-flag
	  (warn "[my-msg] call footstep-planning for field ~%")
	  (setq field-footstep (footstep-planning-armin (make-coords :pos (scale 1000 (float-vector 1 1 0)) :rpy (list 0 0 0)) (make-coords :pos (scale 1000 (float-vector 17.35 9 0)) :rpy (list 0 0 0)))))

	(warn "[my-msg] walk field~%")
	(send *ri* :bdi_footstep field-footstep)
	(unix:usleep (* 20 1000 1000))
	(send *ri* :wait-for-bdi-walk)
	;; (unix:usleep (* 160 1000 1000))

	(ros::spin-once)
	(setq tmp-pose (send *ri* :get :pose-on-gazebo))

	(when call-field-footstep-flag
	  (warn "[my-msg] walk kill footpstep_planner~%")
	  (unix:system "kill $(ps aux | grep footstep_planner | grep roslaunch | awk '{print $2}')")
	  (unix:system "pkill rviz")
	  (unix:usleep (* 15 1000 1000)))

	(warn "[my-msg] launch footpstep_planner.launch again~%")
	(unix:system "roslaunch hrpsys_gazebo footstep_planner.launch MAP_FILE\:=$(rospack find hrpsys_gazebo)/maps/stepping_map.yaml &")
	(unix:usleep (* 15 1000 1000))

	(warn "[my-msg] walk stepping block~%")
	(send *ri* :bdi_footstep (footstep-planning-armin (make-coords :pos (scale 1000 (v+ (float-vector 0.5 1.55 0) (float-vector trans-x trans-y 0))) :rpy (list trans-yaw 0 0)) (make-coords :pos (scale 1000 (float-vector 6.2 1.55 0)) :rpy (list 0 0 0))))
	(unix:usleep (* 20 1000 1000))
	(send *ri* :wait-for-bdi-walk)
	;; (unix:usleep (* 60 1000 1000))

	(warn "[my-msg] save log file to ~a~%" save-dir-name)

	(ros::spin-once)
	(unix:system (format nil "echo \"~1d \: ~1d ~1d ~1d \: ~a \: ~a \" >> ~a/memo.txt"
			     try-num trans-x trans-y trans-yaw tmp-pose (send *ri* :get :pose-on-gazebo) save-dir-name))

	(unix:system (format nil "cp /tmp/qual_task_1/ ~a/~1d -r" save-dir-name try-num))
	(unix:usleep (* 1000 1000))

	(setq trans-x (+ trans-x trans-x-step))
	(when (> trans-x trans-x-max)
	  (setq trans-y (+ trans-y trans-y-step))
	  (setq trans-x (- trans-x-max)))
	(when (> trans-y trans-y-max)
	  (setq trans-yaw (+ trans-yaw trans-yaw-step))
	  (setq (trans-y (- trans-y-max))))
	(when (> trans-yaw trans-yaw-max)
	  (setq (trans-yaw (- trans-yaw-max)))
	  (return nil))

	(warn "[my-msg] finish qual_task_1~%")

	))))

(exit)

