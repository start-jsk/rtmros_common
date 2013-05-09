;; roseus loop-qual-task-1.l

(load "package://hrpsys_gazebo/euslisp/atlas-interface.l")

(let* ((try-num 0))
  (while t
    (let* ((trans-x-max 0.15)
	   (trans-y-max 0.05)
	   (trans-yaw-max 0.025)
	   (trans-x-step 0.05)
	   (trans-y-step 0.02)
	   (trans-yaw-step 0.025)
	   (trans-x (- trans-x-max))
	   (trans-y (- trans-y-max))
	   (trans-yaw (- trans-yaw-max)))
      (while t

	(incf try-num)

	(unix:system "pkill roslaunch")

	(warn "[my-msg] start qual_task_1~%")

	(warn "[my-msg] launch qual_task_1.launch~%")
	(unix:system "roslaunch atlas_utils qual_task_1.launch &")

	(unix:usleep (* 10 1000 1000))
	(warn "[my-msg] launch atlas_hrpsys.launch~%")
	(unix:system "roslaunch hrpsys_gazebo atlas_hrpsys.launch &")

	(unix:usleep (* 10 1000 1000))
	(warn "[my-msg] launch footstep_planner.launch~%")
	(unix:system "roslaunch hrpsys_gazebo footstep_planner.launch MAP_FILE\:=$(rospack find hrpsys_gazebo)/maps/field_map.yaml &")

	(unix:usleep (* 10 1000 1000))
	(warn "[my-msg] call (atlas-init)~%")
	(atlas-init :set-user-mode nil)
	(unix:usleep (* 1000 1000))
	(warn "[my-msg] walk field~%")
	(send *ri* :bdi_footstep (footstep-planning-armin (make-coords :pos (scale 1000 (float-vector 1 1 0)) :rpy (list 0 0 0)) (make-coords :pos (scale 1000 (float-vector 17.25 9.575 0)) :rpy (list 0 0 0))))


	(unix:usleep (* 160 1000 1000))
	(warn "[my-msg] walk kill footpstep_planner~%")
	(unix:system "kill $(ps aux | grep footstep_planner | grep roslaunch | awk '{print $2}')")
	(unix:usleep (* 20 1000 1000))
	(warn "[my-msg] launch footpstep_planner.launch again~%")
	(unix:system "roslaunch hrpsys_gazebo footstep_planner.launch MAP_FILE\:=$(rospack find hrpsys_gazebo)/maps/stepping_map.yaml &")

	(unix:usleep (* 20 1000 1000))
	(warn "[my-msg] walk stepping block~%")

	(send *ri* :bdi_footstep (footstep-planning-armin (make-coords :pos (scale 1000 (v+ (v+ (float-vector 0.5 1.55 0) (float-vector -0.1875 -0.05 0)) (float-vector trans-x trans-y 0))) :rpy (list trans-yaw 0 0)) (make-coords :pos (scale 1000 (float-vector 6.2 1.55 0)) :rpy (list 0 0 0))))

	(unix:usleep (* 80 1000 1000))
	(unix:system (format nil "echo ~1d    ~1d ~1d ~1d $(rostopic echo /atlas/atlas_sim_interface_state/pos_est -n 1 | grep x | head -n 1 | cut -f 2- -d\:) >> ~/Desktop/qual_task_1_0508/memo.txt" try-num trans-x trans-y trans-yaw))

	(unix:usleep (* 1000 1000))
	(warn "[my-msg] pkill roslaunch~%")
	(unix:system "pkill roslaunch")
	(unix:usleep (* 10 1000 1000))

	(unix:system (format nil "cp /tmp/qual_task_1/ ~/Desktop/qual_task_1/~1d -r" try-num))

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

