(load "package://hrpsys_ros_bridge/euslisp/rtm-ros-robot-interface.l")
(require :hrp4c "package://hrpsys_gazebo/models/atlas.l")
(ros::load-ros-manifest "hrpsys_gazebo")

(ros::roseus "atlas_interface")

(defmethod atlas-robot
  (:init-ending ()
   (prog1
       (send-super :init-ending)
     ;; (setq larm-root-link (send self :larm :shoulder-y :child-link))
     ;; (setq rarm-root-link (send self :rarm :shoulder-y :child-link))
     )
   ))

(defclass atlas-interface
  :super rtm-ros-robot-interface
  :slots (l_hand_grasp_pub r_hand_grasp_pub))
(defmethod atlas-interface
  (:init ()
   (prog1 (send-super :init :robot atlas-robot)
     (setq l_hand_grasp_pub (ros::advertise "/sandia_hands/l_hand/simple_grasp" sandia_hand_msgs::SimpleGrasp 1)
           r_hand_grasp_pub (ros::advertise "/sandia_hands/r_hand/simple_grasp" sandia_hand_msgs::SimpleGrasp 1))
;     (send self :add-controller :larm-controller)
;     (send self :add-controller :rarm-controller)
;     (send self :add-controller :head-controller)
;     (send self :add-controller :torso-controller)
     ))
  (:grasp
   (value &optional (arm :arms))
   (let ((msg (instance sandia_hand_msgs::SimpleGrasp :init :name "cylindrical" :closed_amount value)))
     (case arm
       (:arms
        (send self :grasp value :larm)
        (send self :grasp value :rarm))
       (:larm
        (ros::publish "/sandia_hands/l_hand/simple_grasp" msg))
       (:rarm
        (ros::publish "/sandia_hands/r_hand/simple_grasp" msg))
       )))
  (:start-grasp (&optional (arm :arms)) (send self :grasp 1 arm))
  (:stop-grasp  (&optional (arm :arms)) (send self :grasp 0 arm))
  )

(defun atlas-init
  (&key (set-reset-pose t))
  (if (not (boundp '*ri*)) (setq *ri* (instance atlas-interface :init)))
  (if (not (boundp '*atlas*)) (setq *atlas* (instance atlas-robot :init)))
  (send_atlas_control_mode "User")
  (when set-reset-pose
    (send *atlas* :reset-pose)
    ;; send reset pose to real robot
    (send *ri* :angle-vector (send *atlas* :angle-vector) 3000)
    (send *ri* :wait-interpolation))
  )

(defun atlas-tool-look ()
  (send *atlas* :head :neck-p :joint-angle 50)
  (send *ri* :angle-vector (send *atlas* :angle-vector) 3000)
  (send *ri* :wait-interpolation)
  )

(defun atlas-tool-grasp ()
  (send *atlas* :larm :shoulder-r :joint-angle -50)
  (send *atlas* :larm :shoulder-y :joint-angle  40)
  (send *atlas* :larm :elbow-p    :joint-angle  90)
  (send *atlas* :larm :elbow-y    :joint-angle  90)
  (send *atlas* :larm :wrist-y    :joint-angle   0)
  (send *atlas* :larm :wrist-p    :joint-angle -20)

  (send *atlas* :rarm :shoulder-r :joint-angle  50)
  (send *atlas* :rarm :shoulder-y :joint-angle  40)
  (send *atlas* :rarm :elbow-p    :joint-angle -90)
  (send *atlas* :rarm :elbow-y    :joint-angle  90)
  (send *atlas* :rarm :wrist-y    :joint-angle   0)
  (send *atlas* :rarm :wrist-p    :joint-angle -20)

  ;; (send *atlas* :legs :knee-p :joint-angle 30)
  ;; (send *atlas* :legs :crotch-p :joint-angle -15)
  ;; (send *atlas* :legs :ankle-p :joint-angle -15)
  ;; (send *atlas* :move-centroid-on-foot :both '(:rleg :lleg :rarm :larm))

  (send *ri* :angle-vector (send *atlas* :angle-vector) 20000)
  (send *ri* :wait-interpolation)

  (send *atlas* :rarm :inverse-kinematics (make-coords :pos #f(370 -350 200)) :rotation-axis :z)
;;  (send *atlas* :move-centroid-on-foot :both '(:rleg :lleg :rarm :larm))
  (send *ri* :angle-vector (send *atlas* :angle-vector) 15000)
  (send *ri* :wait-interpolation)

  (send *ri* :start-grasp :rarm)
  (unix:usleep 3)

  (send *atlas* :rarm :inverse-kinematics (make-coords :pos #f(550 0 400)) :rotation-axis :z :use-torso t)
;;  (send *atlas* :move-centroid-on-foot :both '(:rleg :lleg :rarm :larm))
  (send *ri* :angle-vector (send *atlas* :angle-vector) 15000)
  (send *ri* :wait-interpolation)

  (send *ri* :stop-grasp :rarm)
  (unix:usleep 3)
  )

(defun atlas-walk-pattern-gen
  (&key (no-gen nil) (fname nil))
  ;; limit crotch-y range which are originally inf and -inf
  (let (pattern-list
        (fname (if fname fname
		 (format nil "~A/atlas-walk-pattern"
			 (ros::resolve-ros-path "package://hrpsys_gazebo/build")))))
    (unless no-gen
      (format t ";; generate pattern~%")
      (setq pattern-list
	    (send *atlas* :calc-walk-pattern-from-footstep-list
		  (send *atlas* :go-pos-params->footstep-list
			500 150 45) ;; x[mm] y[mm] th[rad]
		  :debug-view :no-message :dt 0.01))
      (format t ";; dump pattern file~%")
      (send *ri* :dump-seq-pattern-file pattern-list fname))
    (format t ";; load pattern file~%")
    (send *ri* :loadPattern fname)
    ))

(defun atlas-walk
  ()
  (send_atlas_control_mode "Stand")
  (atlas-init :set-reset-pose nil)
  (objects (list *atlas*))
  (send *atlas* :angle-vector (send *ri* :state :potentio-vector))
  (send *ri* :angle-vector (send *atlas* :angle-vector) 2000)
  (send_atlas_control_mode "User")
  (atlas-walk-pattern-gen)
  )

(ros::advertise "atlas/control_mode" std_msgs::String 1)
(defun send_atlas_control_mode
  (&optional (mode "Stand"))
  (let ((msg (instance std_msgs::String :init)))
    ;; set publish topics
    (send msg :data mode)
    (ros::publish "atlas/control_mode" msg)))


