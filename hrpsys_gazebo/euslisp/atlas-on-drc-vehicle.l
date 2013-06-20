(load "package://hrpsys_gazebo/euslisp/atlas-interface.l")
(load "package://hrpsys_gazebo/euslisp/atlas-applications.l")
(load "package://hrpsys_gazebo/euslisp/atlas-vehicle-motion.l")
(load "package://hrpsys_gazebo/euslisp/atlas-vehicle-motion-drive.l")
(load "package://hrpsys_gazebo/euslisp/atlas-utils.l")

;; (unless (boundp '*drcv*)
;;   (setq *drcv* 
;; 	  (with-open-file 
;; 	   (f (format nil "~a/~a" (unix:getenv "HOME")
;;                   "ros/fuerte/rtm-ros-robotics/rtmros_common/hrpsys_gazebo/models/drc-vehicle.l"))
;;        (read f)))
;;   )

;; no walk and telep in side of vehicle by murooka
;;  (all-demo-task1 :real t :wait nil :no-walk-and-telep t)
(ros::advertise "/drc_world/robot_enter_car" geometry_msgs::Pose 1)

(defun init-atlas-drive-car
  (&key (jump t) (real t) (nominal t))
  (let ((msg (instance geometry_msgs::Pose :init)) (time 3.0))
    (atlas-init)
    (make-drcv-for-drive)
    ;; init pose
    (real2model)
    (sit-manip-pose)
    (when real
      (model2real)
      )
    ;; move to drcv
    (cheat-ride-drcv-eus :mode :assistant)
    (when jump
      (cheat-ride-drcv-drcsim :real real :nominal nominal :mode :assistant)
      )
    ;; drawing
    (objects (list *atlas* *drcv*))
    )
  )

(defun make-drcv-for-drive
  ()
  (make-vehicle)
  (let* ((offset (float-vector 0 200 0)))
    ;; front-left-pipe-left
    (send *drcv* :add-handle
          (make-cascoords :coords (send (send (send *drcv* :handle :name :upper-front-pipe) :locate offset :local) :copy-worldcoords)
                          :name :front-left-pipe-left))
    ;; front-left-pipe-right
    (send *drcv* :add-handle
          (make-cascoords :coords (send (send (send *drcv* :handle :name :upper-front-pipe) :locate (scale -1 offset) :local) :copy-worldcoords)
                          :name :front-left-pipe-left))
    ;; upper-right-pipe
    (send *drcv* :add-handle
          (make-cascoords :pos #f(140 -575 1900) :rot (transpose (make-matrix 3 3 (list #f(0 0 -1) #f(0 -1 0) #f(-1 0 0)))) :name :upper-right-pipe))
    ;; gas-pedal
    (send *drcv* :add-handle
          (make-cascoords :coords (send (elt (send *drcv* :links) 7) :copy-worldcoords)
                          :name :gas-pedal))
    (send *drcv* :add-handle
          (make-cascoords :coords (send (elt (send *drcv* :links) 10) :copy-worldcoords)
                          :name :hand-brake))
    )
  )

(defun cheat-ride-drcv-eus
  (&key (mode :drive))
  (sit-manip-pose)
  (case mode
    (:murooka (send *atlas* :move-to (make-coords :pos #f(-48.855 -500.837 1136.816) :rpy #f(0 -0.3 -1.355e-20)) :world))
    (:assistant (send *atlas* :move-to (make-coords :pos #f(-48.855 -200.837 1136.816) :rpy #f(0.0 -0.3 -1.355e-20)) :world)) ;; center
    (:center (send *atlas* :move-to (make-coords :pos #f(-48.855 0.837 1136.816) :rpy #f(0.0 -0.3 -1.355e-20)) :world)) ;; center
    (:drive (send *atlas* :move-to (make-coords :pos #f(-48.855 200.837 1136.816) :rpy #f(0.0 -0.3 -1.355e-20)) :world)) ;; drive seat
    )
  )

(defun cheat-ride-drcv-drcsim
  (&key (real t) (nominal t) (mode :drive))
  (let ((msg (instance geometry_msgs::Pose :init)) (time 3.0))
    (case mode 
      ;; assistant seat
      (:assistant (send msg :position :y -0.6)
                  (send msg :position :z 0.01)
                  )
      ;; center
      (:center (send msg :position :y -0.4)
               (send msg :position :z 0.01)
               )
      ;; drive seat
      (:drive (send msg :position :y -0.2)
              (send msg :position :z 0.01)
              )
      )
    (while (> time 0)
      (ros::publish "/drc_world/robot_enter_car" msg)
      (unix::sleep 1)
      (setq time (1- time))
      )
    (when nominal
      (set-nominal)
      )
    )
  )

;; (defun sit-pose
;;   (&key (real nil))
;;   (send *atlas* :angle-vector
;;         #f(5.99088 18.2231 1.22162 51.1004 82.7136 14.3384
;;                    -77.9845 16.0836 -29.054 -27.1039 91.4626 2.21792
;;                    79.9998 15.0057 -30.8494 49.4995 -13.1432 -48.3562
;;                    28.3614 -21.7029 5.78518 29.0349 2.74359 -45.669
;;                    -28.3614 -10.2848 18.3346 -5.75568)
;;         )
;;   (when real
;;     (model2real)
;;     )
;;   )

;; need more leg_mhx angle?
(defun sit-manip-pose
  (&key (real nil))
  (send *atlas* :reset-manip-pose)
  (send *atlas* :lleg :angle-vector #f(70.0 0.057931 -65.0 0.0 -5.0 -0.013939))
  (send *atlas* :rleg :angle-vector #f(70.0 0.057931 -65.0 0.0 -5.0 -0.013939))  
  (when real
    (model2real)
    )
  )

(defun move-to-sit-seat-pose
  (&key (real nil))
  (real2model)
  ;; (set-high-gain :arms)
  ;; insert lleg
  (send *atlas* :lleg-knee-p :joint-angle 140)
  (send *atlas* :lleg-crotch-y :joint-angle -10)
  (when real
    (model2real :wait t)
    (warn "waiting...") (read-line)
    )
  (send *atlas* :lleg :angle-vector
        #f(7.61529 -18.242 -23.3566 26.929 17.3499 -18.3346))
  (set-high-gain :legs)
  (when real
    (model2real :wait t)
    (warn "waiting...") (read-line)
    )
  ;; preserve larm 
  (let ((av (send *atlas* :larm :angle-vector)))
    (set-seat-pose)
    (send *atlas* :larm :angle-vector av))
  (set-default-gain)
  (when real
    (model2real :wait t)
    )
  (warn "waiting...") (read-line)
  (ungrasp :arm :larm :real real)
  (set-seat-pose)
  (when real
    (model2real :wait t)
    )
  )

(defun murooka-end-pose
  (&key (real nil))
  ;; init pose
  (reset-pose)
  (model2real)
  (cheat-ride-drcv-drcsim :real real :nominal t :mode :assistant)
  (cheat-ride-drcv-eus :mode :murooka)
  (send *atlas* :larm :inverse-kinematics (send (send *drcv* :handle :name :upper-right-pipe) :worldcoords) :rotation-axis :z :debug-view t)
  (send *atlas* :rarm :angle-vector #f(0 90.0 0.0 60.0 0.0 0.0))

  ;; #f(6.34609 -44.1048 15.2562 48.2583 19.4723 89.5129 -79.7713 -85.3878
  ;;            11.0342 -44.243 0.0 24.981 79.5049 -5.34668 76.0922 30.3127 5.32417
  ;;            -38.7813 -26.929 2.94764 27.8039 97.4379 11.949 -98.8007 -16.7553
  ;;            -1.76944 -13.4926 -35.0))
  ;; #f(7.01098 44.2812 -40.3596 37.166 46.6875 21.9166 -37.3478 -89.9423 26.7111 -113.661 83.0993 24.981 -20.2294
  ;;            30.0854 -89.8809 89.8312 22.6494 30.0099 -26.8675 16.0729 46.4267 80.0454 -14.4574 25.2868 -1.07875 33.0919 4.83296 -35.0)

  )

(defun approach-to-handle-brake
  (&key (real nil))
  (let ((tgt-hand-brake-coords (send (send *drcv* :handle :name :hand-brake) :copy-worldcoords)))
    (send tgt-hand-brake-coords :locate #f(-100 0 100) :local)
    (real2model)
    (send *atlas* :torso-waist-p :joint-angle 40)
    (send *atlas* :rarm :inverse-kinematics tgt-hand-brake-coords
          :rotation-axis nil :debug-view t)
    (when real
      (model2real))
    )
  )

(defun slide-on-drive-seat
  (&key (real t))
  (real2model)
  (pre-slide-pose)
  (model2real)
  (warn "waiting...") (read-line)
  (hook-pose)
  (model2real)
  (warn "waiting...") (read-line)
  (slide-pose)
  (model2real)
  )

(defun pre-slide-pose
  (&key (init nil) (real t))
  (when init
    (sit-manip-pose)
    )
  (send *atlas* :larm-shoulder-r :joint-angle 0)
  (send *atlas* :larm-shoulder-y :joint-angle 0)
  (send *atlas* :larm-wrist-p :joint-angle 0)
  (send *atlas* :larm-elbow-y :joint-angle 180)
  (send *atlas* :larm-elbow-p :joint-angle 0)
  )

(defun hook-pose
  (&key (init nil) (real t))
  (when init
    (sit-manip-pose)
    )
  (send *atlas* :lleg-crotch-p :joint-angle -90)
  (send *atlas* :rleg-crotch-p :joint-angle -90)  
  (send *atlas* :larm-shoulder-r :joint-angle -50)
  )

(defun slide-pose
  (&key (init nil) (real t))
  (when init
    (sit-manip-pose)
    )
  (send *atlas* :larm-wrist-p :joint-angle 0)
  (send *atlas* :larm-elbow-p :joint-angle 135)
  )
