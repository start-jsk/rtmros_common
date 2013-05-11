
;;
;; for grasping tool
;;
(defun atlas-tool-look ()
  (send *atlas* :head :neck-p :joint-angle 50)
  (send *ri* :angle-vector (send *atlas* :angle-vector) 2000)
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

  (send *ri* :angle-vector (send *atlas* :angle-vector) 5000)
  (send *ri* :wait-interpolation)

  (send *atlas* :rarm :inverse-kinematics (make-coords :pos #f(410 -315 215)) :rotation-axis :z)
;;  (send *atlas* :rarm :inverse-kinematics (make-coords :pos #f(370 -350 200)) :rotation-axis :z)
;;  (send *atlas* :move-centroid-on-foot :both '(:rleg :lleg :rarm :larm))
  (send *ri* :angle-vector (send *atlas* :angle-vector) 5000)
  (send *ri* :wait-interpolation)

  (send *ri* :start-grasp :rarm)
  (unix:usleep 3)

  (send *atlas* :rarm :inverse-kinematics (make-coords :pos #f(565 15 400)) :rotation-axis :z :use-torso t)
;;  (send *atlas* :rarm :inverse-kinematics (make-coords :pos #f(550 0 400)) :rotation-axis :z :use-torso t)
;;  (send *atlas* :move-centroid-on-foot :both '(:rleg :lleg :rarm :larm))
  (send *ri* :angle-vector (send *atlas* :angle-vector) 5000)
  (send *ri* :wait-interpolation)

  (send *ri* :stop-grasp :rarm)
  (unix:usleep 3)
  )

;;
;; for footstepping
;;
(defun publish-base-footprint (&optional (x 0) (y 0) (th 0)) ;; 425 1550 0
  (send *atlas* :angle-vector (send *ri* :state :potentio-vector))
  (let ((fmd (send *atlas* :foot-midcoords))
        icds cds)
    (send fmd :translate (float-vector 48 0 0)) ;; too big ??
    ;;(setq cds (send (send *atlas* :pelvis :worldcoords) :transformation fmd))
    (setq icds (send fmd :transformation (send *atlas* :pelvis :worldcoords)))
    ;;(send *tfb* :send-transform cds "/pelvis" "/base_footprint")
    ;;(send *tfb* :send-transform cds "/odom" "/base_footprint")
    (send *tfb* :send-transform icds "/base_footprint" "/odom")
    (send *tfb* :send-transform (make-coords :pos (float-vector x y 0.0) :rpy (list th 0 0))
          "/map" "/base_footprint")
    ))

(defun view-footsteps (&optional (x 0.0) (y 0.0) (th 0) (dst (make-coords :pos (float-vector 6250 1550 0))))
  (publish-base-footprint x y th)
  (let ((footsteps
         (footstep-planning-armin (make-coords :pos (float-vector x y 0) :rpy (list th 0 0))
                                  dst)))
    (when (y-or-n-p "Can you apply these footsteps ? ")
      (send *ri* :bdi_footstep footsteps :swing-height 0.31))
    ))

(defun demo-stepping (&optional (init t) (teleport nil) (exec t))
  (when init (atlas-init :set-user-mode nil))
  (setq *points-marker-topicname* "/points_marker")
  (ros::advertise *points-marker-topicname* visualization_msgs::Marker)

  (when teleport
    (send *ri* :teleport-to (make-coords :pos (float-vector 16150 8000 2000)))
    (unix::sleep 3)
    (send *ri* :bdi-go-pos 0.2 0 0)
    (warn "input any key!!")
    (do-until-key)
    (read-line))

  (start-tilt-laser 1.2)
  (get-tilt-laser) ;; ignore first one
  (setq *laser-pt* (get-tilt-laser))
  (setq *laser-msg* (pointcloud->marker-msg
                     *laser-pt*
                     (instance std_msgs::header :init :frame_id "/pelvis")
                     :color #f(0 1 1)))
  (walking-on-stepping-block)
  )

(defun walking-on-stepping-block (&optional (xx 500.0) (yy 1550.0) (th 0.0))
  (let ((x-pos xx)
        (y-pos yy)
        (th-pos th)
        (pos-diff 5.0)
        (th-diff 0.01)
        (moved nil))
    (publish-base-footprint x-pos y-pos th-pos)
    (ros::publish *points-marker-topicname* *laser-msg*)
    (block :loop
      (while t
        (do-until-key
         (let ((str (read-line)))
           ;;(pprint str)
           (cond
            ((string= "h" str)
             (setq y-pos (+ y-pos pos-diff) moved t)
             )
            ((string= "j" str)
             (setq x-pos (- x-pos pos-diff) moved t)
             )
            ((string= "k" str)
             (setq x-pos (+ x-pos pos-diff) moved t)
             )
            ((string= "l" str)
             (setq y-pos (- y-pos pos-diff) moved t)
             )
            ((string= "r" str)
             (setq th-pos (- th-pos th-diff) moved t)
             )
            ((string= "R" str)
             (setq th-pos (+ th-pos th-diff) moved t)
             )
            ((string= "+" str)
             (setq pos-diff (* 2 pos-diff))
             (setq th-diff (* 2 th-diff))
             )
            ((string= "-" str)
             (setq pos-diff (* 0.5 pos-diff))
             (setq th-diff (* 0.5 th-diff))
             )
            ((string= "=" str)
             (setq pos-diff 5.0
                   th-diff 0.01)
             )
            ((string= "u" str)
             (setq moved t)
             )
            ((string= "w" str)
             ;; wipe step..
             )
            ((string= "f" str)
             (publish-base-footprint x-pos y-pos th-pos)
             (ros::publish *points-marker-topicname* *laser-msg*)
             (view-footsteps x-pos y-pos th-pos)
             )
            ((string= "z" str)
             (setq x-pos 350.0
                   y-pos 1550.0
                   th-pos 0.0)
             )
            ((string= "a" str)
             (publish-base-footprint x-pos y-pos th-pos)
             (ros::publish *points-marker-topicname* *laser-msg*)
             (view-footsteps x-pos y-pos th-pos
                             (make-coords :pos (float-vector 400 1550 0)))
             )
            ((string= "s" str)
             (start-tilt-laser 1.2)
             (get-tilt-laser) ;; ignore first one
             (setq *laser-pt* (get-tilt-laser))
             (setq *laser-msg* (pointcloud->marker-msg
                                *laser-pt*
                                (instance std_msgs::header :init :frame_id "/pelvis")
                                :color #f(0 1 1)))
             (setq moved t)
             )
            ((string= "q" str)
             (return-from :loop)
             ))
           (cond
            (moved
             (setq moved nil)
             (warn "~A ~A ~A~%" x-pos y-pos th-pos)
             (publish-base-footprint x-pos y-pos th-pos)
             (ros::publish *points-marker-topicname* *laser-msg*))
            (t
             (publish-base-footprint x-pos y-pos th-pos)
             ))
           )))
        ))
    ))
