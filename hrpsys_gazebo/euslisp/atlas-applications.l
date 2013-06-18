
;;
;; for grasping tool
;;

(defun get-point (&key (wait t) (timeout 15.0))
  (ros::spin-once)
  (if wait
      (progn
        (send *ri* :put :point nil)
        (while (not (send *ri* :get :point))
          (unix::usleep (* 100 1000))
          (decf timeout 0.1)
          (if (< timeout 0.0)
              (return-from get-point))
          (ros::spin-once)))
    (unless (send *ri* :get :point)
      (return-from get-point)))
  (make-coords :pos (send (send *atlas* :worldcoords) :transform-vector (send *ri* :get :point))))

(defun get-pipe (&key (draw nil))
  (let* ((p1 (get-point))
	 (p2 (get-point))
	 (vy (send (send *atlas* :worldcoords) :rotate-vector (float-vector 1 0 0)))
	 (vz (normalize-vector (v- (send p1 :pos) (send p2 :pos))))
	 (vx (v* vy vz))
	 (pos (midpoint 0.5 (send p1 :pos) (send p2 :pos)))
	 ret)
    (setq ret (make-coords :pos pos :rot (transpose (make-matrix 3 3 (list vx vy vz)))))
    (when draw
      (send p1 :draw-on :flush t :size 200 :width 5)
      (send p2 :draw-on :flush t :size 200 :width 5)
      (send ret :draw-on :flush t :size 200 :width 5 :color #f(1 0 0)))
    ret))

(defun reach-point (&key (real t) (arm :larm))
  (send *atlas* arm :inverse-kinematics (get-point) :rotation-axis nil)
  (when real (model2real)))

(defun reach-pipe (&key (real t) (arm :larm))
  (send *atlas* arm :inverse-kinematics (get-pipe :draw t) :translation-axis t :rotation-axis :z :rthre (/ pi 10) :debug-view nil)
  (draw-end-coords arm)
  (when real (model2real)))

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
      (send *ri* :bdi_footstep footsteps :swing-height 0.31)
      t)
    ))

(defun move-to-viewpose ()
  (send *atlas* :angle-vector
        (send *ri* :state :potentio-vector))
  (send *ri* :angle-vector
        (send *atlas* :angle-vector) 1200)
  (send *ri* :wait-interpolation)
  (send *ri* :set-simulation-mode :mode :user)
  ;;
  (send *atlas* :stand-pose)
  (send *atlas* :head :neck-p :joint-angle 20)
  ;;
  (send *ri* :angle-vector
        (send *atlas* :angle-vector) 2400)
  (send *ri* :wait-interpolation)
  (send *ri* :set-simulation-mode :mode :stand))

(defun move-to-standpose ()
  (send *atlas* :angle-vector
        (send *ri* :state :potentio-vector))
  (send *ri* :angle-vector
        (send *atlas* :angle-vector) 1200)
  (send *ri* :wait-interpolation)
  (send *ri* :set-simulation-mode :mode :user)
  ;;
  (send *atlas* :stand-pose)
  ;;
  (send *ri* :angle-vector
        (send *atlas* :angle-vector) 2400)
  (send *ri* :wait-interpolation)
  (send *ri* :set-simulation-mode :mode :stand))

(defun get-laser-pointcloud (&key (tilt-speed 1.2) (crop t))
  (start-tilt-laser tilt-speed)
  (get-tilt-laser) ;; ignore first one
  (setq *laser-pt* (get-tilt-laser))
  (start-tilt-laser 0.0) ;; stop tilting
  ;;
  (cond
   (crop
    (send *laser-pt* :filter
          :key #'(lambda (p) (and (< (elt p 0) 8000.0) (< -7000 (elt p 1) 7000))))
    (setq *laser-msg* (pointcloud->marker-msg
                       *laser-pt*
                       (instance std_msgs::header :init :frame_id "/pelvis")
                       :color #f(0 1 1)))
    *laser-msg*)
   (t
    *laser-pt*
    ))
  )

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

  (get-laser-pointcloud)

  (walking-on-stepping-block)
  )

;;
;; parameters
;;
(defvar *initial-x* 850.0)
(defvar *initial-y* 1550.0)
;;
(defvar *mid0-destination-x* 1550.0)
(defvar *mid0-destination-y* 1550.0)
(defvar *mid0-destination-th* 0.0)
;;
;;(defvar *mid-destination-y* 2050.0)
(defvar *mid-destination-x* 2120.0)
;;(defvar *mid-destination-y* 2050.0)
(defvar *mid-destination-y* 2075.0)
(defvar *mid-destination-th* 0.0)
;;
(defvar *mid2-destination-x* 2750.0)
(defvar *mid2-destination-y* 2100.0)
(defvar *mid2-destination-th* 0.0)
;;
(defvar *mid3-destination-x* 3250.0)
(defvar *mid3-destination-y* 1550.0)
(defvar *mid3-destination-th* 0.0)
;;
(defvar *final-destination-x* 6250.0)
(defvar *final-destination-y* 1550.0)
(defvar *final-destination-th* 0.0)
;; 1150
;;              2050 (200) 2750
;;    (150) 1550               3250 (150)
;;                                       3650
;;

(setq *laser-msg* nil)
(defun walking-on-stepping-block (&optional (xx 400.0) (yy 1550.0) (th 0.0))
  (let ((x-pos xx)
        (y-pos yy)
        (th-pos th)
        (pos-diff 5.0)
        (th-diff 0.01)
        (moved nil))
    (publish-base-footprint x-pos y-pos th-pos)
    (when *laser-msg*
      (ros::publish *points-marker-topicname* *laser-msg*))
    (warn ";; input commands ...~%");;
    (while t
      (do-until-key
       (publish-base-footprint x-pos y-pos th-pos)
       (unix::usleep (* 50 1000)))
      (let ((str (read-line nil nil)))
        (cond ;; selecting command
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
          (warn ";; trans: ~A, rotate: ~A~%" pos-diff th-diff)
          )
         ((string= "-" str)
          (setq pos-diff (* 0.5 pos-diff))
          (setq th-diff (* 0.5 th-diff))
          (warn ";; trans: ~A, rotate: ~A~%" pos-diff th-diff)
          )
         ((string= "=" str)
          (setq pos-diff 5.0
                th-diff 0.01)
          (warn ";; trans: ~A, rotate: ~A~%" pos-diff th-diff)
          )
         ((string= "z" str)
          (setq x-pos *initial-x*
                y-pos *initial-y*
                th-pos 0.0)
          )
         ((string= "Z" str)
          (setq x-pos *mid-destination-x*
                y-pos *mid-destination-y*
                th-pos 0.0)
          )
         ((string= "u" str)
          (setq moved t)
          )
         ((string= "w" str)
          ;; wipe step..
          )
         ((string= "f" str) ;; footstep to goal
          (publish-base-footprint x-pos y-pos th-pos)
          (ros::publish *points-marker-topicname* *laser-msg*)
          (view-footsteps x-pos y-pos th-pos
                          (make-coords :pos (float-vector *final-destination-x* *final-destination-y* 0)))
          )
         ((or (string= "A" str) (string= "a0" str) (string= "a" str)
              (string= "a1" str) (string= "a2" str) (string= "a3" str)) ;; footstep to each block
          (let (ppx ppy tth)
            (cond
             ((string= "A" str)
              (setq ppx *initial-x*
                    ppy *initial-y*
                    tth 0.0))
             ((string= "a0" str)
              (setq ppx *mid0-destination-x*
                    ppy *mid0-destination-y*
                    tth *mid0-destination-th*))
             ((or (string= "a" str)
                  (string= "a1" str))
              (setq ppx *mid-destination-x*
                    ppy *mid-destination-y*
                    tth *mid-destination-th*))
             ((string= "a2" str)
              (setq ppx *mid2-destination-x*
                    ppy *mid2-destination-y*
                    tth *mid2-destination-th*))
             ((string= "a3" str)
              (setq ppx *mid3-destination-x*
                    ppy *mid3-destination-y*
                    tth *mid3-destination-th*)))
            (publish-base-footprint x-pos y-pos th-pos)
            (ros::publish *points-marker-topicname* *laser-msg*)
            (when
                (view-footsteps x-pos y-pos th-pos
                                (make-coords :pos (float-vector ppx ppy tth)))
              (setq x-pos ppx
                    y-pos ppy
                    th-pos tth)))
          )
         ((string= "s" str) ;; update laser
          (get-laser-pointcloud)
          (setq moved t)
          )
         ((string= "v" str)
          (move-to-viewpose)
          )
         ((string= "V" str)
          (move-to-stand-pose)
          )
         ((string= "q" str)
          (return)
          ))
        ;; check moved
        (cond
         (moved
          (setq moved nil)
          (warn "~A ~A ~A~%" x-pos y-pos th-pos)
          (publish-base-footprint x-pos y-pos th-pos)
          (when *laser-msg*
            (ros::publish *points-marker-topicname* *laser-msg*)))
         (t
          (publish-base-footprint x-pos y-pos th-pos)
          ))
        ))
    ))

#|
(demo-stepping)
hjkl 並行移動
rR 回転（R逆回転R）
+- 移動と回転の量の調整
=  移動と回転の量のリセット
a  2番目のブロックまでfootstep
f  ゴールまでfootstep
A  初期位置までfootstep（初期位置がずれている場合）
s  laserのアップデート（移動したあとは必須。。。）

(demo-stepping nil t)
ステップブロック前まで歩くのを省略できます。
とするとテレポート（予選では使ってはいけないコマンド）して始めます。
|#
