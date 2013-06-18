#|
使い方
  drcsimが起動してから
  roseus atlas-sample.l
  (init)    ;; robot-interfaceをつなぐ、k-effortを255にセットする
  (fall-down)などで適当にこかす

  *仰向けの場合
    -> (stand-up)  (途中で足が地面につくまでread-lineで末ようになっているので、ついたらenterをオス
  *うつ伏せの場合
    -> (rolling-over)  これは放置しておけばいい
  *かえるとび?
    -< (frog)  do-until-keyなので、enterおすまでジャンプしつづける
|#
;;(require "package://hrpsys_gazebo/euslisp/atlas-interface.l")

;;
(defun init
  (&key (fall nil))
  (if (not (boundp '*atlas*)) (setq *atlas* (instance atlas-robot :init)))
  (if (not (boundp '*irtviewer*)) (make-irtviewer))
  (cond
   (fall
    (send *atlas* :torso-waist-p :joint-angle 40)
    (send *atlas* :lleg-ankle-p :joint-angle -20)
    (send *atlas* :rleg-ankle-p :joint-angle -20)
    (atlas-init :set-reset-pose nil))
   (t
    (atlas-init :set-reset-pose nil)
    (adapt-pose)
    ))
  (send *ri* :angle-vector (send *atlas* :angle-vector))
  (set-k-effort)
  ;;(setq r *atlas*)

  ;;(imu-init)                  ;;subscribe /atlus/imu
  ;;(forcetorque-init)          ;;subscribe /atlus_msg/ForceTorqueSensor
  (gain-setting-for-standup)

  (objects (list *atlas*))
  )

(defun gain-setting-for-standup ()
  ;;; setting for stand-up
  (send *ri* :set-servo-gain-percentage "back_lbz" 1000.0)
  (send *ri* :set-servo-gain-percentage "back_mby" 200.0)
  (send *ri* :set-servo-gain-percentage "back_ubx" 200.0)
  (dolist (j (flatten (send *atlas* :arms :joint-list)))
    (send *ri* :set-servo-gain-percentage (send j :name) 150.0))
  (dolist (j (flatten (send *atlas* :legs :joint-list)))
    (send *ri* :set-servo-gain-percentage (send j :name) 100.0))
  (send *ri* :set-servo-gain-percentage "l_leg_uhz" 1000.0)
  (send *ri* :set-servo-gain-percentage "r_leg_uhz" 1000.0)
  )

;;わざと転倒させる (前に傾く）
(defun fall-down
  (&key (run t) (tm 3000))
  (send *atlas* :reset-pose)
  (send *atlas* :torso-waist-p :joint-angle 40)
  (send *atlas* :lleg-ankle-p :joint-angle -20)
  (send *atlas* :rleg-ankle-p :joint-angle -20)
  (send *atlas* :larm :angle-vector (float-vector   90 17 0.0 -27 -93 0.0))
  (send *atlas* :rarm :angle-vector (float-vector  -90 17 0.0  27 -93 0.0))
  (send *irtviewer* :draw-objects)
  (when run (send-pose tm))
  )

;;とりあえずeusから動くようにする (ひどい実装orz)
(defun set-k-effort
  (&optional (k-effort 255))
#|
  (let (com)
    (setq com "rostopic pub -1 /atlas/atlas_sim_interface_command atlas_msgs/AtlasSimInterfaceCommand '{behavior: 1, k_effort: [")
    (dotimes (i 27)
      (setq com (concatenate string com (format nil "~a, " k-effort))))
    (setq com (concatenate string com 
		   (format nil "~a]}'" k-effort)))
    (unix:system com)
    )
|#
  (send *ri* :set-simulation-mode :mode :user :effort k-effort)
  )

#|
 起き上がり
 0. うつ伏せになる
 1. まっすぐにする (pose1)
 2. 足を曲げる、足首を地面と水平にする2 (pose2)
|#
;; 1. まっすぐにする
(defun pose1 (&key (run t) (tm 3000))
  (send *atlas* :lleg :angle-vector #f(0 0 0 0 0 0))
  (send *atlas* :rleg :angle-vector #f(0 0 0 0 0 0))
  (send *atlas* :torso-waist-p :joint-angle 0)
  (send *atlas* :torso-waist-r :joint-angle 0)
  (send *atlas* :torso-waist-y :joint-angle 0)
  (send *atlas* :larm :angle-vector #f(0 0 0 0 0 0 ))
  (send *atlas* :rarm :angle-vector #f(0 0 0 0 0 0 ))
  ;;
  (send *atlas* :lleg-crotch-p :joint-angle 10)
  (send *atlas* :rleg-crotch-p :joint-angle 10)

  (send *irtviewer* :draw-objects)
  (when run (send-pose tm))
  )
;;  2. 足を曲げる、足首を地面と水平にする2
;;     腰を曲げる、腕を曲げる
;;    (土下座のイメージ)
(defun pre-pose2 (&key (run t) (tm 1000))
  ;;(send *atlas* :lleg-knee-p :joint-angle 140)   ;;140が可動域
  ;;(send *atlas* :rleg-knee-p :joint-angle 140)
  (send *atlas* :lleg-ankle-p :joint-angle -20)  ;;-40が可動域
  (send *atlas* :rleg-ankle-p :joint-angle -20)
  (send *atlas* :lleg-ankle-r :joint-angle -10)
  (send *atlas* :rleg-ankle-r :joint-angle 10)

  ;;(send *atlas* :lleg-crotch-p :joint-angle -100)  ;;-100が可動域
  ;;(send *atlas* :rleg-crotch-p :joint-angle -100)
  ;;(send *atlas* :lleg-crotch-y :joint-angle 30)
  ;;(send *atlas* :rleg-crotch-y :joint-angle -30)
  (send *atlas* :rleg-crotch-r :joint-angle -10)
  (send *atlas* :lleg-crotch-r :joint-angle 10)
  (send *irtviewer* :draw-objects)
  (when run (send-pose tm))
  )
(defun pose2 (&key (run t) (tm 3000))
  ;;足を曲げる
  (send *atlas* :lleg-knee-p :joint-angle 140)   ;;140が可動域
  (send *atlas* :rleg-knee-p :joint-angle 140)
  (send *atlas* :lleg-ankle-p :joint-angle -40)  ;;-40が可動域
  (send *atlas* :rleg-ankle-p :joint-angle -40)
  (send *atlas* :lleg-ankle-r :joint-angle -30)
  (send *atlas* :rleg-ankle-r :joint-angle 30)

  (send *atlas* :lleg-crotch-p :joint-angle -100)  ;;-100が可動域
  (send *atlas* :rleg-crotch-p :joint-angle -100)
  (send *atlas* :lleg-crotch-y :joint-angle 30)
  (send *atlas* :rleg-crotch-y :joint-angle -30)
  (send *atlas* :rleg-crotch-r :joint-angle -30)
  (send *atlas* :lleg-crotch-r :joint-angle 30)

  ;;(send *atlas* :lleg-crotch-p :joint-angle -150)   ;;いまいち
  ;;(send *atlas* :rleg-crotch-p :joint-angle -150)
  ;;腰を曲げる
  (send *atlas* :torso-waist-p :joint-angle 40)
  ;;腕を曲げる
  (send *atlas* :larm-shoulder-y :joint-angle -80)
  (send *atlas* :larm-shoulder-r :joint-angle -40)
  (send *atlas* :larm-elbow-p :joint-angle 80)
  (send *atlas* :larm-elbow-y :joint-angle -20)

  (send *atlas* :rarm-shoulder-y :joint-angle -80)
  (send *atlas* :rarm-shoulder-r :joint-angle 40)  ;;これは左右が逆？
  (send *atlas* :rarm-elbow-p :joint-angle -80)    ;;これは左右が逆？
  (send *atlas* :rarm-elbow-y :joint-angle -20)
  ;;
  (send *irtviewer* :draw-objects)
  (when run (send-pose tm))
  )

;; 3. 手と腰を使ってシコ状態に遷移させる
(defun pose3 (&key (run t) (tm 5000))
  (pose2 :run nil)
  ;;
  (send *atlas* :larm-elbow-p :joint-angle 20)
  (send *atlas* :rarm-elbow-p :joint-angle -20)
  ;;(send *atlas* :torso-waist-p :joint-angle 70)
  (send *atlas* :torso-waist-p :joint-angle 50)
  ;;
  (send *atlas* :larm :shoulder-r :joint-angle -55)
  (send *atlas* :rarm :shoulder-r :joint-angle 55)
  ;;
  (send *irtviewer* :draw-objects)
  (when run (send-pose tm))
  )
;;
;; 4. 腰を元に戻し、足もあげていく
(defun pose4-1 (&key (run nil) (tm 4000))
  (pose2 :run nil)
  ;;
  (send *atlas* :larm-elbow-p :joint-angle 10)
  (send *atlas* :rarm-elbow-p :joint-angle -10)
  (send *atlas* :larm-shoulder-y :joint-angle -20)
  (send *atlas* :rarm-shoulder-y :joint-angle -20)
  ;;
  (send *atlas* :torso-waist-p :joint-angle 20)
  ;;
#|
  (send *atlas* :lleg-crotch-p :joint-angle -60)  ;;-100が可動域
  (send *atlas* :rleg-crotch-p :joint-angle -60)
  (send *atlas* :lleg-crotch-y :joint-angle 30)
  (send *atlas* :rleg-crotch-y :joint-angle -30)

  (send *atlas* :rleg-crotch-r :joint-angle -10)
  (send *atlas* :lleg-crotch-r :joint-angle 10)
  ;;
  (send *atlas* :lleg-knee-p :joint-angle 80)   ;;140が可動域
  (send *atlas* :rleg-knee-p :joint-angle 80)
  (send *atlas* :lleg-ankle-p :joint-angle -20)  ;;-40が可動域
  (send *atlas* :rleg-ankle-p :joint-angle -20)

  (send *atlas* :lleg-ankle-r :joint-angle -10)
  (send *atlas* :rleg-ankle-r :joint-angle 10)
|#
  ;;
  (send *irtviewer* :draw-objects)
  (when run (send-pose tm))
  )

;; 4. 腰を元に戻し、足もあげていく (中間姿勢２：一度にゆっくりしたとしてもこけるっぽい, なんで？)
(defun pose4-2 (&key (run nil) (tm 4000))
  (pose4-1 :run nil)
  ;;
  (send *atlas* :larm-shoulder-y :joint-angle 0)
  (send *atlas* :rarm-shoulder-y :joint-angle 0)
  ;;
  (send *atlas* :lleg-crotch-p :joint-angle -80)  ;;-100が可動域
  (send *atlas* :rleg-crotch-p :joint-angle -80)
  (send *atlas* :lleg-crotch-y :joint-angle 30)
  (send *atlas* :rleg-crotch-y :joint-angle -30)
  (send *atlas* :rleg-crotch-r :joint-angle -10)
  (send *atlas* :lleg-crotch-r :joint-angle 10)
  ;;
  (send *atlas* :lleg-knee-p :joint-angle 100)   ;;140が可動域
  (send *atlas* :rleg-knee-p :joint-angle 100)
  (send *atlas* :lleg-ankle-p :joint-angle -40)  ;;-40が可動域
  (send *atlas* :rleg-ankle-p :joint-angle -40)
  (send *atlas* :lleg-ankle-r :joint-angle -20)
  (send *atlas* :rleg-ankle-r :joint-angle 20)
  ;;
  (send *irtviewer* :draw-objects)
  (when run (send-pose tm))
  )

;; 4. 腰を元に戻し、足もあげていく (中間姿勢3：一度にゆっくりしたとしてもこけるっぽい, なんで？)
(defun pose4-3 (&key (run nil) (tm 4000))
  (pose4-1 :run nil)
  ;;
  (send *atlas* :torso-waist-p :joint-angle -5)
  ;;
  (send *atlas* :lleg-crotch-p :joint-angle -40)  ;;-100が可動域
  (send *atlas* :rleg-crotch-p :joint-angle -40)
  (send *atlas* :lleg-crotch-y :joint-angle 20)
  (send *atlas* :rleg-crotch-y :joint-angle -20)

  (send *atlas* :rleg-crotch-r :joint-angle -10)
  (send *atlas* :lleg-crotch-r :joint-angle 10)
  ;;
  (send *atlas* :lleg-knee-p :joint-angle 70)   ;;140が可動域
  (send *atlas* :rleg-knee-p :joint-angle 70)
  (send *atlas* :lleg-ankle-p :joint-angle -30)  ;;-40が可動域
  (send *atlas* :rleg-ankle-p :joint-angle -30)
  (send *atlas* :lleg-ankle-r :joint-angle -10)
  (send *atlas* :rleg-ankle-r :joint-angle 10)
  ;;
  (send *irtviewer* :draw-objects)
  (when run (send-pose tm))
  )

;; 4. 腰を元に戻し、足もあげていく　最終姿勢：pose4-4の姿勢は、walkの初期姿勢と同じ,この状態ならk-effortを0にしてもこけなくなる
(defun pose4-4 (&key (run nil) (tm 4000))
  (send *atlas* :angle-vector
        #f(-0.000611 0.076327 0.002034 28.5397 114.638 -0.264813 -74.6804 17.1935 0.017428 -28.5384 114.637 0.268241 74.6782 17.1937 0.018326 29.5172 -3.85612 -14.613 4.16939 -13.4334 17.3866 29.2973 4.51093 -14.4278 -5.09418 -13.4808 -17.3512 -0.061066))
  ;;
  (send *irtviewer* :draw-objects)
  (when run (send-pose tm))
  )

;;
(defun stand-up (&key (recover t) (time-scale 1.0) (wait-times 40))
  ;;recover = tの場合、最後こけてしまっていたらやりなおす
  ;;(send *ri* :start-grasp)
  (ungrasp :real t)
  ;;(send *ri* :stop-grasp)
  (tagbody
   :start
   ;;仰向けかうつ伏せかを判定し、仰向けならうつ伏せに移行
   (cond
    ((eq :aomuke (check_utubuse_or_not))
     (print "detect aomuke situation, do rolling-over")
     (rolling-over)
     )
    (t
     (print "set k-effort 255")
     (set-k-effort 255)))
   (unix:sleep 2)
   ;;
   (print "set pose1")
   (pose1 :tm (* time-scale 2000))
   (send *ri* :wait-interpolation)
   (print "set pose2")
   (pre-pose2 :tm (* time-scale 1000))
   (send *ri* :wait-interpolation)
   (pose2 :tm (* time-scale 2400))
   (send *ri* :wait-interpolation)
   (print "set pose3")
   (pose3 :tm (* time-scale 4000))
   (send *ri* :wait-interpolation)

   (warn "wait for totally landing, then push enter key~%")
   (let ((counter 0))
     (while (not (setq foot-on-ground-flag (check_feet_on_ground)))  ;;地面に足がつくまで
       (if (not foot-on-ground-flag)
           (print "  not yet foot on ground, then please wait..."))
       (unix:sleep 1)
       (incf counter)
       (when (> counter wait-times)
         (print "something wrong is occured! retry!")
         (pose1)
         (send *ri* :wait-interpolation)
         (unix:sleep 1)
         (when (eq :aomuke (check_utubuse_or_not))
           (print "detect aomuke situation, do rolling-over")
           (rolling-over)
           )
         (go :start)
         )))
   ;;
   (unix:sleep 1)
   (print "set pose4-1")
   (pose4-1 :tm (* time-scale 2000) :run t)
   (send *ri* :wait-interpolation)
   ;;
   (print "set pose4-2")
   (pose4-2 :tm (* time-scale 2000) :run t)
   (send *ri* :wait-interpolation)
   ;;
   (print "set pose4-3")
   (pose4-3 :tm (* time-scale 2000) :run t)
   (send *ri* :wait-interpolation)
   ;;
   ;;ここから怪しいがたまに成功する
   (print "set pose4-4(final pose)")
   (pose4-4 :tm (* time-scale 4000) :run t)
   (send *ri* :wait-interpolation)
#|
  (unix:sleep 1)
  (print "set k-effort 200")
  (set-k-effort 200)   ;;walking mode
  ;;(unix:sleep 3)
  (print "set k-effort 100")
  (set-k-effort 100)   ;;walking mode
  ;;(unix:sleep 3)
  (print "set k-effort 0")
  (set-k-effort 0)   ;;walking mode
|#
   ;;(unix:sleep 3)
   ;;失敗していたらやりなおす
   (when recover
     (ros::spin-once)
     ;;体幹のpitch姿勢が10の中に入っていなければこけていると思っていい
     (when (> (abs (elt *atlas_rpy* 1)) 10)   
       (print "detect standing-up failure, retry!")
       (unix:sleep 1)
       (go :start)))
   ))
;;
;;仰向け→うつ伏せになる
;;
;; 作戦、ブリッジしてバランスを崩す。
(defun rolling-over-old
  ()
  (pose1)
  (send *ri* :wait-interpolation)
  ;;
  ;;万歳
  (send *atlas* :rarm-shoulder-y :joint-angle 90)
  (send *atlas* :larm-shoulder-y :joint-angle 90)
  (send *atlas* :rarm-shoulder-r :joint-angle -90)
  (send *atlas* :larm-shoulder-r :joint-angle 90)
  (send *atlas* :larm-elbow-y :joint-angle -90)
  (send *atlas* :rarm-elbow-y :joint-angle 90)
  (send-pose 1500)
  (send *ri* :wait-interpolation)
  ;;
  ;;ブリッジ！
  (send *atlas* :rleg-crotch-p :joint-angle 0)
  (send *atlas* :lleg-crotch-p :joint-angle 0)
  ;;(send *atlas* :rleg-knee-p :joint-angle 90)
  (send *atlas* :rleg-knee-p :joint-angle 0)
  ;;(send *atlas* :lleg-knee-p :joint-angle 90)
  ;;(send *atlas* :rarm-elbow-p :joint-angle -90)
  (send *atlas* :larm-elbow-p :joint-angle 0)
  (send *atlas* :larm-shoulder-r :joint-angle -50)
  ;;(send *atlas* :lleg-crotch-p :joint-angle -140)
  ;;(send *atlas* :lleg-crotch-y :joint-angle -40)
  (send-pose 500)
  (send *ri* :wait-interpolation)
  (read-line)

  (send *atlas* :lleg-crotch-p :joint-angle 90)
  (send *atlas* :rarm-shoulder-r :joint-angle 0)
  (send-pose 500)
  (send *ri* :wait-interpolation)
  (read-line)

  (send *atlas* :larm-shoulder-r :joint-angle 150)
  (send-pose 500)
  (send *ri* :wait-interpolation)

  (pose1)

  ;;(send *atlas* :torso-waist-y :joint-angle 40)  ;;実機(simulator)は腰のyは動かない気がする
  ;;(send *atlas* :torso-waist-r :joint-angle 40)
  ;;(send-pose 15000)
  )

#| 
 がちゃがちゃやると回転している
|#
(defun rolling-over
  ()
  (set-k-effort 255)   ;;eus control mode
  (unix:usleep 1)
  (pose1)
  (send *ri* :wait-interpolation)
  ;;
  (pose2)
  (send *ri* :wait-interpolation)
  ;;
  (send *atlas* :rarm-shoulder-y :joint-angle 90)
  (send *atlas* :larm-elbow-p :joint-angle 0)
  (send *atlas* :rarm-elbow-p :joint-angle 0)
  (send *atlas* :larm-shoulder-r :joint-angle -80)
  (send *atlas* :rarm-shoulder-r :joint-angle 80)
  (send-pose)
  (send *ri* :wait-interpolation)
  ;;(read-line)
  (unix:sleep 5)
  ;;
  ;;(pose2 :run nil)
  (send *atlas* :torso-waist-p :joint-angle 0)
  (send *atlas* :lleg :angle-vector #f(0 0 0 0 0 0))
  (send *atlas* :rleg :angle-vector #f(0 0 0 0 0 0))
  (send *atlas* :larm-shoulder-y :joint-angle 90)
  (send *atlas* :rarm-shoulder-r :joint-angle 30)
  ;;(send *atlas* :rarm-shoulder-r :joint-angle -90)
  ;;(send *atlas* :larm :angle-vector #f(0 0 0 0 0 0))
  ;;(send *atlas* :larm-shoulder-r :joint-angle -90)
  (send-pose)
  (unix:sleep 5)
  ;;(read-line)
  (pose1)
  )
(defun rolling-over-reverse () ;; utubuse -> aomuke
  (set-k-effort 255)   ;;eus control mode
  (unix:usleep 1)
  (pose1)
  (send *ri* :wait-interpolation)
  (unix:sleep 2)
  ;;
  (pose2)
  (send *ri* :wait-interpolation)
  (unix:sleep 2)
  ;;
  (send *atlas* :rarm-elbow-p :joint-angle 0)
  (send *atlas* :larm-elbow-p :joint-angle 0)
  (send *atlas* :larm-shoulder-r :joint-angle -80)
  (send-pose)
  (send *ri* :wait-interpolation)
  (send *atlas* :rarm-shoulder-r :joint-angle 80)
  (send *atlas* :rarm-shoulder-y :joint-angle 90)
  (send-pose)
  (send *ri* :wait-interpolation)
  ;;(read-line)
  (unix:sleep 2)
  ;;
  ;;(pose2 :run nil)
  (send *atlas* :torso-waist-p :joint-angle 0)
  (send *atlas* :lleg :angle-vector #f(0 0 0 0 0 0))
  (send *atlas* :lleg :crotch-p :joint-angle -60)
  (send *atlas* :rleg :angle-vector #f(0 0 0 0 0 0))
  (send *atlas* :rarm-shoulder-r :joint-angle 30)
  (send-pose)
  (send *ri* :wait-interpolation)
  (unix:sleep 3)

  (send *atlas* :larm :angle-vector #f(0 0 0 -60 -20 0))
  (send-pose)
  (send *ri* :wait-interpolation)
  (unix:sleep 2)

  ;; skip ???
  (send *atlas* :larm :shoulder-y :joint-angle 60)
  (send *atlas* :rleg :crotch-p :joint-angle -40)
  (send-pose 2000)
  (send *ri* :wait-interpolation)
  (send *atlas* :rarm :shoulder-r :joint-angle -120)
  (send-pose 2000)
  (unix:sleep 2)

  (pose1)
  )

#|
 4足歩行モード（蛙飛びが一番楽かな)
|#
(defun frog-pose1 (&key (run t) (tm 3000))
  (pose1 :run nil)
  ;;
#|
  (send *atlas* :larm-shoulder-y :joint-angle -80)
  (send *atlas* :larm-shoulder-r :joint-angle -40)
  (send *atlas* :larm-elbow-p :joint-angle 0)
  (send *atlas* :larm-elbow-y :joint-angle -20)
  ;;
  (send *atlas* :rarm-shoulder-y :joint-angle -80)
  (send *atlas* :rarm-shoulder-r :joint-angle 40)  ;;これは左右が逆？
  (send *atlas* :rarm-elbow-p :joint-angle 0)    ;;これは左右が逆？
  (send *atlas* :rarm-elbow-y :joint-angle -20)
|#
  ;;
  ;;(send *atlas* :torso-waist-p :joint-angle -10)
  (send *atlas* :torso-waist-p :joint-angle 0)
  (send *atlas* :larm-shoulder-r :joint-angle 20)
  (send *atlas* :rarm-shoulder-r :joint-angle -20)
  (send *atlas* :lleg-ankle-p :joint-angle 20)
  (send *atlas* :rleg-ankle-p :joint-angle 20)
  (when run (send-pose tm))
  )

;;なるべく足部を地面に接触させるのがポイント
(defun frog-pose2 (&key (run t) (tm 3000))
  (pose2 :run nil)
  ;;
  (send *atlas* :torso-waist-p :joint-angle 40)
  (send *atlas* :lleg-knee-p :joint-angle 120)
  (send *atlas* :rleg-knee-p :joint-angle 120)
  (send *atlas* :larm-elbow-p :joint-angle 120)
  (send *atlas* :rarm-elbow-p :joint-angle -120)
  (when run (send-pose tm))
  )

(defun frog ()
  (do-until-key
   ;;(frog-pose2)
   (pose2)
   (send *ri* :wait-interpolation)
   (unix:sleep 3)
   ;;
   ;;(frog-pose1 :tm 500)
   (pose1 :tm 2000)
   (send *ri* :wait-interpolation)
   (unix:sleep 5))
  )

#|
  左右の足が地面にちゃんとついているかどうかの判定
  起き上がりの際に使う
  *atlas_forcetorque* に力センサの値は格納される
|#
(defun check_feet_on_ground (&optional (average-times 40))
  (let ((lf-force (instantiate float-vector 6))
        (rf-force (instantiate float-vector 6))
        (ret-flag nil) (cntr 0))
    (setq *atlas_forcetorque* nil)
    (while (< cntr average-times)
      (ros::spin-once)   ;;ROSのセンサtopicを更新
      (when  *atlas_forcetorque*
        (v+ (cdr (assoc :l_foot *atlas_forcetorque*)) lf-force lf-force)  ;; #f(fx fy fz mx my mz)
        (v+ (cdr (assoc :r_foot *atlas_forcetorque*)) rf-force rf-force)  ;; #f(fx fy fz mx my mz)
        (incf cntr)
        (pprint (list lf-force rf-force))
        (setq *atlas_forcetorque* nil))
      )
    (scale (/ 1.0 cntr) lf-force lf-force)
    (scale (/ 1.0 cntr) rf-force rf-force)

    ;;
    ;;なお、atlasは、足首力センサは３成分のみ fz, mx, my
    ;;  両足で地面にたっている条件
    ;;    lfz, rfz > 300
    ;;    root( lmx*lmx +  lmy*lmy ) < 33
    ;;    root( rmx*rmx +  rmy*rmy ) < 33
    (pprint (list
             (aref lf-force 2)
             (aref rf-force 2)
             (sqrt (+ (* (aref lf-force 3) (aref lf-force 3)) (* (aref lf-force 4) (aref lf-force 4))))
             (sqrt (+ (* (aref rf-force 3) (aref rf-force 3)) (* (aref rf-force 4) (aref rf-force 4))))))
    (if (and (or (and (> (aref lf-force 2) 270)
                      (> (aref rf-force 2) 270))
                 (> (+ (aref lf-force 2) (aref rf-force 2)) 700))
             (< (sqrt (+ (* (aref lf-force 3) (aref lf-force 3)) (* (aref lf-force 4) (aref lf-force 4)))) 33)
             (< (sqrt (+ (* (aref rf-force 3) (aref rf-force 3)) (* (aref rf-force 4) (aref rf-force 4)))) 33)
             )
        (setq ret-flag t))
    ret-flag))
#|
  仰向け？うつ伏せ？
|#
(defun check_utubuse_or_not
  ()
  (let (ret)
    (ros::spin-once)
    (cond
     ((> (elt *atlas_rpy* 1) 70)
      (setq ret :utubuse)
      )
     ((< (elt *atlas_rpy* 1) -70)
      (setq ret :aomuke))
     (t
      (setq ret :unknown)))
    ret))

;;;;;

;;安全のためエイリアスする
(warn "(stand-up)~%")
(defun stand-up ()
  (warn ";call (stand-up2)~%")
  (stand-up2))

(defun negaeri-from-aomuke
  ()
  (send *atlas* :angle-vector (fill (send *atlas* :angle-vector) 0))
  (send *atlas* :larm-shoulder-y :joint-angle -90)
  (send *atlas* :rarm-shoulder-y :joint-angle 90)
  (send *atlas* :torso-waist-y :joint-angle -35)
  (send *atlas* :rleg-crotch-p :joint-angle -90)
  (send *atlas* :lleg-crotch-p :joint-angle -90)
  (send *ri* :angle-vector (send *atlas* :angle-vector) 5000)
  (send *ri* :wait-interpolation)

  (send *atlas* :larm-shoulder-r :joint-angle 90)
  (send *atlas* :larm-shoulder-y :joint-angle -50)
  (send *atlas* :torso-waist-y :joint-angle 35)
  (send *atlas* :rarm-shoulder-r :joint-angle 60)
  (send *ri* :angle-vector (send *atlas* :angle-vector) 1000)
  (send *ri* :wait-interpolation)

  (send *atlas* :larm-shoulder-r :joint-angle -50)
  (send *atlas* :larm-shoulder-y :joint-angle 90)
  (send *ri* :angle-vector (send *atlas* :angle-vector) 1000)
  (send *ri* :wait-interpolation)

  (send *atlas* :rarm-shoulder-r :joint-angle 30)
  (send *atlas* :rarm-shoulder-y :joint-angle -90)
  (send *ri* :angle-vector (send *atlas* :angle-vector) 1000)
  (send *ri* :wait-interpolation)

  (send *atlas* :rleg-crotch-p :joint-angle 0)
  (send *atlas* :lleg-crotch-p :joint-angle 0)
  (send *atlas* :torso-waist-y :joint-angle 0)
  (send *ri* :angle-vector (send *atlas* :angle-vector) 1000)
  (send *ri* :wait-interpolation)

  (send *atlas* :angle-vector (fill (send *atlas* :angle-vector) 0))
  (send *ri* :angle-vector (send *atlas* :angle-vector) 1000)
  (send *ri* :wait-interpolation)
  )

(defun negaeri-from-utubuse
  ()
  (send *atlas* :angle-vector (fill (send *atlas* :angle-vector) 0))
  (send *atlas* :larm-shoulder-y :joint-angle -90)
  (send *atlas* :rarm-shoulder-y :joint-angle 90)
  (send *atlas* :torso-waist-y :joint-angle -35)
  (send *atlas* :rleg-crotch-p :joint-angle -90)
  (send *atlas* :lleg-crotch-p :joint-angle -90)
  (send *ri* :angle-vector (send *atlas* :angle-vector) 5000)
  (send *ri* :wait-interpolation)

  (send *atlas* :rarm-shoulder-r :joint-angle -90)
  (send *atlas* :rarm-shoulder-y :joint-angle 50)
  (send *atlas* :larm-shoulder-r :joint-angle -60)
  (send *ri* :angle-vector (send *atlas* :angle-vector) 1000)
  (send *ri* :wait-interpolation)

  (send *atlas* :rarm-shoulder-r :joint-angle 50)
  (send *atlas* :rarm-shoulder-y :joint-angle -90)
  (send *ri* :angle-vector (send *atlas* :angle-vector) 1000)
  (send *ri* :wait-interpolation)

  (send *atlas* :larm-shoulder-r :joint-angle -30)
  (send *atlas* :larm-shoulder-y :joint-angle 90)
  (send *atlas* :torso-waist-y :joint-angle -35)
  (send *atlas* :rleg-crotch-p :joint-angle 0)
  (send *atlas* :lleg-crotch-p :joint-angle 0)
  (send *ri* :angle-vector (send *atlas* :angle-vector) 1000)
  (send *ri* :wait-interpolation)

  (send *atlas* :angle-vector (fill (send *atlas* :angle-vector) 0))
  (send *ri* :angle-vector (send *atlas* :angle-vector) 1000)
  (send *ri* :wait-interpolation)
  )

(warn "(negaeri)~%")
(defun negaeri ;;どちらの向きから実行しても裏返る
  ()
  (if (< 10 (distance (fill (send *atlas* :angle-vector) 0) (send *atlas* :angle-vector)))
    (progn
      (warn ";set zero-pose to detect aomuke or utubuse~%")
      (send *atlas* :angle-vector (fill (send *atlas* :angle-vector) 0))
      (send *ri* :angle-vector (send *atlas* :angle-vector) 5000)
      (send *ri* :wait-interpolation)))
  (cond
    ((eq :aomuke (check_utubuse_or_not))
     (warn ";aomuke detected~%")
     (negaeri-from-aomuke))
    ((eq :utubuse (check_utubuse_or_not))
     (warn ";utubuse detected~%")
     (negaeri-from-utubuse))
    (t
      (warn ";state unknown! use (negaeri-from-aomuke) or (negaeri-from-utubuse)~%"))
    )
  )

(defun stand-up2 (&key (time-scale 1.0) (wait-times 40) (jump t))
  (grasp :real t)
  ;;(send *ri* :start-grasp)
  (tagbody
   :start
   ;;仰向けかうつ伏せかを判定し、仰向けならうつ伏せに移行
   (atlas-set-servo-gain-by-torque-limit 1)
   (cond
    ((eq :aomuke (check_utubuse_or_not))
     (print ";detect aomuke situation, do rolling-over")
     (negaeri-from-aomuke)
     )
    (t
     (print ";set k-effort 255")
     (set-k-effort 255)))
   (unix:sleep 2)
   ;;
   (print ";set pose1")
   (pose1 :tm (* time-scale 2000))
   (send *ri* :wait-interpolation)
   (print ";set pose2")
   (pre-pose2 :tm (* time-scale 1000))
   (send *ri* :wait-interpolation)
   (pose2 :tm (* time-scale 2400))
   (send *ri* :wait-interpolation)
   (print ";set pose3")
   (pose3 :tm (* time-scale 4000))
   (atlas-set-servo-gain-by-torque-limit 0.01)
   (send *ri* :wait-interpolation)
   (unix:sleep 5)
   (atlas-set-servo-gain-by-torque-limit 1)
   (unix:sleep 1)

   (send *atlas* :larm :angle-vector #f(41.1796 119.496 6.70627 -63.7998 67.5002 -0.054577))
   (send *ri* :angle-vector (send *atlas* :angle-vector) (* time-scale 3000))
   (send *ri* :wait-interpolation)
   (send *atlas* :angle-vector #f(-0.355849 49.9885 -0.129764 41.1796 119.496 6.70627 -63.7998 67.5002 -0.054577 -54.0 120.0 -9.51683 29.5998 -92.2502 -0.030691 140.375 -24.981 -100.046 28.3614 -39.6945 30.1193 140.375 24.981 -99.9811 -28.3614 -39.7298 -29.8737 -0.438736))
   (send *ri* :angle-vector (send *atlas* :angle-vector) (* time-scale 5000))
   (send *ri* :wait-interpolation)
   (unix:sleep 1)

   (send *atlas* :reset-pose)
   (send *atlas* :lleg :angle-vector #f(123.714 0 -71.9519 0 -51.5662 0))
   (send *atlas* :rleg :angle-vector #f(123.714 0 -71.9519 0 -51.5662 0))
   (send *ri* :angle-vector (send *atlas* :angle-vector) (* time-scale 5000))
   (send *ri* :wait-interpolation)
   ;;股が開いているのでなんとかして閉じたい

   ;;ジャンプして足が離れた瞬間にとじる
   (when jump
     (let ((prev-pose (send *atlas* :angle-vector)))
       (atlas-set-servo-gain-by-torque-limit 0.1)
       (send *atlas* :lleg :inverse-kinematics (send (send *atlas* :lleg :end-coords :copy-worldcoords) :translate #f(0 0 -50) :world) :rotation-axis t :debug-view :no-message)
       (send *atlas* :rleg :inverse-kinematics (send (send *atlas* :rleg :end-coords :copy-worldcoords) :translate #f(0 0 -50) :world) :rotation-axis t :debug-view :no-message)
       (send *ri* :angle-vector (send *atlas* :angle-vector) (* time-scale 100))
       (send *ri* :wait-interpolation)
       (send *ri* :angle-vector prev-pose (* time-scale 100))
       (send *ri* :wait-interpolation)
       (atlas-set-servo-gain-by-torque-limit 1)
     ))

   (send *atlas* :reset-pose)
   (send *atlas* :lleg :inverse-kinematics (send (send *atlas* :lleg :end-coords :copy-worldcoords) :translate #f(0 0 70) :world) :rotation-axis t :debug-view :no-message)
   (send *atlas* :rleg :inverse-kinematics (send (send *atlas* :rleg :end-coords :copy-worldcoords) :translate #f(0 0 70) :world) :rotation-axis t :debug-view :no-message)
   (send *atlas* :move-centroid-on-foot :both '(:lleg :rleg))

   (send *ri* :angle-vector (send *atlas* :angle-vector) 10000)
   (send *ri* :wait-interpolation)
   (send *atlas* :angle-vector (send *ri* :state :potentio-vector))
   ))

