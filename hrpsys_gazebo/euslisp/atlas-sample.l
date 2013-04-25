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
(load "./atlas-interface.l")
;;
(warn "(init)~%")
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
    (adapt)
    ))
  (send *ri* :angle-vector (send *atlas* :angle-vector))
  (set-k-effort)
  (setq r *atlas*)
  ;;
  (imu-init)
  (objects (list *atlas*))
  )
#|
  for utility
|#
(defun send-pose
  (&optional (s-time 3000))
  (send *ri* :angle-vector (send *atlas* :angle-vector) s-time))
(defun adapt
  ()
  (send *atlas* :angle-vector (send *ri* :state :potentio-vector))
  (send *irtviewer* :draw-objects)
  )
(defun reset-pose
  ()
  (send *atlas* :reset-pose))

;;わざと転倒させる (前に傾く）
(defun fall-down
  (&key (run t) (s-time 3000))
  (send *atlas* :reset-pose)
  (send *atlas* :torso-waist-p :joint-angle 40)
  (send *atlas* :lleg-ankle-p :joint-angle -20)
  (send *atlas* :rleg-ankle-p :joint-angle -20)
  (send *irtviewer* :draw-objects)
  (when run 
    (send-pose s-time))
  )

;;とりあえずeusから動くようにする (ひどい実装orz)
(defun set-k-effort
  (&optional (k-effort 255))
  (let (com)
    (setq com "rostopic pub -1 /atlas/atlas_sim_interface_command atlas_msgs/AtlasSimInterfaceCommand '{behavior: 1, k_effort: [")
    (dotimes (i 27)
      (setq com (concatenate string com (format nil "~a, " k-effort))))
    (setq com (concatenate string com 
			   (format nil "~a]}'" k-effort)))
    (unix:system com)
    ))

#|
 起き上がり
 0. うつ伏せになる
 1. まっすぐにする (pose1)
 2. 足を曲げる、足首を地面と水平にする2 (pose2)
|#
;; 1. まっすぐにする
(defun pose1
  (&key (run t) (s-time 3000))
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
  (when run (send-pose s-time))
  )
;;  2. 足を曲げる、足首を地面と水平にする2
;;     腰を曲げる、腕を曲げる
;;    (土下座のイメージ)
(defun pose2
  (&key (run t) (s-time 3000))
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
  (when run (send-pose s-time))
  )

;; 3. 手と腰を使ってシコ状態に遷移させる
(defun pose3
  (&key (run t) (s-time 5000))
  (pose2 :run nil)
  ;;
  (send *atlas* :larm-elbow-p :joint-angle 20)
  (send *atlas* :rarm-elbow-p :joint-angle -20)
  ;;(send *atlas* :torso-waist-p :joint-angle 70)
  (send *atlas* :torso-waist-p :joint-angle 50)
  ;;
  (send *irtviewer* :draw-objects)
  (when run (send-pose s-time))
  )
;;
;; 4. 腰を元に戻し、足もあげていく
(defun pose4-1
  (&key (run nil) (s-time 30000))
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
  (when run (send-pose s-time))
  )

;; 4. 腰を元に戻し、足もあげていく (中間姿勢２：一度にゆっくりしたとしてもこけるっぽい, なんで？)
(defun pose4-2
  (&key (run nil) (s-time 30000))
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
  (when run (send-pose s-time))
  )

;; 4. 腰を元に戻し、足もあげていく (中間姿勢3：一度にゆっくりしたとしてもこけるっぽい, なんで？)
(defun pose4-3
  (&key (run nil) (s-time 30000))
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
  (when run (send-pose s-time))
  )
  
;; 4. 腰を元に戻し、足もあげていく　最終姿勢：pose4-4の姿勢は、walkの初期姿勢と同じ,この状態ならk-effortを0にしてもこけなくなる
(defun pose4-4
  (&key (run nil) (s-time 30000))
  (send *atlas* :angle-vector
	#f(-0.000611 0.076327 0.002034 28.5397 114.638 -0.264813 -74.6804 17.1935 0.017428 -28.5384 114.637 0.268241 74.6782 17.1937 0.018326 29.5172 -3.85612 -14.613 4.16939 -13.4334 17.3866 29.2973 4.51093 -14.4278 -5.09418 -13.4808 -17.3512 -0.061066))
  ;;
  (send *irtviewer* :draw-objects)
  (when run (send-pose s-time))
  )

;;
(defun stand-up
  ()
  (print "set k-effort 255")
  (set-k-effort 255)
  ;;(unix:sleep 1)
  ;;
  (print "set pose1")
  (pose1 :s-time 15000)
  (send *ri* :wait-interpolation)
  (print "set pose2")
  (pose2 :s-time 15000)
  (send *ri* :wait-interpolation)
  (print "set pose3")
  (pose3)
  (send *ri* :wait-interpolation)
  (warn "wait for totally landing, then push enter key~%")
  (setq w (read-line))
  (when (equal w "e")
    (fall-down :s-time 15000)
    (send *ri* :wait-interpolation)
    (return-from stand-up))
  (print "set pose4-1")
  (pose4-1 :s-time 20000 :run t)
  (send *ri* :wait-interpolation)
  ;;
  (print "set pose4-2")
  (pose4-2 :s-time 20000 :run t)
  (send *ri* :wait-interpolation)
  ;;
  (print "set pose4-3")
  (pose4-3 :s-time 20000 :run t)
  (send *ri* :wait-interpolation)
  ;;
  ;;ここから怪しいがたまに成功する
  (print "set pose4-4(final pose)")
  (pose4-4 :s-time 50000 :run t)
  (send *ri* :wait-interpolation)
  (unix:sleep 1)
  (print "set k-effort 200")
  (set-k-effort 200)   ;;walking mode
  ;;(unix:sleep 3)
  (print "set k-effort 100")
  (set-k-effort 100)   ;;walking mode
  ;;(unix:sleep 3)
  (print "set k-effort 0")
  (set-k-effort 0)   ;;walking mode
  ;;(unix:sleep 3)
  (print "if atlas keep standing, walking mode!")
  )

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

#|
  ROS-API経由でatlasのセンサ情報を取得
  /atlas/imu/orientation x,y,z,wで読める
|#

(defun imu-init
  ()
  (ros::subscribe "/atlas/imu" sensor_msgs::Imu #'get_imu_quaternion 1)
  )
(defvar *atlas_rpy* nil)
;;ros:spin-onceすれば値が更新される
(defun get_imu_quaternion
  (msg)
  (let* ((q (send msg :orientation))
	 )
    ;;(print imu-orientation)
    (setq *atlas_rpy*
	  (mapcar
	   #'rad2deg
	   (car (rpy-angle (quaternion2matrix 
			    (float-vector (send q :w) 
					  (send q :x) 
					  (send q :y) 
					  (send q :z)))))))
    ))
;;

#|
 4足歩行モード（蛙飛びが一番楽かな)
|#
(defun frog-pose1
  (&key (run t) (s-time 3000))
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
  (when run (send-pose s-time))
  )

;;なるべく足部を地面に接触させるのがポイント
(defun frog-pose2
  (&key (run t) (s-time 3000))
  (pose2 :run nil)
  ;;
  (send *atlas* :torso-waist-p :joint-angle 40)
  (send *atlas* :lleg-knee-p :joint-angle 120)
  (send *atlas* :rleg-knee-p :joint-angle 120)
  (send *atlas* :larm-elbow-p :joint-angle 120)
  (send *atlas* :rarm-elbow-p :joint-angle -120)
  
  (when run (send-pose s-time))
  )

(defun frog
  ()
  (do-until-key 
   ;;(frog-pose2) 
   (pose2) 
   (send *ri* :wait-interpolation) 
   (unix:sleep 3) 
   ;;
   ;;(frog-pose1 :s-time 500) 
   (pose1 :s-time 2000)
   (send *ri* :wait-interpolation) 
   (unix:sleep 5))

;;回転かあ