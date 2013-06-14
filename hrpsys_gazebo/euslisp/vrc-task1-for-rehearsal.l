;; hrpsysを立ち上げている状態で

roscd hrpsys_gazebo/euslisp
roseus atlas-vehicle-motion.l 

;; 初期化
(my-init :real t)
(real2model)
(model2real :time 500)
(set-user)

;; 車に干渉しないように右手を上げて，歩くように膝を曲げた姿勢になる
(send *atlas* :angle-vector #f(0.0 0.0 0.0 23.5 114.5 0.0 -75.0 17.0 0.0 -135.0 129.6 -71.6129 79.9998 60.7501 0.0 61.5619 -1.79235 -35.568 1.79235 -25.994 0.012938 61.5446 1.91967 -35.5619 -1.91967 -25.9828 0.01294 0.0))
(send *atlas* :fix-leg-to-coords (make-coords))
(send *atlas* :move-centroid-on-foot :both '(:rleg :lleg)
      :target-centroid-pos (v+ (float-vector 30 0 0) (apply #'midpoint 0.5 (send-all (send *atlas* :legs :end-coords :parent) :worldpos))))
(model2real)
(grasp :arm :rarm :real t)
(ungrasp :arm :larm :real t)

;; 歩行
(send *ri* :start-st)
(send *ri* :cmd-vel-mode)

;; 別ターミナルで
rosrun pr2_teleop teleop_pr2_keyboard
;; を立ち上げて操縦
;; 模様のある地面と道路の境目を踏まないように，その直前で歩幅を広くする必要があるらしい

;; 車の真横まで移動したら(send *ri* :cmd-vel-mode)のターミナルでEnter押すと抜ける

;; 曲げていた膝を伸ばす
(send *atlas* :legs :move-end-pos #f(0 0 -70) :world)
(model2real)
(send *ri* :stop-st)

;; 車乗り込み動作
(get-in-vehicle-side :real t :wait nil :teleport nil)




;; 以下参考 ;;

;; 車の横に瞬間移動
(get-in-vehicle-side-teleport :real t)

;; 車の横に瞬間移動ありの乗り込み
(get-in-vehicle-side :real t)


;; IMUを見てhrpsys-walkで回転角度を合わせるのを追加した
;; 以下で車の右隣まで歩く
;スタートした直後(bdiモードのまま)
(set-imu-data-offset)
;ここで(set-user) (reset-pose)などする
(rotate-with-imu 0)
(send *ri* :go-pos 7 0 0)
(send *ri* :go-pos 0 0 90)
(rotate-with-imu 90)
(send *ri* :go-pos 10 0 0)
(send *ri* :go-pos 0 0 90)
(rotate-with-imu 180)
(send *ri* :go-pos 4.5 0 0)

