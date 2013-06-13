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

;; walk
(send *ri* :start-st)
(send *ri* :cmd-vel-mode)

;; 別ターミナルで
rosrun pr2_teleop teleop_pr2_keyboard
;; を立ち上げて操縦

;; 車の真横まで移動したら(send *ri* :cmd-vel-mode)のターミナルでEnter押すと抜ける

;; prepare for getting in vehicle
(send *atlas* :legs :move-end-pos #f(0 0 -70) :world)
(model2real)

;; get in vehicle
(get-in-vehicle-side :real t :wait nil :teleport nil)
