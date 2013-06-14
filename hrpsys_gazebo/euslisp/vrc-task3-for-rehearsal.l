(send *ri* :teleport-to (make-coords :pos #f(-2700 -4200 1000)))
(progn (real2model) (model2real :wait t :time 10) (set-user) (set-kp 4000))

(objects (list *atlas*))
(send *atlas* :reset-manip-pose :base-down t)
(send *atlas* :arms :move-end-pos #f(-100 0 0) :world :rotation-axis nil)
(send *atlas* :hand :rarm :open-pose2)
(progn (model2real) (hand-model2real :time 1000) (send *ri* :wait-interpolation))

(send *atlas* :reset-manip-pose :base-down t)
(send *atlas* :rarm :inverse-kinematics
      (send (make-coords :pos #f(400 -400 1300) :rpy (list 0 0 -pi/2)) :rotate pi/2 :y :world)
      :debug-view :no-message :thre 5 :stop 200
      :rotation-axis :z)
(send *atlas* :move-centroid-on-foot
	 :both '(:rleg :lleg :rarm)
	 :target-centroid-pos (send *atlas* :calc-target-centroid-pos :both '(:rleg :lleg)))
(progn (model2real) (hand-model2real :time 1000) (send *ri* :wait-interpolation))

(send *atlas* :rarm :inverse-kinematics
      (send (make-coords :pos #f(400 -450 1100) :rpy (list 0 0 -pi/2)) :rotate pi/2 :y :world)
      :debug-view :no-message :thre 5 :stop 200 :rotation-axis :z)
(send *atlas* :move-centroid-on-foot
	 :both '(:rleg :lleg :rarm)
	 :target-centroid-pos (send *atlas* :calc-target-centroid-pos :both '(:rleg :lleg)))
(progn (model2real) (hand-model2real :time 1000) (send *ri* :wait-interpolation))

(send *atlas* :hand :rarm :angle-vector #f(0.0 90.0 40.0 0.0 90.0 40.0 0.0 90.0 40.0 0.0 50.0 20.0))
(set-hand-kp 1)
(progn (model2real) (hand-model2real :time 1000) (send *ri* :wait-interpolation))
(set-hand-kp 200)

(send *atlas* :rarm :inverse-kinematics
      (send (make-coords :pos #f(400 -450 1150) :rpy (list 0 0 -pi/2)) :rotate pi/2 :y :world)
      :debug-view :no-message :thre 5 :stop 200 :rotation-axis :z)
(send *atlas* :move-centroid-on-foot
	 :both '(:rleg :lleg :rarm)
	 :target-centroid-pos (send *atlas* :calc-target-centroid-pos :both '(:rleg :lleg)))
(progn (model2real) (hand-model2real :time 1000) (send *ri* :wait-interpolation))

(send *atlas* :torso :waist-y :joint-angle 35)
(send *atlas* :rarm :inverse-kinematics
      (send (make-coords :pos #f(400 -400 1300) :rpy (list 0 0 -pi/2)) :rotate pi/2 :y :world)
      :debug-view :no-message :thre 5 :stop 200 :rotation-axis :z)

(send *atlas* :rarm :move-end-pos #f(0 0 -100) :world :debug-view :no-message :thre 5)
