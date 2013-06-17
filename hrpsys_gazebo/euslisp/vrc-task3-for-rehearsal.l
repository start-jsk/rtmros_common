(progn (load "atlas-interface.l")(atlas-init :set-reset-pose nil :set-user-mode nil))
(send *ri* :teleport-to (make-coords :pos #f(-2700 -4200 1000)))
(progn (real2model) (model2real :wait t :time 10) (set-user) (set-kp 4000))


(defun init-pose-for-hose ()
  (objects (list *atlas*))
  (send *atlas* :reset-manip-pose :base-down t)
  (send *atlas* :arms :move-end-pos #f(-100 0 0) :world :rotation-axis nil)
  ;;(send *atlas* :larm :angle-vector #f(30.0 90.0 0.0 -79.9998 15.0 0.0))
  (send *atlas* :hand :rarm :open-pose2)
  (progn (model2real) (hand-model2real :time 1000) (send *ri* :wait-interpolation))
  )

(defun reach-and-pickup-hose
  (&key (hose-grasp-x-pos 400)
	(hose-grasp-y-pos -550))
  (send *atlas* :reset-manip-pose :base-down t)
  ;;(send *atlas* :larm :angle-vector #f(30.0 90.0 0.0 -79.9998 15.0 0.0))
  (send *atlas* :rarm :inverse-kinematics
	(send (make-coords :pos (float-vector hose-grasp-x-pos hose-grasp-y-pos 1300) :rpy (list 0 0 -pi/2)) :rotate pi/2 :y :world)
	:debug-view :no-message :thre 5 :stop 200
	:rotation-axis :z)
  (send *atlas* :move-centroid-on-foot
	:both '(:rleg :lleg :rarm)
	:target-centroid-pos (send *atlas* :calc-target-centroid-pos :both '(:rleg :lleg)))
  (progn (model2real :time 2000) (hand-model2real :time 1000) (send *ri* :wait-interpolation))

  (send *atlas* :rarm :inverse-kinematics
	(send (make-coords :pos (float-vector hose-grasp-x-pos hose-grasp-y-pos 1100) :rpy (list 0 0 -pi/2)) :rotate pi/2 :y :world)
	:debug-view :no-message :thre 5 :stop 200 :rotation-axis :z)
  (send *atlas* :move-centroid-on-foot
	:both '(:rleg :lleg :rarm)
	:target-centroid-pos (send *atlas* :calc-target-centroid-pos :both '(:rleg :lleg)))
  (progn (model2real :time 2000) (hand-model2real :time 1000) (send *ri* :wait-interpolation))

  (send *atlas* :hand :rarm :angle-vector #f(0.0 90.0 40.0 0.0 90.0 40.0 0.0 90.0 40.0 0.0 50.0 20.0))
  (set-hand-kp 1)
  (progn (model2real) (hand-model2real :time 1000) (send *ri* :wait-interpolation))
  (dotimes (i 50) (set-hand-kp (* (+ i 1) 1))(unix:usleep 200000))

  (send *atlas* :rarm :inverse-kinematics
	(send (make-coords :pos (float-vector hose-grasp-x-pos hose-grasp-y-pos 1250) :rpy (list 0 0 -pi/2)) :rotate pi/2 :y :world)
	:debug-view :no-message :thre 5 :stop 200 :rotation-axis :z)
  (send *atlas* :move-centroid-on-foot
	:both '(:rleg :lleg :rarm)
	:target-centroid-pos (send *atlas* :calc-target-centroid-pos :both '(:rleg :lleg)))
  (progn (model2real :time 2000) (hand-model2real :time 1000) (send *ri* :wait-interpolation))
  )

(defun twist-torso ()
  (progn
    (send *atlas* :torso :waist-y :joint-angle -35)
    (progn (model2real :time 1000) (hand-model2real :time 1000) (send *ri* :wait-interpolation))
    (send *atlas* :torso :waist-y :joint-angle 35)
    (progn (model2real :time 5000) (hand-model2real :time 1000) (send *ri* :wait-interpolation))
    (send *atlas* :torso :waist-y :joint-angle 0)
    (progn (model2real :time 1000) (hand-model2real :time 1000) (send *ri* :wait-interpolation))
    ;; (send *atlas* :torso :waist-y :joint-angle 35)
    ;; (send *atlas* :rarm :inverse-kinematics
    ;; 	(send (make-coords :pos #f(500 200 1300) :rpy (list 0 0 -pi/2)) :rotate pi/2 :y :world)
    ;; 	:debug-view :no-message :thre 5 :stop 200 :rotation-axis nil)
    ;; (send *atlas* :move-centroid-on-foot
    ;; 	:both '(:rleg :lleg :rarm)
    ;; 	:target-centroid-pos (send *atlas* :calc-target-centroid-pos :both '(:rleg :lleg)))
    ;; (progn (model2real :time 7000) (hand-model2real :time 1000) (send *ri* :wait-interpolation))
    ))

(defun grasp-hose-by-larm ()
  (send *atlas* :rarm :inverse-kinematics
  	(send (send (make-coords :pos #f(500 -80 1300) :rpy (list 0 0 -pi/2)) :rotate pi/2 :y :world) :rotate (deg2rad -30) :x)
  	:debug-view :no-message :thre 10 :stop 200 :rotation-axis :z)  
  (send *atlas* :larm :inverse-kinematics
  	(send (send (make-coords :pos #f(400 80 1400) :rpy (list 0 0 pi/2)) :rotate pi/2 :y :world) :rotate (deg2rad 30) :x)
  	:debug-view :no-message :thre 10 :stop 200 :rotation-axis :z) 
  (send *atlas* :move-centroid-on-foot
  	:both '(:rleg :lleg)
  	:target-centroid-pos (send *atlas* :calc-target-centroid-pos :both '(:rleg :lleg)))
  (progn (model2real) (hand-model2real :time 1000) (send *ri* :wait-interpolation))

  (send *atlas* :larm :inverse-kinematics
  	(send (send (make-coords :pos #f(500 80 1300) :rpy (list 0 0 pi/2)) :rotate pi/2 :y :world) :rotate (deg2rad 30) :x)
  	:debug-view :no-message :thre 10 :stop 200 :rotation-axis :z) 
  (send *atlas* :move-centroid-on-foot
  	:both '(:rleg :lleg)
  	:target-centroid-pos (send *atlas* :calc-target-centroid-pos :both '(:rleg :lleg)))
  (progn (model2real :time 1000) (hand-model2real :time 1000) (send *ri* :wait-interpolation))

  (send *atlas* :hand :larm :angle-vector #f(0.0 90.0 40.0 0.0 90.0 40.0 0.0 90.0 40.0 0.0 90.0 20.0))
  (progn (model2real) (hand-model2real :time 1000) (send *ri* :wait-interpolation))

  (send *atlas* :hand :larm :grasp-pose)
  (progn (model2real) (hand-model2real :time 1000) (send *ri* :wait-interpolation))

  (send *atlas* :hand :rarm :open-pose2)
  (progn (model2real) (hand-model2real :time 1000) (send *ri* :wait-interpolation))

  (send *atlas* :rarm :inverse-kinematics
  	(send (send (make-coords :pos #f(500 -80 1400) :rpy (list 0 0 -pi/2)) :rotate pi/2 :y :world) :rotate (deg2rad -30) :x)
  	:debug-view :no-message :thre 10 :stop 200 :rotation-axis :z)
  (progn (model2real) (hand-model2real :time 1000) (send *ri* :wait-interpolation))

  (send *atlas* :rarm :inverse-kinematics
  	(send (send (make-coords :pos #f(500 -400 1400) :rpy (list 0 0 -pi/2)) :rotate pi/2 :y :world) :rotate (deg2rad -30) :x)
  	:debug-view :no-message :thre 10 :stop 200 :rotation-axis :z)
  (progn (model2real) (hand-model2real :time 1000) (send *ri* :wait-interpolation))
  )

(defun grasp-hose-by-rarm ()
  (send *atlas* :larm :inverse-kinematics
  	(send (send (make-coords :pos #f(500 80 1500) :rpy (list 0 0 pi/2)) :rotate pi/2 :y :world) :rotate (deg2rad 30) :x)
  	:debug-view :no-message :thre 10 :stop 200 :rotation-axis :z) 
  (send *atlas* :rarm :wrist-y :joint-angle -90)
  (send *atlas* :rarm :inverse-kinematics
  	(send (send (make-coords :pos #f(500 -400 1200) :rpy (list 0 0 pi/2)) :rotate pi/2 :y :world) :rotate (deg2rad 30) :x)
  	:debug-view :no-message :thre 10 :stop 200 :rotation-axis :z)
  (progn (model2real) (hand-model2real :time 1000) (send *ri* :wait-interpolation))

  ;; (send *atlas* :rarm :inverse-kinematics
  ;;   	(send (send (make-coords :pos #f(500 -400 1100) :rpy (list 0 0 pi/2)) :rotate pi/2 :y :world) :rotate (deg2rad 30) :x)
  ;;   	:debug-view :no-message :thre 10 :stop 200 :rotation-axis :z)
  ;; (progn (model2real) (hand-model2real :time 1000) (send *ri* :wait-interpolation))
  (send *atlas* :rarm :inverse-kinematics
  	(send (send (make-coords :pos #f(500 -100 1200) :rpy (list 0 0 pi/2)) :rotate pi/2 :y :world) :rotate (deg2rad 30) :x)
  	:debug-view :no-message :thre 10 :stop 200 :rotation-axis :z)
  (progn (model2real) (hand-model2real :time 1000) (send *ri* :wait-interpolation))

  (send *atlas* :larm :inverse-kinematics
  	(send (send (make-coords :pos #f(500 0 1300) :rpy (list 0 0 pi/2)) :rotate pi/2 :y :world) :rotate (deg2rad 30) :x)
  	:debug-view :no-message :thre 10 :stop 200 :rotation-axis :z) 
  (progn (model2real) (hand-model2real :time 1000) (send *ri* :wait-interpolation))

  (send *atlas* :hand :rarm :angle-vector #f(0.0 90.0 40.0 0.0 90.0 40.0 0.0 90.0 40.0 0.0 50.0 20.0))
  (set-hand-kp 1)
  (progn (model2real) (hand-model2real :time 1000) (send *ri* :wait-interpolation))
  (dotimes (i 50) (set-hand-kp (* (+ i 1) 1))(unix:usleep 200000))

  (send *atlas* :hand :larm :open-pose2)
  (set-hand-kp 1)
  (progn (model2real) (hand-model2real :time 1000) (send *ri* :wait-interpolation))
  (dotimes (i 50) (set-hand-kp (* (+ i 1) 1))(unix:usleep 200000))

  (send *atlas* :larm :angle-vector #f(110.0 90.0 30.0 -79.9998 15.0 0.0))
  (send *atlas* :move-centroid-on-foot
  	:both '(:rleg :lleg)
  	:target-centroid-pos (send *atlas* :calc-target-centroid-pos :both '(:rleg :lleg)))
  (progn (model2real) (hand-model2real :time 1000) (send *ri* :wait-interpolation))

  (send *atlas* :rarm :inverse-kinematics
  	(send (send (make-coords :pos #f(500 -80 1300) :rpy (list 0 0 -pi/2)) :rotate pi/2 :y :world) :rotate (deg2rad -30) :x)
  	:debug-view :no-message :thre 10 :stop 200 :rotation-axis :z)  
  (send *atlas* :move-centroid-on-foot
  	:both '(:rleg :lleg)
  	:target-centroid-pos (send *atlas* :calc-target-centroid-pos :both '(:rleg :lleg)))
  (progn (model2real) (hand-model2real :time 1000) (send *ri* :wait-interpolation))
  )


(defun do-all-for-hose
  ()
  (init-pose-for-hose)
  (reach-and-pickup-hose)
  (grasp-hose-by-larm)
  )
(do-all-for-hose)
 
#|

;;
(send *atlas* :hand :larm :reset-pose)
(progn (model2real) (hand-model2real :time 1000) (send *ri* :wait-interpolation))

(send *atlas* :fix-leg-to-coords (make-coords))
(send *atlas* :larm :inverse-kinematics
  	(send (send (make-coords :pos #f(450 200 1050) :rpy (list 0 0 pi/2)) :rotate pi/2 :y :world) :rotate (deg2rad 0) :x)
  	:debug-view :no-message :thre 10 :stop 200 :rotation-axis :z)
(progn (model2real) (hand-model2real :time 1000) (send *ri* :wait-interpolation))

(send *atlas* :hand :larm :angle-vector #f(0.0 30.0 0.0 0.0 30.0 0.0 0.0 30.0 0.0 0.0 90.0 0.0))
(progn (model2real) (hand-model2real :time 1000) (send *ri* :wait-interpolation))

(send *atlas* :move-centroid-on-foot
  	:both '(:rleg :lleg :larm)
  	:target-centroid-pos (send *atlas* :calc-target-centroid-pos :both '(:rleg :lleg) 0.2))
(progn (model2real :time 1500) (hand-model2real :time 1000) (send *ri* :wait-interpolation))

(send *atlas* :lleg :move-end-pos #f(0 0 20) :world)
(progn (model2real :time 700) (hand-model2real :time 1000) (send *ri* :wait-interpolation))
(send *atlas* :lleg :move-end-pos #f(0 100 0) :world)
(progn (model2real :time 700) (hand-model2real :time 1000) (send *ri* :wait-interpolation))
(send *atlas* :lleg :move-end-pos #f(0 0 -20) :world)
(progn (model2real :time 700) (hand-model2real :time 1000) (send *ri* :wait-interpolation))

(send *atlas* :move-centroid-on-foot
  	:both '(:rleg :lleg :larm)
  	:target-centroid-pos (send *atlas* :calc-target-centroid-pos :both '(:rleg :lleg) ))
(progn (model2real) (hand-model2real :time 1000) (send *ri* :wait-interpolation))

(send *atlas* :larm :inverse-kinematics
  	(send (send (make-coords :pos #f(450 300 1050) :rpy (list 0 0 pi/2)) :rotate pi/2 :y :world) :rotate (deg2rad 0) :x)
  	:debug-view :no-message :thre 10 :stop 200 :rotation-axis :z)
(send *atlas* :torso :waist-r :joint-angle 0)
(progn (model2real) (hand-model2real :time 1000) (send *ri* :wait-interpolation))

(send *atlas* :move-centroid-on-foot
  	:both '(:rleg :lleg :larm)
  	:target-centroid-pos (send *atlas* :calc-target-centroid-pos :both '(:rleg :lleg) 0.8))
(progn (model2real :time 1500) (hand-model2real :time 1000) (send *ri* :wait-interpolation))

(send *atlas* :rleg :move-end-pos #f(0 0 20) :world)
(progn (model2real :time 700) (hand-model2real :time 1000) (send *ri* :wait-interpolation))
(send *atlas* :rleg :move-end-pos #f(0 100 0) :world)
(progn (model2real :time 700) (hand-model2real :time 1000) (send *ri* :wait-interpolation))
(send *atlas* :rleg :move-end-pos #f(0 0 -20) :world)
(progn (model2real :time 700) (hand-model2real :time 1000) (send *ri* :wait-interpolation))
(send *atlas* :move-centroid-on-foot
  	:both '(:rleg :lleg :larm)
  	:target-centroid-pos (send *atlas* :calc-target-centroid-pos :both '(:rleg :lleg)))
(progn (model2real :time 1500) (hand-model2real :time 1000) (send *ri* :wait-interpolation))



;;

(send *atlas* :torso :waist-y :joint-angle 10)
(send *atlas* :move-centroid-on-foot
  	:both '(:rleg :lleg)
  	:target-centroid-pos (send *atlas* :calc-target-centroid-pos :both '(:rleg :lleg)))
(progn (model2real) (hand-model2real :time 1000) (send *ri* :wait-interpolation))
)

(send *atlas* :rarm :inverse-kinematics
  	(send (send (make-coords :pos #f(500 -400 1300) :rpy (list 0 0 -pi/2)) :rotate pi/2 :y :world) :rotate (deg2rad -30) :x)
  	:debug-view :no-message :thre 10 :stop 200 :rotation-axis :z)  

(send *atlas* :rarm :inverse-kinematics
  	(send (send (make-coords :pos #f(500 -400 1300) :rpy (list 0 0 pi/2)) :rotate pi/2 :y :world) :rotate (deg2rad 0) :x)
  	:debug-view :no-message :thre 10 :stop 200 :rotation-axis :z)

(send *ri* :set-auto-balancer-param
       :default-zmp-offsets (list (float-vector 30 20 0) (float-vector 30 -20 0)))
(send *ri* :set-gait-generator-param
         :default-double-support-ratio 0.5
         :default-step-time 1.0
         :default-step-height 0.05
         ;;:stride-parameter #f(0.25 0.05 10)
	 :stride-parameter #f(0.25 0.05 30))
(send *ri* :set-st-param
         :k-tpcc-p #f(1.5 1.5)
         :k-tpcc-x #f(5 5)
	 :k-brot-p #f(5 5)
         :k-brot-tc #f(0.1 0.1))
(progn (send *ri* :start-st) (send *ri* :go-pos -0.1 0 0) (send *ri* :stop-st))

;; new


(send *atlas* :torso :waist-y :joint-angle 35)
(send *atlas* :rarm :inverse-kinematics
      (send (make-coords :pos #f(400 -400 1300) :rpy (list 0 0 -pi/2)) :rotate pi/2 :y :world)
      :debug-view :no-message :thre 5 :stop 200 :rotation-axis :z)

(send *atlas* :rarm :inverse-kinematics
      (send (make-coords :pos #f(500 200 1300) :rpy (list 0 0 -pi/2)) :rotate pi/2 :y :world)
      :debug-view :no-message :thre 5 :stop 200 :rotation-axis nil)
(send *atlas* :move-centroid-on-foot
	 :both '(:rleg :lleg :rarm)
	 :target-centroid-pos (send *atlas* :calc-target-centroid-pos :both '(:rleg :lleg)))
p
(send *atlas* :rarm :move-end-pos #f(0 0 -100) :world :debug-view :no-message :thre 5)
(progn (send *atlas* :torso :waist-y :joint-angle -35) (progn (model2real :time 5000) (hand-model2real :time 1000) (send *ri* :wait-interpolation)) (send *atlas* :torso :waist-y :joint-angle 35) (progn (model2real :time 5000) (hand-model2real :time 1000) (send *ri* :wait-interpolation)))

(send *atlas* :hand :larm :open-pose2)
(send *atlas* :larm :inverse-kinematics
      (send (make-coords :pos #f(350 450 1100) :rpy (list 0 0 pi/2)) :rotate pi/2 :y :world)
      :debug-view :no-message :thre 5 :stop 200 :rotation-axis :z)
(send *atlas* :move-centroid-on-foot
	 :both '(:rleg :lleg :rarm)
	 :target-centroid-pos (send *atlas* :calc-target-centroid-pos :both '(:rleg :lleg)))

(send *atlas* :hand :larm :open-pose2)
(send *atlas* :larm :inverse-kinematics
      (send (make-coords :pos #f(500 450 1070) :rpy (list 0 0 pi/2)) :rotate pi/2 :y :world)
      :debug-view :no-message :thre 5 :stop 200 :rotation-axis :z)
(send *atlas* :move-centroid-on-foot
	 :both '(:rleg :lleg :rarm)
	 :target-centroid-pos (send *atlas* :calc-target-centroid-pos :both '(:rleg :lleg)))

|#