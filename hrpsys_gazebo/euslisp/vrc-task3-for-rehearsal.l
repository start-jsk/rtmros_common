(progn (load "atlas-interface.l")(atlas-init :set-reset-pose nil :set-user-mode nil))
(send *ri* :teleport-to (make-coords :pos #f(-2700 -4100 1000)))
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
	(hose-grasp-y-pos -650))
  (send *atlas* :reset-manip-pose :base-down t)
  ;;(send *atlas* :larm :angle-vector #f(30.0 90.0 0.0 -79.9998 15.0 0.0))
  (send *atlas* :rarm :inverse-kinematics
	(send (make-coords :pos (float-vector hose-grasp-x-pos -550 1300) :rpy (list 0 0 -pi/2)) :rotate pi/2 :y :world)
	:debug-view :no-message :thre 5 :stop 200
	:rotation-axis :z)
  (send *atlas* :move-centroid-on-foot
	:both '(:rleg :lleg :rarm)
	:target-centroid-pos (send *atlas* :calc-target-centroid-pos :both '(:rleg :lleg)))
  (progn (model2real :time 2000) (hand-model2real :time 1000) (send *ri* :wait-interpolation))

  (send *atlas* :rarm :inverse-kinematics
	(send (make-coords :pos (float-vector hose-grasp-x-pos hose-grasp-y-pos 1300) :rpy (list 0 0 -pi/2)) :rotate pi/2 :y :world)
	:debug-view :no-message :thre 5 :stop 200
	:rotation-axis :z)
  (send *atlas* :move-centroid-on-foot
	:both '(:rleg :lleg :rarm)
	:target-centroid-pos (send *atlas* :calc-target-centroid-pos :both '(:rleg :lleg)))
  (progn (model2real :time 1000) (hand-model2real :time 1000) (send *ri* :wait-interpolation))

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

(defun twist-torso2 ()
  (progn
    ;;(send *atlas* :torso :waist-y :joint-angle -35)
    ;;(progn (model2real :time 1000) (hand-model2real :time 1000) (send *ri* :wait-interpolation))
    (send *atlas* :torso :waist-y :joint-angle 35)
    (send *atlas* :move-centroid-on-foot
	  :both '(:rleg :lleg :rarm)
	  :target-centroid-pos (send *atlas* :calc-target-centroid-pos :both '(:rleg :lleg)))
    (progn (model2real :time 5000) (hand-model2real :time 1000) (send *ri* :wait-interpolation))
    (send *atlas* :torso :waist-y :joint-angle 0)
    (progn (model2real :time 5000) (hand-model2real :time 1000) (send *ri* :wait-interpolation))
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

  (send *atlas* :hand :larm :angle-vector #f(0.0 90.0 40.0 0.0 90.0 40.0 0.0 90.0 40.0 0.0 90.0 70.0))
  ;;(send *atlas* :hand :larm :grasp-pose)
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

(defun move-base-rot (ang)
  (let* ((prc (send *atlas* :copy-worldcoords))
	 (pav (send *atlas* :angle-vector))
	 (lc (mapcar #'(lambda (ll) (send *atlas* ll :end-coords :copy-worldcoords)) '(:rleg :lleg))))
    (send *atlas* :rotate (deg2rad ang) :z :world)
    (let ((ret
	   (mapcar #'(lambda (ll c)
		       (send *atlas* ll :inverse-kinematics c))
		   '(:rleg :lleg) lc)))
      (if (every #'identity ret)
	  (send *atlas* :angle-vector)
	(progn
	  (send *atlas* :newcoords prc)
	  (send *atlas* :angle-vector pav)
	  nil)
	)
      )))

(defun mate-hose-by-larm-0 ()
  (send *atlas* :reset-manip-pose :base-down t)
  (send *atlas* :rarm :angle-vector #f(-56.6679 67.8617 -23.1484 22.3583 -47.8993 28.1287))
  (send *atlas* :torso :waist-y :joint-angle 35)
  (move-base-rot 10)
  (send *atlas* :larm :inverse-kinematics
  	(send (send (make-coords :pos #f(300 400 1500) :rpy (list 0 0 -pi/2)) :rotate 0 :y :world) :rotate (deg2rad 30) :x)
  	:debug-view :no-message :thre 10 :stop 200 :rotation-axis nil)
  (send *atlas* :rarm :angle-vector #f(-118.773 102.047 -37.5247 79.9989 39.1444 2.12699))
  (send *atlas* :move-centroid-on-foot
  	:both '(:rleg :lleg)
  	:target-centroid-pos (send *atlas* :calc-target-centroid-pos :both '(:rleg :lleg)))
  (progn (model2real :time 2000) (hand-model2real :time 1000) (send *ri* :wait-interpolation))

  (send *atlas* :larm :inverse-kinematics
  	(send (send (make-coords :pos #f(300 400 1100) :rpy (list 0 0 -pi/2)) :rotate 0 :y :world) :rotate (deg2rad 30) :x)
  	:debug-view :no-message :thre 10 :stop 200 :rotation-axis nil)
  (send *atlas* :move-centroid-on-foot
  	:both '(:rleg :lleg)
  	:target-centroid-pos (send *atlas* :calc-target-centroid-pos :both '(:rleg :lleg)))
  (progn (model2real :time 2000) (hand-model2real :time 1000) (send *ri* :wait-interpolation))

  ;; (send *atlas* :hand :larm :angle-vector #f(0.0 90.0 00.0 0.0 90.0 00.0 0.0 90.0 00.0 0.0 90.0 70.0))
  ;; (progn (model2real :time 2000) (hand-model2real :time 1000) (send *ri* :wait-interpolation))
  ;; (send *atlas* :hand :larm :angle-vector #f(0.0 90.0 70.0 0.0 90.0 70.0 0.0 90.0 70.0 0.0 90.0 70.0))
  ;; (progn (model2real :time 2000) (hand-model2real :time 1000) (send *ri* :wait-interpolation))
  )


(defun test-ik1
  (target-coords &key (rotation-axis :z) (init-pose nil))
  (when init-pose
    (send *atlas* :reset-manip-pose :base-down t)
    (send *atlas* :rarm :angle-vector #f(-56.6679 67.8617 -23.1484 22.3583 -47.8993 28.1287))
    (send *atlas* :torso :waist-y :joint-angle 35)
    (move-base-rot 15))
  (with-assoc-move-target
   (mt :move-target (send (send (send *atlas* :larm :end-coords :copy-worldcoords) :rotate (deg2rad -30) :x) :translate (float-vector 0 80 100))
       :parent-link (send *atlas* :larm :end-coords :parent))
   (let* ((ll (send *atlas* :link-list (send (car mt) :parent)))
	  (wei (fill (instantiate float-vector (send *atlas* :calc-target-joint-dimension ll)) 1.0)))
     (dolist (idx (mapcar #'(lambda (j) (position j (send-all (send *atlas* :link-list (send *atlas* :larm :end-coords :parent)) :joint))) (send *atlas* :torso :joint-list)))
       (setf (elt wei idx) 0.05))
     (send *atlas* :inverse-kinematics
	   target-coords
	   :link-list ll
	   :move-target (car mt)
	   :weight wei
	   :debug-view :no-message :thre 10 :stop 200 :rotation-axis rotation-axis)))
  (send *atlas* :move-centroid-on-foot
	:both '(:rleg :lleg :larm)
	:target-centroid-pos (send *atlas* :calc-target-centroid-pos :both '(:rleg :lleg))
	:centroid-thre 40)
  )

(defun mate-hose-by-larm-1 ()
  (send *atlas* :reset-manip-pose :base-down t)
  (send *atlas* :rarm :angle-vector #f(-56.6679 67.8617 -23.1484 22.3583 -47.8993 28.1287))
  (send *atlas* :torso :waist-y :joint-angle 35)
  (move-base-rot 15)
  (with-assoc-move-target
   (mt :move-target (send (send (send *atlas* :larm :end-coords :copy-worldcoords) :rotate (deg2rad -30) :x) :translate (float-vector 0 80 100))
       :parent-link (send *atlas* :larm :end-coords :parent))
   (let* ((ll (send *atlas* :link-list (send (car mt) :parent)))
	  (wei (fill (instantiate float-vector (send *atlas* :calc-target-joint-dimension ll)) 1.0)))
     (dolist (idx (mapcar #'(lambda (j) (position j (send-all (send *atlas* :link-list (send *atlas* :larm :end-coords :parent)) :joint))) (send *atlas* :torso :joint-list)))
       (setf (elt wei idx) 0.05))
     (send *atlas* :inverse-kinematics
	   (send (make-coords :pos #f(200 600 1100) :rpy (list 0 0 -pi/2)) :rotate (deg2rad 0) :z :world)
	   :link-list ll
	   :move-target (car mt)
	   :weight wei
	   :debug-view :no-message :thre 10 :stop 200 :rotation-axis :z)))
  (send *atlas* :move-centroid-on-foot
	:both '(:rleg :lleg :larm)
	:target-centroid-pos (send *atlas* :calc-target-centroid-pos :both '(:rleg :lleg))
	:centroid-thre 40)
  (progn (model2real :time 2000) (hand-model2real :time 1000) (send *ri* :wait-interpolation))

  (with-assoc-move-target
   (mt :move-target (send (send (send *atlas* :larm :end-coords :copy-worldcoords) :rotate (deg2rad -30) :x) :translate (float-vector 0 80 100))
       :parent-link (send *atlas* :larm :end-coords :parent))
   (setq *ikc* (send (car mt) :copy-worldcoords)))

  (send *atlas* :hand :larm :open-pose2)
  (progn (model2real :time 2000) (hand-model2real :time 1000) (send *ri* :wait-interpolation))
  (test-ik1 (send (send *ikc* :copy-worldcoords) :translate (float-vector 0 0 -50) :world) :init-pose t :rotation-axis t)
  (progn (model2real :time 2000) (hand-model2real :time 1000) (send *ri* :wait-interpolation))
  (test-ik1 (send (send (send *ikc* :copy-worldcoords) :translate (float-vector 0 0 -50) :world) :rotate (deg2rad -40) :z) :init-pose t :rotation-axis t)
  (progn (model2real :time 2000) (hand-model2real :time 1000) (send *ri* :wait-interpolation))
  (test-ik1 (send (send (send *ikc* :copy-worldcoords) :translate (float-vector 0 0 0) :world) :rotate (deg2rad -40) :z) :init-pose t :rotation-axis t)
  (progn (model2real :time 2000) (hand-model2real :time 1000) (send *ri* :wait-interpolation))
  (send *atlas* :hand :larm :angle-vector #f(0.0 90.0 40.0 0.0 90.0 40.0 0.0 90.0 40.0 0.0 90.0 70.0))
  (progn (model2real :time 2000) (hand-model2real :time 1000) (send *ri* :wait-interpolation))
  (test-ik1 (send (send *ikc* :copy-worldcoords) :rotate (deg2rad 0) :z) :init-pose t :rotation-axis t)
  (progn (model2real :time 2000) (hand-model2real :time 1000) (send *ri* :wait-interpolation))
  
  (with-assoc-move-target
   (mt :move-target (send (send (send *atlas* :larm :end-coords :copy-worldcoords) :rotate (deg2rad -30) :x) :translate (float-vector 0 80 100))
       :parent-link (send *atlas* :larm :end-coords :parent))
   (send *atlas* :inverse-kinematics
	 (send (send (car mt) :copy-worldcoords) :translate (float-vector 0 0 30))
	 :link-list (send *atlas* :link-list (send (car mt) :parent))
	 :move-target (car mt)
	 :debug-view :no-message :thre 10 :stop 200 :rotation-axis :z))
  (send *atlas* :move-centroid-on-foot
  	:both '(:rleg :lleg :larm)
  	:target-centroid-pos (send *atlas* :calc-target-centroid-pos :both '(:rleg :lleg))
	:centroid-thre 40)
  (progn (model2real :time 2000) (hand-model2real :time 1000) (send *ri* :wait-interpolation))
  )

(defun do-all-for-hose
  ()
  (init-pose-for-hose)
  (reach-and-pickup-hose)
  (grasp-hose-by-larm)
  (mate-hose-by-larm-0)
  )
;;(do-all-for-hose)
 
#|
(send *atlas* :hand :larm :grasp-pose)
(send *ri* :start-impedance
      :larm :force-gain #f(0 0 0)
      :moment-gain #f(0 0 1)
      ;;:m-r 5 :d-r 100 :k-r 200
      :m-r 5 :d-r 50 :k-r 0.5)


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