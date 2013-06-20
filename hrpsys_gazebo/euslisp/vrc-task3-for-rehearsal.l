(defmacro null-output (&rest bodies)
  `(with-all-output->file "/dev/null" ,@bodies))

;; with-all-output->file : (with-all-output->file "filename" (form1) (form2) .. )
(defmacro with-all-output->file (fname &rest bodies)
  (let ((tmp-stdout (gensym))
        (tmp-stderr (gensym))
        (null-port (gensym))
        (ret (gensym)))
    `(progn
       (let ((,tmp-stdout *standard-output*)
             (,tmp-stderr *error-output*)
             (,ret nil))
         (unwind-protect
             (with-open-file (,null-port ,fname :direction :output)
               (setq *standard-output* ,null-port)
               (setq *error-output* ,null-port)
               (setq ,ret (progn ,@bodies))
               )
           (progn
             (setq *standard-output* ,tmp-stdout)
             (setq *error-output* ,tmp-stdout)
             ,ret))))))

(defun init-pose-for-hose ()
  (objects (list *atlas*))
  (send *atlas* :reset-manip-pose :base-down t)
  (send *atlas* :arms :move-end-pos #f(-100 0 0) :world :rotation-axis nil)
  (send *atlas* :head :neck-p :joint-angle 50)
  ;;(send *atlas* :larm :angle-vector #f(30.0 90.0 0.0 -79.9998 15.0 0.0))
  (send *atlas* :hand :rarm :open-pose2)
  (progn (model2real) (hand-model2real :time 1000) (send *ri* :wait-interpolation))
  )

(defun look-at-hose ()
  (stop-gp)
  (send *atlas* :reset-manip-pose :base-down t)
  (send *atlas* :arms :move-end-pos #f(-100 0 0) :world :rotation-axis nil)
  (send *atlas* :head :neck-p :joint-angle 50)
  (send *atlas* :torso :waist-y :joint-angle -20)
  (send *atlas* :move-centroid-on-foot
	:both '(:rleg :lleg)
	:target-centroid-pos (send *atlas* :calc-target-centroid-pos :both '(:rleg :lleg)))
  (progn (model2real :time 2000) (hand-model2real :time 1000) (send *ri* :wait-interpolation))
  )

(defun get-point2 (&optional (itr 4))
  (let ((ret nil))
    (dotimes (i itr)
      (let ((pp (send (get-point) :worldpos)))
	(format t ";; ~A~%" pp)
	(if (y-or-n-p ";; ok?") (push pp ret))))
    (if (> (length ret) 0)
	(scale (/ 1.0 (length ret)) (reduce #'v+ ret :initial-value #f( 0 0 0))))
    ))

;; (setq *thp* #f(569.484 -650.012 1068.87)) ;; typical value
(defun reach-and-pickup-hose
  (&key (target-pos *thp*)
	(hose-grasp-x-pos 400)
	(hose-grasp-y-pos -650))
  (send *atlas* :reset-manip-pose :base-down t)
  (send *atlas* :rarm :inverse-kinematics
	(send (make-coords :pos (float-vector 400 -550 1300) :rpy (list 0 0 -pi/2)) :rotate pi/2 :y :world)
	:debug-view :no-message :thre 5 :stop 200
	:rotation-axis :z)
  (send *atlas* :head :neck-p :joint-angle 50)
  (send *atlas* :move-centroid-on-foot
	:both '(:rleg :lleg :rarm)
	:target-centroid-pos (send *atlas* :calc-target-centroid-pos :both '(:rleg :lleg)))
  (warn ";; reaching via point 1~%")
  (progn (model2real :time 2000) (hand-model2real :time 1000 ) (send *ri* :wait-interpolation))
  
  (send *atlas* :inverse-kinematics
	(send (send (make-coords :pos (v+ target-pos #f(0 -30 100)) :rpy (list 0 0 -pi/2)) :rotate pi/2 :y :world) :rotate (deg2rad -40) :z :world)
	:move-target (send *atlas* :get :rarm-hand-coords-for-hose2)
	:link-list (send *atlas* :link-list (send (send *atlas* :get :rarm-hand-coords-for-hose2) :parent) (send *atlas* :rarm :root-link))
	:debug-view :no-message :thre 20 :stop 200
	:rotation-axis :z)
  (send *atlas* :head :neck-p :joint-angle 50)
  (send *atlas* :move-centroid-on-foot
	:both '(:rleg :lleg :rarm)
	:target-centroid-pos (send *atlas* :calc-target-centroid-pos :both '(:rleg :lleg)))
  (warn ";; reaching via point 2~%")
  (progn (model2real :time 1000) (hand-model2real :time 1000 ) (send *ri* :wait-interpolation))
  (snapshot-image :rarm)
  (unless (y-or-n-p ";; continue?~%") (return-from reach-and-pickup-hose nil))

  (send *atlas* :inverse-kinematics
	(send (send (make-coords :pos (v+ target-pos #f(0 -30 0)) :rpy (list 0 0 -pi/2)) :rotate pi/2 :y :world) :rotate (deg2rad -40) :z :world)
	:move-target (send *atlas* :get :rarm-hand-coords-for-hose2)
	:link-list (send *atlas* :link-list (send (send *atlas* :get :rarm-hand-coords-for-hose2) :parent) (send *atlas* :rarm :root-link))
	:debug-view :no-message :thre 20 :stop 200
	:rotation-axis :z)
  (send *atlas* :head :neck-p :joint-angle 50)
  (send *atlas* :move-centroid-on-foot
	:both '(:rleg :lleg :rarm)
	:target-centroid-pos (send *atlas* :calc-target-centroid-pos :both '(:rleg :lleg)))
  (warn ";; reaching~%")
  (progn (model2real :time 1000) (hand-model2real :time 1000 ) (send *ri* :wait-interpolation))
  (snapshot-image :rarm)
  (unless (y-or-n-p ";; continue?~%") (return-from reach-and-pickup-hose nil))

  (send *atlas* :hand :rarm :angle-vector #f(0.0 90.0 40.0 0.0 90.0 40.0 0.0 90.0 40.0 0.0 50.0 20.0))
  (set-hand-kp 1)
  (progn (model2real) (hand-model2real :time 1000 ) (send *ri* :wait-interpolation))
  (dotimes (i 50) (set-hand-kp (* (+ i 1) 1))(unix:usleep 200000))

  (send *atlas* :rarm :inverse-kinematics
	(send (make-coords :pos (float-vector hose-grasp-x-pos hose-grasp-y-pos 1250) :rpy (list 0 0 -pi/2)) :rotate pi/2 :y :world)
	:debug-view :no-message :thre 5 :stop 200 :rotation-axis :z)
  (send *atlas* :head :neck-p :joint-angle 50)
  (send *atlas* :move-centroid-on-foot
	:both '(:rleg :lleg :rarm)
	:target-centroid-pos (send *atlas* :calc-target-centroid-pos :both '(:rleg :lleg)))
  (progn (model2real :time 2000) (hand-model2real :time 1000 ) (send *ri* :wait-interpolation))
  (format t ";; completion = ~A~%" (progn (dotimes (i 10) (ros::spin-once)) *completion-score*))
  )

(defun reach-and-pickup-hose-org
  (&key (hose-grasp-x-pos 400)
	(hose-grasp-y-pos -650)
	(x-off 0) (y-off 0))
  (send *atlas* :reset-manip-pose :base-down t)
  ;;(send *atlas* :larm :angle-vector #f(30.0 90.0 0.0 -79.9998 15.0 0.0))
  (send *atlas* :rarm :inverse-kinematics
	;;(send (make-coords :pos (float-vector hose-grasp-x-pos -550 1300) :rpy (list 0 0 -pi/2)) :rotate pi/2 :y :world)
	(send (make-coords :pos (float-vector 400 -550 1300) :rpy (list 0 0 -pi/2)) :rotate pi/2 :y :world)
	:debug-view :no-message :thre 5 :stop 200
	:rotation-axis :z)
  (send *atlas* :head :neck-p :joint-angle 50)
  (send *atlas* :move-centroid-on-foot
	:both '(:rleg :lleg :rarm)
	:target-centroid-pos (send *atlas* :calc-target-centroid-pos :both '(:rleg :lleg)))
  (progn (model2real :time 2000) (hand-model2real :time 1000 ) (send *ri* :wait-interpolation))

  (send *atlas* :rarm :inverse-kinematics
	(send (make-coords :pos (float-vector hose-grasp-x-pos hose-grasp-y-pos 1300) :rpy (list 0 0 -pi/2)) :rotate pi/2 :y :world)
	:debug-view :no-message :thre 5 :stop 200
	:rotation-axis :z)
  (send *atlas* :head :neck-p :joint-angle 50)
  (send *atlas* :move-centroid-on-foot
	:both '(:rleg :lleg :rarm)
	:target-centroid-pos (send *atlas* :calc-target-centroid-pos :both '(:rleg :lleg)))
  (progn (model2real :time 1000) (hand-model2real :time 1000 ) (send *ri* :wait-interpolation))
  (snapshot-image :rarm)
  (unless (y-or-n-p ";; continue?~%") (return-from reach-and-pickup-hose nil))

  (send *atlas* :rarm :inverse-kinematics
	(send (make-coords :pos (float-vector hose-grasp-x-pos hose-grasp-y-pos 1100) :rpy (list 0 0 -pi/2)) :rotate pi/2 :y :world)
	:debug-view :no-message :thre 5 :stop 200 :rotation-axis :z)
  (send *atlas* :head :neck-p :joint-angle 50)
  (send *atlas* :move-centroid-on-foot
	:both '(:rleg :lleg :rarm)
	:target-centroid-pos (send *atlas* :calc-target-centroid-pos :both '(:rleg :lleg)))
  (progn (model2real :time 2000) (hand-model2real :time 1000 ) (send *ri* :wait-interpolation))
  (snapshot-image :rarm)
  (unless (y-or-n-p ";; continue?~%") (return-from reach-and-pickup-hose nil))

  (send *atlas* :hand :rarm :angle-vector #f(0.0 90.0 40.0 0.0 90.0 40.0 0.0 90.0 40.0 0.0 50.0 20.0))
  (set-hand-kp 1)
  (progn (model2real) (hand-model2real :time 1000 ) (send *ri* :wait-interpolation))
  (dotimes (i 50) (set-hand-kp (* (+ i 1) 1))(unix:usleep 200000))

  (send *atlas* :rarm :inverse-kinematics
	(send (make-coords :pos (float-vector hose-grasp-x-pos hose-grasp-y-pos 1250) :rpy (list 0 0 -pi/2)) :rotate pi/2 :y :world)
	:debug-view :no-message :thre 5 :stop 200 :rotation-axis :z)
  (send *atlas* :head :neck-p :joint-angle 50)
  (send *atlas* :move-centroid-on-foot
	:both '(:rleg :lleg :rarm)
	:target-centroid-pos (send *atlas* :calc-target-centroid-pos :both '(:rleg :lleg)))
  (progn (model2real :time 2000) (hand-model2real :time 1000 ) (send *ri* :wait-interpolation))
  (format t ";; completion = ~A~%" (progn (dotimes (i 10) (ros::spin-once)) *completion-score*))
  )

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
  (progn (model2real) (hand-model2real :time 1000 ) (send *ri* :wait-interpolation))
  (snapshot-image :larm)

  (send *atlas* :larm :inverse-kinematics
  	(send (send (make-coords :pos #f(500 80 1300) :rpy (list 0 0 pi/2)) :rotate pi/2 :y :world) :rotate (deg2rad 30) :x)
  	:debug-view :no-message :thre 10 :stop 200 :rotation-axis :z) 
  (send *atlas* :move-centroid-on-foot
  	:both '(:rleg :lleg)
  	:target-centroid-pos (send *atlas* :calc-target-centroid-pos :both '(:rleg :lleg)))
  (progn (model2real :time 1000) (hand-model2real :time 1000 ) (send *ri* :wait-interpolation))
  (snapshot-image :larm)

  (send *atlas* :hand :larm :angle-vector #f(0.0 90.0 40.0 0.0 90.0 40.0 0.0 90.0 40.0 0.0 90.0 20.0))
  (progn (model2real) (hand-model2real :time 1000 ) (send *ri* :wait-interpolation))

  (send *atlas* :hand :larm :angle-vector #f(0.0 90.0 40.0 0.0 90.0 40.0 0.0 90.0 40.0 0.0 90.0 50.0))
  ;;(send *atlas* :hand :larm :grasp-pose)
  (progn (model2real) (hand-model2real :time 1000 ) (send *ri* :wait-interpolation))

  (send *atlas* :hand :rarm :open-pose2)
  (progn (model2real) (hand-model2real :time 1000 ) (send *ri* :wait-interpolation))

  (send *atlas* :rarm :inverse-kinematics
  	(send (send (make-coords :pos #f(500 -80 1400) :rpy (list 0 0 -pi/2)) :rotate pi/2 :y :world) :rotate (deg2rad -30) :x)
  	:debug-view :no-message :thre 10 :stop 200 :rotation-axis :z)
  (progn (model2real) (hand-model2real :time 1000 ) (send *ri* :wait-interpolation))

  (send *atlas* :rarm :inverse-kinematics
  	(send (send (make-coords :pos #f(500 -400 1400) :rpy (list 0 0 -pi/2)) :rotate pi/2 :y :world) :rotate (deg2rad -30) :x)
  	:debug-view :no-message :thre 10 :stop 200 :rotation-axis :z)
  (progn (model2real) (hand-model2real :time 1000 ) (send *ri* :wait-interpolation))
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
  (progn (model2real :time 2000) (hand-model2real :time 1000 ) (send *ri* :wait-interpolation))

  (send *atlas* :larm :inverse-kinematics
  	(send (send (make-coords :pos #f(300 400 1100) :rpy (list 0 0 -pi/2)) :rotate 0 :y :world) :rotate (deg2rad 30) :x)
  	:debug-view :no-message :thre 10 :stop 200 :rotation-axis nil)
  (send *atlas* :move-centroid-on-foot
  	:both '(:rleg :lleg)
  	:target-centroid-pos (send *atlas* :calc-target-centroid-pos :both '(:rleg :lleg)))
  (progn (model2real :time 2000) (hand-model2real :time 1000 ) (send *ri* :wait-interpolation))

  ;; (send *atlas* :hand :larm :angle-vector #f(0.0 90.0 00.0 0.0 90.0 00.0 0.0 90.0 00.0 0.0 90.0 70.0))
  ;; (progn (model2real :time 2000) (hand-model2real :time 1000) (send *ri* :wait-interpolation))
  ;; (send *atlas* :hand :larm :angle-vector #f(0.0 90.0 70.0 0.0 90.0 70.0 0.0 90.0 70.0 0.0 90.0 70.0))
  ;; (progn (model2real :time 2000) (hand-model2real :time 1000) (send *ri* :wait-interpolation))

  (send *atlas* :reset-manip-pose :base-down t)
  (send *atlas* :rarm :angle-vector #f(-56.6679 67.8617 -23.1484 22.3583 -47.8993 28.1287))
  (send *atlas* :torso :waist-y :joint-angle 35)
  (move-base-rot 15)
  (with-assoc-move-target
   (mt :move-target (send *atlas* :get :larm-hand-coords-for-hose)
       :parent-link (send *atlas* :larm :end-coords :parent))
   (let* ((ll (send *atlas* :link-list (send (car mt) :parent)))
	  (wei (fill (instantiate float-vector (send *atlas* :calc-target-joint-dimension ll)) 1.0)))
     (dolist (idx (mapcar #'(lambda (j) (position j (send-all (send *atlas* :link-list (send *atlas* :larm :end-coords :parent)) :joint))) (send *atlas* :torso :joint-list)))
       (setf (elt wei idx) 0.05))
     (send *atlas* :inverse-kinematics
	   (send (make-coords :pos #f(200 600 1200) :rpy (list 0 0 -pi/2)) :rotate (deg2rad 0) :z :world)
	   :link-list ll
	   :move-target (car mt)
	   :weight wei
	   :debug-view :no-message :thre 10 :stop 200 :rotation-axis :z)))
  (send *atlas* :rarm :wrist-y :joint-angle -90)
  (send *atlas* :move-centroid-on-foot
	:both '(:rleg :lleg :larm)
	:target-centroid-pos (send *atlas* :calc-target-centroid-pos :both '(:rleg :lleg))
	:centroid-thre 40)
  (progn (model2real :time 2000) (hand-model2real :time 1000 ) (send *ri* :wait-interpolation))

  (setq *ikc* (send (send *atlas* :get :larm-hand-coords-for-hose) :copy-worldcoords))
  ;;(move-hose-end-from-ikc (float-vector 0 0 0) 0)
  ;;(move-hose-end-from-ikc (float-vector -100 70 300) 0)
  (move-hose-end-from-ikc (float-vector 0 0 0) 0)
  )

(defun test-ik1
  (target-coords &key (rotation-axis :z) (init-pose nil) (thre 10))
  (let ((pav (send *atlas* :angle-vector))
	(prc (send *atlas* :copy-worldcoords))
	(ret t))

    (when init-pose
      (send *atlas* :reset-manip-pose :base-down t)
      (send *atlas* :rarm :angle-vector #f(-56.6679 67.8617 -23.1484 22.3583 -47.8993 28.1287))
      (send *atlas* :torso :waist-y :joint-angle 35)
      (move-base-rot 15))

    (with-assoc-move-target
     (mt :move-target (send *atlas* :get :larm-hand-coords-for-hose)
	 :parent-link (send *atlas* :larm :end-coords :parent))
     (let* ((ll (send *atlas* :link-list (send (car mt) :parent)))
	    (wei (fill (instantiate float-vector (send *atlas* :calc-target-joint-dimension ll)) 1.0)))
       (dolist (idx (mapcar #'(lambda (j) (position j (send-all (send *atlas* :link-list (send *atlas* :larm :end-coords :parent)) :joint))) (send *atlas* :torso :joint-list)))
	 (setf (elt wei idx) 0.05))
       (setq ret
	     (and ret
		  (send *atlas* :inverse-kinematics
			target-coords
			:link-list ll
			:move-target (car mt)
			:weight wei
			:debug-view :no-message :thre thre :stop 200 :rotation-axis rotation-axis)))))
    (setq ret
	  (and ret
	       (send *atlas* :move-centroid-on-foot
		     :both '(:rleg :lleg :larm)
		     :target-centroid-pos (send *atlas* :calc-target-centroid-pos :both '(:rleg :lleg))
		     :centroid-thre 40)))
    (send *atlas* :head :neck-p :joint-angle 50)
    (if ret
	(send *atlas* :angle-vector)
      (progn
	(send *atlas* :angle-vector pav)
	(send *atlas* :newcoords prc)
	nil)
      )
    ))

;; (move-hose-end-from-ikc (float-vector 0 0 0) 0 t nil)
(defun move-hose-end-from-ikc
  (&optional (dif-pos (float-vector 0 0 0)) (zang 0) (xang 0) (rotation-axis t) (check nil) (thre 2.5))
  (test-ik1
   (send (send (send (send *ikc* :copy-worldcoords) :translate dif-pos :world)
	       :rotate (deg2rad zang) :z :world)
	 :rotate (deg2rad xang)
	 (let ((xx (send *ikc* :rotate-vector #f(0 1 0)))) (setf (elt xx 2) 0) (normalize-vector xx))
	 :world)
   :init-pose t :rotation-axis rotation-axis :thre thre)
  (send *atlas* :rarm :wrist-y :joint-angle -30)
  (progn (model2real :time 1000) (hand-model2real :time 1000 ) (send *ri* :wait-interpolation))
  (when check
    (unix:sleep 1)
    (format t ";; force ~A, moment ~A~%"
	    (send (send *atlas* :force-sensor :lhsensor) :rotate-vector (send *ri* :force-vector :larm))
	    (send (send *atlas* :force-sensor :lhsensor) :rotate-vector (send *ri* :moment-vector :larm)))
    (unix:sleep 1)
    (snapshot-image :rarm)
    (snapshot-image :head)
    (format t ";; completion = ~A~%" (progn (dotimes (i 10) (ros::spin-once)) *completion-score*))
    ))

(defun release-hose-end ()
  (setq *ikc* (send (send *atlas* :get :larm-hand-coords-for-hose) :copy-worldcoords))
  (let ((ret t)
	(ang 10))
    (while ret
      (setq ret (test-ik1 (send (send *ikc* :copy-worldcoords) :rotate (deg2rad ang) :z)
			  :init-pose t :rotation-axis t :thre 2.5))
      (format t ";; rot hose ang ~A~%" ang)
      (setq ang (+ ang 10))
      (progn (model2real :time 2000) (hand-model2real :time 1000 ) (send *ri* :wait-interpolation))
      (if (> ang 50) (setq ret nil))
      ))
  (y-or-n-p ";; open hand?~%")
  (send *atlas* :hand :larm :open-pose2)
  (progn (model2real :time 1000) (hand-model2real :time 1000 ) (send *ri* :wait-interpolation))
  ;;
  ;;
  ;;(move-hose-end-from-ikc (float-vector -100 0 -100) 0 0 :z)
  (move-hose-end-from-ikc (float-vector -70 0 -70) 0 0 :z)
  (send *atlas* :hand :larm :angle-vector #f(0.0 90.0 40.0 0.0 90.0 40.0 0.0 90.0 40.0 0.0 90.0 70.0))
  (progn (model2real :time 1000) (hand-model2real :time 1000 ) (send *ri* :wait-interpolation))
  (y-or-n-p ";; move1?~%")
  (move-hose-end-from-ikc (float-vector -100 -100 -50) 0 0 nil)
  (move-hose-end-from-ikc (float-vector -200 -200 -50) 0 0 nil)
  (y-or-n-p ";; move2?~%")
  (send *atlas* :rarm :angle-vector #f(-135.0 67.8617 -23.1484 70.0 0.0 -30.0))
  (send *atlas* :larm :angle-vector #f(135.0 67.8617 23.1484 -70.0 0.0 -30.0))
  (send *atlas* :move-centroid-on-foot
	:both '(:rleg :lleg)
	:target-centroid-pos (send *atlas* :calc-target-centroid-pos :both '(:rleg :lleg))
	:centroid-thre 40)
  (progn (model2real :time 5000) (hand-model2real :time 1000 ) (send *ri* :wait-interpolation))
  (tack-arm-pose)
  (progn (model2real :time 5000) (hand-model2real :time 1000 ) (send *ri* :wait-interpolation))
  (send *atlas* :hand :larm :open-pose2)
  (progn (model2real :time 1000) (hand-model2real :time 1000 ) (send *ri* :wait-interpolation))
  )

(defun tack-arm-pose
  (&key (arm :arms))
  (send *atlas* :reset-manip-pose :base-down t)
  (case arm
    (:arms (send *atlas* :rarm :angle-vector #f(-135.0 67.8617 -23.1484 70.0 0.0 -30.0))
	   (send *atlas* :larm :angle-vector #f(135.0 67.8617 23.1484 -70.0 0.0 -30.0)))
    (:rarm (send *atlas* :rarm :angle-vector #f(-135.0 67.8617 -23.1484 70.0 0.0 -30.0))
	   (send *atlas* :larm :angle-vector #f(30.0 90.0 0.0 -79.9998 15.0 0.0)))
    (:larm (send *atlas* :larm :angle-vector #f(135.0 67.8617 23.1484 -70.0 0.0 -30.0))
	   (send *atlas* :rarm :angle-vector #f(-30.0 90.0 0.0 79.9998 15.0 0.0)))
    )
  (send *atlas* :head :neck-p :joint-angle 50)
  (send *atlas* :move-centroid-on-foot
	:both '(:rleg :lleg)
	:target-centroid-pos (send *atlas* :calc-target-centroid-pos :both '(:rleg :lleg))
	:centroid-thre 40)
  )

(defun go-pos2 ()
  (start-gp)
  (gp3 -0.2 0 0)
  (tack-arm-pose :arm :larm)
  (progn (model2real :time 5000) (hand-model2real :time 1000 ) (send *ri* :wait-interpolation))
  (gp3 0 0 45)
  (gp3 0.2 0 0)
  )

(defun go-pos3 ()
  ;;(gp3 0.2 0 0)
  (stop-gp)
  (snapshot-image :all)
  )

;; (prepare-for-rotate-hose #f(500 100 1400) 0 5000 :z nil)
;; last height = 1220
(defun prepare-for-rotate-hose
  (&optional (pos #f(500 100 1300)) (zang 0) (tm 5000) (rotation-axis :z) (use-snapshot-image nil) (init-pose nil))
  (if init-pose (tack-arm-pose :arm :larm))
  (test-ik1 
   (send (make-coords :pos pos :rpy (list 0 0 pi/2)) :rotate (deg2rad zang) :z :world)
   :rotation-axis rotation-axis)
  (progn (model2real :time tm) (hand-model2real :time 1000 ) (send *ri* :wait-interpolation))
  (when use-snapshot-image
    (unix:sleep 1)
    (snapshot-image :larm))
  (send (send *atlas* :get :larm-hand-coords-for-hose) :worldpos)
  )

;; (prepare-for-rotate-hose2 0 0 0 5000 :z nil)
(defun prepare-for-rotate-hose2
  (&optional (hx 0) (hy 0) (hz 0) (tm 5000) (rotation-axis :z) (use-snapshot-image nil))
  (test-ik1
   (make-coords
    :pos
    (v+ (v+ (v+ (send (send *atlas* :get :larm-hand-coords-for-hose) :worldpos)
		(let ((xx (send (send *atlas* :get :larm-hand-coords-for-hose) :rotate-vector #f(0 1 0))))
		  (setf (elt xx 2) 0)
		  (scale hy (normalize-vector xx))))
	    (let ((xx (send (send *atlas* :get :larm-hand-coords-for-hose) :rotate-vector #f(0 0 1))))
	      (setf (elt xx 2) 0)
	      (scale hz (normalize-vector xx))))
	(let ((xx (send (send *atlas* :get :larm-hand-coords-for-hose) :rotate-vector #f(1 0 0))))
	  (setf (elt xx 0) 0)
	  (setf (elt xx 1) 0)
	  (scale hx (normalize-vector xx))))
    :rot (send (send *atlas* :get :larm-hand-coords-for-hose) :worldrot))
   :rotation-axis rotation-axis)
  (progn (model2real :time tm) (hand-model2real :time 1000 ) (send *ri* :wait-interpolation))
  (when use-snapshot-image
    (unix:sleep 1)
    (snapshot-image :larm))
  (send (send *atlas* :get :larm-hand-coords-for-hose) :worldpos)
  )

(defun open-hand
  (&key (arm :larm))
  (send *atlas* :hand arm :open-pose2)
  (hand-model2real :time 1000 )
  )

(defun open-hand-for-hose-rotate
  ()
  (send *atlas* :hand :larm :open-pose2)
  ;;(test-ik1 (send (send *ikc* :copy-worldcoords) :translate (float-vector 15 -15 40) :world) :init-pose nil :rotation-axis t)
  ;;(progn (model2real :time 2000) (hand-model2real :time 3000 ) (send *ri* :wait-interpolation))
  (prepare-for-rotate-hose2 -40 30 0 2000 t t)
  (prepare-for-rotate-hose2 -20 0 0 1000 t t)
  )

(defun grasp-hand
  (&key (arm :larm))
  (send *atlas* :hand arm :angle-vector #f(0.0 90.0 40.0 0.0 90.0 40.0 0.0 90.0 40.0 0.0 90.0 45.0))
  (hand-model2real :time 1000 )
  )

;; (send *ikc* :translate (float-vector 0 0 -30))
(defun rotate-hose (&key (init-ikc t))
  (if init-ikc (setq *ikc* (send (send *atlas* :get :larm-hand-coords-for-hose) :copy-worldcoords)))
  (let ((ret t))
    (while ret
      (open-hand-for-hose-rotate)
      ;;(send *atlas* :hand :larm :open-pose2)
      ;;(test-ik1 (send (send *ikc* :copy-worldcoords) :translate (float-vector 15 -15 40) :world) :init-pose nil :rotation-axis t)
      ;;(prepare-for-rotate-hose2 40 -20 -20 2000 :z t) 
      ;;(progn (model2real :time 2000) (hand-model2real :time 3000 ) (send *ri* :wait-interpolation))
      (tack-arm-pose :arm :larm)
      (test-ik1 (send (send *ikc* :copy-worldcoords) :translate (float-vector 0 0 60) :world) :init-pose nil :rotation-axis t)
      (send *atlas* :head :look-at (send (send *atlas* :get :larm-hand-coords-for-hose) :worldpos) :debug-view nil)
      (progn (model2real :time 700) (hand-model2real :time 700) (send *ri* :wait-interpolation))
      (snapshot-image :larm)
      (do ((ang 10 (+ ang 30))) ((< 50 ang))
	(test-ik1 (send (send (send *ikc* :copy-worldcoords) :translate (float-vector -40 0 0) :world) :rotate (deg2rad ang) :z) :init-pose nil :rotation-axis t)
	(send *atlas* :head :look-at (send (send *atlas* :get :larm-hand-coords-for-hose) :worldpos) :debug-view nil)
	(progn (model2real :time 700) (hand-model2real :time 700) (send *ri* :wait-interpolation)))
      (tack-arm-pose :arm :larm)
      (test-ik1 (send (send (send *ikc* :copy-worldcoords) :translate (float-vector -40 0 -30) :world) :rotate (deg2rad 50) :z) :init-pose nil :rotation-axis t)
      (send *atlas* :head :look-at (send (send *atlas* :get :larm-hand-coords-for-hose) :worldpos) :debug-view nil)
      (progn (model2real :time 700) (hand-model2real :time 700) (send *ri* :wait-interpolation))
      (snapshot-image :larm)
      (test-ik1 (send (send (send *ikc* :copy-worldcoords) :translate (float-vector 0 0 0) :world) :rotate (deg2rad 50) :z) :init-pose nil :rotation-axis t)
      (send *atlas* :head :look-at (send (send *atlas* :get :larm-hand-coords-for-hose) :worldpos) :debug-view nil)
      (progn (model2real :time 700) (hand-model2real :time 700) (send *ri* :wait-interpolation))
      (send *atlas* :hand :larm :angle-vector #f(0.0 90.0 10.0 0.0 90.0 10.0 0.0 90.0 10.0 0.0 90.0 40.0))
      (progn (model2real :time 1000) (hand-model2real :time 1000) (send *ri* :wait-interpolation))
      (send *atlas* :hand :larm :angle-vector #f(0.0 90.0 40.0 0.0 90.0 40.0 0.0 90.0 40.0 0.0 90.0 60.0))
      (progn (model2real :time 1000) (hand-model2real :time 1000) (send *ri* :wait-interpolation))
      (snapshot-image :larm)
      ;;(do ((ang 50 (+ ang -10))) ((< ang -30))
      (do ((ang 50 (+ ang -10))) ((< ang -40))
	(tack-arm-pose :arm :larm)
	(test-ik1 (send (send (send *ikc* :copy-worldcoords) :translate (float-vector 0 0 -30) :world) :rotate (deg2rad ang) :z) :init-pose nil :rotation-axis t)
	;;(test-ik1 (send (send (send *ikc* :copy-worldcoords) :translate (float-vector 0 0 0) :world) :rotate (deg2rad ang) :z) :init-pose nil :rotation-axis t)
	(send *atlas* :head :look-at (send (send *atlas* :get :larm-hand-coords-for-hose) :worldpos) :debug-view nil)
	(progn (model2real :time 700) (hand-model2real :time 700) (send *ri* :wait-interpolation)))
      (send *atlas* :hand :larm :angle-vector #f(0.0 00.0 0.0 0.0 90.0 40.0 0.0 90.0 40.0 0.0 90.0 45.0))
      (progn (model2real :time 2000) (hand-model2real :time 2000) (send *ri* :wait-interpolation))
      (snapshot-image :all)
      (format t ";; completion = ~A~%" (progn (dotimes (i 10) (ros::spin-once)) *completion-score*))
      (setq ret (y-or-n-p ";; continue?"))
      ))
  (send *atlas* :hand :larm :open-pose2)
  (progn (model2real :time 2000) (hand-model2real :time 2000) (send *ri* :wait-interpolation))
  (progn (prepare-for-rotate-hose #f(500 100 1400)) (unix:sleep 1) (snapshot-image :larm))
  (tack-arm-pose :arm :larm)
  (progn (model2real :time 4000) (hand-model2real :time 2000) (send *ri* :wait-interpolation))
  (format t ";; completion = ~A~%" (progn (dotimes (i 10) (ros::spin-once)) *completion-score*))
  )

(defun prepare-for-rotate-valve
  (hand-offset-pos)
  (send *atlas* :reset-manip-pose :base-down t)
  (send *atlas* :head :neck-p :joint-angle 50)
  (send *atlas* :rarm :move-end-pos hand-offset-pos :world :rotation-axis :z)
  (send *atlas* :move-centroid-on-foot
	:both '(:rleg :lleg :larm :rarm)
	:target-centroid-pos (send *atlas* :calc-target-centroid-pos :both '(:rleg :lleg))
	:centroid-thre 40)
  (progn (model2real :time 3000) (hand-model2real :time 2000) (send *ri* :wait-interpolation))
  )

(defun rotate-valve ()
  (do-until-key
   (dolist (pos (list #f(0 70 0) #f(0 0 70) #f(0 -70 0) #f(0 0 -70)))
     (send *atlas* :rarm :move-end-pos pos :world :rotation-axis :z)
     (progn (model2real :time 1000) (hand-model2real :time 1000) (send *ri* :wait-interpolation))
     )
   (format t ";; completion = ~A~%" (progn (dotimes (i 10) (ros::spin-once)) *completion-score*)))
  )

(defun setup-hand-coords-for-hose ()
  (unless (send *atlas* :get :larm-hand-coords-for-hose)
    (let ((lc (make-cascoords
	     :coords (send (send (send *atlas* :larm :end-coords :copy-worldcoords) :rotate (deg2rad -40) :x) :translate (float-vector 60 80 120))
	     :name :larm-hand-coords-for-hose)))
    (send (send *atlas* :larm :end-coords :parent) :assoc lc)
    (send *atlas* :put :larm-hand-coords-for-hose lc)
    ))
  (unless (send *atlas* :get :rarm-hand-coords-for-hose)
    (let ((lc (make-cascoords
	       :coords (send (send (send *atlas* :rarm :end-coords :copy-worldcoords) :rotate (deg2rad 40) :x) :translate (float-vector 60 -80 120))
	       :name :rarm-hand-coords-for-hose)))
      (send (send *atlas* :rarm :end-coords :parent) :assoc lc)
      (send *atlas* :put :rarm-hand-coords-for-hose lc)
      ))
  (unless (send *atlas* :get :rarm-hand-coords-for-hose2)
    (let ((lc (make-cascoords
	       :coords (send (send (send *atlas* :rarm :end-coords :copy-worldcoords) :rotate (deg2rad 40) :x) :translate (float-vector 60 -100 50))
	       :name :rarm-hand-coords-for-hose2)))
      (send (send *atlas* :rarm :end-coords :parent) :assoc lc)
      (send *atlas* :put :rarm-hand-coords-for-hose2 lc)
      ))
  )

(defun init-vrc-task3 ()
  (null-output (load "package://hrpsys_gazebo/euslisp/atlas-interface.l"))
  (null-output (atlas-init :set-reset-pose nil :set-user-mode nil))
  (real2model)
  (model2real :wait t :time 10)
  (set-user)
  (set-kp 4000)
  (send *ri* :set-st-param
	:k-tpcc-p #f(2.5 2.5)
	:k-tpcc-x #f(5 5)
	:k-brot-p #f(0.1 0.1)
	:k-brot-tc #f(0.1 0.1))
  (send *ri* :set-gait-generator-param
         :default-double-support-ratio 0.2
         :default-step-time 1.0
         :default-step-height 0.1
	 :stride-parameter #f(0.20 0.05 20))
  (send *atlas* :walk-pose)
  (send *atlas* :head :neck-p :joint-angle 40)
  (model2real :wait t :time 5000)
  (setup-hand-coords-for-hose)
  )

(defun move-to-table (&key (init-pose t))
  (if init-pose (init-pose-for-hose))
  (send *ri* :set-st-param
	:k-tpcc-p #f(1.5 1.5)
	:k-tpcc-x #f(5.0 5.0)
	:k-brot-p #f(0.05 0.05)
	:k-brot-tc #f(0.1 0.1))
  ;;(send *ri* :teleport-to (make-coords :pos #f(-2700 -4100 1000)))
  ;;(send *ri* :teleport-to (make-coords :pos #f(-3200 -4000 1000))) ;; far
  ;;(progn (real2model) (model2real :wait t :time 10) (set-user) (set-kp 4000))
  ;;(unix:sleep 10)
  (send *ri* :start-auto-balancer)
  (send *ri* :stop-st)
  (let ((ret t))
    (while ret
      (warn ";; point on image_view~%")
      (let ((gp (get-point)))
	(when gp
	  (let* ((goal-pos-from-get-point
		  ;;(send gp :transform-vector (v+ #F(0 0 0) #f(-541.243 97.2492 -1028.51))))
		  (send gp :transform-vector (v+ #F(0 0 0) #f(-525.243 97.2492 -1028.51))))
		 (dif (scale 1e-3
			 (send (apply #'midcoords 0.5 (send *atlas* :legs :end-coords :copy-worldcoords))
			       :inverse-transform-vector goal-pos-from-get-point))))
	    ;;(warn ";; go pos diff ~A ~%" (scale 1e3 dif))
	    (warn ";; go pos diff ~A ~%" (scale 1e3 dif))
	(if (y-or-n-p ";; go pos ?~%")
	    (progn
	      ;;(send *ri* :start-auto-balancer)
	      (send *ri* :start-st)
	      (send *ri* :go-pos-no-wait (elt dif 0) 0 0)
	      (send *ri* :wait-foot-steps)
	      (unix:usleep 100000)
	      (send *ri* :go-pos-no-wait 0 (elt dif 1) 0)
	      (send *ri* :wait-foot-steps)
	      (unix:usleep 100000)
	      (print ";; enter rot value")
	      (let ((rotv (read-from-string (read-line))))
		(when (y-or-n-p ";; rot?~%")
		  (send *ri* :go-pos-no-wait 0 0 rotv)
		  (send *ri* :wait-foot-steps)))
	      (send *ri* :stop-st)
	      ;;(send *ri* :stop-auto-balancer)
	      )
	  (setq ret nil))))))) 
  (send *ri* :set-st-param
	:k-tpcc-p #f(2.5 2.5)
	:k-tpcc-x #f(5 5)
	:k-brot-p #f(0.1 0.1)
	:k-brot-tc #f(0.1 0.1))
 ;; (let ((ret t))
  ;;   (while ret
  ;;     (warn ";; point on image_view~%")
  ;;     (let ((gp (get-point)))
  ;; 	(when gp
  ;; 	  (let* ((goal-pos-from-get-point
  ;; 		  (send gp :transform-vector #f(-541.243 97.2492 -1028.51)))
  ;; 		 (dif (scale 1e-3
  ;; 			 (send (apply #'midcoords 0.5 (send *atlas* :legs :end-coords :copy-worldcoords))
  ;; 			       :inverse-transform-vector goal-pos-from-get-point))))
  ;; 	    ;;(warn ";; go pos diff ~A ~%" (scale 1e3 dif))
  ;; 	    (warn ";; go pos diff ~A ~%" (scale 1e3 dif))
  ;; 	    (setq dif (v+ dif (scale 1e-3 (float-vector 30 20 0))))
  ;; 	(if (y-or-n-p ";; go pos ?~%")
  ;; 	    (progn
  ;; 	      (send *ri* :start-auto-balancer)
  ;; 	      (send *ri* :start-st)
  ;; 	      (send* *ri* :go-pos (concatenate cons (subseq dif 0 2) (list 0)))
  ;; 	      (send *ri* :stop-st))
  ;; 	  (setq ret nil)))))))
  )

(defun gp1 (xx yy th &optional (use-snapshot-image t))
  (send *ri* :go-pos-no-wait xx yy th)
  (send *ri* :wait-foot-steps)
  (unix:usleep 100000)
  (if use-snapshot-image (snapshot-image :head))
  )

(defun gp3 (xx yy th)
  (send *ri* :go-pos-no-wait xx yy th)
  (send *ri* :wait-foot-steps)
  (unix:usleep 100000)
  )

(defun start-gp 
  ()
  (send *ri* :start-auto-balancer)
  (send *ri* :start-st)
  )

(defun stop-gp (&optional (use-snapshot-image t))
  (send *ri* :stop-st)
  (send *ri* :stop-auto-balancer)
  (if use-snapshot-image (snapshot-image :head))
  )
  
(defun pickup-hose
  ()
  (reach-and-pickup-hose)
  (unix:sleep 2)
  (grasp-hose-by-larm)
  (mate-hose-by-larm-0)
  )

(defun check-hand-av ()
  (dolist (arm '(:rarm :larm))
    (grasp-hand :arm arm)
    (send *ri* :hand arm :wait-interpolation)
    (format t ";; grasp ~A ~A~%" arm (v- (get-model-hand-av arm) (get-real-hand-av arm)))
    (open-hand :arm arm)
    (send *ri* :hand arm :wait-interpolation)
    (format t ";; open ~A ~A~%" arm (v- (get-model-hand-av arm) (get-real-hand-av arm)))
    )
  )

(defun usage ()
  ;;(warn ";; (init-vrc-task3) ~%")
  (warn ";; (init-pose-for-hose) ~%")
  ;;(warn ";; (move-to-table) ~%")
  (warn ";; (move-to-table :init-pose nil) ~%")
  (warn ";; (look-at-hose) ~%")
  (warn ";; (setq *thp* (get-point2 4)) ~%")
  (warn ";; (pickup-hose) ~%")
  (warn ";; (reach-and-pickup-hose) ~%")
  (warn ";; (grasp-hose-by-larm) ~%")
  (warn ";; (mate-hose-by-larm-0) ~%")
  (warn ";; (release-hose-end) ~%")
  (warn ";; (gp3 0 0 0) ~%")
  (warn ";; (go-pos2) ~%")
  (warn ";; (go-pos3) ~%")
  (warn ";; (move-hose-end-from-ikc #f(0 0 0) 0 0 t t) ~%")
  (warn ";; (setq *h-pos* (get-point2 4)) ~%")
  (warn ";; (setq *p-pos* (get-point2 4)) ~%")
  (warn ";; (prepare-for-rotate-hose #f(500 100 1400) 0 4000 :z t) ~%")
  (warn ";; (prepare-for-rotate-hose #f(480 -50 1300) -30 2000 :z t) ~%")
  (warn ";; (prepare-for-rotate-hose2 0 0 0 2000 :z t) ~%")
  (warn ";; (open-hand-for-hose-rotate)~%")
  (warn ";; (open-hand :arm :larm)~%")
  (warn ";; (grasp-hand :arm :larm)~%")
  (warn ";; (send *ikc* :translate #f(0 0 -30)) ~%")
  (warn ";; (rotate-hose) ~%")
  (warn ";; (progn (open-hand-for-hose-rotate) (prepare-for-rotate-hose #f(480 -50 1300) -37 3000 :z t t) (prepare-for-rotate-hose2 30 0 0 2000 :z t)) ~%")
  (warn ";; (prepare-for-rotate-valve #f(0 0 0)) ~%")
  (warn ";; (rotate-valve) ~%")
  (warn ";; (tack-arm-pose :arm :larm) ~%")
  (warn ";; (progn (model2real :time 4000) (hand-model2real :time 4000) (send *ri* :wait-interpolation)) ~%")
  (warn ";; (snapshot-image :all) ~%")
  (warn ";; (snapshot-image :head) ~%")
  (warn ";; (snapshot-image :rarm) ~%")
  (warn ";; (snapshot-image :larm) ~%")
  )
(usage)

(progn
  (null-output (load "package://hrpsys_gazebo/euslisp/atlas-interface.l"))
  (unless (and x::*display* (> x::*display* 0))
    (load "package://hrpsys_gazebo/euslisp/no-display"))
  (unless (boundp '*ri*)
    (null-output (atlas-init :set-reset-pose nil :set-user-mode nil))
    (send *ri* :hand :rarm :wait-interpolation)
    (send *ri* :hand :larm :wait-interpolation)
    (init-vrc-task3))
  (unless (boundp '*completion-score*)
    (defparameter *completion_score* 0))
  )

(defun score-callback-for-task3
  (msg)
  (format t ";; score = ~A~%" (send msg :completion_score))
  (defparameter *completion-score* (send msg :completion_score))
  )

(ros::subscribe "/vrc_score"
                atlas_msgs::vrcscore
                #'score-callback-for-task3)
 
#|
;; go pos candidate
(gp1 2.5 0 0)
(gp1 3.5 0 0)
(gp1 0 0 100)
(gp1 8 0 0)
(gp1 0 0 100)
(gp1 6 0 0)
(gp1 0 0 -90)
(gp1 2 0 0)
(gp1 0 0 -70)
(gp1 1.5 0 0)

;;
(defun rot-handle (ang)
  (let* ((arms '(:rarm :larm))
	 (ac (mapcar #'(lambda (a) (make-cascoords :coords (send *atlas* a :end-coords :copy-worldcoords))) arms))
	 (cen-c (make-cascoords :pos (send (apply #'midcoords 0.5 ac) :worldpos))))
    (send cen-c :assoc (car ac))
    (send cen-c :assoc (cadr ac))
    (send cen-c :rotate (deg2rad ang) :x)
    (print ac)
    ))

  (with-assoc-move-target
   (mt :move-target (send (send (send *atlas* :rarm :end-coords :copy-worldcoords) :rotate (deg2rad 30) :x) :translate (float-vector 60 -80 100))
       :parent-link (send *atlas* :rarm :end-coords :parent))
     (send *atlas* :rarm :inverse-kinematics
           (send (send (send (send *atlas* :larm :end-coords :copy-worldcoords) :rotate (deg2rad -30) :x) :translate (float-vector 60 80 100)) :translate (float-vector 0 0 0))
	   :move-target (car mt)
	   :debug-view :no-message :thre 10 :stop 200 :rotation-axis :z))


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
;;
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

;;
(defun grasp-hose-by-rarm ()
  (send *atlas* :larm :inverse-kinematics
  	(send (send (make-coords :pos #f(500 80 1500) :rpy (list 0 0 pi/2)) :rotate pi/2 :y :world) :rotate (deg2rad 30) :x)
  	:debug-view :no-message :thre 10 :stop 200 :rotation-axis :z) 
  (send *atlas* :rarm :wrist-y :joint-angle -90)
  (send *atlas* :rarm :inverse-kinematics
  	(send (send (make-coords :pos #f(500 -400 1200) :rpy (list 0 0 pi/2)) :rotate pi/2 :y :world) :rotate (deg2rad 30) :x)
  	:debug-view :no-message :thre 10 :stop 200 :rotation-axis :z)
  (send *atlas* :head :neck-p :joint-angle 50)
  (progn (model2real) (hand-model2real :time 1000) (send *ri* :wait-interpolation))

  ;; (send *atlas* :rarm :inverse-kinematics
  ;;   	(send (send (make-coords :pos #f(500 -400 1100) :rpy (list 0 0 pi/2)) :rotate pi/2 :y :world) :rotate (deg2rad 30) :x)
  ;;   	:debug-view :no-message :thre 10 :stop 200 :rotation-axis :z)
  ;; (progn (model2real) (hand-model2real :time 1000) (send *ri* :wait-interpolation))
  (send *atlas* :rarm :inverse-kinematics
  	(send (send (make-coords :pos #f(500 -100 1200) :rpy (list 0 0 pi/2)) :rotate pi/2 :y :world) :rotate (deg2rad 30) :x)
  	:debug-view :no-message :thre 10 :stop 200 :rotation-axis :z)
  (send *atlas* :head :neck-p :joint-angle 50)
  (progn (model2real) (hand-model2real :time 1000) (send *ri* :wait-interpolation))

  (send *atlas* :larm :inverse-kinematics
  	(send (send (make-coords :pos #f(500 0 1300) :rpy (list 0 0 pi/2)) :rotate pi/2 :y :world) :rotate (deg2rad 30) :x)
  	:debug-view :no-message :thre 10 :stop 200 :rotation-axis :z) 
  (send *atlas* :head :neck-p :joint-angle 50)
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


|#