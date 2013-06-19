
;; \file    atlas-vehicle-motion-get-in.l
;; \author  Murooka Masaki
;; \date    2013/06/15
;; \brief   vrc_task_1関連 車乗り込み行動要素関数
;;


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;; vehicle motion [get-in]
(defun get-in-for-task1
  (&key (real nil) (wait nil) (telep nil) (after-walk nil))

  (warning-message 2 "[vhc-get-in] start (get-in-for-task1)~%")

  (make-vehicle)
  (setq *lr-reverse* t)

  (unless after-walk
    (telep-in-get-in-for-task1 :real real :wait wait :telep telep))

  (grasp-rarm-in-get-in-for-task1 :real real :wait wait)

  (lift-rleg-in-get-in-for-task1 :real real :wait wait)

  (lift-whole-body-in-get-in-for-task1 :real real :wait wait)

  ;; (grasp-larm-in-get-in-for-task1 :real real :wait wait)
  ;; (grasp-rarm-2-in-get-in-for-task1 :real real :wait wait)
  ;; (lift-lleg-in-get-in-for-task1 :real real :wait wait)

  (warning-message 2 "[vhc-get-in] finish (get-in-for-task1)~%")
  )

(defun telep-in-get-in-for-task1
  (&key (real nil) (wait nil) (init nil) (telep t))

  (warning-message 2 "[vhc-get-in] start (telep-in-get-in-for-task1)~%")

  (make-vehicle)
  (setq *lr-reverse* t)

  (when init
    (warning-message 2 "[vhc-get-in] call (my-init)~%")
    (my-init :real real)
    (send *irtviewer* :draw-objects))

  (when (and real telep)
    (warning-message 2 "[vhc-get-in] call (telep-start) and (stand-up-start)~%")
    (my-init :real real)
    (telep-start)
    (stand-up-start))

  (when real
    (warning-message 2 "[vhc-get-in] call (manip)~%")
    (when wait (my-wait))
    (manip)
    (send *irtviewer* :draw-objects))

  (warning-message 2 "[vhc-get-in] grasp rarm and ungrasp larm~%")
  (when wait (my-wait))
  (grasp :arm :rarm :real real)
  (ungrasp :arm :larm :real real)
  (send *irtviewer* :draw-objects)

  (warning-message 2 "[vhc-get-in] angle-vector before teleport~%")
  (when wait (my-wait))
  (send *atlas* :angle-vector #f(0.0 0.0 0.0 23.5 114.5 0.0 -75.0 17.0 0.0 -135.0 129.6 -71.6129 79.9998 60.7501 0.0 61.5619 -1.79235 -35.568 1.79235 -25.994 0.012938 61.5446 1.91967 -35.5619 -1.91967 -25.9828 0.01294 0.0))
  (send *irtviewer* :draw-objects)
  (when real (model2real :wait t))

  (warning-message 2 "[vhc-get-in] call :teleport~%")
  (when wait (my-wait))
  (when (and real telep) 
    (let* ((telep-coords (send (send *vehicle-side-pos* :copy-worldcoords) :translate #f(0 0 950) :world))
	   (telep-pos (send telep-coords :worldpos))
	   (telep-rpy (car (rpy-angle (send telep-coords :rot)))))
      (when *lr-reverse*
	(setq telep-coords (make-coords :pos (float-vector (elt telep-pos 0) (- (elt telep-pos 1)) (elt telep-pos 2))
					:rpy (list (- (elt telep-rpy 0)) (elt telep-rpy 1) (- (elt telep-rpy 2)))))
	;; (send telep-coords :translate #f(-70 10 0) :local)
	)
      (send *ri* :teleport-to telep-coords nil :keep-effort t)))
  (send *atlas* :fix-leg-to-coords *vehicle-side-pos* '(:lleg :rleg))
  (send *irtviewer* :draw-objects)

  (make-vehicle)

  (warning-message 2 "[vhc-get-in] finish (telep-in-get-in-for-task1)~%")
  )

(defun grasp-rarm-in-get-in-for-task1
  (&key (real nil) (wait nil))

  (warning-message 2 "[vhc-get-in] start (grasp-rarm-in-get-in-for-task1) [get-in phase1]~%")

  (when real
    (warning-message 2 "[vhc-get-in] set high-gain :all in grasp rarm [get-in phase1]~%")
    (when wait (my-wait))
    (set-high-gain))

  (let* ((pipe-coords
	  (send (send (send *drcv* :handle :name :upper-left-pipe) :copy-worldcoords) :translate #f(0 0 0) :local)))

    (when real
      (warning-message 2 "[vhc-get-in] set high-gain :rarm and zero-gain :head in grasp rarm [get-in phase1]~%")
      (when wait (my-wait))
      (set-high-gain :rarm)
      (set-zero-gain :head))

    (warning-message 2 "[vhc-get-in] angle-vector1 in grasp rarm [get-in phase1]~%")
    (when wait (my-wait))
    ;; (if real (real2model))
    (send *atlas* :bdi-stand-pose)
    (fix-leg-vehicle-side)
    (send *atlas* :rarm :inverse-kinematics 
	  (send (send pipe-coords :copy-worldcoords)
		:transform (make-coords :pos #f(516 84 98) :rpy '(0.374725 0.848509 2.09956)))
	  :target-centroid-pos (send (send (send (send *atlas* :foot-midcoords) :copy-worldcoords) :translate #f(-50 0 0)) :pos)
	  :centroid-thre 500
	  :rotation-axis nil :thre 5)
    (fix-leg-vehicle-side)
    (move-centroid)
    (send *irtviewer* :draw-objects)
    (when real (model2real :wait t))

    (warning-message 2 "[vhc-get-in] angle-vector2 in grasp rarm [get-in phase1]~%")
    (when wait (my-wait))
    (send *atlas* :rarm :inverse-kinematics
	  (send (send pipe-coords :copy-worldcoords)
		:transform (make-coords :pos #f(112 189 104) :rpy '(-1.09049 0.919819 0.813976)))
	  :target-centroid-pos (send (send (send (send *atlas* :foot-midcoords) :copy-worldcoords) :translate #f(-50 0 0)) :pos)
	  :centroid-thre 500
	  :rotation-axis nil :thre 5)
    (fix-leg-vehicle-side)
    (move-centroid)
    (send *irtviewer* :draw-objects)
    (when real (model2real :wait t))

    (warning-message 2 "[vhc-get-in] ungrasp rarm in grasp rarm [get-in phase1]~%")
    (when wait (my-wait))
    (ungrasp :arm :rarm :real real)
    (send *irtviewer* :draw-objects)
    (when real (unix::usleep (* 2 1000 1000)))

    (warning-message 2 "[vhc-get-in] angle-vector3 in grasp rarm [get-in phase1]~%")
    (when wait (my-wait))
    (send *atlas* :rarm :inverse-kinematics
	  (send (send pipe-coords :copy-worldcoords)
		:transform (make-coords :pos #f(58 104 30) :rpy '(0 0 0)))
	  :target-centroid-pos (send (send (send (send *atlas* :foot-midcoords) :copy-worldcoords) :translate #f(-50 0 0)) :pos)
	  :centroid-thre 500
	  :translation-axis :z :rotation-axis :z :thre 5)
    (fix-leg-vehicle-side)
    (move-centroid)
    (send *irtviewer* :draw-objects)
    (when real (model2real :wait t))

    ;; (if nil
    (if real
	(progn
	  (warning-message 2 "[vhc-get-in] reach until touch in grasp rarm [get-in phase1]~%")
	  (when wait (my-wait))
	  (reach-until-touch
	   :rarm
	   (v- (send pipe-coords :worldpos)
	       (send (send (send pipe-coords :copy-worldcoords)
			   :transform (make-coords :pos #f(58 104 0) :rpy '(0 0 0))) :worldpos))
	   75)
	  (save-last))
      (progn
	(warning-message 2 "[vhc-get-in] angle-vector4 in grasp rarm [get-in phase1]~%")
	(when wait (my-wait))
	(send *atlas* :rarm :inverse-kinematics
	      pipe-coords
	      :target-centroid-pos (send (send (send (send *atlas* :foot-midcoords) :copy-worldcoords) :translate #f(-50 0 0)) :pos)
	      :centroid-thre 500
	      :rotation-axis :z :thre 5)
	(fix-leg-vehicle-side)
	(move-centroid)
	(send *irtviewer* :draw-objects)
	(when real (model2real :wait t))))

    (warning-message 2 "[vhc-get-in] pregrasp rarm in grasp rarm [get-in phase1]~%")
    (when wait (my-wait))
    (pregrasp :arm :rarm :real real)
    (send *irtviewer* :draw-objects)
    (when real (unix::usleep (* 3 1000 1000)))

    (when real
      (progn
        (warning-message 2 "[vhc-get-in] reach until touch 2 in grasp rarm [get-in phase1]~%")
        (when wait (my-wait))
        (reach-until-touch
         :rarm
         (v- (send pipe-coords :worldpos)
             (send (send (send pipe-coords :copy-worldcoords)
                         :transform (make-coords :pos #f(0 -1 2) :rpy '(0 0 0)) :world) :worldpos))
         100)
        (save-last)))

    (when real
      (warning-message 2 "[vhc-get-in] set middle-gain :rarm in grasp rarm [get-in phase1]~%")
      (when wait (my-wait))
      (set-middle-gain :rarm))

    (warning-message 2 "[vhc-get-in] grasp rarm in grasp rarm [get-in phase1]~%")
    (when wait (my-wait))
    (grasp :arm :rarm :real real)
    (send *irtviewer* :draw-objects)
    (when real (unix::usleep (* 3 1000 1000)))

    (when real
      (warning-message 2 "[vhc-get-in] adapt angle-vector in grasp rarm [get-in phase1]~%")
      (when wait (my-wait))
      (real2model)
      (send *irtviewer* :draw-objects)
      (model2real :time 1000 :wait t))

    (when real
      (warning-message 2 "[vhc-get-in] set high-gain :rarm in grasp rarm [get-in phase1]~%")
      (when wait (my-wait))
      (set-high-gain :rarm))

    (when real
      (when wait (my-wait))
      (warning-message 2 "[vhc-get-in] call (set-user) in grasp rarm [get-in phase1]~%")
      (set-user))

    )

  (warning-message 2 "[vhc-get-in] finish (grasp-rarm-in-get-in-for-task1) [get-in phase1]~%")
  )

(defun lift-rleg-in-get-in-for-task1
  (&key (real nil) (wait nil))

  (warning-message 2 "[vhc-get-in] start (lift-rleg-in-get-in-for-task1) [get-in phase2]~%")

  (when real
    (warning-message 2 "[vhc-get-in] set zero-gain :larm in lift rleg [get-in phase2]~%")
    (when wait (my-wait))
    (set-zero-gain :larm))

  (warning-message 2 "[vhc-get-in] lift right leg on ground in lift rleg [get-in phase2]~%")
  (when wait (my-wait))
  (let* ((tc (list (send (send *atlas* :rleg :end-coords :copy-worldcoords) :translate #f(195 0 500) :world)
		   nil
		   nil
		   (send (send (send *atlas* :worldcoords) :copy-worldcoords) :translate #f(0 300 0) :world))))
    (with-move-target-link-list
     (mt ll *atlas* '(:rleg :lleg :rarm :larm))
     (send *atlas* :fullbody-inverse-kinematics
	   (mapcar #'(lambda (c m) (if c c (send m :copy-worldcoords))) tc mt)
	   :link-list ll :move-target mt
	   :collision-avoidance-link-pair nil
           :revert-if-fail nil 	   :stop 1000	   
	   :translation-axis '(t t t nil)
	   :rotation-axis '(t t t nil)
	   :thre '(5 5 10 10)
	   :rthre (list (deg2rad 1) (deg2rad 1) (deg2rad 2) (deg2rad 2))
	   :target-centroid-pos (send (send (send *atlas* :lleg :end-coords :copy-worldcoords) :translate #f(-10 0 0)) :pos)
	   :centroid-thre 500)))
  (send *irtviewer* :draw-objects)
  (when real (model2real :wait t))
  (save-last)

  (warning-message 2 "[vhc-get-in] turn right leg in yaw direction in lift rleg [get-in phase2]~%")
  (when wait (my-wait))
  (send (send *atlas* :rleg :crotch-y) :joint-angle -35)
  (send *irtviewer* :draw-objects)
  (when real (model2real :wait t :time 1000))

  (warning-message 2 "[vhc-get-in] move right leg in vehicle in lift rleg [get-in phase2]~%")
  (when wait (my-wait))
  (let* ((tc (list (send (send *atlas* :rleg :end-coords :copy-worldcoords) :translate #f(150 -60 25) :local)
		   nil
		   nil
		   (send (send (send *atlas* :worldcoords) :copy-worldcoords) :translate #f(0 300 0) :world))))
    (with-move-target-link-list
     (mt ll *atlas* '(:rleg :lleg :rarm :larm))
     (send *atlas* :fullbody-inverse-kinematics
	   (mapcar #'(lambda (c m) (if c c (send m :copy-worldcoords))) tc mt)
	   :link-list ll :move-target mt
	   :collision-avoidance-link-pair nil
           :revert-if-fail nil 	   :stop 1000	   
	   :translation-axis '(t t t nil)
	   :rotation-axis '(t t t nil)
	   :thre '(5 5 10 10)
	   :rthre (list (deg2rad 1) (deg2rad 1) (deg2rad 2) (deg2rad 2))
	   :target-centroid-pos (send (send (send *atlas* :lleg :end-coords :copy-worldcoords) :translate #f(-10 0 0)) :pos)
	   :centroid-thre 500)))
  (send *irtviewer* :draw-objects)
  (when real (model2real :wait t :time 1000))
  (save-last)

  (warning-message 2 "[vhc-get-in] take down right leg in vehicle in lift rleg [get-in phase2]~%")
  (when wait (my-wait))
  (let* ((tc (list 
	      (send (send *atlas* :rleg :end-coords :copy-worldcoords) :translate #f(0 0 -150) :world)
	      ;; (if real
	      ;; 	  (send (send *atlas* :rleg :end-coords :copy-worldcoords) :translate #f(0 0 -100) :world)
	      ;; 	(send (send *atlas* :rleg :end-coords :copy-worldcoords) :translate #f(0 0 -150) :world))
	      nil
	      nil
	      (send (send (send *atlas* :worldcoords) :copy-worldcoords) :translate #f(0 300 0) :world))))
    (with-move-target-link-list
     (mt ll *atlas* '(:rleg :lleg :rarm :larm))
     (send *atlas* :fullbody-inverse-kinematics
	   (mapcar #'(lambda (c m) (if c c (send m :copy-worldcoords))) tc mt)
	   :link-list ll :move-target mt
	   :collision-avoidance-link-pair nil
           :revert-if-fail nil 	   :stop 1000	   
	   :translation-axis '(t t t nil)
	   :rotation-axis '(t t t nil)
	   :thre '(5 5 10 10)
	   :rthre (list (deg2rad 1) (deg2rad 1) (deg2rad 2) (deg2rad 2))
	   :target-centroid-pos (send (send (send *atlas* :lleg :end-coords :copy-worldcoords) :translate #f(-10 0 0)) :pos)
	   :centroid-thre 500)))
  (send *irtviewer* :draw-objects)
  (when real (model2real :wait t))
  (save-last)

  ;; (when real
  ;;   (warning-message 2 "[vhc-get-in] take down right leg until touch in vehicle in lift rleg [get-in phase2]~%")
  ;;   (when wait (my-wait))
  ;;   (reach-until-touch :rleg #f(0 0 -1) 200)
  ;;   (save-last))

  (when real
    (warning-message 2 "[vhc-get-in] adapt right leg in lift rleg [get-in phase2]~%")
    (when wait (my-wait))
    (adapt-leg :rleg)
    (save-last))

  (warning-message 2 "[vhc-get-in] lift left leg on ground in lift rleg [get-in phase2]~%")
  (when wait (my-wait))
  (let* ((tc (list nil
		   (send (send *atlas* :lleg :end-coords :copy-worldcoords) :translate #f(75 -125 100) :world)
		   nil
		   (send (send (send *atlas* :worldcoords) :copy-worldcoords) :translate #f(0 300 0) :world))))
    (with-move-target-link-list
     (mt ll *atlas* '(:rleg :lleg :rarm :larm))
     (send *atlas* :fullbody-inverse-kinematics
	   (mapcar #'(lambda (c m) (if c c (send m :copy-worldcoords))) tc mt)
	   :link-list ll :move-target mt
	   :collision-avoidance-link-pair nil
           :revert-if-fail nil 	   :stop 1000	   
	   :translation-axis '(t t t nil)
	   :rotation-axis '(t t t nil)
	   :thre '(5 5 10 10)
	   :rthre (list (deg2rad 1) (deg2rad 1) (deg2rad 2) (deg2rad 2))
	   ;; :target-centroid-pos (send (send (send (send *atlas* :foot-midcoords) :copy-worldcoords) :translate #f(10 -30 0)) :pos) ;; hogehoge
	   :target-centroid-pos (send (send (send (send *atlas* :foot-midcoords) :copy-worldcoords) :translate #f(20 -20 0)) :pos)
	   :centroid-thre 500)))
  (send *irtviewer* :draw-objects)
  (when real (model2real :wait t :time 2000))
  (save-last)

  (warning-message 2 "[vhc-get-in] take down left leg on ground in lift rleg [get-in phase2]~%")
  (when wait (my-wait))
  (let* ((tc (list 
	      nil
	      (make-coords :pos (send (send (send *atlas* :lleg :end-coords :copy-worldcoords) :translate #f(75 -125 -100) :world) :worldpos))
	      ;; (if real
	      ;; 	  (make-coords :pos (send (send (send *atlas* :lleg :end-coords :copy-worldcoords) :translate #f(75 -125 -75) :world) :worldpos))
	      ;; 	(make-coords :pos (send (send (send *atlas* :lleg :end-coords :copy-worldcoords) :translate #f(75 -125 -100) :world) :worldpos)))
	      nil
	      (send (send (send *atlas* :worldcoords) :copy-worldcoords) :translate #f(0 300 0) :world))))
    (with-move-target-link-list
     (mt ll *atlas* '(:rleg :lleg :rarm :larm))
     (send *atlas* :fullbody-inverse-kinematics
	   (mapcar #'(lambda (c m) (if c c (send m :copy-worldcoords))) tc mt)
	   :link-list ll :move-target mt
	   :collision-avoidance-link-pair nil
           :revert-if-fail nil 	   :stop 1000	   
	   :translation-axis '(t t t nil)
	   :rotation-axis '(t t t nil)
	   :thre '(5 5 10 10)
	   :rthre (list (deg2rad 1) (deg2rad 1) (deg2rad 2) (deg2rad 2))
	   ;; :target-centroid-pos (send (send (send (send *atlas* :foot-midcoords) :copy-worldcoords) :translate #f(10 -30 0)) :pos) ;; hogehoge
	   :target-centroid-pos (send (send (send (send *atlas* :foot-midcoords) :copy-worldcoords) :translate #f(20 -20 0)) :pos)
	   :centroid-thre 500)))
  (send *irtviewer* :draw-objects)
  (when real (model2real :wait t :time 2000))
  (save-last)

  ;; (when real
  ;;   (warning-message 2 "[vhc-get-in] take down left leg until touch in vehicle in lift rleg [get-in phase2]~%")
  ;;   (when wait (my-wait))
  ;;   (reach-until-touch :lleg #f(0 0 -1) 200)
  ;;   (save-last))

  (when real
    (warning-message 2 "[vhc-get-in] adapt left leg in lift rleg [get-in phase2]~%")
    (when wait (my-wait))
    (adapt-leg :lleg)
    (save-last))

  (warning-message 2 "[vhc-get-in] lift right leg on vehicle in lift rleg [get-in phase2]~%")
  (when wait (my-wait))
  (let* ((tc (list (send (send *atlas* :rleg :end-coords :copy-worldcoords) :translate #f(0 0 100) :world)
		   nil
		   nil
		   (send (send (send *atlas* :worldcoords) :copy-worldcoords) :translate #f(0 300 0) :world))))
    (with-move-target-link-list
     (mt ll *atlas* '(:rleg :lleg :rarm :larm))
     (send *atlas* :fullbody-inverse-kinematics
  	   (mapcar #'(lambda (c m) (if c c (send m :copy-worldcoords))) tc mt)
  	   :link-list ll :move-target mt
  	   :collision-avoidance-link-pair nil
           :revert-if-fail nil 	   :stop 1000  	   
	   :translation-axis '(t t t nil)
  	   :rotation-axis '(t t t nil)
	   :thre '(5 5 10 10)
	   :rthre (list (deg2rad 1) (deg2rad 1) (deg2rad 2) (deg2rad 2))
	   :target-centroid-pos (send (send (send *atlas* :lleg :end-coords :copy-worldcoords) :translate #f(20 0 0)) :pos)
  	   :centroid-thre 500)))
  (when real (model2real :wait t))
  (send *irtviewer* :draw-objects)
  (save-last)

  (warning-message 2 "[vhc-get-in] move right leg in vehicle in lift rleg [get-in phase2]~%")
  (when wait (my-wait))
  (let* ((tc (list 
	      (send (send *atlas* :rleg :end-coords :copy-worldcoords) :translate #f(100 -150 25) :local)
	      ;; (send (send *atlas* :rleg :end-coords :copy-worldcoords) :translate #f(100 -125 25) :local) ;; hogehoge
	      nil
	      nil
	      (send (send (send *atlas* :worldcoords) :copy-worldcoords) :translate #f(0 300 0) :world))))
    (with-move-target-link-list
     (mt ll *atlas* '(:rleg :lleg :rarm :larm))
     (send *atlas* :fullbody-inverse-kinematics
  	   (mapcar #'(lambda (c m) (if c c (send m :copy-worldcoords))) tc mt)
  	   :link-list ll :move-target mt
  	   :collision-avoidance-link-pair nil
           :revert-if-fail nil 	   :stop 1000  	   
	   :translation-axis '(t t t nil)
  	   :rotation-axis '(t t t nil)
	   :thre '(5 5 10 10)
	   :rthre (list (deg2rad 1) (deg2rad 1) (deg2rad 2) (deg2rad 2))
	   :target-centroid-pos (send (send (send *atlas* :lleg :end-coords :copy-worldcoords) :translate #f(20 0 0)) :pos)
  	   :centroid-thre 500)))
  (when real (model2real :wait t))
  (send *irtviewer* :draw-objects)
  (save-last)

  (warning-message 2 "[vhc-get-in] take down right leg inner on vehicle in lift rleg [get-in phase2]~%")
  (when wait (my-wait))
  (let* ((tc (list 
	      (send (send *atlas* :rleg :end-coords :copy-worldcoords) :translate #f(0 0 -120) :world)
	      ;; (if real
	      ;; 	  (send (send *atlas* :rleg :end-coords :copy-worldcoords) :translate #f(0 0 -70) :world)
	      ;; 	(send (send *atlas* :rleg :end-coords :copy-worldcoords) :translate #f(0 0 -120) :world))
	      nil
	      nil
	      (send (send (send *atlas* :worldcoords) :copy-worldcoords) :translate #f(0 300 0) :world))))
    (with-move-target-link-list
     (mt ll *atlas* '(:rleg :lleg :rarm :larm))
     (send *atlas* :fullbody-inverse-kinematics
  	   (mapcar #'(lambda (c m) (if c c (send m :copy-worldcoords))) tc mt)
  	   :link-list ll :move-target mt
  	   :collision-avoidance-link-pair nil
           :revert-if-fail nil 	   :stop 1000  	   
	   :translation-axis '(t t t nil)
  	   :rotation-axis '(t t t nil)
	   :thre '(5 5 10 10)
	   :rthre (list (deg2rad 1) (deg2rad 1) (deg2rad 2) (deg2rad 2))
	   :target-centroid-pos (send (send (send *atlas* :lleg :end-coords :copy-worldcoords) :translate #f(20 0 0)) :pos)
  	   :centroid-thre 500)))
  (when real (model2real :wait t))
  (send *irtviewer* :draw-objects)
  (save-last)

  ;; (when real
  ;;   (warning-message 2 "[vhc-get-in] take down right leg until touch in vehicle in lift rleg [get-in phase2]~%")
  ;;   (when wait (my-wait))
  ;;   (reach-until-touch :rleg #f(0 0 -1) 200)
  ;;   (save-last))

  (when real
    (warning-message 2 "[vhc-get-in] adapt right leg in lift rleg [get-in phase2]~%")
    (when wait (my-wait))
    (adapt-leg :rleg)
    (save-last))

  (warning-message 2 "[vhc-get-in] finish (lift-rleg-in-get-in-for-task1) [get-in phase2]~%")
  )

(defun lift-whole-body-in-get-in-for-task1
  (&key (real nil) (wait nil))

  (warning-message 2 "[vhc-get-in] start (lift-whole-body-in-get-in-for-task1) [get-in phase-final]~%")

  (warning-message 2 "[vhc-get-in] set zero-gain :lleg and middle-gain :larm in lift whole body [get-in phase-final]~%")
  (when wait (my-wait))
  (set-middle-gain :lleg)
  (set-middle-gain :larm)

  (warning-message 2 "[vhc-get-in] angle-vector1 in lift whole body [get-in phase-final]~%")
  (when wait (my-wait))
  (send *atlas* :larm :inverse-kinematics (send (send (send *atlas* :larm :end-coords) :copy-worldcoords) :translate #f(600 -500 1000) :world) :revert-if-fail nil)
  (send *atlas* :lleg :inverse-kinematics (send (send (send *atlas* :lleg :end-coords) :copy-worldcoords) :translate #f(0 200 500) :world) :revert-if-fail nil)
  (send *atlas* :rarm :inverse-kinematics (send (send (send *atlas* :rarm :end-coords) :copy-worldcoords) :translate #f(0 150 -300) :world) :revert-if-fail nil)
  (send *atlas* :rleg :inverse-kinematics (send (send (send *atlas* :rleg :end-coords) :copy-worldcoords) :translate #f(0 150 -300) :world) :revert-if-fail nil)
  (send *irtviewer* :draw-objects)
  (when real (model2real :wait t))
  (save-last)

  (warning-message 2 "[vhc-get-in] angle-vector2 in lift whole body [get-in phase-final]~%")
  (when wait (my-wait))
  (send *atlas* :lleg :inverse-kinematics (send (send (send *atlas* :lleg :end-coords) :copy-worldcoords) :translate #f(0 -100 100) :world) :revert-if-fail nil)
  (send *atlas* :rarm :inverse-kinematics (send (send (send *atlas* :rarm :end-coords) :copy-worldcoords) :translate #f(300 1000 -300) :world) :revert-if-fail nil)
  (send *atlas* :rleg :inverse-kinematics (send (send (send *atlas* :rleg :end-coords) :copy-worldcoords) :translate #f(200 1000 -300) :world) :revert-if-fail nil)
  (send *irtviewer* :draw-objects)
  (when real (model2real :wait t))
  (save-last)

  (warning-message 2 "[vhc-get-in] angle-vector3 in lift whole body [get-in phase-final]~%")
  (when wait (my-wait))
  ;; (send *atlas* :lleg :inverse-kinematics (send (send (send *atlas* :lleg :end-coords) :copy-worldcoords) :translate #f(500 0 0) :world) :revert-if-fail nil)
  ;; (send *atlas* :rarm :inverse-kinematics (send (send (send *atlas* :rarm :end-coords) :copy-worldcoords) :translate #f(500 0 0) :world) :revert-if-fail nil)
  ;; (send *atlas* :rleg :inverse-kinematics (send (send (send *atlas* :rleg :end-coords) :copy-worldcoords) :translate #f(500 0 0) :world) :revert-if-fail nil)
  (send *atlas* :lleg :inverse-kinematics (send (send (send *atlas* :lleg :end-coords) :copy-worldcoords) :translate #f(500 0 50) :world) :revert-if-fail nil)
  (send *atlas* :rarm :inverse-kinematics (send (send (send *atlas* :rarm :end-coords) :copy-worldcoords) :translate #f(500 0 50) :world) :revert-if-fail nil)
  (send *atlas* :rleg :inverse-kinematics (send (send (send *atlas* :rleg :end-coords) :copy-worldcoords) :translate #f(500 0 50) :world) :revert-if-fail nil)
  (send *irtviewer* :draw-objects)
  (when real (model2real :wait t))
  (save-last)

  (warning-message 2 "[vhc-get-in] finish (lift-whole-body-in-get-in-for-task1) [get-in phase-final]~%")
  )




;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;; not used new !!!

(defun grasp-larm-in-get-in-for-task1
  (&key (real nil) (wait nil))

  (warning-message 2 "[vhc-get-in] start (grasp-larm-in-get-in-for-task1) [get-in phase3]~%")

  (let* ((pipe-coords
	  (send (send (send *drcv* :handle :name :front-left-pipe) :copy-worldcoords) :translate #f(0 0 0) :local)))

    ;; (when real
    ;;   (warning-message 2 "[vhc-get-in] adapt angle-vector :larm in grasp larm [get-in phase3]~%")
    ;;   (when wait (my-wait))
    ;;   (real2model :larm)
    ;;   (send *irtviewer* :draw-objects)
    ;;   (model2real :time 1000 :wait t))    

    (when real
      (warning-message 2 "[vhc-get-in] set high-gain :larm in grasp larm [get-in phase3]~%")
      (when wait (my-wait))
      (set-high-gain :larm))

    (warning-message 2 "[vhc-get-in] angle-vector1 in grasp larm [get-in phase3]~%")
    (when wait (my-wait))
    (let* ((tc (list nil
		     nil
		     nil
		     (send (send pipe-coords :copy-worldcoords) :translate #f(300 -600 -100) :local))))
      (with-move-target-link-list
       (mt ll *atlas* '(:rleg :lleg :rarm :larm))
       (send *atlas* :fullbody-inverse-kinematics
	     (mapcar #'(lambda (c m) (if c c (send m :copy-worldcoords))) tc mt)
	     :link-list ll :move-target mt
	     :collision-avoidance-link-pair nil
	     :translation-axis '(t t t :z)
	     :rotation-axis '(t t t :z)
	     :thre '(5 5 10 50)
	     :rthre (list (deg2rad 2) (deg2rad 2) (deg2rad 2) (deg2rad 30))
	     :target-centroid-pos (send (send (send (send *atlas* :foot-midcoords) :copy-worldcoords) :translate #f(25 0 0)) :pos)
	     :centroid-thre 500)))
    (send *irtviewer* :draw-objects)
    (when real (model2real :time 4000 :wait t))
    (save-last)

    (warning-message 2 "[vhc-get-in] angle-vector2 in grasp larm [get-in phase3]~%")
    (when wait (my-wait))
    (let* ((tc (list nil
		     nil
		     nil
		     (send (send pipe-coords :copy-worldcoords) :translate #f(50 -300 0) :local))))
      (with-move-target-link-list
       (mt ll *atlas* '(:rleg :lleg :rarm :larm))
       (send *atlas* :fullbody-inverse-kinematics
	     (mapcar #'(lambda (c m) (if c c (send m :copy-worldcoords))) tc mt)
	     :link-list ll :move-target mt
	     :collision-avoidance-link-pair nil
	     :translation-axis '(t t t :z)
	     :rotation-axis '(t t t nil)
	     :thre '(5 5 10 50)
	     :rthre (list (deg2rad 2) (deg2rad 2) (deg2rad 2) (deg2rad 30))
	     :target-centroid-pos (send (send (send (send *atlas* :foot-midcoords) :copy-worldcoords) :translate #f(25 0 0)) :pos)
	     :centroid-thre 500)))
    (send *irtviewer* :draw-objects)
    (when real (model2real :time 4000 :wait t))
    (save-last)

    (warning-message 2 "[vhc-get-in] pregrasp larm in grasp larm [get-in phase3]~%")
    (when wait (my-wait))
    (pregrasp :arm :larm :real real)
    (send *irtviewer* :draw-objects)
    (when real (unix::usleep (* 3 1000 1000)))

    (if nil
        ;; (if real
	(progn
	  (warning-message 2 "[vhc-get-in] reach until touch in grasp larm [get-in phase3]~%")
	  (when wait (my-wait))
	  (reach-until-touch
	   :larm
	   (v- (send (send (send pipe-coords :copy-worldcoords) :translate #f(0 0 -200) :local) :worldpos)
	       (send (send (send pipe-coords :copy-worldcoords) :translate #f(50 -300 0) :local) :worldpos))
	   50)
	  (save-last))
      (progn
	(warning-message 2 "[vhc-get-in] angle-vector3 in grasp larm [get-in phase3]~%")
	(when wait (my-wait))
	(let* ((tc (list nil
			 nil
			 nil
			 (send (send pipe-coords :copy-worldcoords) :translate #f(0 0 -200) :local))))
	  (with-move-target-link-list
	   (mt ll *atlas* '(:rleg :lleg :rarm :larm))
	   (send *atlas* :fullbody-inverse-kinematics
		 (mapcar #'(lambda (c m) (if c c (send m :copy-worldcoords))) tc mt)
		 :link-list ll :move-target mt
		 :collision-avoidance-link-pair nil
		 :translation-axis '(t t t t)
		 :rotation-axis '(t t t :z)
		 :thre '(5 5 10 10)
		 :rthre (list (deg2rad 2) (deg2rad 2) (deg2rad 2) (deg2rad 2))
		 :target-centroid-pos (send (send (send (send *atlas* :foot-midcoords) :copy-worldcoords) :translate #f(25 0 0)) :pos)
		 :centroid-thre 500)))
	(send *irtviewer* :draw-objects)
	(when real (model2real :wait t))
	(save-last)))

    (when real
      (warning-message 2 "[vhc-get-in] set middle-gain :larm in grasp larm [get-in phase3]~%")
      (when wait (my-wait))
      (set-middle-gain :larm))

    (warning-message 2 "[vhc-get-in] grasp larm and ungrasp rarm in grasp larm [get-in phase3]~%")
    (when wait (my-wait))
    (grasp :arm :larm :real real)
    (send *irtviewer* :draw-objects)
    (when real (unix::usleep (* 2 1000 1000)))
    (ungrasp :arm :rarm :real real)
    (send *irtviewer* :draw-objects)
    (when real (unix::usleep (* 2 1000 1000)))

    (when real
      (warning-message 2 "[vhc-get-in] adapt angle-vector in grasp larm [get-in phase3]~%")
      (when wait (my-wait))
      (real2model)
      (send *irtviewer* :draw-objects)
      (model2real :time 1000 :wait t))

    (when real
      (warning-message 2 "[vhc-get-in] set high-gain :larm in grasp larm [get-in phase3]~%")
      (when wait (my-wait))
      (set-high-gain :larm))

    (warning-message 2 "[vhc-get-in] angle-vector for release pipe with right arm in grasp larm [get-in phase3]~%")
    (when wait (my-wait))
    (let* ((tc (list nil
		     nil
		     (send (send (send *drcv* :handle :name :upper-left-pipe) :copy-worldcoords) :transform (make-coords :pos #f(80 200 0) :rpy '(0 0 0)) :local)
		     nil)))
      (with-move-target-link-list
       (mt ll *atlas* '(:rleg :lleg :rarm :larm))
       (send *atlas* :fullbody-inverse-kinematics
	     (mapcar #'(lambda (c m) (if c c (send m :copy-worldcoords))) tc mt)
	     :link-list ll :move-target mt
	     :collision-avoidance-link-pair nil
	     :translation-axis '(t t t t)
	     :rotation-axis '(t t nil t)
	     :thre '(5 5 10 10)
	     :rthre (list (deg2rad 1) (deg2rad 1) (deg2rad 2) (deg2rad 2))
	     :target-centroid-pos (send (send (send (send *atlas* :foot-midcoords) :copy-worldcoords) :translate #f(25 50 0)) :pos)
	     :centroid-thre 500)))
    (send *irtviewer* :draw-objects)
    (when real (model2real :wait t))
    (save-last)
    )

  (warning-message 2 "[vhc-get-in] finish (grasp-larm-in-get-in-for-task1) [get-in phase3]~%")
  )

(defun grasp-rarm-2-in-get-in-for-task1
  (&key (real nil) (wait nil))

  (warning-message 2 "[vhc-get-in] start (grasp-rarm-2-in-get-in-for-task1) [get-in phase4]~%")

  (let* ((pipe-coords
	  (send (send (send *drcv* :handle :name :upper-front-pipe) :copy-worldcoords) :translate #f(0 0 0) :local)))

    (warning-message 2 "[vhc-get-in] angle-vector1 in grasp rarm 2 [get-in phase4]~%")
    (when wait (my-wait))
    (let* ((tc (list nil
		     nil
		     (send (send pipe-coords :copy-worldcoords) :translate #f(-200 200 0) :local)
		     nil)))
      (with-move-target-link-list
       (mt ll *atlas* '(:rleg :lleg :rarm :larm))
       (send *atlas* :fullbody-inverse-kinematics
	     (mapcar #'(lambda (c m) (if c c (send m :copy-worldcoords))) tc mt)
	     :link-list ll :move-target mt
	     :collision-avoidance-link-pair nil
	     :translation-axis '(t t :z t)
	     :rotation-axis '(t t :z t)
	     :thre '(5 5 50 10)
	     :rthre (list (deg2rad 2) (deg2rad 2) (deg2rad 2) (deg2rad 2))
	     :target-centroid-pos (send (send (send (send *atlas* :foot-midcoords) :copy-worldcoords) :translate #f(50 25 0)) :pos)
	     :centroid-thre 500)))
    (send *irtviewer* :draw-objects)
    (when real (model2real :time 4000 :wait t))
    (save-last)

    (warning-message 2 "[vhc-get-in] pregrasp rarm in grasp rarm 2 [get-in phase4]~%")
    (when wait (my-wait))
    (pregrasp :arm :rarm :real real)
    (send *irtviewer* :draw-objects)
    (when real (unix::usleep (* 3 1000 1000)))

    (if nil
        ;; (if real
	(progn
	  (warning-message 2 "[vhc-get-in] reach until touch in grasp rarm 2 [get-in phase4]~%")
	  (when wait (my-wait))
	  (reach-until-touch
	   :rarm
	   (v- (send pipe-coords :worldpos)
	       (send (send (send pipe-coords :copy-worldcoords) :translate #f(-150 100 0) :local) :worldpos))
	   50)
	  (save-last))
      (progn
	(warning-message 2 "[vhc-get-in] angle-vector2 in grasp rarm 2 [get-in phase4]~%")
	(when wait (my-wait))
	(let* ((tc (list nil
			 nil
			 pipe-coords
			 nil)))
	  (with-move-target-link-list
	   (mt ll *atlas* '(:rleg :lleg :rarm :larm))
	   (send *atlas* :fullbody-inverse-kinematics
		 (mapcar #'(lambda (c m) (if c c (send m :copy-worldcoords))) tc mt)
		 :link-list ll :move-target mt
		 :collision-avoidance-link-pair nil
		 :translation-axis '(t t t t)
		 :rotation-axis '(t t :z t)
		 :thre '(5 5 10 10)
		 :rthre (list (deg2rad 2) (deg2rad 2) (deg2rad 2) (deg2rad 2))
		 :target-centroid-pos (send (send (send (send *atlas* :foot-midcoords) :copy-worldcoords) :translate #f(50 25 0)) :pos)
		 :centroid-thre 500)))
	(send *irtviewer* :draw-objects)
	(when real (model2real :wait t))
	(save-last)))

    (when real
      (warning-message 2 "[vhc-get-in] set middle-gain :rarm in grasp rarm 2 [get-in phase4]~%")
      (when wait (my-wait))
      (set-middle-gain :rarm))

    (warning-message 2 "[vhc-get-in] grasp rarm in grasp rarm 2 [get-in phase4]~%")
    (when wait (my-wait))
    (grasp :arm :rarm :real real)
    (send *irtviewer* :draw-objects)
    (when real (unix::usleep (* 3 1000 1000)))

    (when real
      (warning-message 2 "[vhc-get-in] adapt angle-vector in grasp rarm 2 [get-in phase4]~%")
      (when wait (my-wait))
      (real2model)
      (send *irtviewer* :draw-objects)
      (model2real :time 1000 :wait t))

    (when real
      (warning-message 2 "[vhc-get-in] set high-gain :rarm in grasp rarm 2 [get-in phase4]~%")
      (when wait (my-wait))
      (set-high-gain :rarm))


    (warning-message 2 "[vhc-get-in] ungrasp larm in grasp rarm 2 [get-in phase4]~%")
    (when wait (my-wait))
    (ungrasp :arm :larm :real real)
    (send *irtviewer* :draw-objects)
    (when real (unix::usleep (* 3 1000 1000)))
    )

  (warning-message 2 "[vhc-get-in] finish (grasp-rarm-2-in-get-in-for-task1) [get-in phase4]~%")
  )

(defun lift-lleg-in-get-in-for-task1
  (&key (real nil) (wait nil))

  (warning-message 2 "[vhc-get-in] start (lift-lleg-in-get-in-for-task1) [get-in phase5]~%")

  (when real
    (warning-message 2 "[vhc-get-in] set zero-gain :lleg and :larm in lift lleg [get-in phase5]~%")
    (when wait (my-wait))
    (set-zero-gain :lleg)
    (set-zero-gain :larm))

  (warning-message 2 "[vhc-get-in] lift left leg on ground in lift lleg [get-in phase5]~%")
  (when wait (my-wait))
  (let* ((tc (list nil
		   nil
		   nil
		   nil))
	 (target-limbs '(:rleg :lleg :rarm :larm))
	 (links-dof (send *atlas* :calc-target-joint-dimension
			  (mapcar #'(lambda (x) (send *atlas* :link-list (send *atlas* x :end-coords :parent))) target-limbs)))
	 (nv (instantiate float-vector (+ 6 links-dof)))
	 (root-dest-pos #f(100 400 1200)))
    (with-move-target-link-list
     (mt ll *atlas* target-limbs)
     (send *atlas* :fullbody-inverse-kinematics
	   (mapcar #'(lambda (c m) (if c c (send m :copy-worldcoords))) tc mt)
	   :link-list ll :move-target mt
	   :collision-avoidance-link-pair nil
           :revert-if-fail nil 	   :stop 100
	   :translation-axis '(t nil t nil)
	   :rotation-axis '(t nil t nil)
	   :thre '(100 100 100 100)
	   :rthre (list (deg2rad 20) (deg2rad 20) (deg2rad 20) (deg2rad 20))
	   :null-space
	   #'(lambda ()
	       ;; move down base-link by feedback base height
	       (setf (elt nv 0) (* 0.0001 (- (elt root-dest-pos 0) (elt (send *atlas* :worldpos) 0))))
	       (setf (elt nv 1) (* 0.0001 (- (elt root-dest-pos 1) (elt (send *atlas* :worldpos) 1))))
	       (setf (elt nv 2) (* 0.0001 (- (elt root-dest-pos 2) (elt (send *atlas* :worldpos) 2))))
	       nv)
	   ;; :target-centroid-pos (send (send (send *atlas* :rleg :end-coords :copy-worldcoords) :translate #f(0 50 0)) :pos)
	   :centroid-thre 500)))
  (let* ((tc (list nil
		   nil
		   nil
		   nil))
	 (target-limbs '(:rleg :lleg :rarm :larm))
	 (links-dof (send *atlas* :calc-target-joint-dimension
			  (mapcar #'(lambda (x) (send *atlas* :link-list (send *atlas* x :end-coords :parent))) target-limbs)))
	 (nv (instantiate float-vector (+ 6 links-dof)))
	 (root-dest-pos #f(100 400 1400)))
    (with-move-target-link-list
     (mt ll *atlas* target-limbs)
     (send *atlas* :fullbody-inverse-kinematics
	   (mapcar #'(lambda (c m) (if c c (send m :copy-worldcoords))) tc mt)
	   :link-list ll :move-target mt
	   :collision-avoidance-link-pair nil
           :revert-if-fail nil 	   :stop 100
	   :translation-axis '(t nil t nil)
	   :rotation-axis '(t nil t nil)
	   :thre '(100 100 100 100)
	   :rthre (list (deg2rad 20) (deg2rad 20) (deg2rad 20) (deg2rad 20))
	   :null-space
	   #'(lambda ()
	       ;; move down base-link by feedback base height
	       (setf (elt nv 0) (* 0.0001 (- (elt root-dest-pos 0) (elt (send *atlas* :worldpos) 0))))
	       (setf (elt nv 1) (* 0.0001 (- (elt root-dest-pos 1) (elt (send *atlas* :worldpos) 1))))
	       (setf (elt nv 2) (* 0.0001 (- (elt root-dest-pos 2) (elt (send *atlas* :worldpos) 2))))
	       nv)
	   ;; :target-centroid-pos (send (send (send *atlas* :rleg :end-coords :copy-worldcoords) :translate #f(0 50 0)) :pos)
	   :centroid-thre 500)))
  (let* ((tc (list nil
		   nil
		   nil
		   nil))
	 (target-limbs '(:rleg :lleg :rarm :larm))
	 (links-dof (send *atlas* :calc-target-joint-dimension
			  (mapcar #'(lambda (x) (send *atlas* :link-list (send *atlas* x :end-coords :parent))) target-limbs)))
	 (nv (instantiate float-vector (+ 6 links-dof)))
	 (root-dest-pos #f(100 400 1600)))
    (with-move-target-link-list
     (mt ll *atlas* target-limbs)
     (send *atlas* :fullbody-inverse-kinematics
	   (mapcar #'(lambda (c m) (if c c (send m :copy-worldcoords))) tc mt)
	   :link-list ll :move-target mt
	   :collision-avoidance-link-pair nil
           :revert-if-fail nil 	   :stop 100
	   :translation-axis '(t nil t nil)
	   :rotation-axis '(t nil t nil)
	   :thre '(100 100 100 100)
	   :rthre (list (deg2rad 20) (deg2rad 20) (deg2rad 20) (deg2rad 20))
	   :null-space
	   #'(lambda ()
	       ;; move down base-link by feedback base height
	       (setf (elt nv 0) (* 0.0001 (- (elt root-dest-pos 0) (elt (send *atlas* :worldpos) 0))))
	       (setf (elt nv 1) (* 0.0001 (- (elt root-dest-pos 1) (elt (send *atlas* :worldpos) 1))))
	       (setf (elt nv 2) (* 0.0001 (- (elt root-dest-pos 2) (elt (send *atlas* :worldpos) 2))))
	       nv)
	   ;; :target-centroid-pos (send (send (send *atlas* :rleg :end-coords :copy-worldcoords) :translate #f(0 50 0)) :pos)
	   :centroid-thre 500)))
  (send *irtviewer* :draw-objects)
  (when real (model2real :wait t))
  (save-last)

  (warning-message 2 "[vhc-get-in] move left leg in vehicle in lift lleg [get-in phase5]~%")
  (when wait (my-wait))
  (let* ((tc (list nil
		   nil
		   nil
		   nil))
	 (target-limbs '(:rleg :lleg :rarm :larm))
	 (links-dof (send *atlas* :calc-target-joint-dimension
			  (mapcar #'(lambda (x) (send *atlas* :link-list (send *atlas* x :end-coords :parent))) target-limbs)))
	 (nv (instantiate float-vector (+ 6 links-dof)))
	 (root-dest-pos #f(100 200 1600)))
    (with-move-target-link-list
     (mt ll *atlas* target-limbs)
     (send *atlas* :fullbody-inverse-kinematics
	   (mapcar #'(lambda (c m) (if c c (send m :copy-worldcoords))) tc mt)
	   :link-list ll :move-target mt
	   :collision-avoidance-link-pair nil
           :revert-if-fail nil 	   :stop 100
	   :translation-axis '(t nil t nil)
	   :rotation-axis '(t nil t nil)
	   :thre '(100 100 100 100)
	   :rthre (list (deg2rad 20) (deg2rad 20) (deg2rad 20) (deg2rad 20))
	   :null-space
	   #'(lambda ()
	       ;; move down base-link by feedback base height
	       (setf (elt nv 0) (* 0.0001 (- (elt root-dest-pos 0) (elt (send *atlas* :worldpos) 0))))
	       (setf (elt nv 1) (* 0.0001 (- (elt root-dest-pos 1) (elt (send *atlas* :worldpos) 1))))
	       (setf (elt nv 2) (* 0.0001 (- (elt root-dest-pos 2) (elt (send *atlas* :worldpos) 2))))
	       nv)
	   ;; :target-centroid-pos (send (send (send *atlas* :rleg :end-coords :copy-worldcoords) :translate #f(0 50 0)) :pos)
	   :centroid-thre 500)))
  (let* ((tc (list nil
		   nil
		   nil
		   nil))
	 (target-limbs '(:rleg :lleg :rarm :larm))
	 (links-dof (send *atlas* :calc-target-joint-dimension
			  (mapcar #'(lambda (x) (send *atlas* :link-list (send *atlas* x :end-coords :parent))) target-limbs)))
	 (nv (instantiate float-vector (+ 6 links-dof)))
	 (root-dest-pos #f(100 100 1600)))
    (with-move-target-link-list
     (mt ll *atlas* target-limbs)
     (send *atlas* :fullbody-inverse-kinematics
	   (mapcar #'(lambda (c m) (if c c (send m :copy-worldcoords))) tc mt)
	   :link-list ll :move-target mt
	   :collision-avoidance-link-pair nil
           :revert-if-fail nil 	   :stop 100
	   :translation-axis '(t nil t nil)
	   :rotation-axis '(t nil t nil)
	   :thre '(100 100 100 100)
	   :rthre (list (deg2rad 20) (deg2rad 20) (deg2rad 20) (deg2rad 20))
	   :null-space
	   #'(lambda ()
	       ;; move down base-link by feedback base height
	       (setf (elt nv 0) (* 0.0001 (- (elt root-dest-pos 0) (elt (send *atlas* :worldpos) 0))))
	       (setf (elt nv 1) (* 0.0001 (- (elt root-dest-pos 1) (elt (send *atlas* :worldpos) 1))))
	       (setf (elt nv 2) (* 0.0001 (- (elt root-dest-pos 2) (elt (send *atlas* :worldpos) 2))))
	       nv)
	   ;; :target-centroid-pos (send (send (send *atlas* :rleg :end-coords :copy-worldcoords) :translate #f(0 50 0)) :pos)
	   :centroid-thre 500)))
  (send *irtviewer* :draw-objects)
  (when real (model2real :wait t))
  (save-last)

  (warning-message 2 "[vhc-get-in] finish (lift-lleg-in-get-in-for-task1) [get-in phase5]~%")  )




;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;; not used old !!!

(defun get-in-vehicle-side
  (&key (real nil) (wait nil) (only-teleport nil) (teleport t))

  (get-in-vehicle-side-teleport :real real :wait wait :teleport teleport)

  (when only-teleport (return-from get-in-vehicle-side nil))

  (get-in-vehicle-side-grasp-upper-left-pipe-with-right-hand :real real :wait wait)

  (get-in-vehicle-side-get-in-right-leg :real real :wait wait)

  (get-in-vehicle-side-get-in-right-leg-inner :real real :wait wait)

  (get-in-vehicle-side-grasp-front-left-pipe-with-left-hand :real real :wait wait)

  (get-in-vehicle-side-get-in-left-leg :real real :wait wait)
  )

(defun get-in-vehicle-side-grasp-upper-left-pipe-with-left-hand
  (&key (real nil) (wait nil))

  (let* ((pipe-coords
	  (send (send (send *drcv* :handle :name :upper-left-pipe) :copy-worldcoords) :translate #f(10 -60 200) :local)))

    ;; (when real
    ;;   (warning-message 2 "[vhc-get-in] adapt angle-vector~%")
    ;;   (when wait (my-wait))
    ;;   (real2model)
    ;;   (model2real)
    ;;   (send *irtviewer* :draw-objects)
    ;;   (model2real :wait t))

    (when real
      (warning-message 2 "[vhc-get-in] set larm kp 100 ~%")
      (when wait (my-wait))
      (atlas-set-servo-gain-by-torque-limit 1.0 :larm))

    (warning-message 2 "[vhc-get-in] angle-vector1 for reaching pipe with left arm~%")
    (when wait (my-wait))
    (let* ((tc (list nil
		     nil
		     nil
		     (send (send pipe-coords :copy-worldcoords) :translate #f(100 -300 100) :local))))
      (with-move-target-link-list
       (mt ll *atlas* '(:rleg :lleg :rarm :larm))
       (send *atlas* :fullbody-inverse-kinematics
	     (mapcar #'(lambda (c m) (if c c (send m :copy-worldcoords))) tc mt)
	     :link-list ll :move-target mt
	     :collision-avoidance-link-pair nil
	     :translation-axis '(t t t t)
	     :rotation-axis '(t t t :z)
	     :thre '(5 5 10 10)
	     :centroid-thre 500)))
    (send *irtviewer* :draw-objects)
    (when real (model2real :wait t))

    (warning-message 2 "[vhc-get-in] angle-vector2 for reaching pipe with left arm~%")
    (when wait (my-wait))
    (let* ((tc (list nil
		     nil
		     nil
		     pipe-coords)))
      (with-move-target-link-list
       (mt ll *atlas* '(:rleg :lleg :rarm :larm))
       (send *atlas* :fullbody-inverse-kinematics
	     (mapcar #'(lambda (c m) (if c c (send m :copy-worldcoords))) tc mt)
	     :link-list ll :move-target mt
	     :collision-avoidance-link-pair nil
	     :translation-axis '(t t t t)
	     :rotation-axis '(t t t :z)
	     :thre '(5 5 10 10)
	     :rthre (list (deg2rad 1) (deg2rad 1) (deg2rad 2) (deg2rad 2))
	     :centroid-thre 500)))
    (send *irtviewer* :draw-objects)
    (when real (model2real :wait t))

    (warning-message 2 "[vhc-get-in] grasp larm and ungrasp rarm~%")
    (when wait (my-wait))
    (grasp :arm :larm :real real)
    (send *irtviewer* :draw-objects)
    (when real (unix::usleep (* 5 1000 1000)))
    (ungrasp :arm :rarm :real real)
    (send *irtviewer* :draw-objects)
    (when real (unix::usleep (* 5 1000 1000)))

    (warning-message 2 "[vhc-get-in] angle-vector1 for release pipe with right arm~%")
    (when wait (my-wait))
    (let* ((tc (list nil
		     nil
		     (send (send pipe-coords :copy-worldcoords) :transform (make-coords :pos #f(100 200 0) :rpy '(0 0 0)))
		     nil)))
      (with-move-target-link-list
       (mt ll *atlas* '(:rleg :lleg :rarm :larm))
       (send *atlas* :fullbody-inverse-kinematics
	     (mapcar #'(lambda (c m) (if c c (send m :copy-worldcoords))) tc mt)
	     :link-list ll :move-target mt
	     :collision-avoidance-link-pair nil
	     :translation-axis '(t t t t)
	     :rotation-axis '(t t :z t)
	     :thre '(5 5 10 10)
	     :centroid-thre 500)))
    (send *irtviewer* :draw-objects)
    (when real (model2real :wait t))
    ))

(defun get-in-vehicle-side-get-in-right-leg-inner2
  (&key (real nil) (wait nil))

  (warning-message 2 "[vhc-get-in] lift right leg on vehicle~%")
  (when wait (my-wait))
  (let* ((tc (list (send (send *atlas* :rleg :end-coords :copy-worldcoords) :translate #f(0 0 100) :world)
		   nil
		   (send (send (send *atlas* :worldcoords) :copy-worldcoords) :translate #f(0 -200 0) :world)
		   nil)))
    (with-move-target-link-list
     (mt ll *atlas* '(:rleg :lleg :rarm :larm))
     (send *atlas* :fullbody-inverse-kinematics
  	   (mapcar #'(lambda (c m) (if c c (send m :copy-worldcoords))) tc mt)
  	   :link-list ll :move-target mt
  	   :collision-avoidance-link-pair nil
           :revert-if-fail nil 	   :stop 1000  	   
	   :translation-axis '(t t nil t)
  	   :rotation-axis '(t t nil t)
	   :thre '(5 5 10 10)
	   ;; :target-centroid-pos (send (send (send *atlas* :lleg :end-coords :copy-worldcoords) :translate #f(0 0 0)) :pos)
  	   :centroid-thre 500)))
  (when real (model2real :wait t))
  (send *irtviewer* :draw-objects)

  (warning-message 2 "[vhc-get-in] move right leg in vehicle~%")
  (when wait (my-wait))
  (let* ((tc (list (send (send *atlas* :rleg :end-coords :copy-worldcoords) :translate #f(75 -100 0) :local)
		   nil
		   (send (send (send *atlas* :worldcoords) :copy-worldcoords) :translate #f(0 -200 0) :world)
		   nil)))
    (with-move-target-link-list
     (mt ll *atlas* '(:rleg :lleg :rarm :larm))
     (send *atlas* :fullbody-inverse-kinematics
  	   (mapcar #'(lambda (c m) (if c c (send m :copy-worldcoords))) tc mt)
  	   :link-list ll :move-target mt
  	   :collision-avoidance-link-pair nil
           :revert-if-fail nil 	   :stop 1000  	   
	   :translation-axis '(t t nil t)
  	   :rotation-axis '(t t nil t)
	   :thre '(5 5 10 10)
	   ;; :target-centroid-pos (send (send (send *atlas* :lleg :end-coords :copy-worldcoords) :translate #f(0 0 0)) :pos)
  	   :centroid-thre 500)))
  (when real (model2real :wait t))
  (send *irtviewer* :draw-objects)

  (warning-message 2 "[vhc-get-in] take down right leg on vehicle~%")
  (when wait (my-wait))
  (let* ((tc (list (send (send *atlas* :rleg :end-coords :copy-worldcoords) :translate #f(0 0 -100) :world)
		   nil
		   (send (send (send *atlas* :worldcoords) :copy-worldcoords) :translate #f(0 -200 0) :world)
		   nil)))
    (with-move-target-link-list
     (mt ll *atlas* '(:rleg :lleg :rarm :larm))
     (send *atlas* :fullbody-inverse-kinematics
  	   (mapcar #'(lambda (c m) (if c c (send m :copy-worldcoords))) tc mt)
  	   :link-list ll :move-target mt
  	   :collision-avoidance-link-pair nil
           :revert-if-fail nil 	   :stop 1000  	   
	   :translation-axis '(t t nil t)
  	   :rotation-axis '(t t nil t)
	   :thre '(5 5 10 10)
	   ;; :target-centroid-pos (send (send (send *atlas* :lleg :end-coords :copy-worldcoords) :translate #f(0 0 0)) :pos)
  	   :centroid-thre 500)))
  (when real (model2real :wait t))
  (send *irtviewer* :draw-objects)
  )

(defun lift-lleg-in-get-in-for-task1-old
  (&key (real nil) (wait nil))

  (warning-message 2 "[vhc-get-in] start (lift-lleg-in-get-in-for-task1) [get-in phase5]~%")

  (warning-message 2 "[vhc-get-in] lift left leg on ground in lift lleg [get-in phase5]~%")
  (when wait (my-wait))
  (let* ((tc (list nil
		   (send (send *atlas* :lleg :end-coords :copy-worldcoords) :translate #f(0 50 200) :world)
		   nil
		   nil)))
    (with-move-target-link-list
     (mt ll *atlas* '(:rleg :lleg :rarm :larm))
     (send *atlas* :fullbody-inverse-kinematics
	   (mapcar #'(lambda (c m) (if c c (send m :copy-worldcoords))) tc mt)
	   :link-list ll :move-target mt
	   :collision-avoidance-link-pair nil
           :revert-if-fail nil 	   :stop 1000	   
	   :translation-axis '(t t t t)
	   :rotation-axis '(t :z t t)
	   :thre '(5 5 10 10)
	   :rthre (list (deg2rad 2) (deg2rad 2) (deg2rad 2) (deg2rad 2))
	   :target-centroid-pos (send (send (send *atlas* :rleg :end-coords :copy-worldcoords) :translate #f(0 50 0)) :pos)
	   :centroid-thre 500)))
  (send *irtviewer* :draw-objects)
  (when real (model2real :wait t :time 1000))
  (save-last)

  (warning-message 2 "[vhc-get-in] lift left leg in air in lift lleg [get-in phase5]~%")
  (when wait (my-wait))
  (let* ((tc (list nil
		   (send (send *atlas* :lleg :end-coords :copy-worldcoords) :translate #f(-25 25 300) :world)
		   nil
		   nil)))
    (with-move-target-link-list
     (mt ll *atlas* '(:rleg :lleg :rarm :larm))
     (send *atlas* :fullbody-inverse-kinematics
	   (mapcar #'(lambda (c m) (if c c (send m :copy-worldcoords))) tc mt)
	   :link-list ll :move-target mt
	   :collision-avoidance-link-pair nil
           :revert-if-fail nil 	   :stop 1000	   
	   :translation-axis '(t t t t)
	   :rotation-axis '(t :z t t)
	   :thre '(5 5 10 10)
	   :rthre (list (deg2rad 2) (deg2rad 2) (deg2rad 2) (deg2rad 2))
	   :target-centroid-pos (send (send (send *atlas* :rleg :end-coords :copy-worldcoords) :translate #f(0 50 0)) :pos)
	   :centroid-thre 500)))
  (send *irtviewer* :draw-objects)
  (when real (model2real :wait t :time 1000))
  (save-last)

  (warning-message 2 "[vhc-get-in] move left leg in air in lift lleg [get-in phase5]~%")
  (when wait (my-wait))
  (let* ((tc (list nil
		   (send (send *atlas* :lleg :end-coords :copy-worldcoords) :translate #f(0 -300 25) :world)
		   nil
		   nil)))
    (with-move-target-link-list
     (mt ll *atlas* '(:rleg :lleg :rarm :larm))
     (send *atlas* :fullbody-inverse-kinematics
	   (mapcar #'(lambda (c m) (if c c (send m :copy-worldcoords))) tc mt)
	   :link-list ll :move-target mt
	   :collision-avoidance-link-pair nil
           :revert-if-fail nil 	   :stop 1000	   
	   :translation-axis '(t t t t)
	   :rotation-axis '(t :z t t)
	   :thre '(5 5 10 10)
	   :rthre (list (deg2rad 2) (deg2rad 2) (deg2rad 2) (deg2rad 2))
	   :target-centroid-pos (send (send (send *atlas* :rleg :end-coords :copy-worldcoords) :translate #f(0 50 0)) :pos)
	   :centroid-thre 500)))
  (send *irtviewer* :draw-objects)
  (when real (model2real :wait t :time 1000))
  (save-last)

  (warning-message 2 "[vhc-get-in] take down left leg on vehicle in lift lleg [get-in phase5]~%")
  (when wait (my-wait))
  (let* ((tc (list nil
		   (send (send *atlas* :lleg :end-coords :copy-worldcoords) :translate #f(0 0 -125) :world)
		   nil
		   nil)))
    (with-move-target-link-list
     (mt ll *atlas* '(:rleg :lleg :rarm :larm))
     (send *atlas* :fullbody-inverse-kinematics
	   (mapcar #'(lambda (c m) (if c c (send m :copy-worldcoords))) tc mt)
	   :link-list ll :move-target mt
	   :collision-avoidance-link-pair nil
           :revert-if-fail nil 	   :stop 1000	   
	   :translation-axis '(t t t t)
	   :rotation-axis '(t :z t t)
	   :thre '(5 5 10 10)
	   :rthre (list (deg2rad 2) (deg2rad 2) (deg2rad 2) (deg2rad 2))
	   :target-centroid-pos (send (send (send *atlas* :rleg :end-coords :copy-worldcoords) :translate #f(0 50 0)) :pos)
	   :centroid-thre 500)))
  (send *irtviewer* :draw-objects)
  (when real (model2real :wait t :time 1000))
  (save-last)

  (warning-message 2 "[vhc-get-in] finish (lift-lleg-in-get-in-for-task1) [get-in phase5]~%")
  )

