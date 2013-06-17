
;; \file    atlas-vehicle-motion-others.l
;; \author  Murooka Masaki
;; \date    2013/06/15
;; \brief   vrc_task_1関連 その他の行動要素関数
;;


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;; vehicle motion [init]

(defun init-for-task1
  (&key (real nil) (wait nil) (auto nil))

  (warning-message 2 "[vhc-init] start (init-for-task1)~%")

  (my-init :real real)

  (when auto
    (send *ri* :teleport-to (make-coords :pos (v+ (send *vehicle-start-pos* :worldpos) #f(0 -3500 950)) :rpy (list -pi/2 0 0))))

  (when real
    (real2model)
    (model2real :time 100)
    (set-user))

  (warning-message 2 "[vhc-init] finish (init-for-task1)~%")
  )


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;; vehicle motion [walk]

(defun walk-for-task1
  (&key (real nil) (wait nil) (auto nil))

  (warning-message 2 "[vhc-walk] start (walk-for-task1)~%")

  (prepare-walking-in-walk-for-task1 :real real :wait wait)

  (cond (auto (walk-auto-in-walk-for-task1 :real real :wait wait))
	(t (walk-in-walk-for-task1 :real real :wait wait)))

  (finish-in-walk-for-task1 :real real :wait wait)

  (warning-message 2 "[vhc-walk] finish (walk-for-task1)~%")
)

(defun prepare-walking-in-walk-for-task1
  (&key (real nil) (wait nil))

  (warning-message 2 "[vhc-walk] start (prepare-walking-in-walk-for-task1)~%")

  ;; 歩行のために膝を曲げて，車と干渉しないように右手を上げる
  (send *atlas* :angle-vector #f(0.0 0.0 0.0 23.5 114.5 0.0 -75.0 17.0 0.0 -135.0 129.6 -71.6129 79.9998 60.7501 0.0 61.5619 -1.79235 -35.568 1.79235 -25.994 0.012938 61.5446 1.91967 -35.5619 -1.91967 -25.9828 0.01294 0.0))
  (fix-leg)
  (send *atlas* :move-centroid-on-foot :both '(:rleg :lleg)
	:target-centroid-pos (v+ (float-vector 50 0 0) (apply #'midpoint 0.5 (send-all (send *atlas* :legs :end-coords :parent) :worldpos))))
  (grasp :arm :rarm :real real)
  (ungrasp :arm :larm :real real)
  (when real (model2real :wait t))

  (warning-message 2 "[vhc-walk] finish (prepare-walking-in-walk-for-task1)~%")
  )

(defun walk-in-walk-for-task1
  (&key (real nil) (wait nil))

  (warning-message 2 "[vhc-walk] start (walk-in-walk-for-task1)~%")

  (when real
    (send *ri* :start-st)
    (warning-message 2 "[vhc-walk] start (cmd-vel-mode)~%")
    (warning-message 1 " rosrun pr2_teleop teleop_pr2_keyboard~%")
    (send *ri* :cmd-vel-mode)
    (warning-message 2 "[vhc-walk] finish (cmd-vel-mode)~%")
    )

  (warning-message 2 "[vhc-walk] finish (walk-in-walk-for-task1)~%")
  )

(defun walk-auto-in-walk-for-task1
  (&key (real nil) (wait nil))

  (warning-message 2 "[vhc-walk] start (walk-auto-in-walk-for-task1)~%")

  (when real
    (send *ri* :start-st)
    (warning-message 2 "[vhc-walk] start :go-pos~%")
    (send *ri* :go-pos 1.0 0 0)
    ;; (send *ri* :go-pos 5.5 0 0)
    (warning-message 2 "[vhc-walk] finish :go-pos~%")
    )

  (warning-message 2 "[vhc-walk] finish (walk-auto-in-walk-for-task1)~%")
  )

(defun finish-in-walk-for-task1
  (&key (real nil) (wait nil))

  (warning-message 2 "[vhc-walk] start (finish-in-walk-for-task1)~%")

  (fix-leg)
  (send *atlas* :legs :move-end-pos #f(0 0 -70) :world)
  (when real
    (model2real :wait t)
    (send *ri* :stop-st))

  (warning-message 2 "[vhc-walk] finish (finish-in-walk-for-task1)~%")
  )


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;; other functions

(defun telep-side
  (&key (real nil) (wait nil) (init nil) (telep t))

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

  (warning-message 2 "[vhc-get-in] call :teleport~%")
  (when wait (my-wait))
  (when (and real telep) 
    (let* ((telep-coords (send *vehicle-side-pos* :copy-worldcoords))
	   (telep-pos (send telep-coords :worldpos))
	   (telep-rpy (car (rpy-angle (send telep-coords :rot)))))
      (setq telep-coords (make-coords :pos (float-vector (elt telep-pos 0) (- (elt telep-pos 1)) (elt telep-pos 2))
				      :rpy (list (- (elt telep-rpy 0)) (elt telep-rpy 1) (- (elt telep-rpy 2)))))
      (send telep-coords :translate #f(-70 10 0) :local)
      (send telep-coords :rotate -pi/2 :z)
      (send *ri* :teleport-to (send (send telep-coords :copy-worldcoords) :translate #f(0 0 950) :world) nil :keep-effort t)
      (send *atlas* :fix-leg-to-coords telep-coords '(:lleg :rleg))))
  (send *irtviewer* :draw-objects)

  (make-vehicle)
  )

