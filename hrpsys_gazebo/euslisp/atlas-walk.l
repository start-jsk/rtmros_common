(defun start-walk
  (&key (real t) (vel-mode nil) (before-get-in-vehicle nil))

  (let (prev-av lleg-av rleg-av)

    (when real
      (real2model)
      (model2real :wait t :time 10)
      (atlas-set-servo-gain-by-torque-limit 1.0 :legs)
      (atlas-set-servo-gain-by-torque-limit 2.0 :torso)
      (atlas-set-servo-gain-by-torque-limit 4.0 :arms)
      (atlas-set-servo-gain-by-torque-limit 5.0 :head)
      (set-user))

    (setq prev-av (send *atlas* :angle-vector))

    (when before-get-in-vehicle
      (setq *lr-reverse* t)
      (send *atlas* :angle-vector
	    #f(0.0 0.0 0.0 23.5 114.5 0.0 -75.0 17.0 0.0 -135.0 129.6 -71.6129 79.9998 60.7501 0.0 61.5619 -1.79235 -35.568 1.79235 -25.994 0.012938 61.5446 1.91967 -35.5619 -1.91967 -25.9828 0.01294 0.0))

      (setq prev-av (send *atlas* :angle-vector))
      (grasp :arm :rarm :real real)
      (ungrasp :arm :larm :real real)
      )

    ;; 足だけwalk-poseにする
    (send *atlas* :walk-pose)

    (setq lleg-av (send *atlas* :lleg :angle-vector))
    (setq rleg-av (send *atlas* :rleg :angle-vector))
    (send *atlas* :angle-vector prev-av)
    (send *atlas* :lleg :angle-vector lleg-av)
    (send *atlas* :rleg :angle-vector rleg-av)
    (send *atlas* :head :neck-p :joint-angle 50)
    (fix-leg)
    (move-centroid)

    (when real
      (model2real :wait t :time 5000)
      (let ((prev-av (send *atlas* :angle-vector))
            (max-distance 10.0)
            (timeout 10)
            )
        (when (< max-distance (check-foot-distance))
          (warn ";; legs end-coords difference = ~A ;set legs to very-high-gain~%" (check-foot-distance))
          (atlas-set-servo-gain-by-torque-limit 0.01 :legs)
          (while (and (< 0 timeout) (< max-distance (check-foot-distance)))
            (warn ";; legs end-coords difference = ~A~%" (check-foot-distance))
            (unix::usleep (* 500 1000))
            (decf timeout))
          (atlas-set-servo-gain-by-torque-limit 1.0 :legs)
          (model2real :wait t :time 5000) ; wait 5s in sim time
          ))
      (send *ri* :start-st)
      (when vel-mode
	(warning-message 1 " rosrun pr2_teleop teleop_pr2_keyboard~%")
	(send *ri* :cmd-vel-mode))
      ))
  t)

(defun start-walk2 ()
  (start-walk :before-get-in-vehicle t))

(defun check-foot-distance ()
  (let ((model-av (send *atlas* :angle-vector))
        model-diff real-diff)
    (setq model-diff
          (distance (send *atlas* :lleg :end-coords :worldpos)
                    (send *atlas* :rleg :end-coords :worldpos)))
    ;; (send *atlas* :angle-vector (send *ri* :state :potentio-vector))
    (real2model)
    (setq real-diff
          (distance (send *atlas* :lleg :end-coords :worldpos)
                    (send *atlas* :rleg :end-coords :worldpos)))
    (send *atlas* :angle-vector model-av)
    (abs (- model-diff real-diff))))

(defun finish-walk
  (&key (real t) (mode :user))

  (let ((prev-av (send *atlas* :angle-vector))
        lleg-av rleg-av)
    (when real
      (when (or (equal mode :stand) (equal mode :manip) (equal mode :my-manip) (equal mode :user-change-pose))
        (modify-leg-stride #f(0 63.5 0)))
      (send *ri* :stop-auto-balancer)
      (send *ri* :stop-st))
    (when (or (equal mode :stand) (equal mode :manip) (equal mode :my-manip) (equal mode :user-change-pose))
      ;; 足だけbdi-stand-poseにする
      (send *atlas* :bdi-stand-pose)
      (setq lleg-av (send *atlas* :lleg :angle-vector))
      (setq rleg-av (send *atlas* :rleg :angle-vector))
      (send *atlas* :angle-vector prev-av)
      (send *atlas* :lleg :angle-vector lleg-av)
      (send *atlas* :rleg :angle-vector rleg-av)
      (fix-leg)
      (move-centroid)
      (when real (model2real :wait t :time 5000)))

    (when real
      (case mode
        (:stand (reset-bdi-controller) (set-stand))
        (:manip (reset-bdi-controller) (set-manip))
        (:my-manip (real2model) (model2real :wait t) (reset-bdi-controller) (manip))
        ;;(:my-manip (manip :smooth t))
        ))
    t))

(defun finish-walk2 ()
  (finish-walk :mode :my-manip))

(defun walk-clickpos
  ()

  (let* ((robot-coords (send (send *atlas* :foot-midcoords) :copy-worldcoords))
         (clicked-coords (get-point))
         (walk-pos
          (when clicked-coords
            (scale 0.001 (send (send robot-coords
                                     :transformation clicked-coords) :worldpos)))))
    (when walk-pos
      (warn "[walk-clickpos] x: ~a~%  OK?  " (elt walk-pos 0))
      (unless (y-or-n-p) (return-from walk-clickpos))
      (send *ri* :go-pos (elt walk-pos 0) 0 0))))

(defun turn-clickpos
  ()

  (let* ((robot-coords (send (send *atlas* :foot-midcoords) :copy-worldcoords))
         (clicked-coords (get-point))
         (walk-pos
          (when clicked-coords
            (scale 0.001 (send (send robot-coords
                                     :transformation clicked-coords) :worldpos))))
         (turn-theta (rad2deg (atan2 (elt walk-pos 1) (elt walk-pos 0)))))
    (when walk-pos
      (warn "[turn-clickpos] theta: ~a~%  OK?  " turn-theta)
      (unless (y-or-n-p) (return-from turn-clickpos))
      (send *ri* :go-pos 0 0 turn-theta))))

(defun walk
  (x)

  ;; (warn "[walk] x: ~a~%  OK?  " x)
  ;; (unless (y-or-n-p) (return-from walk))
  (send *ri* :go-pos x 0 0))

(defun turn
  (theta &optional (lr nil))

  (setq theta (cond ((equal lr :left) (abs theta))
                    ((equal lr :right) (- (abs theta))(abs theta))
                    (t theta)))
  ;; (warn "[walk] theta: ~a~%  OK?  " theta)
  ;; (unless (y-or-n-p) (return-from turn))
  (send *ri* :go-pos 0 0 theta))

(defun go-pos
  (x y theta)

  (send *ri* :go-pos x y theta))

(defun move-neck
  (&optional (rel-angle 10) &key (zero nil))

  (real2model)
  (case rel-angle
        (:zero (send *atlas* :head :neck-p :joint-angle 50))
        (t (send *atlas* :head :neck-p :joint-angle rel-angle :relative t)))
  (fix-leg)
  (move-centroid)
  (model2real :wait t :time 4000))

(defun move-waist
  (&optional (rel-angle 10) &key (zero nil))

  (real2model)
  (case rel-angle
    (:zero (send *atlas* :torso :waist-y :joint-angle 0))
    (t (send *atlas* :torso :waist-y :joint-angle rel-angle :relative t)))
  (fix-leg)
  (move-centroid)
  (model2real :wait t :time 4000))

(defun snapshot-hand
  ()

  (let ((prev-angle (send *atlas* :rarm :angle-vector)))
    (send *atlas* :rarm :shoulder-r :joint-angle 20)
    (send *atlas* :rarm :shoulder-y :joint-angle 30)
    (send *atlas* :rarm :elbow-p :joint-angle -90)
    (send *atlas* :rarm :elbow-y :joint-angle 110)
    (send *atlas* :rarm :wrist-y :joint-angle -20)
    (fix-leg)
    (move-centroid)
    (model2real :wait t :time 4000)
    (unix::system "rosservice call /rhand_snap/snapshot")
    (unix::system "rosservice call /lhand_snap/snapshot")
    (send *atlas* :rarm :angle-vector prev-angle)
    (model2real :wait t :time 4000)))

(defun walk-over-edge
  ()

  (finish-walk)

  (send *atlas* :angle-vector
        #f(0.0 0.0 0.0 23.5 114.5 0.0 -75.0 17.0 0.0 -23.5 114.5 0.0 75.0 17.0 0.0 102.266 -2.30694 -59.397 2.30694 -42.6426 0.025311 102.252 2.46955 -59.3977 -2.46955 -42.6279 0.025303 0.0))
  (model2real :wait t :time 5000)

  ;; (send *atlas* :angle-vector
  ;;       #f(0.0 61.9711 0.0 55.2 110.4 22.166 26.2001 -47.2501 0.0 -55.2 110.4 -22.166 -26.2001 -47.2501 0.0 102.266 -2.30694 -59.397 2.30694 -42.6426 0.025311 102.252 2.46955 -59.3977 -2.46955 -42.6279 0.025303 0.0))
  (send *atlas* :angle-vector
        #f(0.0 29.2896 0.0 55.2 110.4 22.166 26.2001 -47.2501 0.0 -55.2 110.4 -22.166 -26.2001 -47.2501 0.0 102.266 -2.30694 -59.397 2.30694 -42.6426 0.025311 102.252 2.46955 -59.3977 -2.46955 -42.6279 0.025303 0.0))
  (model2real :wait t :time 5000)

  (send *atlas* :angle-vector
        #f(0.0 29.2896 0.0 23.5 110.4 0.0 -6.19986 -47.2501 0.0 -23.5 110.4 0.0 6.19986 -47.2501 0.0 102.266 -2.30694 -59.397 2.30694 -42.6426 0.025311 102.252 2.46955 -59.3977 -2.46955 -42.6279 0.025303 0.0))
  (model2real :wait t :time 5000)

  ;; (send *atlas* :angle-vector
  ;;       #f(0.0 -36.0734 0.0 75.6 110.4 44.0146 -61.9998 20.25 0.0 -75.6 110.4 -44.0146 61.9998 20.25 0.0 38.4 -2.30694 -59.397 2.30694 -42.6426 0.025311 38.4 2.30694 -59.397 -2.30694 -42.6426 -0.025311 0.0))
  (send *atlas* :angle-vector
        #f(0.0 22.1849 0.0 75.6 110.4 38.265 -61.9998 20.25 0.0 -75.6 110.4 -38.265 61.9998 20.25 0.0 38.4 -2.30694 -59.397 2.30694 -42.6426 0.025311 38.4 2.30694 -59.397 -2.30694 -42.6426 -0.025311 0.0))
  (model2real :wait t :time 3000)

  (stand-up2)
  )

;; (progn
;;   (start-walk)
;;   (turn 90)
;;   (finish-walk)
;;   (start-walk :before-get-in-vehicle t)
;;   (walk 0.5)
;;   (finish-walk :mode :my-manip)
;;   (get-in-for-task1 :real t))

