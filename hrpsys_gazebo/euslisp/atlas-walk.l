(defun start-walk
  (&key (real t) (vel-mode nil))

  (when real
    (real2model)
    (model2real :wait t :time 10)

    (atlas-set-servo-gain-by-torque-limit 1.0 :legs)
    (atlas-set-servo-gain-by-torque-limit 2.0 :torso)
    (atlas-set-servo-gain-by-torque-limit 4.0 :arms)
    (atlas-set-servo-gain-by-torque-limit 5.0 :head)
    (unix::usleep (* 5 1000 1000))

    (set-user))

  ;; (send *atlas* :legs :move-end-pos #f(0 0 70) :world)
  (send *atlas* :head :neck-p :joint-angle 50)
  (fix-leg)
  (move-centroid)

  ;; 足だけwalk-poseにする
  (let ((prev-av (send *atlas* :angle-vector))
        lleg-av rleg-av)
    (send *atlas* :walk-pose)
    (setq lleg-av (send *atlas* :lleg :angle-vector))
    (setq rleg-av (send *atlas* :rleg :angle-vector))
    (send *atlas* :angle-vector prev-av)
    (send *atlas* :lleg :angle-vector lleg-av)
    (send *atlas* :rleg :angle-vector rleg-av)
    (send *atlas* :move-centroid-on-foot :both '(:lleg :rleg))
    )

  (when real
    (atlas-set-servo-gain-by-torque-limit 0.01 :legs)
    (unix::usleep (* 5 1000 1000))
    (model2real :wait t :time 10000)
    (atlas-set-servo-gain-by-torque-limit 1.0 :legs)
    (unix::usleep (* 5 1000 1000))
    (send *ri* :start-st)
    (when vel-mode
      (warning-message 1 " rosrun pr2_teleop teleop_pr2_keyboard~%")
      (send *ri* :cmd-vel-mode))
    )
  t)

(defun finish-walk
  (&key (real t) (mode :my-manip))

  (when real
    (send *ri* :stop-auto-balancer)
    (send *ri* :stop-st))
  (when (or (equal mode :stand) (equal mode :manip) (equal mode :my-manip))
    ;; (send *atlas* :legs :move-end-pos #f(0 0 -70) :world)
    (send *atlas* :rleg :move-end-pos #f(0 -50 -70) :world)
    (send *atlas* :lleg :move-end-pos #f(0 50 -70) :world)
    (when real
      (atlas-set-servo-gain-by-torque-limit 0.01 :legs)
      (unix::usleep (* 5 1000 1000))
      (model2real :wait t :time 5000)
      (atlas-set-servo-gain-by-torque-limit 1.0 :legs)
      (unix::usleep (* 5 1000 1000))))

  (when real
    (case mode
      (:stand (set-stand))
      (:manip (set-manip))
      (:my-manip (manip))))
  t)

(defun walk-clickpos ()
  (let* ((robot-coords (send (send *atlas* :foot-midcoords) :copy-worldcoords))
         (clicked-coords (get-point))
         (walk-pos
          (when clicked-coords
            (scale 0.001 (send (send robot-coords
                                     :transformation clicked-coords) :worldpos)))))
    (when walk-pos
      (warn "[walk-clickpos] x: ~a~%" (elt walk-pos 0))
      (send *ri* :go-pos (elt walk-pos 0) 0 0))))

(defun walk-turn
  (&key (theta 90) (lr :left))

  (setq theta (if (equal lr :left) (abs theta) (- (abs theta))))
  (send *ri* :go-pos 0 0 theta))

