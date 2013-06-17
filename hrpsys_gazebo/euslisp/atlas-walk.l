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
  (send *atlas* :rleg :move-end-pos #f(0 50 70) :world)
  (send *atlas* :lleg :move-end-pos #f(0 -50 70) :world)

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
  t
  )




