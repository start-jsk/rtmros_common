(defun start-walk
  (&key (vel-mode nil))

  (real2model)
  (model2real :wait t :time 10)

  (atlas-set-servo-gain-by-torque-limit 1.0 :legs)
  (atlas-set-servo-gain-by-torque-limit 2.0 :torso)
  (atlas-set-servo-gain-by-torque-limit 4.0 :arms)
  (atlas-set-servo-gain-by-torque-limit 5.0 :head)

  (set-user)

  ;; (send *atlas* :walk-pose)
  (send *atlas* :legs :move-end-pos #f(0 0 70) :world)
  (model2real :wait t)

  (send *ri* :start-st)
  (when vel-mode
    (send *ri* :cmd-vel-mode))
  t
  )


(defun finish-walk
  (&key (mode :my-manip))

  ;; (send *atlas* :bdi-stand-pose)
  (send *atlas* :legs :move-end-pos #f(0 0 -70) :world)
  (when real
    (model2real :wait t)
    (send *ri* :stop-st))
  (case mode
    (:stand (set-stand))
    (:manip (set-manip))
    (:my-manip (manip)))
  t
  )




