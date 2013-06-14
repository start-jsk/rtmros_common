(progn 
  (load "package://hrpsys_gazebo/euslisp/atlas-interface.l")
  (atlas-init :set-reset-pose nil :set-user-mode nil)
  (real2model)
  (model2real :wait t :time 10)
  (set-user)
  (set-kp 4000)
  (send *atlas* :walk-pose)
  (model2real :wait t)
  (send *ri* :start-st)
  (send *ri* :cmd-vel-mode)
  (send *ri* :stop-st)
  )

