(load "package://hrpsys_gazebo/euslisp/atlas-interface.l")
;; application for task1
(load "package://hrpsys_gazebo/euslisp/atlas-calc-position.l")

(defun wobj (obj) ;; write object to standard-output
  (dump-structure *standard-output* obj))
