#!/usr/bin/env roseus

;; $ rtmlaunch hrpsys_ros_bridge samplerobot.launch
;; $ roseus samplerobot-sequence-player.l "(samplerobot-sequence-player-demo)"

(load "package://hrpsys_ros_bridge/euslisp/samplerobot-interface.l")

(defun samplerobot-sequence-player-init ()
  (samplerobot-init)
  t)

(defun samplerobot-sequence-player-demo0 ()
  "1. :angle-vector (setJointAngles) and :wait-interpolation (waitInterpolation)"
  (send *ri* :angle-vector (send *sr* :angle-vector) 700)
  (send *ri* :wait-interpolation)
  (eps= (distance (send *ri* :state :potentio-vector) (send *sr* :angle-vector)) 0.0)
  )

(defun samplerobot-sequence-player-demo1 ()
  "2. :set-interpolation-mode"
  (send *ri* :set-interpolation-mode :linear)
  (send *sr* :reset-manip-pose)
  (send *ri* :angle-vector (send *sr* :angle-vector) 500)
  (send *ri* :wait-interpolation)
  (send *ri* :set-interpolation-mode :hoffarbib)
  (send *sr* :reset-pose)
  (send *ri* :angle-vector (send *sr* :angle-vector) 500)
  (send *ri* :wait-interpolation)
  t)

(defun samplerobot-sequence-player-demo2 ()
  "3. limb controller trajectory action"
  ;; Initialize pose
  (send *sr* :reset-pose)
  (send *ri* :angle-vector (send *sr* :angle-vector) 500)
  (send *ri* :wait-interpolation)
  (let ((limbs '(:rarm :larm :torso :rleg :lleg :rhand :lhand)))
    ;; Add joint group just for test
    (mapcar #'(lambda (l) (send *ri* :add-joint-group l)) limbs)
    (let ((ret (mapcar
              #'(lambda (limb)
                  (let ((dif-ja 0.5) ;; [deg]
                        (av0 (send *sr* :reset-pose)))
                    ;; Set pose
                    (mapcar #'(lambda (tmp-limb)
                                (mapcar #'(lambda (jn)
                                            (send (send *sr* :joint jn) :joint-angle dif-ja :relative t))
                                        (cdr (assoc :joint-names (car (send *ri* (read-from-string (format nil "~A-controller" tmp-limb))))))))
                            limbs)
                    (let ((av1 (send *sr* :angle-vector))
                          (ctype (read-from-string (format nil "~A-controller" limb))))
                      ;; Send pose just for specific limb controller
                      (send *ri* :angle-vector (send *sr* :angle-vector) 200 ctype)
                      (send *ri* :wait-interpolation ctype)
                      (send *ri* :state)
                      (let ((ret-all
                             (mapcar #'(lambda (jnt)
                                         (let ((idx (position jnt (send *sr* :joint-list))))
                                           (eps= (elt (send (send *ri* :robot) :angle-vector) idx)
                                                 ;; If jnt is included in ctype, (send *ri* :robot)'s joint angle is same as av1. Otherwise av0.
                                                 (if (member (send jnt :name) (cdr (assoc :joint-names (car (send *ri* ctype)))) :test #'equal)
                                                     (elt av1 idx)
                                                   (elt av0 idx)))))
                                     (send *sr* :joint-list))))
                        (send *ri* :remove-joint-group limb)
                        (send *ri* :wait-interpolation-seq)
                        (every #'identity ret-all)
                        ))))
              limbs)))
      (every #'identity ret)
      )))

(defun samplerobot-sequence-player-demo ()
  (samplerobot-sequence-player-init)
  (samplerobot-sequence-player-demo0)
  (samplerobot-sequence-player-demo1)
  (samplerobot-sequence-player-demo2)
  )

(warn ";; (samplerobot-sequence-player-demo)~%")