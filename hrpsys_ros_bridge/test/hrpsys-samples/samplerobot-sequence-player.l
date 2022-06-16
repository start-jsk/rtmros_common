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
                      ;; fix for https://github.com/fkanehiro/hrpsys-base/pull/1297
                      ;; need to send reset-pose to default sequence controller
                      (send *ri* :angle-vector av0 500)
                      (send *ri* :wait-interpolation)
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

(defun samplerobot-sequence-player-demo3 ()
  "4. :angle-vector-sequence-full (setJointAnglesSequenceFull)"
  (labels ((calc-wrench
            ()
            (let ((flist) (mlist))
              (multiple-value-setq
               (flist mlist)
               (send *sr* :calc-contact-wrenches-from-total-wrench (list (send *sr* :lleg :end-coords :worldpos) (send *sr* :rleg :end-coords :worldpos))))
              (apply #'concatenate float-vector (append (mapcar #'(lambda (f m) (concatenate float-vector f m)) flist mlist) (make-list 2 :initial-element (instantiate float-vector 6))))
              ))
           (save-log
            ()
            (send *ri* :set-log-maxlength 2)
            (send *ri* :start-log)
            (unix:usleep 100000)
            (send *ri* :save-log "/tmp/test-samplerobot-seq-full" :set-robot-date-string nil)
            )
           (get-float-vector-from-log
            (fname)
            (with-open-file
             (f fname :direction :input)
             (let ((ln))
               (subseq (read-from-string (format nil "#f(~A)" (read-line f nil nil))) 1))))
           (check-log
            (ref-angle-vector ref-root-pos ref-root-rpy ref-zmp ref-wrench)
            (save-log)
            (let ((dif-angle-vector (distance (get-float-vector-from-log "/tmp/test-samplerobot-seq-full.sh_qOut") ref-angle-vector))
                  (dif-root-pos (distance (get-float-vector-from-log "/tmp/test-samplerobot-seq-full.sh_basePosOut") ref-root-pos))
                  (dif-root-rpy (distance (get-float-vector-from-log "/tmp/test-samplerobot-seq-full.sh_baseRpyOut") ref-root-rpy))
                  (dif-zmp (distance (get-float-vector-from-log "/tmp/test-samplerobot-seq-full.sh_zmpOut") ref-zmp))
                  (dif-wrench (distance (apply #'concatenate float-vector (mapcar #'(lambda (x) (get-float-vector-from-log (format nil "/tmp/test-samplerobot-seq-full.sh_~AOut" x))) (reverse (send-all (send *sr* :force-sensors) :name)))) ;;(reverse) Because :force-sensors and end-effectors of hrpsys are in reverse order.
                                        ref-wrench)))
              (warn ";; Diff angle-vector = ~A, root-pos = ~A, root-rpy = ~A, zmp = ~A, wrench = ~A~%"
                    dif-angle-vector dif-root-pos dif-root-rpy dif-zmp dif-wrench)
              ;; (print (list (get-float-vector-from-log "/tmp/test-samplerobot-seq-full.sh_baseRpyOut") ref-root-rpy))
              ;; (print (list (get-float-vector-from-log "/tmp/test-samplerobot-seq-full.sh_basePosOut") ref-root-pos))
              (and (eps= dif-angle-vector 0.0 (deg2rad 0.1))
                   (eps= dif-root-pos 0.0 (* 1e-3 0.1))
                   (eps= dif-root-rpy 0.0 (deg2rad 0.1))
                   (eps= dif-zmp 0.0 (* 1e-3 0.1))
                   (eps= dif-wrench 0.0 (* 1e-3 0.1))
                   ))))
    (let* ((dt-msec 900) (ret t)
           ;; State1
           (jpos1 (progn (send *sr* :init-pose) (send *sr* :fix-leg-to-coords (make-coords) '(:rleg :lleg)) (send *sr* :angle-vector)))
           (rc1 (send (car (send *sr* :links)) :copy-worldcoords))
           (zmp1 (send (send *sr* :foot-midcoords) :worldpos))
           (wr1 (calc-wrench))
           (optional1 #f(1 1 0 0 1 1 1 1))
           ;; State2
           (jpos2 (progn (send *sr* :reset-pose) (send *sr* :legs :crotch-p :joint-angle -30) (send *sr* :fix-leg-to-coords (make-coords) '(:rleg :lleg)) (send *sr* :move-centroid-on-foot :both '(:rleg :lleg)) (send *sr* :angle-vector)))
           (rc2 (send (car (send *sr* :links)) :copy-worldcoords))
           (zmp2 (send (send *sr* :foot-midcoords) :worldpos))
           (wr2 (calc-wrench))
           (optional2 #f(1 1 0 0 1 1 1 1))
           ;; State3
           (jpos3 (progn (send *sr* :reset-manip-pose) (send *sr* :fix-leg-to-coords (make-coords) '(:rleg :lleg)) (send *sr* :move-centroid-on-foot :both '(:rleg :lleg)) (send *sr* :angle-vector)))
           (rc3 (send (car (send *sr* :links)) :copy-worldcoords))
           (zmp3 (send (send *sr* :foot-midcoords) :worldpos))
           (wr3 (calc-wrench))
           (optional3 #f(1 1 0 0 1 1 1 1)))
      ;; Init
      (send *ri* :angle-vector jpos1 2000)
      (send *ri* :wait-interpolation)
      ;; Check 1
      (send *ri* :angle-vector-sequence-full (list jpos1 jpos2 jpos3) (list dt-msec dt-msec dt-msec))
      (send *ri* :wait-interpolation-seq)
      (setq ret (and ret (check-log (map float-vector #'deg2rad jpos3)
                                    (float-vector 0 0 0) (float-vector 0 0 0)
                                    (float-vector 0 0 0) (instantiate float-vector (length wr3)))))
      ;; Check 2
      (send *ri* :angle-vector-sequence-full (list jpos1 jpos2) (list dt-msec dt-msec)
            :root-coords (list rc1 rc2) :zmp (list zmp1 zmp2)
            :wrench (list wr1 wr2) :optional (list optional1 optional2))
      (send *ri* :wait-interpolation-seq)
      (setq ret (and ret (check-log (map float-vector #'deg2rad jpos2)
                                    (scale 1e-3 (send rc2 :worldpos)) (concatenate float-vector (reverse (car (rpy-angle (send rc2 :worldrot)))))
                                    (scale 1e-3 (send rc2 :inverse-transform-vector zmp2)) wr2)))
      ;; Check 3
      (send *ri* :angle-vector-sequence-full (list jpos1 jpos2 jpos3) (list dt-msec dt-msec dt-msec)
            :root-coords (list rc1 rc2 rc3) :zmp (list zmp1 zmp2 zmp3)
            :wrench (list wr1 wr2 wr3) :optional (list optional1 optional2 optional3))
      (send *ri* :wait-interpolation-seq)
      (setq ret (and ret (check-log (map float-vector #'deg2rad jpos3)
                                    (scale 1e-3 (send rc3 :worldpos)) (concatenate float-vector (reverse (car (rpy-angle (send rc3 :worldrot)))))
                                    (scale 1e-3 (send rc3 :inverse-transform-vector zmp3)) wr3)))
      ;; Return
      ret)))

(defun samplerobot-sequence-player-demo ()
  (samplerobot-sequence-player-init)
  (samplerobot-sequence-player-demo0)
  (samplerobot-sequence-player-demo1)
  (samplerobot-sequence-player-demo2)
  (samplerobot-sequence-player-demo3)
  )

(warn ";; (samplerobot-sequence-player-demo)~%")
