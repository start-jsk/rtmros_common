
;; \file    atlas-vehicle-motion-drive.l
;; \author  Murooka Masaki
;; \date    2013/06/17
;; \brief   vrc_task_1関連 車運転行動要素関数
;;


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;; vehicle motion [drive]
(defun drive-for-task1
  (&key (real nil) (wait nil) (telep nil))

  (warning-message 2 "[vhc-drive] start (drive-for-task1)~%")

  (when telep
    (telep-in-drive-for-task1 :real real :wait wait))

  (move-to-seat-in-drive-for-task1 :real real :wait wait)

  (warning-message 2 "[vhc-drive] finish (drive-for-task1)~%")
  )

(defun telep-in-drive-for-task1
  (&key (real nil) (wait nil))

  (warning-message 2 "[vhc-drive] start (telep-in-drive-for-task1)~%")


  (warning-message 2 "[vhc-drive] finish (telep-in-drive-for-task1)~%")
  )

(defun move-to-seat-in-drive-for-task1
  (&key (real nil) (wait nil))

  (warning-message 2 "[vhc-drive] start (move-to-seat-in-drive-for-task1) [drive phase1]~%")

  (warning-message 2 "[vhc-drive] loop boyon-boyon in move to seat [drive phase1]~%")
  (when wait (my-wait))
  (do-until-key
   (send *atlas* :legs :knee-p :joint-angle 0)
   (send *atlas* :legs :crotch-p :joint-angle 0)
   (send *atlas* :torso :waist-r :joint-angle 40)
   (send *irtviewer* :draw-objects)
   (when real (model2real :time 200 :wait t))
   (send *atlas* :legs :knee-p :joint-angle 90)
   (send *atlas* :legs :crotch-p :joint-angle 90)
   (send *atlas* :torso :waist-r :joint-angle -40)
   (send *irtviewer* :draw-objects)
   (when real (model2real :time 200 :wait t)))

  (warning-message 2 "[vhc-drive] finish (move-to-seat-in-drive-for-task1) [drive phase1]~%")
  )

;;
;; very very dirty code
;;
#|
(setq *lr-reverse* nil)
(real2model)
(send *atlas* :newcoords (make-coords :rpy (mapcar #'deg2rad (get-imu-data))))
;;(move-robot-with-imu)

(model2real)

(set-stand)
(reset-bdi-controller)
;;(send *ri* :set-simulation-mode :mode :user :effort 255)

(setq pt (get-full-cloud))
(setq qq (send pt :filter :key #'(lambda (p) (< (norm p) 1000)) :create t))

(send *atlas* :hand :larm :grasp-pose)
(send *atlas* :hand :rarm :grasp-pose)
(hand-model2real)

(send *atlas* :hand :larm :open-pose)
(send *atlas* :hand :rarm :open-pose)
(hand-model2real)

(send *atlas* :lleg :crotch-y :joint-angle 20)
(model2real :time 2000)

(send *atlas* :lleg :ankle-p :joint-angle 25)
(model2real :time 2000)
|#
(defun pre-seat-pose ()
  )

(defun atlas-reset () (send *atlas* :reset-coords))
(defun set-seat-pose (&optional (yaw 15) (pt 90))
  (send *atlas* :reset-manip-pose)
  (send *atlas* :legs :crotch-p :joint-angle (- pt))
  (send *atlas* :legs :knee-p :joint-angle pt)
  (send *atlas* :legs :crotch-p :joint-angle (- pt))
  (send *atlas* :lleg :crotch-y :joint-angle yaw)
  (send *atlas* :rleg :crotch-y :joint-angle (- yaw))
  (send *atlas* :head :neck-p :joint-angle 35)
  (send *atlas* :reset-coords)
  )
(defun set-default-gain ()
  (atlas-set-servo-gain-by-torque-limit 1.0 :legs)
  (atlas-set-servo-gain-by-torque-limit 2.0 :torso)
  (atlas-set-servo-gain-by-torque-limit 4.0 :arms)
  (atlas-set-servo-gain-by-torque-limit 5.0 :head)
  )
(defun move-world (vec &optional (limb :larm))
  (let ((cds (send *atlas* limb :end-coords :copy-worldcoords)))
    (send cds :translate vec :world)
    (send *atlas* limb :inverse-kinematics cds :rotation-axis nil)
    ))
