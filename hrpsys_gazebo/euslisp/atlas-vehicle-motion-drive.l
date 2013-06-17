
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
   (send *atlas* :torso :waist-r :joint-angle -40)
   (send *irtviewer* :draw-objects)
   (when real (model2real :time 200 :wait t))
   (send *atlas* :legs :knee-p :joint-angle 90)
   (send *atlas* :legs :crotch-p :joint-angle 90)
   (send *atlas* :torso :waist-r :joint-angle 40)
   (send *irtviewer* :draw-objects)
   (when real (model2real :time 200 :wait t)))

  (warning-message 2 "[vhc-drive] finish (move-to-seat-in-drive-for-task1) [drive phase1]~%")
  )
