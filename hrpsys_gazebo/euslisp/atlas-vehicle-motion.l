
;; \file    atlas-vehicle-motion.l
;; \author  Murooka Masaki
;; \date    2013/06/08
;; \brief   vrc_task_1関連関数
;;

(load "package://hrpsys_gazebo/euslisp/atlas-interface.l")
(load "package://hrpsys_gazebo/euslisp/atlas-standup.l")
(load "package://hrpsys_gazebo/euslisp/atlas-calc-position.l")
(load "package://hrpsys_gazebo/euslisp/atlas-vehicle-motion-utils.l")
(load "package://hrpsys_gazebo/euslisp/atlas-interface.l")
(load "package://hrpsys_gazebo/euslisp/t1l5.l")

;; (progn
;;   (my-init :real t)

;;   (setq *lr-reverse* t)
;;   (start-walk)
;;   (turn 90)
;;   (finish-walk)

;;   (setq *lr-reverse* t)
;;   (start-walk :before-get-in-vehicle t)
;;   (walk 0.5)
;;   (finish-walk :mode :manip)
;;   ;; (finish-walk :mode :user-change-pose)

;;   (setq *lr-reverse* t)
;;   (get-in-for-task1 :real t :after-walk t))


;; (get-diff-vehicle-pos)



;; setting
(defun make-vehicle ()
  (unless (boundp '*drcv*)
    (setq *drcv* 
	  (with-open-file 
	   (f (format nil "~a/~a" (unix:getenv "HOME") "ros/fuerte/rtm-ros-robotics/rtmros_common/hrpsys_gazebo/models/drc-vehicle.l")) (read f))))
  ;; upper-front-pipe
  (send *drcv* :add-handle
	(make-cascoords :pos #f(320 150 1920) :rpy (list 0 0 -pi/2) :name :upper-front-pipe))
  ;; (send *drcv* :add-handle
  ;; 	(make-cascoords :pos #f(320 0 1920) :rpy (list 0 0 -pi/2) :name :upper-front-pipe))
  ;; upper-left-pipe
  (send *drcv* :add-handle
	(make-cascoords :pos #f(140 575 1900) :rot (transpose (make-matrix 3 3 (list #f(0 0 -1) #f(0 -1 0) #f(-1 0 0)))) :name :upper-left-pipe))
  ;; front-left-pipe
  (send *drcv* :add-handle
	(make-cascoords :pos #f(500 600 1500) :rpy (list pi (* pi 0.125) 0) :name :front-left-pipe))
  ;; draw
  (if (boundp '*irtviewer*)
      (objects (union (send *irtviewer* :objects) (list *drcv*)))
    (objects (list *drcv*))))

(setq *vehicle-start-pos* (make-coords :pos #f(-10000 2000 0) :rpy (list -pi/2 0 0)))
(setq *vehicle-side-pos* (make-coords :pos #f(175 950 0) :rpy (list 0 0 0)))


;; move
;;;; (stand-up) on starting point [cheating!!]
(defun stand-up-start ()
  (warning-message 1 "[vhc] [cheating!!] called (stand-up-start)~%")
  (send *ri* :teleport-to (make-coords :pos (v+ (send *vehicle-start-pos* :worldpos) #f(0 0 950)) :rpy (list -pi/2 0 0))))
(defun stand-up-road ()
  (warning-message 1 "[vhc] [cheating!!] called (stand-up-start)~%")
  (send *ri* :teleport-to (make-coords :pos (v+ (send *vehicle-side-pos* :worldpos) #f(2000 0 950)) :rpy (list 0 0 0))))

;;;; only teleport to starting point (not set harnessed) [cheating!!]
(defun telep-start ()
  (warning-message 1 "[vhc] [cheating!!] called (telep-start)~%")
  (ros::publish "atlas/set_pose"
		(ros::coords->tf-pose (make-coords :pos (v+ (send *vehicle-start-pos* :worldpos) #f(0 0 950)) :rpy (list -pi/2 0 0)))))

;;;; move atlas model in side of vehicle
(defun fix-leg-vehicle-side (&optional (leg :both))
  (cond
   ((equal leg :both)
    (send *atlas* :fix-leg-to-coords *vehicle-side-pos* '(:lleg :rleg)))
   (t (send *atlas* :newcoords
	    (send (send *vehicle-side-pos* :copy-worldcoords) :transform 
		  (send (send *atlas* leg :end-coords :copy-worldcoords)
			:transformation (send *atlas* :worldcoords) :local) :local)))))



;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; all vehicle motion

(defun lr-rev
  ()

  (setq *lr-reverse* t))


(defun init-task1
  ()

  (load "package://hrpsys_gazebo/euslisp/no-display.l")
  (make-irtviewer)

  (my-init :real t)
  (lr-rev)
  (setq *imu-data-offset* (list 0 0 0)))

(defun all-demo-task1
  (&key (real nil) (wait nil) (auto-walk-and-telep nil) (no-walk-and-telep nil))

  (warning-message 2 "[vhc-init] lr-reverse mode !!!~%")
  (setq *lr-reverse* t)

  (warning-message 2 "[vhc-init] start (all-demo-task1)~%")

  (init-for-task1 :real real :wait wait :auto auto-walk-and-telep)

  (unless no-walk-and-telep
    (walk-for-task1 :real real :wait wait :auto auto-walk-and-telep))

  (get-in-for-task1 :real real :wait wait :telep (or auto-walk-and-telep no-walk-and-telep))

  (drive-for-task1 :real real :wait t :telep nil)

  (warning-message 2 "[vhc-init] finish (all-demo-task1)~%")
  )


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;; vehicle motion [init]
(load "package://hrpsys_gazebo/euslisp/atlas-vehicle-motion-others.l")


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;; vehicle motion [walk]
;; (load "package://hrpsys_gazebo/euslisp/atlas-vehicle-motion-others.l") ;; loaded above


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;; vehicle motion [get-in]
(load "package://hrpsys_gazebo/euslisp/atlas-vehicle-motion-get-in.l")


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;; vehicle motion [drive]
(load "package://hrpsys_gazebo/euslisp/atlas-vehicle-motion-drive.l")



;; usage
(defun usage ()
  (warn ";; demo~%")
  (warn "  (all-demo-task1 :real t :wait nil)~%")
  (warn ";; demo : walk through gait1 and telep in side of vehicle~%")
  (warn "  (all-demo-task1 :real t :wait nil :auto-walk-and-telep t)~%")
  (warn ";; demo : no walk and telep in side of vehicle~%")
  (warn "  (all-demo-task1 :real t :wait nil :no-walk-and-telep t)~%")
  )
