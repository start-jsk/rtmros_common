;; init
(defun my-init
  (&key (real t) (set-reset-pose t))
  (if real
      (unless (boundp '*atlas-init-called*)
	(setq *atlas-init-called* t)
	(atlas-init :set-reset-pose set-reset-pose :set-user-mode nil))
    (if (not (boundp '*atlas*)) (setq *atlas* (atlas-with-hand))))
  (unless (boundp '*translated-atlas-arm-end-coords*)
    (setq *translated-atlas-arm-end-coords* t)
    (send (send *atlas* :larm :end-coords) :translate #f(0 125 0) :local)
    (send (send *atlas* :rarm :end-coords) :translate #f(0 -125 0) :local)
    (send (send (send (send *atlas* :larm :end-coords)
		      :rotate (deg2rad -20) :x) :translate #f(0 0 -30) :local) :translate #f(0 -20 0) :local)
    (send (send (send (send *atlas* :rarm :end-coords)
		      :rotate (deg2rad 20) :x) :translate #f(0 0 -30) :local) :translate #f(0 20 0) :local))
  (when real
    (atlas-set-servo-gain-by-torque-limit 1.0)
    (set-hand-kp 100.0))
  (setq *robot* *atlas*)
  (objects (list *atlas*)))

(defun bar-init ()
  (load "euslib/demo/s-noda/atlas/robot-pose.l"))

(defun stand-up ()
  (send *ri* :teleport-to))


;; hand

;; (send *ri* :hand :larm :send_query "start")
;; (send *ri* :hand :larm :send_query "stop")

(defun hand-model2real (&key (arm :both) (wait nil))
  (cond
   ((equal arm :both) (hand-model2real :arm :larm :wait wait) (hand-model2real :arm :rarm :wait wait))
   (t (set-real-hand-av (get-model-hand-av arm) arm)
      (when wait (send *ri* :hand arm :wait-interpolation)))))

(defun hand-real2model (&key (arm :both))
  (cond
   ((equal arm :both) (hand-real2model :arm :larm) (hand-real2model :arm :rarm))
   (t (set-model-hand-av (get-real-hand-av arm) arm))))

(defun set-real-hand-av (av &optional (arm :both))
  (send *ri* :hand arm :angle-vector av))

(defun get-real-hand-av (&optional (arm :both))
  (send *ri* :hand arm :state :potentio-vector))

(defun set-model-hand-av (av &optional (arm :both))
  (send *atlas* :hand arm :angle-vector av))

(defun get-model-hand-av (&optional (arm :both))
  (send *atlas* :hand arm :angle-vector))

(defun set-hand-kp (kp &optional (arm :both))
  (send *ri* :hand arm :send-command :pgain kp))

(defun grasp (&key (arm :both) (real nil))
  (set-model-hand-av #f(0 90 90 0 90 90 0 90 90 0 20 90) arm)
  ;; (send *ri* :start-grasp (if (equal arm :both) :arms arm))
  (when real (hand-model2real))
  )

(defun ungrasp (&key (arm :both) (real nil))
  (set-model-hand-av #f(0 0 0 0 0 0 0 0 0 0 0 0) arm)
  ;; (send *ri* :stop-grasp (if (equal arm :both) :arms arm))
  (when real (hand-model2real))
  )


;; sim mode
(defun set-user (&optional (e 255))
  (send *ri* :set-simulation-mode :mode :user :effort e))

(defun set-stand (&optional (e 0))
  (send *ri* :set-simulation-mode :mode :stand :effort e))

(defun set-manip (&optional (e 255))
  (send *ri* :set-simulation-mode :mode :manipulate :effort e))

(defun set-walk (&optional (e 0))
  (send *ri* :set-simulation-mode :mode :walk :effort e))

(defun set-step (&optional (e 0))
  (send *ri* :set-simulation-mode :mode :step :effort e))

(defun get-mode ()
  (send *ri* :get-control-mode))

(defun set-effort (&optional (e 0) (limb :all))
  (send *ri* :set-k-effort e limb))

(defun get-effort (&optional limb)
  (send *ri* :get-k-effort limb))

(defun set-kp 
  (&optional (kp 100.0) limb)
  (cond
   ((equal limb :arms) (set-kp kp :larm) (set-kp kp :rarm))
   ((equal limb :legs) (set-kp kp :lleg) (set-kp kp :rleg))
   (t (dolist (j (if limb (send-all (send *atlas* limb :joint-list) :name)
		   (send-all (send *atlas* :joint-list) :name)))
	(send *ri* :set-servo-gain-percentage j kp)))))

(defun manip ()
  (set-user nil)
  (set-effort 0 :legs)
  (set-effort 255 :torso)
  (set-effort 255 :arms)
  (set-effort 255 :head))


;; mode
(defun set-pinned ()
  (send *ri* :atlas-mode "pinned"))

(defun set-pinned-wg ()
  (send *ri* :atlas-mode "pinned_with_gravity"))

(defun set-nominal ()
  (send *ri* :atlas-mode "nominal"))

(defun set-feet ()
  (send *ri* :atlas-mode "feet"))


;; angle vector
(defun real2model ()
  (send *atlas* :angle-vector (send *ri* :state :potentio-vector)))

(defun model2real (&key (time 2000) (wait nil))
  (send *ri* :angle-vector (send *atlas* :angle-vector) time)
  (when wait (send *ri* :wait-interpolation)))

(defun reset-pose (&key (real nil))
  (send *atlas* :stand-pose)
  (send *atlas* :larm :elbow-p :joint-angle -5 :relative t)
  (send *atlas* :rarm :elbow-p :joint-angle 5 :relative t)
  (send *atlas* :head :neck-p :joint-angle 0) 
  ;; (move-centroid)
  (when real (model2real)))


;; angle vector util
(defun mirror-angle-vector-from
  (from)
  (let ((av (send *atlas* from :angle-vector))
        (arm-convert-v (float-vector -1 1 -1 -1 1 1))
        (leg-convert-v (float-vector 1 -1 1 -1 1 -1))
        )
    (case from
      (:lleg
       (setq av (map float-vector #'* leg-convert-v av))
       (send *atlas* :rleg :angle-vector av)
       )
      (:rleg
       (setq av (map float-vector #'* leg-convert-v av))
       (send *atlas* :lleg :angle-vector av)
       )
      (:larm
       (setq av (map float-vector #'* arm-convert-v av))
       (send *atlas* :rarm :angle-vector av)
       )
      (:rarm
       (setq av (map float-vector #'* arm-convert-v av))
       (send *atlas* :larm :angle-vector av)
       ))))

(defun fix-leg (&optional (coord (make-coords)))
  (send *atlas* :fix-leg-to-coords coord '(:rleg :lleg)))

(defun move-centroid ()
  (send *atlas* :move-centroid-on-foot :both '(:lleg :rleg)))

(defun set-last ()
  (send *atlas* :angle-vector *last-atlas-av*)
  (send *atlas* :newcoords *last-atlas-worldcoords*)
  (send *irtviewer* :draw-objects))


;; draw
(defun draw-coord (&optional (coord (make-coords)) (color #f(0 1 0)))
  (send coord :draw-on :flush t :size 200 :width 5 :color color))

(defun draw-end-coords (&optional (limb :larm))
  (draw-coord (send *atlas* limb :end-coords)))

(defun draw-point ()
  (draw-coord (get-point) #f(1 0 0)))


;; imu
(defun move-robot-with-imu (&key (loop-p nil))
  (if loop-p
      (do-until-key
       (real2model)
       (send *atlas* :newcoords (make-coords :rpy (mapcar #'deg2rad (get-imu-data))))
       (send *irtviewer* :draw-objects)
       (x::window-main-one)
       (unix::usleep (* 10 1000)))
    (send *atlas* :newcoords (make-coords :rpy (mapcar #'deg2rad (get-imu-data))))
    ))

(defun get-hrpsys-imu-rpy ()
  (car (rpy-angle (send (send *ri* :state :imucoords) :rot))))

(defun move-robot-with-hrpsys-imu (&key (loop-p nil))
  (if loop-p
      (do-until-key
       (real2model)
       (send *atlas* :newcoords (make-coords :rpy (get-hrpsys-imu-rpy)))
       (send *irtviewer* :draw-objects)
       (x::window-main-one)
       (unix::usleep (* 10 1000)))
    (send *atlas* :newcoords (make-coords :rpy (get-hrpsys-imu-rpy)))
    ))

(defun set-imu-data-offset ()
  (setq *imu-data-offset* (get-imu-data))
  )

(defun get-imu-data-with-offset ()
  (mapcar #'- (get-imu-data) *imu-data-offset*))

(defun rotate-with-imu (zdeg)
  (let ((diff (- zdeg (elt (get-imu-data-with-offset) 0))))
    (cond
      ((< 180 diff)
       (setq diff (- diff 360))
       )
      ((> -180 diff)
       (setq diff (+ diff 360))
       )
      )
    (if (< 1 (abs diff))
      (progn
        (print diff)
        (send *ri* :go-pos 0 0 diff)
        ))
    ))

;; joint
(defmethod atlas-robot
  (:calc-target-centroid-pos-for-one-leg
   (fix-leg &optional (offset))
   (send (send self fix-leg :end-coords) :transform-vector offset))
  (:calc-target-centroid-pos
   (centroid-leg fix-legs
		 &optional (mid 0.5)
		           (offsets
			    (mapcar
			     #'(lambda (x)
				 (float-vector 30 0 0))
			     (make-list (length fix-legs)))))
   (if (eq centroid-leg :both)
       (apply
	#'midpoint mid
	(mapcar
	 #'(lambda (tmp-leg tmp-of)
	     (send self :calc-target-centroid-pos-for-one-leg tmp-leg tmp-of))
	 fix-legs offsets))
     (send self :calc-target-centroid-pos-for-one-leg centroid-leg (car offsets))))
  (:walk-pose
   ()
   (send self :reset-pose)
   (send self :fix-leg-to-coords (make-coords))
   (send self :legs :move-end-pos #f(0 0 70) :world)
   (send self :fix-leg-to-coords (make-coords))
   (send self :move-centroid-on-foot
	 :both '(:rleg :lleg)
	 :target-centroid-pos (send self :calc-target-centroid-pos :both '(:rleg :lleg)))
   )
  (:get-jl
   (limb)
   (cadr (assoc limb *atlas-joint-list*)))
  (:print-jl ()
	     (dolist (jl *atlas-joint-list*) (dolist (j (cadr jl)) (print (append (list (car jl)) (list j) (list (send *atlas* (car jl) j :max-angle)) (list (send *atlas* (car jl) j :min-angle))))))))

(setq *atlas-joint-list*
      (list 
       (list :head (list :neck-p))
       (list :torso (list :waist-r :waist-p :waist-y))
       (list :larm (list :shoulder-r :shoulder-y :elbow-p :elbow-y :wrist-p :wrist-y))
       (list :lleg (list :crotch-r :crotch-p :crotch-y :knee-p :ankle-r :ankle-p ))
       (list :rarm (list :shoulder-r :shoulder-y :elbow-p :elbow-y :wrist-p :wrist-y))
       (list :rleg (list :crotch-r :crotch-p :crotch-y :knee-p :ankle-r :ankle-p ))))


;; model
(defmethod cascaded-link
  (:handle
   (&key name)
   (let ((handle (get self :handle)))
     (dolist (h (flatten handle))
       (when h (send h :worldcoords)))
     (if name (find-if #'(lambda (x) (equal name (send x :name))) handle) handle)))
  (:add-handle 
   (h)
   (let ((handle (get self :handle)))
     (push h handle)
     (setf (get self :handle) handle)
     (send (car bodies) :assoc h))
   ))

(defun make-vehicle ()
  (unless (boundp '*drcv*)
    (setq *drcv* 
	  (with-open-file 
	   (f (format nil "~a/~a" (unix:getenv "HOME") "ros/fuerte/rtm-ros-robotics/rtmros_common/hrpsys_gazebo/models/drc-vehicle.l")) (read f))))
  (send *drcv* :add-handle
	(make-cascoords :pos #f(140 575 1900) :rot (transpose (make-matrix 3 3 (list #f(0 0 -1) #f(0 -1 0) #f(-1 0 0)))) :name :upper-left-pipe))
  (send *drcv* :add-handle
	(make-cascoords :pos #f(500 600 1500) :rpy (list pi (* pi 0.125) 0) :name :front-left-pipe))
  (if (boundp '*irtviewer*)
      (objects (union (send *irtviewer* :objects) (list *drcv*)))
    (objects (list *drcv*))))


;; wait
(defun my-wait ()
  (warn ";; waiting~%")
  (read-line))
