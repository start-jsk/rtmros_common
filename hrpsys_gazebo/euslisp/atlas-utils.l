;; init
(setq *lr-reverse* nil)

(defun my-init
  (&key (real t)
	(set-reset-pose t)
	(servo-gain
	 (list (cons :legs 1.0)
	       (cons :torso 2.0)
	       (cons :arms 4.0)
	       (cons :head 5.0))))
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
    (mapcar
     #'(lambda (k)
	 (atlas-set-servo-gain-by-torque-limit
	  (cdr k) (car k)))
     servo-gain)
    (set-hand-kp 50.0))
  (setq *robot* *atlas*)
  (objects (list *atlas*))
  t)

(defun bar-init ()
  (load "euslib/demo/s-noda/atlas/robot-pose.l"))

(defun stand-up ()
  (send *ri* :teleport-to))


;; hand

;; (send *ri* :hand :larm :send_query "start")
;; (send *ri* :hand :larm :send_query "stop")

(defun hand-model2real (&key (arm :both) (wait nil) (time 1000))
  (cond
   ((equal arm :both) (hand-model2real :arm :larm :wait wait) (hand-model2real :arm :rarm :wait wait))
   (t (set-real-hand-av (get-model-hand-av arm) arm time)
      (when wait (send *ri* :hand arm :wait-interpolation)))))

(defun hand-real2model (&key (arm :both))
  (cond
   ((equal arm :both) (hand-real2model :arm :larm) (hand-real2model :arm :rarm))
   (t (set-model-hand-av (get-real-hand-av arm) arm))))

(defun set-real-hand-av (av &optional (arm :both) (time 1000))
  (when *lr-reverse* (case arm (:larm (setq arm :rarm)) (:rarm (setq arm :larm))))
  (send *ri* :hand arm :angle-vector av time))

(defun get-real-hand-av (&optional (arm :both))
  (when *lr-reverse* (case arm (:larm (setq arm :rarm)) (:rarm (setq arm :larm))))
  (send *ri* :hand arm :state :potentio-vector))

(defun set-model-hand-av (av &optional (arm :both))
  (send *atlas* :hand arm :angle-vector av))

(defun get-model-hand-av (&optional (arm :both))
  (send *atlas* :hand arm :angle-vector))

(defun set-hand-kp (kp &optional (arm :both))
  (when *lr-reverse* (case arm (:larm (setq arm :rarm)) (:rarm (setq arm :larm))))
  (send *ri* :hand arm :send-command :pgain kp))

(defun grasp (&key (arm :both) (real nil))
  (set-model-hand-av #f(0 90 90 0 90 90 0 90 90 0 20 90) arm)
  (when real (hand-model2real))
  )

(defun pregrasp (&key (arm :both) (real nil))
  (set-model-hand-av #f(0 70 0 0 70 0 0 70 0 0 0 0) arm)
  (when real (hand-model2real))
  )

(defun ungrasp (&key (arm :both) (real nil))
  (set-model-hand-av #f(0 0 0 0 0 0 0 0 0 0 0 0) arm)
  (when real (hand-model2real))
  )


;; sim mode
(defun set-user (&optional (e 255))
  (warning-message 1 "[altas-utils] set user-mode~%")
  (send *ri* :set-simulation-mode :mode :user :effort e))

(defun set-stand (&optional (e 0))
  (warning-message 1 "[altas-utils] set stand-mode~%")
  (send *ri* :set-simulation-mode :mode :stand :effort e))

(defun set-manip (&optional (e 255))
  (warning-message 1 "[altas-utils] set manip-mode~%")
  (send *ri* :set-simulation-mode :mode :manipulate :effort e))

(defun set-walk (&optional (e 0))
  (warning-message 1 "[altas-utils] set walk-mode~%")
  (send *ri* :set-simulation-mode :mode :walk :effort e))

(defun set-step (&optional (e 0))
  (warning-message 1 "[altas-utils] set step-mode~%")
  (send *ri* :set-simulation-mode :mode :step :effort e))

(defun get-mode ()
  (send *ri* :get-control-mode))

(defun set-effort (&optional (e 0) (limb :all))
  (warning-message 1 "[altas-utils] set effort <~a> <~a>~%" e limb)
  (when *lr-reverse*
    (case limb (:larm (setq limb :rarm)) (:rarm (setq limb :larm)) (:lleg (setq limb :rleg)) (:rleg (setq limb :lleg))))
  (send *ri* :set-k-effort e limb))

(defun get-effort (&optional limb)
  (send *ri* :get-k-effort limb))

(defun set-kp 
  (&optional (kp 100.0) (limb :all))
  (warning-message 1 "[altas-utils] set kp <~a> <~a>~%" kp limb)
  (when *lr-reverse*
    (case limb (:larm (setq limb :rarm)) (:rarm (setq limb :larm)) (:lleg (setq limb :rleg)) (:rleg (setq limb :lleg))))
  (cond
   ((equal limb :arms) (set-kp kp :larm) (set-kp kp :rarm))
   ((equal limb :legs) (set-kp kp :lleg) (set-kp kp :rleg))
   (t (dolist (j (if (equal limb :all) (send-all (send *atlas* :joint-list) :name)
		   (send-all (send *atlas* limb :joint-list) :name)))
	(send *ri* :set-servo-gain-percentage j kp)))))

(defun set-high-gain (&optional (limb :all))
  (warning-message 1 "[altas-utils] set-high-gain ~a~%" limb)
  (when *lr-reverse*
    (case limb (:larm (setq limb :rarm)) (:rarm (setq limb :larm)) (:lleg (setq limb :rleg)) (:rleg (setq limb :lleg))))
  (atlas-set-servo-gain-by-torque-limit 1.0 limb))

(defun set-middle-gain (&optional (limb :all))
  (warning-message 1 "[altas-utils] set-middle-gain ~a~%" limb)
  (when *lr-reverse*
    (case limb (:larm (setq limb :rarm)) (:rarm (setq limb :larm)) (:lleg (setq limb :rleg)) (:rleg (setq limb :lleg))))
  (atlas-set-servo-gain-by-torque-limit 7.5 limb))

(defun set-low-gain (&optional (limb :all))
  (warning-message 1 "[altas-utils] set-low-gain ~a~%" limb)
  (when *lr-reverse*
    (case limb (:larm (setq limb :rarm)) (:rarm (setq limb :larm)) (:lleg (setq limb :rleg)) (:rleg (setq limb :lleg))))
  (atlas-set-servo-gain-by-torque-limit 30.0 limb))

(defun set-zero-gain (&optional (limb :all))
  (warning-message 1 "[altas-utils] set-zero-gain ~a~%" limb)
  (when *lr-reverse*
    (case limb (:larm (setq limb :rarm)) (:rarm (setq limb :larm)) (:lleg (setq limb :rleg)) (:rleg (setq limb :lleg))))
  (atlas-set-servo-gain-by-torque-limit 1000.0 limb))

(defun manip (&key (smooth nil))
  (warning-message 1 "[altas-utils] set mode and effort for manipulation~%")
  (set-stand nil)

  (when smooth
    ;; (set-effort 0 :torso)
    ;; (set-middle-gain :legs)
    (let ((start-effort (car (get-effort :lleg))))
      (dotimes (i 20)
        (set-effort (* (/ (- 20 i) 20.0) start-effort) :legs))))
  (set-effort 0 :legs)
  (set-effort 255 :torso)
  (set-effort 255 :arms)
  (set-effort 255 :head))


;; mode
(defun set-pinned ()
  (warning-message 1 "[altas-utils] [cheating!!] set pinned~%")
  (send *ri* :atlas-mode "pinned"))

(defun set-pinned-wg ()
  (warning-message 1 "[altas-utils] [cheating!!] set pinned-with-gravity~%")
  (send *ri* :atlas-mode "pinned_with_gravity"))

(defun set-nominal ()
  (warning-message 1 "[altas-utils] [cheating!!] set nominal~%")
  (send *ri* :atlas-mode "nominal"))

(defun set-feet ()
  (warning-message 1 "[altas-utils] [cheating!!] set feet~%")
  (send *ri* :atlas-mode "feet"))

(defun set-harnessed ()
  (warning-message 1 "[altas-utils] [cheating!!] set harnessed~%")
  (send *ri* :atlas-mode "harnessed"))

;; angle vector
(defun real2model ()
  (send *atlas* :angle-vector (send *ri* :state :potentio-vector))
  (when *lr-reverse* (mirror-angle-vector)))

(defun model2real (&key (sleep-time 1500) (time sleep-time) (wait nil))
  (when *lr-reverse* (mirror-angle-vector))
  (send *ri* :angle-vector (send *atlas* :angle-vector) time)
  (when *lr-reverse* (mirror-angle-vector))
  (when wait (send *ri* :wait-interpolation)))

(defun model2real-with-rand-vib (&key (sleep-time 1500) (time sleep-time) (wait nil))
  (model2real :sleep-time time :wait wait))
  ;; (when *lr-reverse* (mirror-angle-vector))
  ;; (let* ((prev-av (send *ri* :state :potentio-vector))
  ;;        (dest-av (send *atlas* :angle-vector))
  ;;        (ik-time 100)
  ;;        (ik-num (/ time ik-time))
  ;;        (rand-angle 0.0) rand-av)
  ;;   (dotimes (i ik-num)
  ;;     (setq rand-av
  ;;           (concatenate float-vector
  ;;                        (mapcar #'(lambda (x) (- x (/ rand-angle 2)))
  ;;                                (mapcar #'random (list rand-angle rand-angle rand-angle rand-angle rand-angle rand-angle)))))
  ;;     (send *atlas* :angle-vector (midpoint (/ (+ i 0.0) ik-num) prev-av dest-av))
  ;;     (send *atlas* :lleg :angle-vector (v+ (send *atlas* :lleg :angle-vector) rand-av))
  ;;     (send *ri* :angle-vector (send *atlas* :angle-vector) ik-time)
  ;;     (send *ri* :wait-interpolation))
  ;;   (send *atlas* :angle-vector dest-av)
  ;;   (send *ri* :angle-vector (send *atlas* :angle-vector) ik-time))
  ;; (when *lr-reverse* (mirror-angle-vector))
  ;; (when wait (send *ri* :wait-interpolation)))

(defun reset-pose (&key (real nil))
  (send *atlas* :stand-pose)
  (send *atlas* :larm :elbow-p :joint-angle -5 :relative t)
  (send *atlas* :rarm :elbow-p :joint-angle 5 :relative t)
  (send *atlas* :head :neck-p :joint-angle 0) 
  ;; (move-centroid)
  (when real (model2real)))


;; angle vector util
(defun mirror-angle-vector
  ()
  (let* ((larm-av (send *atlas* :larm :angle-vector))
	 (rarm-av (send *atlas* :rarm :angle-vector))
	 (lleg-av (send *atlas* :lleg :angle-vector))
	 (rleg-av (send *atlas* :rleg :angle-vector))
	 (torso-av (send *atlas* :torso :angle-vector))
	 (arm-convert-v (float-vector -1 1 -1 -1 1 1))
	 (leg-convert-v (float-vector 1 -1 1 -1 1 -1))
	 (torso-convert-v (float-vector -1 1 -1)))
    (send *atlas* :rarm :angle-vector (map float-vector #'* arm-convert-v larm-av))
    (send *atlas* :larm :angle-vector (map float-vector #'* arm-convert-v rarm-av))
    (send *atlas* :rleg :angle-vector (map float-vector #'* leg-convert-v lleg-av))
    (send *atlas* :lleg :angle-vector (map float-vector #'* leg-convert-v rleg-av))
    (send *atlas* :torso :angle-vector (map float-vector #'* torso-convert-v torso-av))
    ))

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

(defun save-last ()
  (setq *last-atlas-worldcoords* (send (send *atlas* :worldcoords) :copy-worldcoords))
  (setq *last-atlas-av* (send *atlas* :angle-vector)))

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
  (:reset-manip-pose
   (&key (base-down nil))
   (send self :reset-pose)
   (send self :fix-leg-to-coords (make-coords))
   (if base-down (send self :legs :move-end-pos #f(0 0 70) :world))
   (send self :fix-leg-to-coords (make-coords))
   (send self :larm :elbow-p :joint-angle 110)
   (send self :rarm :elbow-p :joint-angle -110)
   (send self :rarm :wrist-p :joint-angle -30)
   (send self :larm :wrist-p :joint-angle 30)
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


;; wait
(defun my-wait ()
  (warn ";; waiting~%")
  (read-line))

(defun static-transition-to-stand ()
  (send *ri* :set-simulation-mode :mode :stand :effort 0)
  (send *ri* :bdi_step_static
        (make-coords :pos (float-vector 0 235 0) :rpy (list 0.15 0 0))
        :l/r :lleg)
  (send *ri* :wait-for-start-behavior 6)
  (send *ri* :wait-for-finish-behavior 6)
  (send *ri* :bdi_step_static
        (make-coords :pos (float-vector 0 -235 0) :rpy (list -0.15 0 0))
        :l/r :rleg)
  (send *ri* :wait-for-start-behavior 6)
  (send *ri* :wait-for-finish-behavior 6)
  )

(defun interpolated-mode-change (&key (e-start 0) (e-end 255) (mode :stand)
                                  (mask) ((:time tm) 3000))
  (let ((step (/ tm (float (abs (- e-end e-start))))))
    (dotimes (i (abs (- e-end e-start)))
      (warn ";; effort = ~d~%"
            (if (> e-end e-start) (+ e-start i 1) (- e-start i 1)))
      (send *ri* :set-simulation-mode :mode mode
            :effort (if (> e-end e-start) (+ e-start i 1) (- e-start i 1))
            :sleep (/ step 1000.0)
            :effort-mask mask)
      )
    ))
;;(interpolated-mode-change :e-start 0 :e-end 20 :mode :user :time 6000)
;;(interpolated-mode-change :e-start 20 :e-end 100 :mode :user :time 6000)
;;(interpolated-mode-change :e-start 20 :e-end 255 :mode :user :time 6000)

(defun manipulation-mode (&key (height 0.8) (yaw 0.0) (lat 0.0))
  (ros::spin-once)
  (unless (eq (get-mode) :manipulate)
    (real2model)
    ;; mask upper body
    (let ((av (send *atlas* :angle-vector))
          tmpl tmpr)
      (send *atlas* :stand-pose)
      (setq tmpl (send *atlas* :lleg :angle-vector))
      (setq tmpr (send *atlas* :rleg :angle-vector))
      (send *atlas* :angle-vector av)
      (send *atlas* :lleg :angle-vector tmpl)
      (send *atlas* :rleg :angle-vector tmpr)
      )
    (model2real :wait t :time 5000)
    (send *ri* :set-simulation-mode :mode :stand
          :effort 255 :effort-mask *manip-mask-vector*)
    (unix::usleep (* 6 1000 1000)) ;; wait 6 sec
    )
  (send *ri* :set-simulation-mode :mode :manipulate
        :effort 255 :effort-mask *manip-mask-vector*
        :use_desired t
        :pelvis_height height :pelvis_yaw yaw :pelvis_lat lat)
  t)

(defun manip-to-stand-mode (&optional (pose-clear nil))
  (send *ri* :set-simulation-mode :mode :manipulate
        :effort 255 :effort-mask *manip-mask-vector*
        :use_desired t
        :pelvis_height 0.9 :pelvis_yaw 0 :pelvis_lat 0)
  (unix::usleep (* 6 1000 1000)) ;; wait 6 sec
  (send *ri* :set-simulation-mode :mode :stand
        :effort 255 :effort-mask *manip-mask-vector*)
  (when pose-clear
    (send *atlas* :stand-pose)
    (model2real :wait t :time 5000)
    (send *ri* :set-simulation-mode :mode :stand :effort 0))
  t
  )

;; 
(defun snapshot-image (&optional (cam :head))
  (let ((req (instance std_srvs::EmptyRequest :init)))
    (if (or (eq cam :head) (eq cam :all))
        (ros::service-call "/head_snap/snapshot" req))
    (if (or (eq cam :arms) (eq cam :all) (eq cam :rarm))
        (ros::service-call "/rhand_snap/snapshot" req))
    (if (or (eq cam :arms) (eq cam :all) (eq cam :larm))
        (ros::service-call "/lhand_snap/snapshot" req))
    ))
;; clear octomap
(defun clear-octomap ()
  (let ((req (instance std_srvs::EmptyRequest :init)))
    (ros::service-call "/octomap_full_server/reset" req)))

;; get-octomap
(defun get-octomap (&optional (wait t) (clear t) (remove-nan))
  (when clear (clear-octomap))
  (when wait (get-tilt-laser))
  ;; (get-tilt-laser)
  (let ((msg (one-shot-subscribe "/octomap_point_cloud_centers"
                                 sensor_msgs::PointCloud2 :timeout 30000)))
    (when msg
      (let ((ret (make-eus-pointcloud-from-ros-msg msg :remove-nan remove-nan)))
        (setf (get ret :frame_id) (send msg :header :frame_id))
        (setf (get ret :stamp) (send msg :header :stamp))
        (setf (get ret :point_step) (send msg :point_step))
        ret
        ))))

;; get-full-cloud
(defun get-full-cloud (&optional (wait t))
  (when wait (get-tilt-laser))
  (get-tilt-laser))

;; (send pt :filter :key #'(lambda (p) (and (> 700 (elt p 0) -700) (> 1400 (elt p 1) 400) (> 700 (elt p 2) -300))) :create t)
;; (pcl::downsample pt 25.0 25.0 25.0)


(defun safety-fall-down nil
  ;; (let* ((tc (list (send (send *atlas* :rleg :end-coords :copy-worldcoords) :translate #f(0 0 450) :world)
  ;; 		   (send (send *atlas* :lleg :end-coords :copy-worldcoords) :translate #f(0 0 450) :world)
  ;; 		   nil
  ;; 		   nil)))
  ;;   (with-move-target-link-list
  ;;    (mt ll *atlas* '(:rleg :lleg :rarm :larm))
  ;;    (send *atlas* :fullbody-inverse-kinematics
  ;; 	   (mapcar #'(lambda (c m) (if c c (send m :copy-worldcoords))) tc mt)
  ;; 	   :link-list ll :move-target mt
  ;; 	   :collision-avoidance-link-pair nil
  ;; 	   :stop 1000
  ;; 	   :translation-axis '(t t t t)
  ;; 	   :rotation-axis '(t t t t)
  ;; 	   :thre '(5 5 10 10)
  ;; 	   :rthre (list (deg2rad 2) (deg2rad 2) (deg2rad 2) (deg2rad 2))
  ;; 	   :target-centroid-pos (send *atlas* :calc-target-centroid-pos :both '(:larm :lleg))
  ;; 	   :centroid-thre 500))
  ;;   )

  (send *atlas* :angle-vector #f(33.5356 46.4636 14.0039 64.2606 95.3945 -24.981 -79.9998 -25.5105 15.0798 -23.0551 150.466 -40.8152 62.3277 6.39616 -79.6342 133.101 2.05166 -69.9106 -0.370845 -51.5662 13.6866 133.288 5.99313 -72.0902 -10.4833 -50.4458 -16.8264 -35.0))
  (progn (model2real :time 5000)(send *ri* :wait-interpolation))

  ;; (send *atlas* :larm :inverse-kinematics
  ;; 	(make-coords :pos #f(-100 400 -250))
  ;; 	:debug-view t :thre 10 :stop 200 :rotation-axis nil)

  ;; (send *atlas* :rarm :inverse-kinematics
  ;; 	(make-coords :pos #f(50 -200 -250))
  ;; 	:debug-view t :thre 10 :stop 200 :rotation-axis nil)

  ;; (send *atlas* :move-centroid-on-foot
  ;; 	:both '(:rleg :lleg )
  ;; 	:target-centroid-pos (send *atlas* :calc-target-centroid-pos :both '(:rleg :lleg))
  ;; 	:centroid-thre 40)

  (send *atlas* :angle-vector #f(33.5356 46.4636 14.0039 64.2606 95.3945 -24.981 -79.9998 -25.5105 15.0798 -26.4997 154.671 -17.5356 69.7715 -18.2152 -72.1545 133.003 -5.39242 -77.1493 6.91985 -44.144 14.013 134.584 -4.13608 -76.1416 -0.52805 -47.872 -16.8734 -35.0))
  (progn (model2real :time 3000) (send *ri* :wait-interpolation))

  ;;(send *atlas* :rleg :knee-p :joint-angle 141)
  ;;(send *atlas* :lleg :knee-p :joint-angle 141)
  ;;(send *atlas* :rleg :ankle-p :joint-angle -20)
  ;;(send *atlas* :lleg :ankle-p :joint-angle -20)

  (send *atlas* :angle-vector #f(33.5356 46.4636 14.0039 64.2606 95.3945 -24.981 -79.9998 -25.5105 15.0798 -26.4997 154.671 -17.5356 69.7715 -18.2152 -72.1545 140.375 -5.39242 -77.1493 6.91985 -20.0 14.013 140.375 -4.13608 -76.1416 -0.52805 -20.0 -16.8734 -35.0))
  (progn (model2real :time 5000) (send *ri* :wait-interpolation))
)