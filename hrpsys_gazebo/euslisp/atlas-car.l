
(require "atlas-gallop-node.l")
(require "atlas-vehicle-motion.l")

(setq *move-step-time* 5000)

(defun real2model
  nil
  (send *robot* :angle-vector (send *ri* :state :potentio-vector)))

(defun model2real
  (&key (time *move-step-time*) (sleep-time time) (wait nil))
  (send *ri* :angle-vector (send *robot* :angle-vector) sleep-time)
  (when wait (send *ri* :wait-interpolation)))

(defun telep
  nil
  (init-pose :move -30)
  (send *robot* :legs :move-end-pos #f(0 0 70))
  (model2real_ 1000)
  (telep-start)
  (let*
      ((telep-coords (send *vehicle-side-pos* :copy-worldcoords))
       (telep-pos (send telep-coords :worldpos))
       (telep-rpy (car (rpy-angle (send telep-coords :rot)))))
    (setq
     telep-coords (make-coords
                   :pos
                   (float-vector
                    (elt telep-pos 0)
                    (- (elt telep-pos 1))
                    (elt telep-pos 2))
                   :rpy
                   (list
                    (- (elt telep-rpy 0))
                    (elt telep-rpy 1)
                    (- (elt telep-rpy 2)))))
    (send telep-coords :translate #f(-70.0 10.0 0.0) :local)
    (send telep-coords :rotate -pi/2 :z)
    (send
     *ri*
     :teleport-to
     (send
      (send telep-coords :copy-worldcoords)
      :translate
      #f(0.0 0.0 950.0)
      :world)
     nil
     :keep-effort
     t))
  (set-user)
  )

(defun crime-car-and-drive
  nil
  (telep)
  (send *robot* :reset-manip-pose)
  (send *robot* :torso :waist-p :joint-angle -90)
  (send *robot* :legs :move-end-pos #F(0 0 100))
  (model2real_ 100)
  (send *robot* :rarm :inverse-kinematics
        (make-coords :pos
                     (v+ (send *robot* :rarm :end-coords :worldpos) #F(-1000 5000 1000)))
        :rotation-axis nil
        :debug-view :no-message
        :stop 100
        :revert-if-fail nil)
  (send *robot* :larm :inverse-kinematics
        (make-coords :pos
                     (v+ (send *robot* :larm :end-coords :worldpos) #F(-1000 -5000 1000)))
        :rotation-axis nil
        :debug-view :no-message
        :stop 100
        :revert-if-fail nil)
  (model2real_ 1000)
  (send *robot* :larm :inverse-kinematics
        (make-coords :pos
                     (v+ (send *robot* :larm :end-coords :worldpos) #F(-1000 0 1000)))
        :rotation-axis nil
        :debug-view :no-message
        :stop 100
        :revert-if-fail nil)
  (send *robot* :rarm :inverse-kinematics
        (make-coords :pos
                     (v+ (send *robot* :rarm :end-coords :worldpos) #F(-1000 0 1000)))
        :rotation-axis nil
        :debug-view :no-message
        :stop 100
        :revert-if-fail nil)
  (model2real_ 1000)
  (send *robot* :angle-vector
        (scale 0 (send *robot* :angle-vector)))
  (model2real_)
  (init-pose :move -30)
  (send *robot* :legs :move-end-pos #f(0 0 70))
  (model2real_ 1000)
  (send *robot* :rarm :inverse-kinematics
        (make-coords :pos
                     (v+ (send *robot* :rarm :end-coords :worldpos) #F(1000 0 0)))
        :rotation-axis nil
        :debug-view :no-message
        :stop 100
        :revert-if-fail nil)
  (send *robot* :larm :inverse-kinematics
        (make-coords :pos
                     (v+ (send *robot* :larm :end-coords :worldpos) #F(1000 0 0)))
        :rotation-axis nil
        :debug-view :no-message
        :stop 100
        :revert-if-fail nil)
  (model2real_ 1000)
  (send *robot* :torso :waist-p :joint-angle 90)
  (do-until-key
   (mapcar
    #'(lambda (j)
        (let ((rand (random 1.0)))
          (send j :joint-angle
                (+ (* rand (send j :min-angle))
                   (* (- 1 rand) (send j :max-angle))))))
    (send *robot* :joint-list))
   (model2real_ 100)))


#|

(telep)

(send *robot* :reset-manip-pose)
(model2real_)

(send *robot* :torso :waist-p :joint-angle 90)
(model2real_)

(send *robot* :larm :inverse-kinematics
      (make-coords :pos
                   (v+ (send *robot* :larm :end-coords :worldpos) #F(-4000 -1000 500)))
      :rotation-axis nil
      :debug-view :no-message
      :revert-if-fail nil)
(model2real_)

(send *robot* :torso :waist-y :joint-angle -90)
(model2real_)

(send *robot* :rarm :inverse-kinematics
      (make-coords :pos
                   (v+ (send *robot* :rarm :end-coords :worldpos) #F(-1000 1000 500)))
      :rotation-axis nil
      :debug-view :no-message
      :revert-if-fail nil)
(model2real_)

(send *robot* :torso :waist-p :joint-angle 0)
(send *robot* :torso :waist-y :joint-angle 0)
(send *robot* :larm :inverse-kinematics
      (make-coords :pos
                   (v+ (send *robot* :larm :end-coords :worldpos) #F(-4000 0 2000)))
      :rotation-axis nil
      :debug-view :no-message
      :revert-if-fail nil)
(send *robot* :rarm :inverse-kinematics
      (make-coords :pos
                   (v+ (send *robot* :rarm :end-coords :worldpos) #F(-4000 0 2000)))
      :rotation-axis nil
      :debug-view :no-message
      :revert-if-fail nil)
(model2real)

(send *robot* :larm :inverse-kinematics
      (make-coords :pos
                   (v+ (send *robot* :larm :end-coords :worldpos) #F(0 1000 0)))
      :rotation-axis nil
      :debug-view :no-message
      :revert-if-fail nil)
(model2real_)

(send *robot* :rarm :inverse-kinematics
      (make-coords :pos
                   (v+ (send *robot* :rarm :end-coords :worldpos) #F(0 0 1000)))
      :rotation-axis nil
      :debug-view :no-message
      :revert-if-fail nil)
(model2real_)

(send *robot* :rarm :inverse-kinematics
      (make-coords :pos
                   (v+ (send *robot* :rarm :end-coords :worldpos) #F(0 -1000 0)))
      :rotation-axis nil
      :debug-view :no-message
      :revert-if-fail nil)
(model2real_)

(send *robot* :larm :inverse-kinematics
      (make-coords :pos
                   (v+ (send *robot* :larm :end-coords :worldpos) #F(4000 0 2000)))
      :rotation-axis nil
      :debug-view :no-message
      :revert-if-fail nil)
(send *robot* :rarm :inverse-kinematics
      (make-coords :pos
                   (v+ (send *robot* :rarm :end-coords :worldpos) #F(4000 0 2000)))
      :rotation-axis nil
      :debug-view :no-message
      :revert-if-fail nil)
(model2real)



;;
(telep)

(send *robot* :reset-manip-pose)
(send *robot* :torso :waist-p :joint-angle -90)
(send *robot* :legs :move-end-pos #F(0 0 100))
(model2real_ 100)

(send *robot* :rarm :inverse-kinematics
      (make-coords :pos
                   (v+ (send *robot* :rarm :end-coords :worldpos) #F(-1000 5000 1000)))
      :rotation-axis nil
      :debug-view :no-message
      :stop 100
      :revert-if-fail nil)
(send *robot* :larm :inverse-kinematics
      (make-coords :pos
                   (v+ (send *robot* :larm :end-coords :worldpos) #F(-1000 -5000 1000)))
      :rotation-axis nil
      :debug-view :no-message
      :stop 100
      :revert-if-fail nil)
(model2real_ 1000)

(send *robot* :larm :inverse-kinematics
      (make-coords :pos
                   (v+ (send *robot* :larm :end-coords :worldpos) #F(-1000 0 1000)))
      :rotation-axis nil
      :debug-view :no-message
      :stop 100
      :revert-if-fail nil)
(send *robot* :rarm :inverse-kinematics
      (make-coords :pos
                   (v+ (send *robot* :rarm :end-coords :worldpos) #F(-1000 0 1000)))
      :rotation-axis nil
      :debug-view :no-message
      :stop 100
      :revert-if-fail nil)
(model2real_ 1000)

(send *robot* :angle-vector
      (scale 0 (send *robot* :angle-vector)))
(model2real_)

(init-pose :move -30)
(send *robot* :legs :move-end-pos #f(0 0 70))
(model2real_ 1000)

(send *robot* :rarm :inverse-kinematics
      (make-coords :pos
                   (v+ (send *robot* :rarm :end-coords :worldpos) #F(1000 0 0)))
      :rotation-axis nil
      :debug-view :no-message
      :stop 100
      :revert-if-fail nil)
(send *robot* :larm :inverse-kinematics
      (make-coords :pos
                   (v+ (send *robot* :larm :end-coords :worldpos) #F(1000 0 0)))
      :rotation-axis nil
      :debug-view :no-message
      :stop 100
      :revert-if-fail nil)
(model2real_ 1000)

(send *robot* :torso :waist-p :joint-angle 90)

(do-until-key
 (mapcar
  #'(lambda (j)
      (let ((rand (random 1.0)))
        (send j :joint-angle
              (+ (* rand (send j :min-angle))
                 (* (- 1 rand) (send j :max-angle))))))
  (send *robot* :joint-list))
 (model2real_ 100))