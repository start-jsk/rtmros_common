(load "package://hrpsys_ros_bridge/scripts/rtm-ros-robot-interface.l")
(require :hrp4c "package://hrpsys_ros_bridge/models/hrp4c.l")

(defclass hrp4c-interface
  :super robot-interface
  :slots ())
(defmethod hrp4c-interface
  (:init (&rest args)
	  (send-super :init :robot hrp4-robot)))

(defun hrp4c-init ()
  (if (not (boundp '*ri*))
      (setq *ri* (instance hrp4c-interface :init)))
  (if (not (boundp '*hrp4c*))
      (setq *hrp4c* (instance hrp4-robot :init)))
  (send *hrp4c* :reset-pose)
  ;; send reset pose to real robot
  (send *ri* :angle-vector (send *hrp4c* :angle-vector) 3000)
  (send *ri* :wait-interpolation)
  )

(defun hrp4c-lookdown ()
  ;; reset manip pose
  (send *hrp4c* :torso :waist-y :joint-angle -30)
  (send *hrp4c* :arms :shoulder-p :joint-angle 40)
  (send *hrp4c* :arms :elbow-p :joint-angle -90)
  (send *hrp4c* :head :neck-p :joint-angle -30)
  (send *ri* :angle-vector (send *hrp4c* :angle-vector) 3000)
  (send *ri* :wait-interpolation)
  )

(defun hrp4c-pickup ()
  (send *hrp4c* :inverse-kinematics
	(make-coords :pos #f(400 -200 600))
	:link-list (send *hrp4c* :link-list (send *hrp4c* :rarm :end-coords :parent))
	:move-target (send *hrp4c* :rarm :end-coords)
	:rotation-axis nil
	:look-at-target t
	:debug-view t)
  (send *ri* :angle-vector (send *hrp4c* :angle-vector) 3000)
  (send *ri* :wait-interpolation)
  )

(defun hrp4c-fullbody-pickup ()
  (let* ((limbs '(:rarm :rleg :lleg))
         (move-target (mapcar #'(lambda (l) (send *hrp4c* l :end-coords)) limbs))
         (link-list (mapcar #'(lambda (mt)
                                (send *hrp4c* :link-list (send mt :parent)))
                            move-target))
         (fix-leg-coords (list (send *hrp4c* :rleg :end-coords :copy-worldcoords)
                               (send *hrp4c* :lleg :end-coords :copy-worldcoords))))
    (send *hrp4c* :reset-pose)
    ;; fullbody-inverse-kinematics requires parameters for rarm, rleg, and lleg
    (send *hrp4c* :fullbody-inverse-kinematics
          (append (list (make-coords :pos #f(250 -200 170))) fix-leg-coords)
          :link-list link-list :move-target move-target
          :rotation-axis (list nil t t) :thre '(5 1 1)
          :look-at-target t
          :cog-gain 2.5 :stop 200 :centroid-thre 20
          :debug-view t))
  (send *ri* :angle-vector (send *hrp4c* :angle-vector) 3000)
  (send *ri* :wait-interpolation)
  )

(ros::roseus "hrp4c_pickup")
(hrp4c-init)
(hrp4c-lookdown)
;;(hrp4c-pickup)
(hrp4c-fullbody-pickup)
(hrp4c-init)
