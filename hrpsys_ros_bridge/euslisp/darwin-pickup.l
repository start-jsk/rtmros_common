(load "package://hrpsys_ros_bridge/scripts/rtm-ros-robot-interface.l")
(require :darwin "package://hrpsys_ros_bridge/models/darwin.l")

(defclass darwin-interface
  :super rtm-ros-robot-interface
  :slots ())
(defmethod darwin-interface
  (:init (&rest args)
	  (send-super :init :robot darwin-robot)))

(defun darwin-init ()
  (setq *ri* (instance darwin-interface :init))
  (setq *darwin* (instance darwin-robot :init))
  (send *darwin* :reset-pose)
  (make-irtviewer :width 320 :height 320)
  (objects (list *darwin*))
  (send *ri* :angle-vector (send *darwin* :angle-vector) 1000)
  (send *ri* :wait-interpolation))
;;
(defun darwin-walk ()
  (let ((walk-data
	 (with-open-file (f (format nil "~A/scripts/darwin-walk-data.l" (ros::resolve-ros-path "package://hrpsys_ros_bridge")) :direction :input)
			 (read f))))
    (send *ri* :set-interpolation-mode :linear)
    (send *ri* :angle-vector-sequence
	  (mapcar #'car walk-data)
	  (mapcar #'cadr walk-data))
    (send *ri* :wait-interpolation)))

(defun darwin-fullbody-pickup ()
  (send *darwin* :reset-pose)
  (send *darwin* :newcoords (make-coords))
  (let* ((limbs '(:rarm :rleg :lleg))
         (move-target (mapcar #'(lambda (l) (send *darwin* l :end-coords)) limbs))
         (link-list (mapcar #'(lambda (mt)
                                (send *darwin* :link-list (send mt :parent)))
                            move-target))
         (fix-leg-coords (list (send *darwin* :rleg :end-coords :copy-worldcoords)
                               (send *darwin* :lleg :end-coords :copy-worldcoords))))
    ;; fullbody-inverse-kinematics requires parameters for rarm, rleg, and lleg
    (send *darwin* :fullbody-inverse-kinematics
          (append (list (make-coords :pos #f(80 -80 120))) fix-leg-coords)
          :link-list link-list :move-target move-target
          :rotation-axis (list nil t t) :thre '(5 2 2)
          :look-at-target t
          :cog-gain 2.5 :stop 300 :centroid-thre 10
          :debug-view :no-message))
  (send *ri* :angle-vector (send *darwin* :angle-vector) 3000)
  (send *ri* :wait-interpolation)
  )
;;
(defun darwin-arm-up-pose ()
  (send *darwin* :reset-pose)
  (send *darwin* :newcoords (make-coords))
  (send *darwin* :rarm :shoulder-p :joint-angle 40)
  (send *darwin* :rarm :elbow-p :joint-angle -130)
  (send *darwin* :head :look-at (send *darwin* :rarm :end-coords :worldpos))
  (objects (list *darwin*))
  (send *ri* :angle-vector (send *darwin* :angle-vector) 1000)
  (send *ri* :wait-interpolation)
  ;;
  (send *darwin* :rarm :shoulder-p :joint-angle -65)
  (send *darwin* :rarm :shoulder-r :joint-angle -60)
  (send *darwin* :rarm :elbow-p :joint-angle -40)
  (send *darwin* :head :look-at (send *darwin* :rarm :end-coords :worldpos))
  (objects (list *darwin*))
  (send *ri* :angle-vector (send *darwin* :angle-vector) 1000)
  (send *ri* :wait-interpolation))

(ros::roseus "darwin_pickup")
(darwin-init)
(darwin-walk)
(darwin-fullbody-pickup)
(darwin-arm-up-pose)
