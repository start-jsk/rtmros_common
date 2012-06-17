#!/usr/bin/env roseus

(ros::load-ros-manifest "image_view2")
(ros::load-ros-manifest "ar_pose")
(load "models/arrow-object.l")

(load "package://hrpsys_ros_bridge/scripts/rtm-ros-robot-interface.l")
(require :kawadahironx "package://hrpsys_ros_bridge/models/kawada-hironx.l")

(ros::roseus "ar_pose_clinet")

(defclass kawadahironx-interface
  :super rtm-ros-robot-interface
  :slots ())
(defmethod kawadahironx-interface
  (:init (&rest args)
	  (send-super :init :robot hironx-robot)))

(setq *ri* (instance kawadahironx-interface :init))
(setq *hiro* (instance hironx-robot :init))


(setq *tfb* (instance ros::transform-broadcaster :init))
(setq *tfl* (instance ros::transform-listener :init))

(defun ar-pose-cb (msg)
  (let ((mrk (instance image_view2::ImageMarker2 :init)) c)
    (dolist (marker (send msg :markers))
      (setq c (ros::tf-pose->coords (send marker :pose :pose)))
      (send mrk :type image_view2::ImageMarker2::*FRAMES*)
      (send mrk :frames (list "/ar_marker_frame"))
      (send *tfb* :send-transform
	    c (send marker :header :frame_id) "/ar_marker_frame"
	    (send marker :header :stamp))
      (ros::ros-info "~A" c)
      (ros::publish "image_marker" mrk)
      )))

(ros::advertise "image_marker" image_view2::ImageMarker2 1)
(ros::subscribe "ar_pose_marker" ar_pose::ARMarkers #'ar-pose-cb)

(ros::rate 10)
(setq *arrow* (arrow))
(send *arrow* :locate #f(0 0 2000))
(objects (list (make-cube 1000 1000 1) *arrow* *hiro*))
(do-until-key
  (ros::spin-once)
  (x::window-main-one)
  (when (send *tfl* :can-transform "WAIST" "/ar_marker_frame" (ros::time 0))
    (send *arrow* :newcoords (send *tfl* :lookup-transform "WAIST" "/ar_marker_frame" (ros::time 0)))
    (setq pos (send *arrow* :worldpos))
    (print pos)
    ;; (send *hiro* :head :look-at pos) .. now working now...
    (if (> (elt pos 1) 0)
        (send *hiro* :head :neck-y :joint-angle 3 :relative t) ;; t . nil
      (send *hiro* :head :neck-y :joint-angle -3 :relative t))
    )
  (send *irtviewer* :draw-objects)
  ;;(send *ri* :angle-vector (send *hiro* :angle-vector) 500)
  ;;(send *ri* :wait-interpolation)
  (ros::sleep)
  )

