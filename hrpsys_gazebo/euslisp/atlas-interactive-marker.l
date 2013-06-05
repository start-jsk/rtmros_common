#!/usr/bin/env roseus

(load "euslib/irteus_proposals/motion-lib-proposal.l")
;; (setq *cycle* 30) ;; Hz

(setq *cycle* 30) ;; Hz
(setq *cycle-times* 3.0) ;;
(setq *limit-angle* 4.5) ;; angle change per (* *cycle* *cycle-times*)

(defun robot-initial-setting ()
  (load "package://hrpsys_gazebo/euslisp/atlas-interface.l")
  ;; robot depending part
  (atlas-init)
  (send *atlas* :reset-pose)
  (send *atlas* :angle-vector
        (send *ri* :state :reference-vector))
  (send *ri* :set-interpolation-mode ":linear")
  (setq *using-robot* *atlas*)
  (setq larm-tip-link :l_hand)
  (setq rarm-tip-link :r_hand)
  (setq lleg-tip-link :l_foot)
  (setq rleg-tip-link :r_foot)
  (setq waist-link :utorso)
  (setq base_frame_link :pelvis)
  ;; /robot depending part
  )

(defun robot-reset-pose ()
  (send *atlas* :reset-pose)
  ;; (send *atlas* :fix-leg-to-coords (make-coords))
  (send *ri* :angle-vector (send *atlas* :angle-vector) 5000)
  )

;; over write functions
(defun _command-func-stop_robot (res_msg)
  (setq *current-mode* :stop)
  (ros::ros-info ";;; stop func ;;;")
  (let ((mkcds (make-coords))
        (mkframe "/pelvis")
        (av (send *using-robot* :angle-vector))
        markers)
    (send *using-robot* :angle-vector
          (send *ri* :state :potentio-vector))
    (case *current-limb*
      ;;(:rleg )
      ;;(:lleg )
      (:rarm
       (send mkcds :transform (send *using-robot* rarm-tip-link :worldcoords))
       (setq markers
             (list (mesh->marker-msg
                    (make-coords)
                    "package://hrpsys_gazebo/models/sandia_right.dae"
                    (instance std_msgs::Header :init :frame_id "/right_palm")
                    :color (float-vector 0 0 1)
                    :alpha 0.6
                    ))))
      (:larm
       (send mkcds :transform (send *using-robot* larm-tip-link :worldcoords))
       (setq markers
             (list (mesh->marker-msg
                    (make-coords)
                    "package://hrpsys_gazebo/models/sandia_left.dae"
                    (instance std_msgs::Header :init :frame_id "/left_palm")
                    :color (float-vector 0 0 1)
                    :alpha 0.6
                    ))))
      )
    (when mkframe
      (ros::ros-info "set-marker-pose")
      (set-marker-pose mkcds :frame mkframe :markers markers))
    (send *using-robot* :angle-vector av)
    ))

;; execute main program
(load "package://interactive_marker_interface/euslisp/interactive-marker-fullbody.l")
