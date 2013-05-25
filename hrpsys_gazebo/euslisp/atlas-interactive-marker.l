#!/usr/bin/env roseus

(load "/home/jskuser/prog/euslib/irteus_proposals/motion-lib-proposal.l")
;; (setq *cycle* 30) ;; Hz

(setq *cycle* 30) ;; Hz
(setq *cycle-times* 3.0) ;;
(setq *limit-angle* 4.5) ;; angle change per (* *cycle* *cycle-times*)

(defun robot-initial-setting ()
  (load "package://hrpsys_gazebo/euslisp/atlas-interface.l")
  ;; robot depending part
  (atlas-init)
  (send *atlas* :reset-pose)
  (send *ri* :angle-vector (send *atlas* :angle-vector) 5000) ;; go to initial pose
  (send *ri* :set-interpolation-mode ":linear") ;; prevent hrp4r to interplorate invalid qRef
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

;; execute main program
(load "package://interactive_marker_interface/euslisp/interactive-marker-fullbody.l")
