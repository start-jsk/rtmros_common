#!/usr/bin/env roseus

;; rtmlaunch hrpsys_ros_bridge samplerobot.launch hrpsys_precreate_rtc:=PDcontroller USE_UNSTABLE_RTC:=true PROJECT_FILE:=`rospack find hrpsys`/share/hrpsys/samples/SampleRobot/SampleRobot.carryobject.xml
(load "package://hrpsys_ros_bridge/euslisp/samplerobot-interface.l")

(defun samplerobot-carry-object-init ()
  ;; Initialize models
  (samplerobot-init)
  (setq *robot* *sr*)
  (send *robot* :reset-pose)
  (send *robot* :fix-leg-to-coords (make-coords))
  (send *robot* :arms :move-end-pos #f(0 0 200) :world :rotation-axis nil)
  (objects (list *robot*))
  (setq *initial-pose* (send *robot* :angle-vector))
  (send *Ri* :angle-vector (Send *robot* :angle-vector) 2000)
  (send *ri* :wait-interpolation)
  ;; Initialize controller params
  (send *ri* :set-gait-generator-param :stride-parameter (float-vector 0.15 0.08 25.0 0.15))
  (send *ri* :set-st-param :is-ik-enable (list t t t t))
  (send *ri* :start-default-unstable-controllers :abc-limbs '(:rleg :lleg :rarm :larm))
  (send *ri* :set-impedance-controller-param :arms :D-p 400 :K-r 1e5 :D-r 1e5)
  (send *ri* :stop-collision-detection)
  (send *ri* :load-forcemoment-offset-params (format nil "~A/samples/SampleRobot/ForceSensorOffset_SampleRobot.txt" (ros::resolve-ros-path "package://hrpsys")))
  )

(defun object-hold-walk ()
  (send *ri* :go-pos -0.3 -0.1 0)
  (send *ri* :go-pos 0 0 30)
  (send *ri* :go-pos 0 0 -30)
  (send *ri* :go-pos 0.3 0.1 0)
  )

(defun samplerobot-carry-object-demo0 (&key (walk-p t) (auto-detect-p t))
  "1. Dual-arm carry up"
  (let ((r-handle-coords (make-coords :pos (float-vector 350 -220 870) :rpy (list 0 0 pi/2)))
        (l-handle-coords (make-coords :pos (float-vector 350  220 870) :rpy (list 0 0 -pi/2))))
    (send *robot* :reset-manip-pose)
    (send *robot* :fix-leg-to-coords (make-coords))
    ;; Via coords
    (send *robot* :fullbody-inverse-kinematics
          (list (send (send r-handle-coords :copy-worldcoords) :translate (float-vector -150 0 -30) :world)
                (send (send l-handle-coords :copy-worldcoords) :translate (float-vector -150 0 -30) :world)
                (send *robot* :rleg :end-coords :copy-worldcoords)
                (send *robot* :lleg :end-coords :copy-worldcoords))
          :move-target (mapcar #'(lambda (x) (send *robot* x :end-coords)) '(:rarm :larm :rleg :lleg))
          :link-list (mapcar #'(lambda (x) (send *robot* :link-list (send *robot* x :end-coords :parent))) '(:rarm :larm :rleg :lleg))
          :cog-null-space nil
          :use-6dof-joint-leg-gain nil)
    (let ((reaching-via-pose (send *robot* :angle-vector)) (reaching-pose))
      (send *Ri* :angle-vector (Send *robot* :angle-vector) 2000)
      (send *ri* :wait-interpolation)
      ;; Reach pose
      (send *robot* :fullbody-inverse-kinematics
            (list r-handle-coords
                  l-handle-coords
                  (send *robot* :rleg :end-coords :copy-worldcoords)
                  (send *robot* :lleg :end-coords :copy-worldcoords))
            :move-target (mapcar #'(lambda (x) (send *robot* x :end-coords)) '(:rarm :larm :rleg :lleg))
            :link-list (mapcar #'(lambda (x) (send *robot* :link-list (send *robot* x :end-coords :parent))) '(:rarm :larm :rleg :lleg))
            :cog-null-space nil
            :use-6dof-joint-leg-gain nil)
      (setq reaching-pose (send *robot* :angle-vector))
      (send *Ri* :angle-vector (Send *robot* :angle-vector) 2000)
      (send *ri* :wait-interpolation)
      (if auto-detect-p
          (send *ri* :set-object-turnaround-ref-force :max-ref-force (* 9.8 6.0))
        (progn
          (send *ri* :set-ref-force (float-vector 0 0 (* 9.8 2.5 -1)) 2000 :arms)
          (send *ri* :wait-interpolation-seq)))
      ;; Lift up
      (send *robot* :fullbody-inverse-kinematics
            (list (send (send (send r-handle-coords :copy-worldcoords) :translate (float-vector 0 0 50) :world) :rotate (deg2rad 5) :z)
                  (send (send (send l-handle-coords :copy-worldcoords) :translate (float-vector 0 0 50) :world) :rotate (deg2rad -5) :z)
                  (send *robot* :rleg :end-coords :copy-worldcoords)
                  (send *robot* :lleg :end-coords :copy-worldcoords))
            :move-target (mapcar #'(lambda (x) (send *robot* x :end-coords)) '(:rarm :larm :rleg :lleg))
            :link-list (mapcar #'(lambda (x) (send *robot* :link-list (send *robot* x :end-coords :parent))) '(:rarm :larm :rleg :lleg))
            :cog-null-space nil
            :use-6dof-joint-leg-gain nil)
      (send *Ri* :angle-vector (Send *robot* :angle-vector) 2000)
      (send *ri* :wait-interpolation)
      (if walk-p (object-hold-walk))
      (send *robot* :angle-vector reaching-pose)
      (send *Ri* :angle-vector (Send *robot* :angle-vector) 2000)
      (send *ri* :wait-interpolation)
      (send *ri* :set-ref-force (float-vector 0 0 0) 2000 :arms)
      (send *ri* :wait-interpolation-seq)
      (send *robot* :angle-vector reaching-via-pose)
      (send *ri* :angle-vector (Send *robot* :angle-vector) 2000)
      (send *ri* :wait-interpolation)
      (send *robot* :angle-vector *initial-pose*)
      (send *ri* :angle-vector (Send *robot* :angle-vector) 2000)
      (send *ri* :wait-interpolation)
      ;; (send *robot* :rarm :wrist-r :joint-angle 20)
      ;; (send *robot* :larm :wrist-r :joint-angle -20)
      )))

(defun samplerobot-carry-object-demo1 (&key (walk-p t) (auto-detect-p t))
  "2. Dual-arm carry up"
  (let ((r-handle-coords (make-coords :pos #f(350 -150 870) :rpy (list 0 0 pi/2))))
    (send *ri* :go-pos 0.02 0.15 0)
    (send *robot* :reset-manip-pose)
    (send *robot* :larm :angle-vector #f(17.8356 9.13759 6.61188 -36.456 0.0 0.0 0.0)) ;; reset-pose
    (send *robot* :fix-leg-to-coords (make-coords))
    (send *robot* :fullbody-inverse-kinematics
          (list (send (send r-handle-coords :copy-worldcoords) :translate (float-vector -250 0 -30) :world)
                (send *robot* :rleg :end-coords :copy-worldcoords)
                (send *robot* :lleg :end-coords :copy-worldcoords))
          :move-target (mapcar #'(lambda (x) (send *robot* x :end-coords)) '(:rarm :rleg :lleg))
          :link-list (mapcar #'(lambda (x) (send *robot* :link-list (send *robot* x :end-coords :parent))) '(:rarm :rleg :lleg))
          :cog-null-space nil
          :use-6dof-joint-leg-gain nil)
    (let ((reaching-via-pose (send *robot* :angle-vector)) (reaching-pose))
      (send *ri* :angle-vector (Send *robot* :angle-vector) 2000)
      (send *ri* :wait-interpolation)
      (send *robot* :fullbody-inverse-kinematics
            (list r-handle-coords
                  (send *robot* :rleg :end-coords :copy-worldcoords)
                  (send *robot* :lleg :end-coords :copy-worldcoords))
            :move-target (mapcar #'(lambda (x) (send *robot* x :end-coords)) '(:rarm :rleg :lleg))
            :link-list (mapcar #'(lambda (x) (send *robot* :link-list (send *robot* x :end-coords :parent))) '(:rarm :rleg :lleg))
            :cog-null-space nil
            :use-6dof-joint-leg-gain nil)
      (send *Ri* :angle-vector (Send *robot* :angle-vector) 2000)
      (send *ri* :wait-interpolation)
      (if auto-detect-p
          (send *ri* :set-object-turnaround-ref-force :max-ref-force (* 9.8 6.0) :limbs '(:rarm))
        (progn
          (send *ri* :set-ref-force (float-vector 0 0 (* 9.8 5.0 -1)) 2000 :rarm)
          (send *ri* :wait-interpolation-seq)
          ))
      (send *robot* :fullbody-inverse-kinematics
            (list (send (send (send r-handle-coords :copy-worldcoords) :translate (float-vector 0 0 50) :world) :rotate (deg2rad 5) :z)
                  (send *robot* :rleg :end-coords :copy-worldcoords)
                  (send *robot* :lleg :end-coords :copy-worldcoords))
            :move-target (mapcar #'(lambda (x) (send *robot* x :end-coords)) '(:rarm :rleg :lleg))
            :link-list (mapcar #'(lambda (x) (send *robot* :link-list (send *robot* x :end-coords :parent))) '(:rarm :rleg :lleg))
            :cog-null-space nil
            :use-6dof-joint-leg-gain nil)
      ;; (send *robot* :rarm :wrist-r :joint-angle 10)
      (send *robot* :rarm :wrist-r :joint-angle 5)
      (send *Ri* :angle-vector (Send *robot* :angle-vector) 300)
      (send *ri* :wait-interpolation)
      (if walk-p (object-hold-walk))
      (send *robot* :angle-vector reaching-pose)
      (send *Ri* :angle-vector (Send *robot* :angle-vector) 2000)
      (send *ri* :wait-interpolation)
      (send *ri* :set-ref-force (float-vector 0 0 0) 2000 :rarm)
      (send *ri* :wait-interpolation-seq)
      (send *robot* :angle-vector reaching-via-pose)
      (send *robot* :fix-leg-to-coords (make-coords))
      (send *ri* :angle-vector (Send *robot* :angle-vector) 2000)
      (send *ri* :wait-interpolation)
      (send *robot* :angle-vector *initial-pose*)
      (send *robot* :fix-leg-to-coords (make-coords))
      (send *ri* :angle-vector (Send *robot* :angle-vector) 2000)
      (send *ri* :wait-interpolation)
      ;; (send *robot* :rarm :wrist-r :joint-angle 20)
      ;; (send *robot* :larm :wrist-r :joint-angle -20)
      )))

(defun data-string-split ;; this function will be replaced by https://github.com/euslisp/EusLisp/issues/16
  (str separator)
  (let ((start-idx 0) (ret))
    (dotimes (i (length str))
      (when (= (elt str i) (elt separator 0))
        (push (subseq str start-idx i) ret)
        (setq start-idx (1+ i))))
    (if (/= start-idx (length str)) (push (subseq str start-idx) ret))
    (reverse ret)))

(defun samplerobot-carry-object-demo2 (&key (auto-detect-p t))
  "3. Dual-arm push"
  (send *Ri* :go-pos -0.45 0 0)
  ;; Get robot's and object's pose (WAIST) from simulation instead of object recognition
  (send *ri* :go-pos 0 0 (rad2deg (- (read-from-string (cadr (data-string-split (car (data-string-split (car (last (data-string-split (find-if #'(lambda (x) (substringp "Pose3D" x)) (piped-fork-returns-list "rtprint localhost:15005/PushBox\\(Robot\\)0.rtc:WAIST -t 0.5 -r 100")) ","))) ")")) "=")))
                                     (read-from-string (cadr (data-string-split (car (data-string-split (car (last (data-string-split (find-if #'(lambda (x) (substringp "Pose3D" x)) (piped-fork-returns-list "rtprint localhost:15005/SampleRobot\\(Robot\\)0.rtc:WAIST -t 0.5 -r 100")) ","))) ")")) "="))))))
  (let ((pushbox-pose (find-if #'(lambda (x) (substringp "Pose3D" x)) (piped-fork-returns-list "rtprint localhost:15005/PushBox\\(Robot\\)0.rtc:WAIST -t 0.5 -r 100")))
        (samplerobot-pose (find-if #'(lambda (x) (substringp "Pose3D" x)) (piped-fork-returns-list "rtprint localhost:15005/SampleRobot\\(Robot\\)0.rtc:WAIST -t 0.5 -r 100"))))
    (send *ri* :go-pos
          (- (-
              (read-from-string (cadr (data-string-split (elt (data-string-split (elt (data-string-split pushbox-pose "(") 2) ",") 1) "=")))
              (read-from-string (cadr (data-string-split (elt (data-string-split (elt (data-string-split samplerobot-pose "(") 2) ",") 1) "="))))
             0.60)
          (* -1 (- (read-from-string (cadr (data-string-split (car (data-string-split (elt (data-string-split pushbox-pose "(") 2) ",")) "=")))
                   (read-from-string (cadr (data-string-split (car (data-string-split (elt (data-string-split samplerobot-pose "(") 2) ",")) "=")))))
          0))
  (let ((r-handle-coords (make-coords :pos (float-vector 480 -220 870) :rpy (list 0 0 pi/2)))
        (l-handle-coords (make-coords :pos (float-vector 480  220 870) :rpy (list 0 0 -pi/2))))
    (send *robot* :reset-manip-pose)
    (send *robot* :fix-leg-to-coords (make-coords))
    ;; Via coords
    (send *robot* :fullbody-inverse-kinematics
          (list (send (send r-handle-coords :copy-worldcoords) :translate (float-vector -150 0 -30) :world)
                (send (send l-handle-coords :copy-worldcoords) :translate (float-vector -150 0 -30) :world)
                (send *robot* :rleg :end-coords :copy-worldcoords)
                (send *robot* :lleg :end-coords :copy-worldcoords))
          :move-target (mapcar #'(lambda (x) (send *robot* x :end-coords)) '(:rarm :larm :rleg :lleg))
          :link-list (mapcar #'(lambda (x) (send *robot* :link-list (send *robot* x :end-coords :parent))) '(:rarm :larm :rleg :lleg))
          :cog-null-space nil
          :use-6dof-joint-leg-gain nil)
    (send *robot* :rarm :wrist-r :joint-angle 90)
    (send *robot* :larm :wrist-r :joint-angle -90)
    (let ((reaching-via-pose (send *robot* :angle-vector)) (reaching-pose))
      (send *Ri* :angle-vector (Send *robot* :angle-vector) 2000)
      (send *ri* :wait-interpolation)
      ;; Reach pose
      (send *robot* :fullbody-inverse-kinematics
            (list r-handle-coords
                  l-handle-coords
                  (send *robot* :rleg :end-coords :copy-worldcoords)
                  (send *robot* :lleg :end-coords :copy-worldcoords))
            :move-target (mapcar #'(lambda (x) (send *robot* x :end-coords)) '(:rarm :larm :rleg :lleg))
            :link-list (mapcar #'(lambda (x) (send *robot* :link-list (send *robot* x :end-coords :parent))) '(:rarm :larm :rleg :lleg))
            :cog-null-space nil
            :use-6dof-joint-leg-gain nil)
      (send *robot* :rarm :wrist-r :joint-angle 90)
      (send *robot* :larm :wrist-r :joint-angle -90)
      (setq reaching-pose (send *robot* :angle-vector))
      (send *Ri* :angle-vector (Send *robot* :angle-vector) 2000)
      (send *ri* :wait-interpolation)
      (if auto-detect-p
          (send *ri* :set-object-turnaround-ref-force :max-ref-force 100 :max-time 2000 :axis (float-vector -1 0 0)) ;; Push forward
        (progn
          ;;(send *ri* :set-ref-force (float-vector 0 0 ) 2000 :rarm)
          ;;(send *ri* :wait-interpolation-seq)
          ))
      (send *ri* :go-pos 0.5 0 0)
      (send *robot* :angle-vector reaching-pose)
      (send *Ri* :angle-vector (Send *robot* :angle-vector) 2000)
      (send *ri* :wait-interpolation)
      (send *ri* :set-ref-force (float-vector 0 0 0) 2000 :rarm)
      (send *ri* :wait-interpolation-seq)
      (send *robot* :angle-vector reaching-via-pose)
      (send *robot* :fix-leg-to-coords (make-coords))
      (send *ri* :angle-vector (Send *robot* :angle-vector) 2000)
      (send *ri* :wait-interpolation)
      (send *robot* :angle-vector *initial-pose*)
      (send *robot* :fix-leg-to-coords (make-coords))
      (send *ri* :angle-vector (Send *robot* :angle-vector) 2000)
      (send *ri* :wait-interpolation)
      ))
  )

(defun samplerobot-carry-object-demo ()
  (samplerobot-carry-object-init)
  (samplerobot-carry-object-demo0)
  (samplerobot-carry-object-demo1)
  (samplerobot-carry-object-demo2)
  )

(warn ";; (samplerobot-carry-object-demo)~%")
