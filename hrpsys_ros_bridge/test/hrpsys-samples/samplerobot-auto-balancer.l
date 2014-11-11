#!/usr/bin/env roseus

;; $ rtmlaunch hrpsys_ros_bridge_tutorials samplerobot.launch
;; $ roseus samplerobot-auto-balancer.l "(samplerobot-auto-balancer-demo)"

(load "package://hrpsys_ros_bridge_tutorials/euslisp/samplerobot-interface.l")

(defun samplerobot-auto-balancer-init ()
  (samplerobot-init)
  (initialize-pose-list)
  t)

(defun test-Pose-List (pose-list initial-pose)
  (dolist (pose pose-list)
    (send *sr* :angle-vector pose)
    (send *ri* :angle-vector (send *sr* :angle-vector) 1000)
    (send *ri* :wait-interpolation)
    (send *sr* :angle-vector initial-pose)
    (send *ri* :angle-vector (send *sr* :angle-vector) 1000)
    (send *ri* :wait-interpolation)))

(defun initialize-pose-list
  ()
  (setq *initial-pose* (send *sr* :reset-pose))
  (send *sr* :arms :shoulder-p :joint-angle -90)
  (send *sr* :arms :elbow-p :joint-angle -20)
  (setq *arm-front-pose* (send *sr* :angle-vector))
  (send *sr* :reset-pose)
  (send *sr* :legs :move-end-pos #f(0 0 70))
  (setq *half-sitting-pose* (send *sr* :angle-vector))
  (send *sr* :reset-pose)
  (send *sr* :fix-leg-to-coords (make-coords))
  (send *sr* :legs :move-end-pos #f(0 0 70))
  (let ((lc (send *sr* :legs :end-coords :copy-worldcoords)))
    (send *sr* :move-coords (send (send (car (send *sr* :links)) :copy-worldcoords) :rotate (deg2rad 10) :x) (car (send *sr* :links)))
    (mapcar #'(lambda (l c) (send *sr* l :inverse-kinematics c)) '(:lleg :rleg) lc)
    (send *sr* :move-centroid-on-foot :both '(:rleg :lleg))
    (setq *root-rot-x-pose* (send *sr* :angle-vector)))
  (send *sr* :reset-pose)
  (send *sr* :fix-leg-to-coords (make-coords))
  (send *sr* :legs :move-end-pos #f(0 0 70))
  (let ((lc (send *sr* :legs :end-coords :copy-worldcoords)))
    (send *sr* :move-coords (send (send (car (send *sr* :links)) :copy-worldcoords) :rotate (deg2rad 20) :y) (car (send *sr* :links)))
    (mapcar #'(lambda (l c) (send *sr* l :inverse-kinematics c)) '(:lleg :rleg) lc)
    (send *sr* :move-centroid-on-foot :both '(:rleg :lleg))
    (setq *root-rot-y-pose* (send *sr* :angle-vector)))
  (setq *pose-list* (list *half-sitting-pose* *root-rot-x-pose* *root-rot-y-pose*))
  (send *sr* :angle-vector *initial-pose*)
  (send *ri* :angle-vector (send *sr* :angle-vector) 2000)
  (send *ri* :wait-interpolation)
  )

(defun samplerobot-auto-balancer-demo0 ()
  "1. AutoBalancer mode by fixing feet"
  (send *ri* :start-auto-balancer)
  (test-pose-list (list *arm-front-pose*) *initial-pose*)
  (send *ri* :stop-auto-balancer)
  (print "Start and Stop AutoBalancer by fixing feet=>OK")
  t)

(defun samplerobot-auto-balancer-demo1 ()
  "2. AutoBalancer mode by fixing hands and feet"
  (send *ri* :start-auto-balancer :limbs '(:rleg :lleg :rarm :larm))
  (test-pose-list (list *arm-front-pose*) *initial-pose*)
  (send *ri* :stop-auto-balancer)
  (print "Start and Stop AutoBalancer by fixing hands and feet=>OK")
  t)

(defun samplerobot-auto-balancer-demo2 ()
  "3. getAutoBalancerParam"
  (pprint (send (send *ri* :get-auto-balancer-param) :slots))
  (print "getAutoBalancerParam() => OK")
  t)

(defun samplerobot-auto-balancer-demo3 ()
  "4. setAutoBalancerParam"
  (let ((zmpoff (list (float-vector 100 0 0) (float-vector 100 0 0))))
    (send *ri* :set-auto-balancer-param :default-zmp-offsets zmpoff)
    (let ((dd (send (send (send *ri* :get-auto-balancer-param) :default_zmp_offsets) :data)))
      (if (and (eps-v= (car zmpoff) (scale 1e3 (subseq dd 0 3))) (eps-v= (cadr zmpoff) (scale 1e3 (subseq dd 3)))) ;; [m] => [mm]
          (print "setAutoBalancerParam() => OK"))))
  (send *ri* :start-auto-balancer)
  (send *ri* :set-auto-balancer-param :default-zmp-offsets (list (float-vector 0 0 0) (float-vector 0 0 0)))
  t)

(defun samplerobot-auto-balancer-demo4 ()
  "5. change base height, base rot x, base rot y, and upper body while AutoBalancer mode"
  (send *ri* :start-auto-balancer)
  (test-pose-list *pose-list* *initial-pose*)
  (send *ri* :stop-auto-balancer)
  )

(defun samplerobot-auto-balancer-demo5 ()
  "1. goPos"
  (send *ri* :go-pos 0.1 0.05 20)
  (print "goPos()=>OK")
  t)

(defun samplerobot-auto-balancer-demo6 ()
  "2. goVelocity and goStop"
  (send *ri* :go-velocity -0.1 -0.05 -20)
  (unix:sleep 1)
  (send *ri* :go-stop)
  (print "goVelocity()=>OK")
  t)

(defun samplerobot-auto-balancer-demo7 ()
  "3. setFootSteps"
  (send *ri* :set-foot-steps
        (list (make-coords :pos (float-vector 0 -90 0) :name :rleg)
              (make-coords :pos (float-vector 0 90 0) :name :lleg)))
  (send *ri* :wait-foot-steps)
  (send *ri* :set-foot-steps
        (list (make-coords :pos (float-vector 0 -90 0) :name :rleg)
              (make-coords :pos (float-vector 150 90 0) :name :lleg)
              (make-coords :pos (float-vector 300 -90 0) :name :rleg)
              (make-coords :pos (float-vector 300 90 0) :name :lleg)))
  (send *ri* :wait-foot-steps)
  (print "setFootSteps()=>OK")
  t)

(defun samplerobot-auto-balancer-demo8 ()
  "4. change base height, base rot x, base rot y, and upper body while walking"
  (send *ri* :start-auto-balancer)
  (send *ri* :go-velocity 0 0 0)
  (test-pose-list *pose-list* *initial-pose*)
  (send *ri* :go-stop)
  (send *ri* :stop-auto-balancer)
  t)

(defun samplerobot-auto-balancer-demo9 ()
  "5. getGaitGeneratorParam"
  (pprint (send (send *ri* :get-gait-generator-param) :slots))
  (print "getGaitGeneratorParam() => OK")
  t)

(defun samplerobot-auto-balancer-demo10 ()
  "6. setGaitGeneratorParam"
  (let ((org-gp (send *ri* :get-gait-generator-param))
        (default-step-time 0.7)
        (default-step-height 0.15)
        (default-double-support-ratio 0.4)
        (default-orbit-type :RECTANGLE))
    (send *ri* :set-gait-generator-param
          :default-step-time default-step-time
          :default-step-height default-step-height
          :default-double-support-ratio default-double-support-ratio
          :default-orbit-type default-orbit-type)
    (let ((gp (send *ri* :get-gait-generator-param)))
      (if (and (eps= (send gp :default_step_time) default-step-time)
               (eps= (send gp :default_step_height) default-step-height)
               (eps= (send gp :default_double_support_ratio) default-double-support-ratio)
               (eq (send gp :default_orbit_type) 2)) ;; rectangle
          (print "setGaitGeneratorParam() => OK")))
    (send *ri* :go-velocity 0.1 0 0)
    (unix:sleep 1)
    (send *ri* :go-stop)
    (send *ri* :set-gait-generator-param
          :default-step-time (send org-gp :default_step_time)
          :default-step-height (send org-gp :default_step_height)
          :default-double-support-ratio (send org-gp :default_double_support_ratio)
          :default-orbit-type (send org-gp :default_orbit_type)
          )
    t))

(defun samplerobot-auto-balancer-demo11 ()
  "7. non-default stride"
  (send *ri* :start-auto-balancer)
  (send *ri* :set-foot-steps
        (list (make-coords :pos (float-vector 0 -90 0) :name :rleg)
              (make-coords :pos (float-vector 150 90 0) :name :lleg)))
  (send *ri* :wait-foot-steps)
  (send *ri* :set-foot-steps
        (list (make-coords :pos (float-vector 0 -90 0) :name :rleg)
              (make-coords :pos (float-vector 0 90 0) :name :lleg)))
  (send *ri* :wait-foot-steps)
  (send *ri* :stop-auto-balancer)
  (print "Non default Stride()=>OK")
  t)

(defun samplerobot-auto-balancer-demo ()
  (samplerobot-auto-balancer-init)

  ;; sample for AutoBalancer mode
  (samplerobot-auto-balancer-demo0)
  (samplerobot-auto-balancer-demo1)
  (samplerobot-auto-balancer-demo2)
  (samplerobot-auto-balancer-demo3)
  (samplerobot-auto-balancer-demo4)

  ;; sample for walk pattern generation by AutoBalancer RTC
  (samplerobot-auto-balancer-demo5)
  (samplerobot-auto-balancer-demo6)
  (samplerobot-auto-balancer-demo7)
  (samplerobot-auto-balancer-demo8)
  (samplerobot-auto-balancer-demo9)
  (samplerobot-auto-balancer-demo10)
  (samplerobot-auto-balancer-demo11)
  )

(warn ";; (samplerobot-auto-balancer-demo)~%")