;;
(load "package://eus_pcl/euslisp/eus_pcl.l")

;;
(setq *floor-z* (+ -842.0 100.0))

(setq *pp0* (with-open-file (f (ros::resolve-ros-path "package://hrpsys_gazebo/euslisp/atlas_vh_pos.l"))
                            (read f)))

(setq *raw-org-points* (cadr *pp0*))
(setq *floor-z* (+ -842.0 100.0))
(setq *org-points* (send *raw-org-points*
                         :filter :key #'(lambda (p) (and (> 0.0 (elt p 1) -3000.0)
                                                         (> 2000 (elt p 0) -2000)
                                                         (> 1000 (elt p 2) *floor-z*))) :create t))
(send *org-points* :point-color #f(1 0 0))

(setq *org-points2* (with-open-file (f (ros::resolve-ros-path "package://hrpsys_gazebo/euslisp/car-right-side.l"))
                            (read f)))
(send *org-points2* :point-color #f(1 1 0))

(setq *org-points3* (with-open-file (f (ros::resolve-ros-path "package://hrpsys_gazebo/euslisp/car-seat.l"))
                            (read f)))
(send *org-points3* :point-color #f(0 1 1))

;; pelvis relative coords
(setq *point-a* (float-vector 230.0 -650.0 380.0))
(setq *point-b* (float-vector 480.0 -360.0 150.0))
(setq *point-mid* (scale 0.5 (v+ *point-a* *point-b*)))
(setq *direction* (normalize-vector (v- *point-a* *point-b*)))

(setq *point-c* (float-vector 830.0 580.0 200.0))
(setq *point-d* (float-vector 700.0 1200.0 200.0))
(setq *point-mid2* (scale 0.5 (v+ *point-c* *point-d*)))
(setq *direction2* (normalize-vector (v- *point-c* *point-d*)))

(setq *point-e* (float-vector 686.092 -161.981 -17.234))
(setq *point-f* (float-vector 646.214 274.355 59.81))
(setq *point-mid3* (scale 0.5 (v+ *point-e* *point-f*)))
(setq *direction3* (normalize-vector (v- *point-e* *point-f*)))

;;(setq pa #f(855.0 -1420.0 380.0))  ;; center of handle
;;(setq pb #f(1135.0 -1110.0 180.0)) ;; left side of dashboard
;;
;; (send *atlas* :torso :waist-y :joint-angle -35)
;; (setq points (progn (get-tilt-laser) (get-tilt-laser)))
;; (setq aa (progn (ros::spin-once) (send *ri* :get :point)))
;; (setq bb (progn (ros::spin-once) (send *ri* :get :point)))
;;
(defun calc-transform-pelvis (in-points pointa pointb
                                        &optional (midp *point-mid*) (orgpt *org-points*))
  (send *atlas* :angle-vector (send *ri* :state :potentio-vector))
  (let ((midcds (send *atlas* :foot-midcoords))
        (point-mid (scale 0.5 (v+ pointa pointb)))
        (direction (normalize-vector (v- pointa pointb)))
        floorz points
        min_x max_x
        min_y max_y
        pre-cds trans rot)
    (setq floorz (+ 100 (elt (send midcds :worldpos) 2)))
    (setq trans (v- midp point-mid))

    (setq pre-cds (make-coords :pos trans))
    ;;(setq rot ;;

    (setq max_x (+ (elt point-mid 0) 1150))
    (setq min_x (- max_x 2400))
    (setq max_y (+ (elt point-mid 1) 500))
    (setq min_y (- max_y 2000))

    (setq points (send in-points :filter :key
                       #'(lambda (p) (and (> max_x (elt p 0) min_x)
                                          (> max_y (elt p 1) min_y)
                                          (> 1000 (elt p 2) floorz)))
                       :create t))
    (let ((npoints
           (send points :transform-points pre-cds :create t))
          cds)
      (setq  cds (pcl::registration-raw npoints orgpt pcl::*REGIST_GICP*))
      (send pre-cds :transform cds :world)
      pre-cds
      )))
;;
;; origin points -> x = 350, y = 300
;;
;;
(defun calc-transform-pelvis2 (in-points pointa pointb
                                        &optional (midp *point-mid2*)
                                        (orgpt *org-points2*))
  (send *atlas* :angle-vector (send *ri* :state :potentio-vector))
  (let ((midcds (send *atlas* :foot-midcoords))
        (point-mid (scale 0.5 (v+ pointa pointb)))
        (direction (normalize-vector (v- pointa pointb)))
        floorz points
        min_x max_x
        min_y max_y
        pre-cds trans rot)
    (setq floorz (+ 100 (elt (send midcds :worldpos) 2)))
    (setq trans (v- midp point-mid))

    (setq pre-cds (make-coords :pos trans))
    (pprint pre-cds)
    (setq max_x (+ (elt point-mid 0) 1150))
    (setq min_x (- max_x 2400))
    (setq min_y (- (elt point-mid 1) 700))
    (setq max_y (+ min_y 2000))

    (setq points (send in-points :filter :key
                       #'(lambda (p) (and (> max_x (elt p 0) min_x)
                                          (> max_y (elt p 1) min_y)
                                          (> 1000 (elt p 2) floorz)))
                       :create t))
    (let ((npoints
           (send points :transform-points pre-cds :create t))
          cds)
      (setq  cds (pcl::registration-raw npoints orgpt pcl::*REGIST_GICP*))
      (pprint cds)
      (send pre-cds :transform cds :world)
      pre-cds
      )))
;;
;; for sitting seat
;; -> 110.0 300.0 -1020.0
;; (setq car (load-mesh-file "/home/lyouhei/.gazebo/models/polaris_ranger_ev/meshes/polaris.dae" :scale 25.4))
;;
(defun calc-transform-pelvis3 (in-points pointa pointb
                                        &optional (midp *point-mid3*)
                                        (orgpt *org-points3*))
  (send *atlas* :reset-coords)
  (send *atlas* :angle-vector (send *ri* :state :potentio-vector))
  (let ((midcds (send *atlas* :foot-midcoords))
        (point-mid (scale 0.5 (v+ pointa pointb)))
        (direction (normalize-vector (v- pointa pointb)))
        floorz points
        min_x max_x
        min_y max_y
        pre-cds trans rot)
    (setq floorz (+ 100 (elt (send midcds :worldpos) 2)))
    (setq trans (v- midp point-mid))

    (setq pre-cds (make-coords :pos trans))
    (pprint pre-cds)
    (setq max_x (+ (elt point-mid 0) 1150))
    (setq min_x (- max_x 1350))
    (setq min_y (- (elt point-mid 1) 1200))
    (setq max_y (+ min_y 2400))

    (setq points (send in-points :filter :key
                       #'(lambda (p) (and (> max_x (elt p 0) min_x)
                                          (> max_y (elt p 1) min_y)
                                          (> 1000 (elt p 2) floorz)))
                       :create t))
    (let ((npoints
           (send points :transform-points pre-cds :create t))
          cds)
      (setq  cds (pcl::registration-raw npoints orgpt pcl::*REGIST_GICP*))
      (pprint cds)
      (send pre-cds :transform cds :world)
      pre-cds
      )))

(defun get-diff-vehicle-pos
  (&key (waist-y-angle 35) (neck-p-angle 30))

  (let ((prev-av (send *atlas* :angle-vector)))

    (set-high-gain)
    (unix::usleep (* 5 1000 1000))


    (setq *lr-reverse* nil)

    (let* (points p1 p2)
      (setq points (progn (get-tilt-laser) (get-tilt-laser)))
      (warn ";; click p1~%")
      (my-wait)
      (setq p1 (progn (ros::spin-once) (send *ri* :get :point)))
      (warn ";; click p2~%")
      (my-wait)
      (setq p2 (progn (ros::spin-once) (send *ri* :get :point)))
      (let* ((res-coords (calc-transform-pelvis2 points p1 p2))
             (res-x (elt (send res-coords :worldpos) 0))
             (res-y (elt (send res-coords :worldpos) 1))
             (dest-x (- 350 150 90))
             (dest-y 300)
             (diff-x (- dest-x res-x))
             (diff-y (- dest-y res-y))
             (diff-yaw (rad2deg (elt (car (rpy-angle (send res-coords :rot))) 0))))
        (warn "diff-x: ~a diff-y: ~a diff-y: ~a~%" diff-x diff-y diff-yaw)

        (setq *lr-reverse* t)
        (real2model)
        (list diff-x diff-y diff-yaw)
        ))
    ))