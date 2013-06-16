;;
(load "package://eus_pcl/euslisp/eus_pcl.l")

;;
(setq *floor-z* (+ -842.0 100.0))
(setq p0 (with-open-file (f "atlas_vh_pos.l") (read f)))
(setq *raw-org-points* (cadr p0))
(setq *floor-z* (+ -842.0 100.0))
(setq *org-points* (send *raw-org-points*
                         :filter :key #'(lambda (p) (and (> 0.0 (elt p 1) -3000.0)
                                                         (> 2000 (elt p 0) -2000)
                                                         (> 1000 (elt p 2) *floor-z*))) :create t))
(send *org-points* :point-color #f(1 0 0))
;; pelvis relative coords
(setq *point-a* (float-vector 230.0 -650.0 380.0))
(setq *point-b* (float-vector 480.0 -360.0 150.0))
(setq *point-mid* (scale 0.5 (v+ *point-a* *point-b*)))
(setq *direction* (normalize-vector (v- *point-a* *point-b*)))

;;(setq pa #f(855.0 -1420.0 380.0))  ;; center of handle
;;(setq pb #f(1135.0 -1110.0 180.0)) ;; left side of dashboard

(defun calc-transform-pelvis (in-points pointa pointb)
  (send *atlas* :angle-vector (send *ri* :state :potentio-vector))
  (let ((midcds (send *atlas* :foot-midcoords))
        (point-mid (scale 0.5 (v+ pointa pointb)))
        (direction (normalize-vector (v- pointa pointb)))
        floorz points
        min_x max_x
        min_y max_y
        pre-cds trans rot)
    ;;(setq floorz (+ 100 (elt (send midcds :worldpos) 2)))
    (setq floorz (+ 100 -842))
    (setq trans (v- *point-mid* point-mid))

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

      (setq  cds (pcl::registration-raw npoints *org-points* pcl::*REGIST_GICP*))
      (send pre-cds :transform cds :world)
      pre-cds
      )))
