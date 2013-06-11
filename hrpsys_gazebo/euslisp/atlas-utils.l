(defun mirror-angle-vector-from
  (from)
  (let ((av (send *atlas* from :angle-vector))
        (arm-convert-v (float-vector -1 1 -1 -1 1 1))
        (leg-convert-v (float-vector 1 -1 1 -1 1 -1))
        )
    (case from
      (:lleg
        (setq av (map float-vector #'* leg-convert-v av))
        (send *atlas* :rleg :angle-vector av)
        )
      (:rleg
        (setq av (map float-vector #'* leg-convert-v av))
        (send *atlas* :lleg :angle-vector av)
        )
      (:larm
        (setq av (map float-vector #'* arm-convert-v av))
        (send *atlas* :rarm :angle-vector av)
        )
      (:rarm
        (setq av (map float-vector #'* arm-convert-v av))
        (send *atlas* :larm :angle-vector av)
        ))))

