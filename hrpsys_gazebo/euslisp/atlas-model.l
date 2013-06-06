(require :atlas "package://hrpsys_gazebo/models/atlas.l")
(require :sandia_hand_left  "package://hrpsys_gazebo/models/sandia_hand_left.l")
(require :sandia_hand_right "package://hrpsys_gazebo/models/sandia_hand_right.l")

(defun atlas-with-hand ()
  (atlas)
  ;; left hand
  (sandia_hand_left)
  (send *sandia_hand_left* :move-to (send *atlas* :l_hand :copy-worldcoords))
  (send *atlas* :l_hand :assoc *sandia_hand_left*)
  (setf (get *atlas* :lhand-model) *sandia_hand_left*)
  ;; right hand
  (sandia_hand_right)
  (send *sandia_hand_right* :move-to (send *atlas* :r_hand :copy-worldcoords))
  (send *atlas* :r_hand :assoc *sandia_hand_right*)
  (setf (get *atlas* :rhand-model) *sandia_hand_right*)
  ;; update bodies
  (let ((atbd (*atlas* . bodies)))
    (setq atbd (append atbd (send *sandia_hand_left* :bodies)))
    (setq atbd (append atbd (send *sandia_hand_right* :bodies)))
    (setq (*atlas* . bodies) atbd)
    )
  *atlas*)
