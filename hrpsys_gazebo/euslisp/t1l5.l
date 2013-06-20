;; roseus atlas-vehicle-motion.l

(defun task1func-1
  ()
  (init-task1)
  (start-walk)
  (rotate-with-imu 0)
  (walk 1)
  )

(defun task1func-2
  ()
  (walk 6)
  (rotate-with-imu 90)
  (walk 8)
  (rotate-with-imu 180)
  (walk 3)
  (rotate-with-imu 90)
  (walk 2.5)
  (rotate-with-imu 180)
  (finish-walk)
  )

(defun task1func-3
  ()
  (start-walk2)
  (walk 0.5)
  (rotate-with-imu 90)
  (go-pos 0 0.55 0)
  (rotate-with-imu 90)
  (walk -0.325)
  (rotate-with-imu 90)
)

(defun task1func-4
  ()
  (finish-walk2)
  (lr-rev)
  (get-in-for-task1 :real t :after-walk t)
  (drive-for-task1 :real t :wait nil)
  )
