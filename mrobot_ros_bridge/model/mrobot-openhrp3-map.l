(unless (boundp '*pickview*) (pickview :no-menu t))

(defun convert-to-vrml (body-lst)
  (dolist (b body-lst)
    (if (not (string= (send b :name) "ground"))
        (progn
          (print b)
          (setf (get b :weight) 1000)
          (setf (get b :face-color) (float-vector 0 0 1)))
      (progn
        (print b)
        (setf (get b :weight) 1000)
        (setf (get b :face-color) (float-vector 0 1 1)))))
  (let ((rl (instance robot-link :init (make-cascoords) body-lst :name "map")))
    (eus2wrl rl "map.wrl" :mode :hrp :fixed t)))

(defun m-string (&key (l1 4000) (l2 3500) (w 300) (h 500) (theta 30))
  (let* ((m1 (make-cube l1 w h :name "m1"))
         (m2 (make-cube l2 w h :name "m2"))
         (m3 (make-cube l2 w h :name "m3"))
         (m4 (make-cube l1 w h :name "m4"))
         (ret (list m1 m2 m3 m4)))

    (send m2 :rotate (deg2rad theta) :z)
    (send m2 :translate (float-vector
                         (- (/ l1 2.0) (* (/ l2 2.0) (cos (deg2rad theta))))
                         (* (/ l2 2.0) (sin (deg2rad (- theta))))
                         0)
          :world)
    
    (send m3 :rotate (deg2rad (- theta)) :z)
    (send m3 :translate (float-vector
                         (- (/ l1 2.0) (* (/ l2 2.0) (cos (deg2rad theta))))
                         (- (* (/ l2 2.0) (sin (deg2rad theta))) l2)
                         0)
          :world)
    
    (send m4 :translate (float-vector 0 (- l2) 0))
    
    (objects ret)
    ret
    )
  )

(defun r-string (&key (l1 4000) (l2 3300) (l3 1500) (l4 l2) (l5 3000)
                      (w 300) (h 500) (theta 30))
  (let* ((r1 (make-cube l1 w h :name "r1"))
         (r2 (make-cube l2 w h :name "r2"))
         (r3 (make-cube l3 w h :name "r3"))
         (r4 (make-cube l4 w h :name "r4"))
         (r5 (make-cube l5 w h :name "r5"))
         (ret (list r1 r2 r3 r4 r5)))

    (send r2 :rotate (deg2rad 90) :z)
    (send r2 :translate (float-vector (/ l1 2) (- (/ l2 2)) 0) :world)

    (send r3 :translate (float-vector (- (/ l1 2) (/ l3 2)) (- l2) 0) :world)

    (send r4 :rotate (deg2rad 90) :z)
    (send r4 :translate (float-vector (- (/ l1 2) l3) (- (/ l2 2)) 0) :world)

    (send r5 :rotate (deg2rad theta) :z)
    (send r5 :translate (float-vector (- (* (/ l5 2.0) (cos (deg2rad theta))) (/ l1 2))
                                      -2500 0) :world)

    (objects ret)
    ret
    )
  )

(defun t-string (&key (l1 3500) (l2 3500) (w 300) (h 500) (theta 30))
  (let* ((t1 (make-cube l1 w h :name "t1"))
         (t2 (make-cube l2 w h :name "t2"))
         (ret (list t1 t2)))
    (send t2 :rotate pi/2 :z)
    (send t2 :translate (float-vector (/ l1 2.0) 0 0) :world)
    
    (objects ret)
    ret
    )
  )

(defun c-string (&key (l1 3000) (l2 3000) (w 300) (h 500) (theta 30))
  (let* ((c1 (make-cube l1 w h :name "c1"))
         (c2 (make-cube l2 w h :name "c2"))
         (c3 (make-cube l2 w h :name "c3"))
         (ret (list c1 c2 c3)))
    (send c2 :rotate pi/2 :z)
    (send c2 :translate (float-vector (/ l1 2.0) (/ l2 -2.0) 0) :world)

    (send c3 :rotate pi/2 :z)
    (send c3 :translate (float-vector (/ l1 -2.0) (/ l2 -2.0) 0) :world)
    
    (objects ret)
    ret
    )
  )

(defun i-string (&key (l1 3500) (l2 2000) (w 300) (h 500) (theta 30))
  (let* ((i1 (make-cube l1 w h :name "i1"))
         (i2 (make-cube l2 w h :name "i2"))
         (i3 (make-cube l2 w h :name "i3"))
         (ret (list i1 i2 i3)))
    (send i2 :rotate pi/2 :z)
    (send i2 :translate (float-vector (/ l1 2.0) 0 0) :world)

    (send i3 :rotate pi/2 :z)
    (send i3 :translate (float-vector (/ l1 -2.0) 0 0) :world)
    
    (objects ret)
    ret
    )
  )

(defun outer-walls (&key (l1 8000) (l2 14000) (w 300) (h 500) (theta 30))
  (let* ((w1 (make-cube l1 w h :name "w1"))
         (w2 (make-cube l2 w h :name "w2"))
         (w3 (make-cube l1 w h :name "w3"))
         (w4 (make-cube l2 w h :name "w2"))
         (w5 (make-cube 4000 4000 h :name "w4"))
         (ret (list w1 w2 w3 w4 w5)))
    (send w2 :rotate pi/2 :z)
    (send w2 :translate (float-vector (/ l1 2.0) (/ l2 -2.0) 0) :world)

    (send w3 :translate (float-vector 0 (- l2) 0) :world)

    (send w4 :rotate pi/2 :z)
    (send w4 :translate (float-vector (/ l1 -2.0) (/ l2 -2.0) 0) :world)

    (send w5 :translate (float-vector (/ 4000 -2.0) (- (+ 6000 2000)) 0) :world)
    
    (objects ret)
    ret
    )
  )

(defun make-objects-map ()
  (let* ((r-objects (r-string))
         (t-objects (t-string))
         (m-objects (m-string))
         (c-objects (c-string))
         (i-objects (i-string))
         (o-objects (outer-walls))
         (ground (make-cube 8000 14000 100))
         (ret (list r-objects t-objects m-objects c-objects i-objects o-objects ground)))
    (send-all r-objects :translate (float-vector 4000 0 0) :world)
    (send-all t-objects :translate (float-vector 2000 -6000 0) :world)
    (send-all c-objects :translate (float-vector 3500 -9000 0) :world)
    (send-all i-objects :translate (float-vector 1900 -13000 0) :world)
    (send-all o-objects :translate (float-vector 2000 (/ 300 2.0) 0) :world)
    (send ground :name "ground")
    (send ground :translate #f(2000 -7000 -350) :world)
    
    (objects ret)
    ret
    )
  )

(defun main-proc ()
  (let ((obj-lst (flatten (make-objects-map))))
    (convert-to-vrml obj-lst)
    ))