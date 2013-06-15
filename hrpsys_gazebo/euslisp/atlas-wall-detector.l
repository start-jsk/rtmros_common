
(defun vget
  (vec key &key (but-val 0.0))
  (if (numberp key)
      (aref vec key)
    (case key
      (:x (aref vec 0))
      (:y (aref vec 1))
      (:z (aref vec 2))
      (t but-val))))

;; #f(x,y) data-list ---> (a,b) of y = ax + b
(defun min-power-2d
  (data-list
   &optional (mirror nil))
  (let*
      ( (Exx (reduce
	      #'+
	      (mapcar
	       #'(lambda (v) (* (vget v :x) (vget v :x))) data-list)))
	(Ex (reduce
	     #'+
	     (mapcar
	      #'(lambda (v) (vget v :x)) data-list)))
	(Ey (reduce
	     #'+
	     (mapcar
	      #'(lambda (v) (vget v :y)) data-list)))
	(Exy (reduce
	      #'+
	      (mapcar #'(lambda (v) (* (vget v :x) (vget v :y))) data-list)))
	(n (length data-list))
	(mother (- (* n Exx) (* Ex Ex)))
        (a (/ (- (* n Exy) (* Ex Ey)) mother))
        (b (/ (- (* Exx Ey) (* Exy Ex)) mother)) )
    (cond
     ((or (zerop mother)
          (and mirror (zerop a)))
      nil)
     (mirror
      (setq a (/ -1.0 a))
      (setq b (/ (+ Ex Ey) n))
      (list a b))
     (t (list a b)))))

;; #f(x,y,z) data-list ---> (a,b,c,d,e,f) of ax+d = by+e = cz+f
(defun min-power-3d
  (data-list)
  (let ( (xyz (list :x :y :z))
	 (get2d
	  #'(lambda (key1 key2)
	      (mapcar
	       #'(lambda (v) (float-vector (vget v key1) (vget v key2)))
	       data-list))) )
    (reduce
     #'(lambda (result key)
	 (or result
	     (let* ( (but-key (remove-if #'(lambda (a) (equal a key)) xyz))
		     (but1 (funcall get2d (car but-key) key))
		     (but2 (funcall get2d (cadr but-key) key))
		     (min-power1 (min-power-2d but1))
		     (min-power2 (min-power-2d but2)) )
	       (cond
		((and (null min-power1) (null min-power2))  nil)
		(t
		 (let ( (ret
			 (list (cons key (list 1 0))
			       (cons (car but-key) min-power1)
			       (cons (cadr but-key) min-power2))) )
		   (list
		    (cdr (assoc :x ret))
		    (cdr (assoc :y ret))
		    (cdr (assoc :z ret)))))))))
     (cons nil xyz))))

(defun min-power-vector-3d
  (data-list)
  (concatenate float-vector
	       (mapcar #'(lambda (list)
			   (cond
			    ((null list) 0)
			    ((zerop (car list)) 1)
			    (t (/ 1.0 (car list)))))
		       (min-power-3d data-list))))

;; #f(x,y) data-list ---> (a,b) of y = ax + b
(defun max-power-2d
  (data-list)
  (let*
      ( (Exx (reduce
	      #'+
	      (mapcar
	       #'(lambda (v) (* (vget v :x) (vget v :x))) data-list)))
	(Ex (reduce
	     #'+
	     (mapcar
	      #'(lambda (v) (vget v :x)) data-list)))
	(Ey (reduce
	     #'+
	     (mapcar
	      #'(lambda (v) (vget v :y)) data-list)))
	(Exy (reduce
	      #'+
	      (mapcar #'(lambda (v) (* (vget v :x) (vget v :y))) data-list)))
	(n (length data-list))
        (b (/ (+ Ex Ey) n))
        (X Exx)
        (Y (* 2 (- (* b Ex) Exy)))
        (Z ( * n b b))
        (Z-X/Y (/ (- Z X) Y))
        (a+ (+ (* -1 Z-X/Y) (sqrt (+ 1 (expt Z-X/Y 2)))))
        (a- (- (* -1 Z-X/Y) (sqrt (+ 1 (expt Z-X/Y 2))))) )
    (list
     (if (minusp Y) a- a+)
     b)))

(defun y-from-coef
  (x coef)
  (+ (nth 1 coef) (* x (nth 0 coef))))

(defun point-cloud-callback
  (msg)
  nil)

(defvar *point-cloud*)
(defun point-cloud-callback-main
  (msg)
  (let ((ret (make-eus-pointcloud-from-ros-msg msg :remove-nan t)))
    (setf (get ret :frame_id) (send msg :header :frame_id))
    (setf (get ret :stamp) (send msg :header :stamp))
    (setf (get ret :point_step) (send msg :point_step))
    (setq *point-cloud* ret)
    (point-cloud-callback msg)
    ))

(defun get-pc-sync
  (&optional (max 100))
  (setq *point-cloud* nil)
  (while (and (plusp (decf max))
              (null *point-cloud*))
    (ros::spin-once)
    (ros::sleep))
  *point-cloud*)

(defun cluster
  (vlist
   &key
   (forward #f(0 0 1))
   (eval-func
    #'(lambda (p)
        (rad2deg
         (acos (v. (normalize-vector (map float-vector #'* #F(1 1 0) p))
                   forward)))))
   (remove-func #'(lambda (p val) (> (aref p 2) 2000)))
   (thre 5)
   (buf)
   (ret))
  (let* ((p (car vlist))
         (val (funcall eval-func (or p #F(1 1 0)))))
    (cond
     ((null vlist) ret)
     ((funcall remove-func p val)
      (cluster
       (cdr vlist)
       :forward forward
       :eval-func eval-func
       :remove-func remove-func
       :thre thre
       :buf buf
       :ret ret))
     ((> (abs (- val (or (cdr (assoc :average buf)) 1e+10)))
         thre)
      (cluster
       (cdr vlist)
       :forward forward
       :eval-func eval-func
       :remove-func remove-func
       :thre thre
       :buf (list (cons :average val)
                  (cons :size 1)
                  (cons :data (list p)))
       :ret
       (if buf
           (cons
            (cons
             (cons
              :coveriance
              (/
               (sqrt
                (apply
                 #'+
                 (mapcar
                  #'(lambda (p) (expt (- (funcall eval-func p)
                                         (cdr (assoc :average buf)))
                                      2))
                  (cdr (assoc :data buf)))))
               (cdr (assoc :size buf))))
             buf)
            ret)
         ret)))
     (t (cluster
         (cdr vlist)
         :forward forward
         :eval-func eval-func
         :remove-func remove-func
         :thre thre
         :buf (list (cons :average
                          (/ (+ val
                                (* (cdr (assoc :size buf))
                                   (cdr (assoc :average buf))))
                             (+ 1 (cdr (assoc :size buf)))))
                    (cons :size (+ 1 (cdr (assoc :size buf))))
                    (cons :data (cons p (cdr (assoc :data buf)))))
         :ret ret)))))

(defun wall-distance
  (&key
   (pc (get-pc-sync))
   (pc-coords (make-coords)))
  (let (plist buf)
    (send *robot* :newcoords (make-coords :rpy (mapcar #'deg2rad (get-imu-data))))
    (send pc :transform pc-coords)
    (send pc :convert-to-world)
;    (objects (list *robot* pt))
    (setq
     plist
     (remove-if
      #'(lambda (p)
          (or (< (cdr (assoc :size p)) 10)
              (> (abs (cdr (assoc :average p))) 30)))
      (cluster
       (send pc :point-list)
       :remove-func #'(lambda (p val) (or (< (aref p 2) 500)
                                          (> (aref p 2) 3000)))
       :thre 5)))
    (setq buf
          (mapcar
           #'(lambda (p)
               (/
                (apply #'+
                       (mapcar #'norm (cdr (assoc :data p))))
                (cdr (assoc :size p))))
           plist))
    (/ (apply #'+ buf) (length buf))))

(defun wall-line
  (&key
   (pc (get-pc-sync))
   (mirror t)
   (pc-coords (make-coords)))
  (let (min)
    (send pc :transform pc-coords)
    (send pc :convert-to-world)
    (setq
     min
     (if mirror
         (max-power-2d
          (mapcar
           #'(lambda (v) (float-vector (aref v 0) (aref v 1)))
           (send pc :point-list)))
       (min-power-2d
        (mapcar
         #'(lambda (v) (float-vector (aref v 0) (aref v 1)))
         (send pc :point-list)))))
    (print min)
    (if (boundp '*irtviewer*)
        (progn
          (send *irtviewer* :viewer :viewsurface :draw-line
                (subseq
                 (send *irtviewer* :viewer :point-to-screen
                       (float-vector
                        -10000 (y-from-coef -10000 min) 0))
                 0 2)
                (subseq
                 (send *irtviewer* :viewer :point-to-screen
                       (float-vector
                        10000 (y-from-coef 10000 min) 0))
                 0 2))
          (send *irtviewer* :viewer :viewsurface :flush)))
    min
    ))

(defun wall-potential
  (&key
   (sampling 100)
   (point-cloud (get-pc-sync))
   (plist (remove-if
           #'(lambda (v) (minusp (- (aref v 2) 1000)))
           (send point-cloud :point-list)))
   (cnt 0)
   (len (length plist))
   (step (/ len (+ 1 sampling)))
   (pc-coords (make-coords))
   (init (progn
           (send point-cloud :transform pc-coords)
           (send point-cloud :convert-to-world)))
   )
  (if (minusp sampling)
      #F(0 0 0)
      (v+
       (scale
        (/ -1.0 (norm (nth cnt plist)))
        (nth cnt plist))
       (wall-potential
        :sampling sampling
        :point-cloud point-cloud
        :plist plist
        :cnt (mod (+ cnt step) len)
        :step step
        :pc-coords pc-coords
        :init nil))))

(ros::roseus-add-msgs "sensor_msgs")
(ros::subscribe "/full_cloud2"
                sensor_msgs::PointCloud2
                #'point-cloud-callback-main)
