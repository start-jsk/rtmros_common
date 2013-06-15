

(require "package://hrpsys_gazebo/euslisp/atlas-interface.l")
(my-init)

(require "package://hrpsys_gazebo/euslisp/atlas-tolot.l")
(require "package://hrpsys_gazebo/euslisp/atlas-standup.l")
(require "package://hrpsys_gazebo/euslisp/atlas-wall-detector.l")
(tolot-init :reverse? t)
(set-user)
(model2real)

(defun gen-dict
  nil
  (list
   (cons :stop
         (list
          (cons :av-list
                (progn
                  (tolot-init)
                  (model2real_ 300)
                  (unix:sleep 1)
                  (let ((buf (send *robot* :angle-vector)))
                    (vector buf buf))))
          (cons :sleep #(300 300))))
   (cons :forward
         (list
          (cons :av-list
                (progn
                  (tolot-init)
                  (send *tolot-hash* :enter :direction #(#f(-100 0 0) #f(-400 0 0)))
                  (gallop-step :reverse? t :sleep 100)
                  (copy-object (send *tolot-hash* :get :dict))))
          (cons :sleep #(100 100))))
   (cons :right
         (list
          (cons :av-list
                (progn
                  (tolot-init)
                  (send *tolot-hash* :enter :direction #(#f(-0 -0 0) #f(-400 -50 0)))
                  (gallop-step :reverse? t :rotate -30 :sleep 100)
                  (copy-object (send *tolot-hash* :get :dict))))
          (cons :sleep #(100 100))))
   (cons :left
         (list
          (cons :av-list
                (progn
                  (tolot-init)
                  (send *tolot-hash* :enter :direction #(#f(-0 -0 0) #f(-400 50 0)))
                  (gallop-step :reverse? t :rotate 30 :sleep 100)
                  (copy-object (send *tolot-hash* :get :dict))))
          (cons :sleep #(100 100))))
   (cons :prot
         (list
          (cons :av-list
                (progn
                  (tolot-init)
                  (send *tolot-hash* :enter :direction #(#F(-400 -200 0) #F(-100 0 0)))
                  (side-gallop-step :reverse? t :rotate 0 :mirror t)
                  (copy-object (send *tolot-hash* :get :dict))))
          (cons :sleep #(100 100))))
   (cons :qrot
         (list
          (cons :av-list
                (progn
                  (tolot-init)
                  (send *tolot-hash* :enter :direction #(#F(-400 200 0) #F(-100 0 0)))
                  (side-gallop-step :reverse? t :rotate 0)
                  (copy-object (send *tolot-hash* :get :dict))))
          (cons :sleep #(100 100))))
   (cons :negaeri 'negaeri-once)))

(defun negaeri-once
  nil
  (negaeri)
  (setq *gallop-mode* :stop))

(defvar *gallop-dict*
  '((:stop (:av-list . #(#f(-0.003048 6.09591 -0.053802 105.109 97.2697 0.0 -26.5584 105.814 0.0 -105.089 97.0922 0.0 26.6117 105.805 0.0 99.6314 1.06171 2.72941 17.1759 -4.57762 -1.29622 99.6322 -0.980218 2.75578 -17.0183 -4.58332 1.34451 0.0) #f(-0.003048 6.09591 -0.053802 105.109 97.2697 0.0 -26.5584 105.814 0.0 -105.089 97.0922 0.0 26.6117 105.805 0.0 99.6314 1.06171 2.72941 17.1759 -4.57762 -1.29622 99.6322 -0.980218 2.75578 -17.0183 -4.58332 1.34451 0.0))) (:sleep . #(300 300))) (:forward (:av-list . #(#f(0.02106 19.9715 -0.063195 108.512 72.4522 0.0 60.0 99.8673 0.0 -108.271 72.0259 0.0 -60.0 99.6771 0.0 89.4923 1.78779 6.2236 17.7434 -3.04163 0.879951 89.4598 -1.65378 6.27114 -17.6136 -3.01217 -0.762856 0.0) #f(-0.006872 -6.1332 -0.051529 103.378 89.4835 0.0 -15.9431 111.961 0.0 -103.391 89.3914 0.0 15.9614 111.968 0.0 132.403 -12.2784 27.2545 25.3349 -51.5662 -18.3346 132.377 12.3472 27.2685 -25.2106 -51.5662 18.3346 0.0))) (:sleep . #(100 100))) (:right (:av-list . #(#f(-0.013919 23.9251 -30.0 107.668 92.2545 0.0 60.0 98.3323 0.0 -107.538 92.0215 0.0 -60.0 98.2446 0.0 93.1815 1.3889 4.89525 17.6166 -4.30003 0.174148 93.177 -1.30705 4.92571 -17.4672 -4.29974 -0.11187 0.0) #f(0.555412 -5.75039 -30.0 105.158 62.8758 0.0 -22.1684 111.615 0.0 -102.662 106.447 0.0 12.2553 111.504 0.0 133.195 -9.81029 27.208 28.3614 -51.5662 -18.3346 130.807 15.5738 28.31 -17.6376 -51.5662 18.3346 0.0))) (:sleep . #(100 100))) (:left (:av-list . #(#f(-0.013919 23.9251 30.0 107.668 92.2545 0.0 60.0 98.3323 0.0 -107.538 92.0215 0.0 -60.0 98.2446 0.0 93.1815 1.3889 4.89525 17.6166 -4.30003 0.174148 93.177 -1.30705 4.92571 -17.4672 -4.29974 -0.11187 0.0) #f(-0.550973 -5.76289 30.0 102.653 106.493 0.0 -12.247 111.495 0.0 -105.157 62.9009 0.0 22.1592 111.618 0.0 130.825 -15.519 28.3084 17.7481 -51.5662 -18.3346 133.181 9.83779 27.2271 -28.3614 -51.5662 18.3346 0.0))) (:sleep . #(100 100))) (:prot (:av-list . #(#f(-34.0992 15.3573 0.0 82.5364 20.9453 0.0 60.0 112.5 0.0 -105.703 0.130668 0.0 -60.0 52.4805 0.0 107.194 -5.91914 -12.2449 23.3883 6.90871 -6.21829 105.621 -6.25553 -3.36497 -6.59642 -4.44814 -1.36072 0.0) #f(33.6347 15.5531 0.0 113.606 76.6479 0.0 -19.8355 78.2388 0.0 -80.7925 89.5449 0.0 51.3878 112.5 0.0 108.927 2.1324 7.48571 19.963 -17.7766 -5.69368 119.599 5.43001 -3.33338 -18.6435 -15.0183 4.08233 0.0))) (:sleep . #(100 100))) (:qrot (:av-list . #(#f(34.7483 15.5818 0.0 105.988 0.135829 0.0 60.0 51.9628 0.0 -81.859 20.6065 0.0 -60.0 112.5 0.0 105.816 6.34649 -3.35018 6.55935 -4.46118 1.54433 107.264 5.91712 -12.2369 -23.4663 7.06727 6.43399 0.0) #f(-33.643 15.5096 0.0 80.8086 89.5727 0.0 -51.4134 112.5 0.0 -113.599 76.3776 0.0 19.9786 78.1938 0.0 119.605 -5.37253 -3.36691 18.7554 -14.9949 -4.04987 108.937 -2.06656 7.48758 -19.8117 -17.7692 5.7331 0.0))) (:sleep . #(100 100))) (:negaeri . negaeri-once)))
;  '((:stop (:av-list . #(#f(-0.003048 6.09591 -0.053802 105.109 97.2697 0.0 -26.5584 105.814 0.0 -105.089 97.0922 0.0 26.6117 105.805 0.0 99.6314 1.06171 2.72941 17.1759 -4.57762 -1.29622 99.6322 -0.980218 2.75578 -17.0183 -4.58332 1.34451 0.0) #f(-0.003048 6.09591 -0.053802 105.109 97.2697 0.0 -26.5584 105.814 0.0 -105.089 97.0922 0.0 26.6117 105.805 0.0 99.6314 1.06171 2.72941 17.1759 -4.57762 -1.29622 99.6322 -0.980218 2.75578 -17.0183 -4.58332 1.34451 0.0))) (:sleep . #(300 300))) (:forward (:av-list . #(#f(0.02106 19.9715 -0.063195 108.512 72.4522 0.0 60.0 99.8673 0.0 -108.271 72.0259 0.0 -60.0 99.6771 0.0 89.4923 1.78779 6.2236 17.7434 -3.04163 0.879951 89.4598 -1.65378 6.27114 -17.6136 -3.01217 -0.762856 0.0) #f(-0.000869 -8.5676 -0.059016 103.058 88.9548 0.0 -15.4401 112.063 0.0 -103.068 88.9449 0.0 15.4401 112.069 0.0 130.294 -12.5845 30.023 27.9989 -51.5662 -18.3346 130.29 12.6254 30.023 -27.9531 -51.5662 18.3346 0.0))) (:sleep . #(100 100))) (:right (:av-list . #(#f(-0.013919 23.9251 -30.0 107.668 92.2545 0.0 60.0 98.3323 0.0 -107.538 92.0215 0.0 -60.0 98.2446 0.0 93.1815 1.3889 4.89525 17.6166 -4.30003 0.174148 93.177 -1.30705 4.92571 -17.4672 -4.29974 -0.11187 0.0) #f(1.14502 -7.98182 -30.0 105.159 63.8736 0.0 -21.3554 111.804 0.0 -102.146 104.352 0.0 12.5502 111.655 0.0 130.881 -11.1906 30.023 28.3614 -51.5662 -18.3346 130.188 14.1449 30.023 -24.6546 -51.5662 18.3346 0.0))) (:sleep . #(100 100))) (:left (:av-list . #(#f(-0.013919 23.9251 30.0 107.668 92.2545 0.0 60.0 98.3323 0.0 -107.538 92.0215 0.0 -60.0 98.2446 0.0 93.1815 1.3889 4.89525 17.6166 -4.30003 0.174148 93.177 -1.30705 4.92571 -17.4672 -4.29974 -0.11187 0.0) #f(-1.1423 -7.99684 30.0 102.134 104.378 0.0 -12.5475 111.646 0.0 -105.164 63.8791 0.0 21.3513 111.808 0.0 130.198 -14.0979 30.023 24.742 -51.5662 -18.3346 130.877 11.2193 30.023 -28.3614 -51.5662 18.3346 0.0))) (:sleep . #(100 100))) (:prot (:av-list . #(#f(34.7483 15.5818 0.0 105.988 0.135829 0.0 60.0 51.9628 0.0 -81.859 20.6065 0.0 -60.0 112.5 0.0 105.816 6.34649 -3.35018 6.55935 -4.46118 1.54433 107.264 5.91712 -12.2369 -23.4663 7.06727 6.43399 0.0) #f(-33.643 15.5096 0.0 80.8086 89.5727 0.0 -51.4134 112.5 0.0 -113.599 76.3776 0.0 19.9786 78.1938 0.0 119.605 -5.37253 -3.36691 18.7554 -14.9949 -4.04987 108.937 -2.06656 7.48758 -19.8117 -17.7692 5.7331 0.0))) (:sleep . #(100 100))) (:qrot (:av-list . #(#f(34.7483 15.5818 0.0 105.988 0.135829 0.0 60.0 51.9628 0.0 -81.859 20.6065 0.0 -60.0 112.5 0.0 105.816 6.34649 -3.35018 6.55935 -4.46118 1.54433 107.264 5.91712 -12.2369 -23.4663 7.06727 6.43399 0.0) #f(-33.643 15.5096 0.0 80.8086 89.5727 0.0 -51.4134 112.5 0.0 -113.599 76.3776 0.0 19.9786 78.1938 0.0 119.605 -5.37253 -3.36691 18.7554 -14.9949 -4.04987 108.937 -2.06656 7.48758 -19.8117 -17.7692 5.7331 0.0))) (:sleep . #(100 100))) (:negaeri . negaeri-once)))

(defvar *gallop-mode* :stop)
(defvar *neck-angle* -40)
(defvar *prev-mode* :stop)
(defvar *mode-dup* 0)

(defun play
  (key)
  (setq *prev-mode* *gallop-mode*)
  (setq *gallop-mode* key)
  (if (eq *prev-mode* *gallop-mode*)
      (setq *mode-dup* (+ *mode-dup* 1))
    (setq *mode-dup* 0))
  (let* ((target (cdr (assoc key *gallop-dict*))))
    (if (functionp target)
        (funcall target)
      (map cons
           #'(lambda (av time)
               (send *robot* :angle-vector av)
               (send *robot* :head :neck-p :joint-angle *neck-angle*)
               (model2real_ time))
           (cdr (assoc :av-list target))
           (cdr (assoc :sleep target))))))

(defun gallop-callback
  (msg)
  (describe msg)
  (let ((buf
         (cdr
          (find-if #'(lambda (a) (eq (aref (send msg :data) 0)
                                     (car a)))
                   (list
                    (cons #\q :qrot)
                    (cons #\p :prot)
                    (cons #\n :negaeri)
                    (cons #\f :forward)
                    (cons #\s :stop)
                    (cons #\r :right)
                    (cons #\l :left))))))
    (if (eq buf :forward)
        (setq *forward-offset* (* -1 (send *auto-controller-hash* :get :compus))))
    (setq *gallop-mode* buf)
    ))

(ros::roseus-add-msgs "std_msgs")
(ros::subscribe "/atlas_gallop_controller/command"
                std_msgs::string
                #'gallop-callback)

(ros::advertise "atlas_gallop_controller/compus"
		std_msgs::string)

(defvar *pub-compus* 0)
(defun publish-compus
  (compus utsubuse step)
  (let ((now
	 (*
	  (if (zerop utsubuse) 1 -1)
	  (* step (/ (+ 180 (round compus)) step)))))
    (if (not (eq now *pub-compus*))
	(ros::publish
	 "atlas_gallop_controller/compus"
	 (instance std_msgs::string :init :data
		   (format nil "~A"
			   (setq *pub-compus* now)))))))

(defun atitude-update
  (&optional (goal (send *auto-controller-hash* :get :goal)))
  (send *robot* :newcoords (make-coords :rpy (mapcar #'deg2rad (get-imu-data))))
  (send *irtviewer* :draw-objects)
;  (send (make-coords) :draw-on :flush t :color #F(1 0 0) :size 500)
  (send *auto-controller-hash*
	:enter
	:compus
	(let ((buf (transform (send *robot* :worldrot) #F(0 0 1))))
	  (* (if (minusp (aref (v* buf goal) 2))
		 -1
	       1)
	     (rad2deg (acos (/ (v. buf goal) (norm buf)))))))
  (send *auto-controller-hash*
	:enter
	:utsubuse
	(if (plusp (nth 1 (get-imu-data)))
	    (+ (send *auto-controller-hash* :get :utsubuse) 1)
	  0))
  (publish-compus
   (send *auto-controller-hash* :get :compus)
   (send *auto-controller-hash* :get :utsubuse)
   10)
  ;; (let ((index (send *auto-controller-hash* :get :index))
  ;;       (size (send *auto-controller-hash* :get :ring-buf-size)))
  ;;   (setf
  ;;    (aref (send *auto-controller-hash* :get :position) index)
  ;;    (cadr (assoc :pos (get-pos-estimation))))
  ;;   (send
  ;;    *auto-controller-hash*
  ;;    :enter
  ;;    :velocity
  ;;    (v-
  ;;     (aref (send *auto-controller-hash* :get :position) index)
  ;;     (aref (send *auto-controller-hash* :get :position)
  ;;           (mod (+ (- index 1) size) size))))
  ;;   (send *auto-controller-hash*
  ;;         :enter
  ;;         :index
  ;;         (mod (+ 1 index) size)))
  (format t "fall:~A compus:~A~%"
	  (send *auto-controller-hash* :get :utsubuse)
	  (send *auto-controller-hash* :get :compus)))

(defvar *forward-offset* 0)
(defun command-subscriber
  nil
  (do-until-key
   (case *gallop-mode*
	 (:forward
	  (forward-controller *forward-offset*)
          (setq *gallop-mode* :forward))
	 (:right (play :prot))
	 (:prot (play :prot))
	 (:left (play :qrot))
	 (:qrot (play :qrot))
	 (:stop (play :stop))
	 (:negaeri (negaeri-once)))
   (ros::spin-once)
   (atitude-update)))

(defvar *auto-controller-hash* (make-hash-table))
(send *auto-controller-hash* :enter :goal #F(0 1 0))
(send *auto-controller-hash* :enter :compus 0)
(send *auto-controller-hash* :enter :utsubuse 0)
;(send *auto-controller-hash* :enter :velocity #f(0 0 0))
;(send *auto-controller-hash* :enter :ring-buf-size 5)
;; (send *auto-controller-hash* :enter :position
;;       (coerce
;;        (make-list
;;         (send *auto-controller-hash* :get :ring-buf-size)
;;         :initial-element #F(0 0 0))
;;        vector))
;; (send *auto-controller-hash* :enter :index 0)

(defun rotate-with-orient
  (orient
   &optional (max 30))
  (while (and (> (decf max) 0)
              (progn
                (atitude-update orient)
                (> (abs (send *auto-controller-hash* :get :compus))
                   30)))
    (if (plusp (send *auto-controller-hash* :get :compus))
        (play :qrot)
      (play :prot))))

(defun forward-controller
  (&optional
   (offset 0)
   (compus (+ offset (send *auto-controller-hash* :get :compus))))
  (cond
   ((or (> (send *auto-controller-hash* :get :utsubuse) 5)
        nil)
;	(> *mode-dup* 50))
    (play :negaeri)
    (play :stop))
   ((> compus 50) (play :qrot))
   ((< compus -50) (play :prot))
   ((> compus 15) (play :left))
   ((< compus -15) (play :right))
   (t (play :forward))))

(defun auto-controller
  nil
  (rotate-with-orient
   (send *auto-controller-hash* :get :goal))
  (do-until-key
   (atitude-update)
   (forward-controller)))

(defun wall-controller
  (&optional (rot 60) (buf))
  (do-until-key
   (setq
    buf
    (car
     (sort
      (mapcar
       #'(lambda (r)
           (rotate-with-orient
            (transform
             (rotation-matrix (deg2rad r) :z)
             (send *auto-controller-hash* :get :goal)))
           (list
            (cons :rot r)
            (cons :dist (wall-distance))
            ))
       (list (* -1 rot) 0 rot))
      #'(lambda (a b)
          (> (cdr (assoc :dist a))
             (cdr (assoc :dist b))))
      )))
   (rotation-matrix (send *auto-controller-hash* :get :goal))
   (dotimes (i 10)
     (forward-controller (cdr (assoc :rot buf))))))

#|

(do-until-key
 (send *robot* :newcoords (make-coords :rpy (mapcar #'deg2rad (get-imu-data))))
 (send *irtviewer* :draw-objects)
 (print (round
         (rad2deg (acos (v. (transform (send *robot* :worldrot) *goal*) *goal*)))))
 (unix:sleep 1))


(setq pt (get-tilt-laser))
(atitude-update)
(send pt :transform (send *robot* :worldcoords))
(send pt :convert-to-world)
(objects (list *robot* pt))

(setq
 a
 (remove-if
  #'(lambda (p)
      (or (< (cdr (assoc :size p)) 10)
          (> (abs (cdr (assoc :average p))) 30)))
  (cluster (send pt :point-list) :thre 5)))

(mapcar
 #'(lambda (p)
     (cons
      (cons
       :distance
       (/
        (apply #'+
               (mapcar #'norm (cdr (assoc :data p))))
        (cdr (assoc :size p))))
      p))
 a)

(mapcar
 #'(lambda (a)
     (mapcar
      #'(lambda (d)
          (send d :draw-on :flush t :color #F(1 0 0) :width 50))
      (cdr (assoc :data a)))
     (read-line))
 a)