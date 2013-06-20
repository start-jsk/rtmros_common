
(require "package://hrpsys_gazebo/euslisp/atlas-interface.l")
(require "package://hrpsys_gazebo/euslisp/atlas-tolot.l")
(require "package://hrpsys_gazebo/euslisp/atlas-standup.l")
(require "package://hrpsys_gazebo/euslisp/atlas-wall-detector.l")

(ros::advertise "atlas_gallop_controller/compus"
                std_msgs::string)
(ros::advertise "atlas_gallop_controller/point"
                std_msgs::string)

(defun gallop-standby
  nil
  (my-init
   :set-reset-pose nil
   :servo-gain
   (list (cons :all 15.0))
   )
  (setq *robot* *atlas*)
  (objects *robot*)
  (if (or (not (boundp '*irtviewer*))
          (null *irtviewer*))
      (progn
        (require "package://hrpsys_gazebo/euslisp/no-display.l")
        (make-irtviewer)))
  (tolot-init :reverse? t)
  (set-user)
  (model2real_ 3000)
  (ros::roseus-add-msgs "std_msgs")
  (ros::subscribe "/atlas_gallop_controller/command"
                  std_msgs::string
                  #'gallop-callback)
  (ros::subscribe "/vrc/bytes/remaining/downlink"
                  std_msgs::string
                  #'remain-callback)
  (ros::subscribe "/vrc/bytes/remaining/uplink"
                  std_msgs::string
                  #'remain-callback)
  (ros::roseus-add-msgs "atlas_msgs")
  (ros::subscribe "/vrc_score"
                  atlas_msgs::vrcscore
                  #'score-callback)
  )

(defun gen-dict
  nil
  (list
   (cons :stop
         (list
          (cons :av-list
                (progn
                  (tolot-init)
                  (model2real_ 300)
                  (let ((buf (copy-object (send *robot* :angle-vector))))
                    (vector buf))))
          (cons :sleep #(1000))))
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
   (cons :tolot
         (list
          (cons :av-list
                (progn
                  (tolot-init)
                  (send *tolot-hash* :enter :direction #(#F(-200 0 0) #F(-200 0 0)))
                  (tolot-step :reverse? t :rotate 0 :sleep 100)
                  (copy-object (send *tolot-hash* :get :dict))))
          (cons :sleep #(100 100))))
   (cons :brot
         (list
          (cons :av-list
                (progn
                  (tolot-init)
                  (send *tolot-hash* :enter :direction #(#F(-20 -30 0) #F(-50 50 0)))
                  (tolot-step :mirror? t :reverse? t :sleep 100 :rotate 30)
                  (copy-object (send *tolot-hash* :get :dict))))
          (cons :sleep #(100 100))))
   (cons :drot
         (list
          (cons :av-list
                (progn
                  (tolot-init)
                  (send *tolot-hash* :enter :direction #(#F(-50 -50 0) #F(-20 30 0)))
                  (tolot-step :reverse? t :sleep 100 :rotate -30)
                  (copy-object (send *tolot-hash* :get :dict))))
          (cons :sleep #(100 100))))
   (cons :negaeri 'negaeri-once)))

(defun negaeri-once
  nil
  (negaeri-from-utubuse)
  (setq *gallop-mode* :stop))

(setq *gallop-dict*
      '((:stop (:av-list . #(#f(-0.003048 6.09591 -0.053802 105.109 97.2697 0.0 -26.5584 105.814 0.0 -105.089 97.0922 0.0 26.6117 105.805 0.0 99.6314 1.06171 2.72941 17.1759 -4.57762 -1.29622 99.6322 -0.980218 2.75578 -17.0183 -4.58332 1.34451 0.0))) (:sleep . #(1000))) (:forward (:av-list . #(#f(0.02106 19.9715 -0.063195 108.512 72.4522 0.0 60.0 99.8673 0.0 -108.271 72.0259 0.0 -60.0 99.6771 0.0 89.4923 1.78779 6.2236 17.7434 -3.04163 0.879951 89.4598 -1.65378 6.27114 -17.6136 -3.01217 -0.762856 0.0) #f(-0.006872 -6.1332 -0.051529 103.378 89.4835 0.0 -15.9431 111.961 0.0 -103.391 89.3914 0.0 15.9614 111.968 0.0 132.403 -12.2784 27.2545 25.3349 -51.5662 -18.3346 132.377 12.3472 27.2685 -25.2106 -51.5662 18.3346 0.0))) (:sleep . #(100 100))) (:right (:av-list . #(#f(-0.013919 23.9251 -30.0 107.668 92.2545 0.0 60.0 98.3323 0.0 -107.538 92.0215 0.0 -60.0 98.2446 0.0 93.1815 1.3889 4.89525 17.6166 -4.30003 0.174148 93.177 -1.30705 4.92571 -17.4672 -4.29974 -0.11187 0.0) #f(0.555412 -5.75039 -30.0 105.158 62.8758 0.0 -22.1684 111.615 0.0 -102.662 106.447 0.0 12.2553 111.504 0.0 133.195 -9.81029 27.208 28.3614 -51.5662 -18.3346 130.807 15.5738 28.31 -17.6376 -51.5662 18.3346 0.0))) (:sleep . #(100 100))) (:left (:av-list . #(#f(-0.013919 23.9251 30.0 107.668 92.2545 0.0 60.0 98.3323 0.0 -107.538 92.0215 0.0 -60.0 98.2446 0.0 93.1815 1.3889 4.89525 17.6166 -4.30003 0.174148 93.177 -1.30705 4.92571 -17.4672 -4.29974 -0.11187 0.0) #f(-0.550973 -5.76289 30.0 102.653 106.493 0.0 -12.247 111.495 0.0 -105.157 62.9009 0.0 22.1592 111.618 0.0 130.825 -15.519 28.3084 17.7481 -51.5662 -18.3346 133.181 9.83779 27.2271 -28.3614 -51.5662 18.3346 0.0))) (:sleep . #(100 100))) (:prot (:av-list . #(#f(-34.0992 15.3573 0.0 82.5364 20.9453 0.0 60.0 112.5 0.0 -105.703 0.130668 0.0 -60.0 52.4805 0.0 107.194 -5.91914 -12.2449 23.3883 6.90871 -6.21829 105.621 -6.25553 -3.36497 -6.59642 -4.44814 -1.36072 0.0) #f(33.6347 15.5531 0.0 113.606 76.6479 0.0 -19.8355 78.2388 0.0 -80.7925 89.5449 0.0 51.3878 112.5 0.0 108.927 2.1324 7.48571 19.963 -17.7766 -5.69368 119.599 5.43001 -3.33338 -18.6435 -15.0183 4.08233 0.0))) (:sleep . #(100 100))) (:qrot (:av-list . #(#f(34.7483 15.5818 0.0 105.988 0.135829 0.0 60.0 51.9628 0.0 -81.859 20.6065 0.0 -60.0 112.5 0.0 105.816 6.34649 -3.35018 6.55935 -4.46118 1.54433 107.264 5.91712 -12.2369 -23.4663 7.06727 6.43399 0.0) #f(-33.643 15.5096 0.0 80.8086 89.5727 0.0 -51.4134 112.5 0.0 -113.599 76.3776 0.0 19.9786 78.1938 0.0 119.605 -5.37253 -3.36691 18.7554 -14.9949 -4.04987 108.937 -2.06656 7.48758 -19.8117 -17.7692 5.7331 0.0))) (:sleep . #(100 100))) (:tolot (:av-list . #(#f(33.8454 17.2987 0.0 114.508 72.3192 0.0 30.0 78.5293 0.0 -81.0531 79.133 0.0 -30.0 112.5 0.0 131.176 -3.63089 -1.91747 21.8434 -29.2754 -10.9932 95.3759 0.494701 5.01827 -16.3715 -2.03611 -1.00262 0.0) #f(-33.8543 17.2572 0.0 81.0632 79.1719 0.0 -50.1101 112.5 0.0 -114.546 72.0034 0.0 22.6569 78.5143 0.0 95.3764 -0.440599 4.98976 16.53 -2.01939 1.03466 131.193 3.64904 -1.9328 -21.7673 -29.2832 10.9956 0.0))) (:sleep . #(100 100))) (:brot (:av-list . #(#f(-35.0 16.8463 30.0 80.8566 86.4222 0.0 30.0 112.5 0.0 -115.55 75.8875 0.0 -30.0 78.7854 0.0 96.8193 -0.984257 5.42203 18.6126 -2.91636 -0.032404 116.36 -0.474493 -12.102 -17.4992 -5.48042 6.10913 0.0) #f(35.0 17.2905 30.0 116.729 80.5532 0.0 -19.0793 80.4611 0.0 -80.5846 86.0286 0.0 52.1699 112.5 0.0 118.804 -1.34965 -11.2334 20.2008 -8.26037 -8.44896 96.8373 1.67013 5.22951 -17.9288 -2.82798 0.533762 0.0))) (:sleep . #(100 100))) (:drot (:av-list . #(#f(34.1561 16.0954 -30.0 111.468 72.9045 0.0 30.0 75.059 0.0 -82.0843 85.5975 0.0 -30.0 112.499 0.0 119.507 2.67421 -10.1894 11.1528 -11.0552 -2.78083 95.3257 -0.139219 6.45923 -16.8345 -2.81909 -1.34271 0.0) #f(-34.3994 16.283 -30.0 82.0099 86.7447 0.0 -52.2256 112.498 0.0 -112.253 73.0025 0.0 22.2897 75.4222 0.0 95.9698 -0.189405 6.20977 17.366 -3.04587 0.929679 116.561 -2.36576 -11.3176 -12.2828 -6.8961 3.3097 0.0))) (:sleep . #(100 100))) (:negaeri . negaeri-once)))



                                        ;  '((:stop (:av-list . #(#f(-0.003048 6.09591 -0.053802 105.109 97.2697 0.0 -26.5584 105.814 0.0 -105.089 97.0922 0.0 26.6117 105.805 0.0 99.6314 1.06171 2.72941 17.1759 -4.57762 -1.29622 99.6322 -0.980218 2.75578 -17.0183 -4.58332 1.34451 0.0) #f(-0.003048 6.09591 -0.053802 105.109 97.2697 0.0 -26.5584 105.814 0.0 -105.089 97.0922 0.0 26.6117 105.805 0.0 99.6314 1.06171 2.72941 17.1759 -4.57762 -1.29622 99.6322 -0.980218 2.75578 -17.0183 -4.58332 1.34451 0.0))) (:sleep . #(300 300))) (:forward (:av-list . #(#f(0.02106 19.9715 -0.063195 108.512 72.4522 0.0 60.0 99.8673 0.0 -108.271 72.0259 0.0 -60.0 99.6771 0.0 89.4923 1.78779 6.2236 17.7434 -3.04163 0.879951 89.4598 -1.65378 6.27114 -17.6136 -3.01217 -0.762856 0.0) #f(-0.006872 -6.1332 -0.051529 103.378 89.4835 0.0 -15.9431 111.961 0.0 -103.391 89.3914 0.0 15.9614 111.968 0.0 132.403 -12.2784 27.2545 25.3349 -51.5662 -18.3346 132.377 12.3472 27.2685 -25.2106 -51.5662 18.3346 0.0))) (:sleep . #(100 100))) (:right (:av-list . #(#f(-0.013919 23.9251 -30.0 107.668 92.2545 0.0 60.0 98.3323 0.0 -107.538 92.0215 0.0 -60.0 98.2446 0.0 93.1815 1.3889 4.89525 17.6166 -4.30003 0.174148 93.177 -1.30705 4.92571 -17.4672 -4.29974 -0.11187 0.0) #f(0.555412 -5.75039 -30.0 105.158 62.8758 0.0 -22.1684 111.615 0.0 -102.662 106.447 0.0 12.2553 111.504 0.0 133.195 -9.81029 27.208 28.3614 -51.5662 -18.3346 130.807 15.5738 28.31 -17.6376 -51.5662 18.3346 0.0))) (:sleep . #(100 100))) (:left (:av-list . #(#f(-0.013919 23.9251 30.0 107.668 92.2545 0.0 60.0 98.3323 0.0 -107.538 92.0215 0.0 -60.0 98.2446 0.0 93.1815 1.3889 4.89525 17.6166 -4.30003 0.174148 93.177 -1.30705 4.92571 -17.4672 -4.29974 -0.11187 0.0) #f(-0.550973 -5.76289 30.0 102.653 106.493 0.0 -12.247 111.495 0.0 -105.157 62.9009 0.0 22.1592 111.618 0.0 130.825 -15.519 28.3084 17.7481 -51.5662 -18.3346 133.181 9.83779 27.2271 -28.3614 -51.5662 18.3346 0.0))) (:sleep . #(100 100))) (:prot (:av-list . #(#f(-34.0992 15.3573 0.0 82.5364 20.9453 0.0 60.0 112.5 0.0 -105.703 0.130668 0.0 -60.0 52.4805 0.0 107.194 -5.91914 -12.2449 23.3883 6.90871 -6.21829 105.621 -6.25553 -3.36497 -6.59642 -4.44814 -1.36072 0.0) #f(33.6347 15.5531 0.0 113.606 76.6479 0.0 -19.8355 78.2388 0.0 -80.7925 89.5449 0.0 51.3878 112.5 0.0 108.927 2.1324 7.48571 19.963 -17.7766 -5.69368 119.599 5.43001 -3.33338 -18.6435 -15.0183 4.08233 0.0))) (:sleep . #(100 100))) (:qrot (:av-list . #(#f(34.7483 15.5818 0.0 105.988 0.135829 0.0 60.0 51.9628 0.0 -81.859 20.6065 0.0 -60.0 112.5 0.0 105.816 6.34649 -3.35018 6.55935 -4.46118 1.54433 107.264 5.91712 -12.2369 -23.4663 7.06727 6.43399 0.0) #f(-33.643 15.5096 0.0 80.8086 89.5727 0.0 -51.4134 112.5 0.0 -113.599 76.3776 0.0 19.9786 78.1938 0.0 119.605 -5.37253 -3.36691 18.7554 -14.9949 -4.04987 108.937 -2.06656 7.48758 -19.8117 -17.7692 5.7331 0.0))) (:sleep . #(100 100))) (:negaeri . negaeri-once)))

(defvar *gallop-mode* :stop)
(defvar *neck-angle* -40)
(defvar *prev-mode* :stop)
(defvar *mode-dup* 0)
(defvar *command-cnt* 0)

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
;  (describe msg)
  (cond
   ((or (null (send msg :data))
	(not (plusp (length (send msg :data)))))
    )
   ((find (aref (send msg :data) 0) (list #\h #\y #\o))
    (case (aref (send msg :data) 0)
	  (#\h
	   (print 'hard-mode)
	   (atlas-set-servo-gain-by-torque-limit 1.0))
	  (#\y
	   (print 'soft-mode)
	   (atlas-set-servo-gain-by-torque-limit 15.0))
	  (#\o
	   (if (> (length (send msg :data)) 3)
	       (let ((ori
		      (read-from-string
		       (format nil "#f(~A)"
			       (subseq (send msg :data) 2)))))
		 (format t "set goal ~A~%" ori)
		 (if (= (length ori) 3)
		     (send *auto-controller-hash* :enter :goal ori)
		   (print 'wrong-orient)))))))
   (t
    (let ((buf
	   (cdr
	    (find-if #'(lambda (a) (eq (aref (send msg :data) 0)
				       (car a)))
		     (list
		      (cons #\a :auto)
		      (cons #\q :qrot)
		      (cons #\p :prot)
		      (cons #\n :negaeri)
		      (cons #\f :forward)
		      (cons #\t :tolot)
		      (cons #\s :stop)
		      (cons #\r :right)
		      (cons #\l :left))))))
      (if (find buf '(:forward :tolot))
	  (setq *forward-offset* (* -1 (send *auto-controller-hash* :get :compus))))
      (if (find buf '(:auto))
	  (progn
	    (setq *forward-offset* 0)
	    (setq *point-cloud* nil)
	    (rotate-with-orient (send *auto-controller-hash* :get :goal))))
      (setq *gallop-mode* buf)
      (setq *command-cnt* 5)
      ))))

(defun atitude-update
  (&optional (goal (send *auto-controller-hash* :get :goal)))
  (if (or (not (boundp '*atlas_rpy*))
          (null *atlas_rpy*))
      (setq *atlas_rpy* '(0 0 0)))
  (send *robot* :newcoords
	(make-coords :rpy (mapcar #'deg2rad *atlas_rpy*)))
  (if (and (boundp '*irtviewer*) *irtviewer*)
      (send *irtviewer* :draw-objects))
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
	(if (plusp (- (nth 1 *atlas_rpy*) 45))
	    (+ (send *auto-controller-hash* :get :utsubuse) 1)
	  0))
  (publish-compus
   (send *auto-controller-hash* :get :compus)
   (send *auto-controller-hash* :get :utsubuse)
   10)
  (format t "fall:~A compus:~A~%"
	  (send *auto-controller-hash* :get :utsubuse)
	  (send *auto-controller-hash* :get :compus)))

(defvar *vrc-score* nil)
(defun score-callback
  (msg)
  (cond
   ((null *vrc-score*)
    (setq *vrc-score* (copy-object msg)))
   ((not (eq (send *vrc-score* :falls)
	     (send msg :falls)))
    (print 'fall)
    (ros::publish
     "atlas_gallop_controller/point"
     (instance std_msgs::string :init :data "f")))
   ((or
     (not (eq (send *vrc-score* :completion_score)
              (send msg :completion_score)))
     (and (stringp (send msg :message))
          (plusp (length (send msg :message)))))
    (print 'score)
    (ros::publish
     "atlas_gallop_controller/point"
     (instance std_msgs::string :init :data "s"))))
  (setq *vrc-score* (copy-object msg)))

(defvar *auto-started* nil)
(defun remain-callback
  (msg)
  (let ((buf (if (and msg (send msg :data)) (read-from-string (send msg :data)))))
    (cond
     (*auto-started* nil)
     ((and (numberp buf) (<= buf 0))
      (setq *auto-started* t)
      (setq *gallop-mode* nil)
      (setq *forward-offset* 0)
      (setq *point-cloud* nil)
      (rotate-with-orient (send *auto-controller-hash* :get :goal))
      (setq *gallop-mode* :auto))
     )))

(defvar *pub-compus* 0)
(defvar *prev-pub-time* (send (ros::time) :now))
(defun publish-compus
  (compus utsubuse step)
  (let ((now
	 (*
	  (if (zerop utsubuse) 1 -1)
	  (* step (/ (+ 180 (round compus)) step))))
	(tm (send (ros::time) :now)))
    (if (and (not (eq now *pub-compus*))
	     (> (send (ros::time- tm *prev-pub-time*) :to-sec) 1))
	(progn
	  (ros::publish
	   "atlas_gallop_controller/compus"
	   (instance std_msgs::string :init :data
		     (format nil "~A"
			     (setq *pub-compus* now))))
	  (setq *prev-pub-time* tm)))))

(defvar *forward-offset* 0)
(defun command-subscriber
  nil
  (do-until-key
   (print *gallop-mode*)
   (case *gallop-mode*
     (:forward
      (forward-controller *forward-offset*)
      (setq *gallop-mode* :forward))
     (:tolot
      (tolot-forward-controller *forward-offset*)
      (setq *gallop-mode* :tolot))
     (:right (play :prot))
     (:prot (play :prot))
     (:left (play :qrot))
     (:qrot (play :qrot))
     (:stop (play :stop))
     (:auto
      (async-wall-controller-step)
      (setq *gallop-mode* :auto))
     (:negaeri (negaeri-once)))
   (ros::spin-once)
   (ros::sleep)
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
   &key
   (max 30)
   (thre 30))
  (while (and (> (decf max) 0)
              (progn
		(ros::spin-once)
		(ros::sleep)
                (atitude-update orient)
                (> (abs (send *auto-controller-hash* :get :compus))
                   thre)))
    (cond
     ((> (send *auto-controller-hash* :get :utsubuse) 5)
      (play :negaeri)
      (play :stop))
     ((plusp (send *auto-controller-hash* :get :compus))
      (play :qrot))
     (t (play :prot)))
    )
  (> max 0)
;  (dotimes (i 5)
;    (if (plusp (send *auto-controller-hash* :get :compus))
;        (play :qrot)
;      (play :prot)))
  )

(defun forward-controller
  (&optional
   (offset 0)
   (compus (+ offset (send *auto-controller-hash* :get :compus))))
  (cond
   ((> (send *auto-controller-hash* :get :utsubuse) 5)
                                        ;	)
    (play :negaeri)
    (play :stop))
   ((> *mode-dup* 50) (dotimes (i 5) (play :right)))
   ((> compus 50) (play :qrot))
   ((< compus -50) (play :prot))
   ((> compus 15) (play :left))
   ((< compus -15) (play :right))
   (t (play :forward))))

(defun tolot-forward-controller
  (&optional
   (offset 0)
   (compus (+ offset (send *auto-controller-hash* :get :compus))))
  (cond
   ((> (send *auto-controller-hash* :get :utsubuse) 5)
                                        ;	)
    (play :negaeri)
    (play :stop))
   ((> *mode-dup* 50) (dotimes (i 5) (play :right)))
   ((> compus 30) (play :qrot))
   ((< compus -30) (play :prot))
   ((> compus 40) (play :drot))
   ((< compus -40) (play :brot))
   (t (play :tolot))))

(defun auto-controller
  nil
  (rotate-with-orient
   (send *auto-controller-hash* :get :goal))
  (do-until-key
   (ros::spin-once)
   (ros::sleep)
   (atitude-update)
   (forward-controller)))

(defvar *laser-data*)
(defun point-cloud-callback
  (msg)
  (ros::spin-once)
  (ros::sleep)
  (atitude-update)
  (let* ((buf (orient-cluster-meta
               :pc *point-cloud*
               :pc-coords (send *robot* :copy-worldcoords)
               :forward (send *auto-controller-hash* :get :goal)))
         (mae (remove-if
                 #'(lambda (deg)
                     (> (abs (- (cdr (assoc :deg deg))
                                (send *auto-controller-hash* :get :compus)))
                        60))
                 buf))
         (sort
          (sort
           mae
           #'(lambda (a b)
               (> (norm (cdr (assoc :average a)))
                  (norm (cdr (assoc :average b))))))))
    (objects (list *Robot* *point-cloud*))
    (setq *laser-data* sort)))

(defun avoid-controller
  (&optional
   (offset #f(0 0 0))
   (forward
    (normalize-vector
     (v+ offset
         (transform
          (send *robot* :worldrot)
          #F(0 0 1)))))
   (revert nil))
  (rotate-with-orient (scale -1 forward) :max 20)
  (dotimes (i 5) (play :forward))
  (if revert (rotate-with-orient forward))
  )

(defun wall-controller
  (&key
   (goal (send *auto-controller-hash* :get :goal))
   buf)
  (do-until-key
   (if (rotate-with-orient
        (send *auto-controller-hash* :get :goal))
       (progn
         (tolot-init)
         (model2real_)
	 (ros::spin-once)
	 (ros::sleep)
         (atitude-update)
         (setq
          buf
          (normalize-vector
           (v+
            goal
            (map float-vector
                 #'*
                 #f(0.05 0.005 0)
                 (wall-potential :pc-coords nil)))))
         (rotate-with-orient buf)
         (setq *forward-offset*
               (* (if (minusp (aref (v* buf goal) 2)) 1 -1)
                  (rad2deg (acos (v. buf goal)))))
                                        ;(atitude-update)
         (dotimes (i 8)
           (ros::spin-once)
           (ros::sleep)
           (atitude-update)
           (forward-controller *forward-offset*)))))
  )

(defun async-wall-controller-step
  nil
  (if *point-cloud*
      (let ((goal (send *auto-controller-hash* :get :goal)))
	(setq
	 buf
	 (normalize-vector
	  (v+
	   goal
	   (map float-vector
		#'*
		#f(0.05 0.005 0)
		(wall-potential
		 :point-cloud *point-cloud*
		 :pc-coords nil
		 :min-height 1500)))))
	(setq *forward-offset*
	      (* (if (minusp (aref (v* buf goal) 2)) 1 -1)
		 (rad2deg (acos (v. buf goal)))))
	(setq *point-cloud* nil)))
  (forward-controller *forward-offset*))

(defun async-wall-controller
  (&key
   (goal (send *auto-controller-hash* :get :goal))
   buf)
  (setq *forward-offset* 0)
  (setq *point-cloud* nil)
  (rotate-with-orient goal)
  (do-until-key
   (ros::spin-once)
   (ros::sleep)
   (atitude-update)
   (async-wall-controller-step)
   ))


(format t
 "~A~%~A~%"
 "[USAGE] (progn (gallop-standby) (command-subscriber))"
 "wget http://www.jsk.t.u-tokyo.ac.jp/~s-noda/AtlasGallopController.jar ; java -jar AtlasGallopController $ROS_IP $ROS_MASTER_URI")

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