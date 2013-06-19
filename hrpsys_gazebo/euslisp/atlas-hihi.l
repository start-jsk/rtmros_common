;;(require "package://hrpsys_gazebo/euslisp/atlas-interface.l")
;(require "atlas-interface.l") ; (progn (require "atlas-interface.l") (atlas-init) (setq *robot* *atlas*) (objects *robot*))

;;------------ initalize

(defmethod rotational-joint
  (:worldrot
   nil
   (send (send self :child-link) :worldrot))
  (:worldpos
   nil
   (send (send self :child-link) :worldpos)))

;; (defvar *gazebo-flag* nil)
;; (if *gazebo-flag*
;;     (progn
;;       (require "atlas-interface.l")
;;       (atlas-init))
(defvar *atlas* (atlas-with-hand)) ;(instance atlas-robot :init)))
(defvar *robot* *atlas*)
(defvar *move-step-time* 100)

(setq
 *atlas-anima-endcoords*
 (progn
   (send *robot* :angle-vector
	 (scale 0 (send *robot* :angle-vector)))
   (append
    (mapcar
     #'(lambda (k)
	 (cons
	  k
	  (let ((c (send (send *robot* :rleg :end-coords) :copy-worldcoords)))
	    (send c :rotate (deg2rad 90) :z)
	    (make-cascoords
	     :pos (copy-seq (send *robot* k :wrist-p :worldpos))
	     :rot (copy-object (send c :worldrot))
	     :name (read-from-string (format nil "~A-animal-endcoords" k))
	     :parent (send *robot* k :wrist-p :child-link)))))
     '(:rarm :larm))
    (mapcar
     #'(lambda (k)
	 (cons
	  (read-from-string (format nil "~A-2" k))
	  (let ((c (send (send *robot* :rleg :end-coords) :copy-worldcoords)))
	    (send c :rotate (deg2rad 90) :z)
	    (send c :translate #F(0 0 -200))
	    (make-cascoords
	     :pos (copy-seq (send *robot* k :wrist-p :worldpos))
	     :rot (copy-object (send c :worldrot))
	     :name (read-from-string (format nil "~A-animal-endcoords" k))
	     :parent (send *robot* k :wrist-p :child-link)))))
     '(:rarm :larm))
    (mapcar
     #'(lambda (k)
    	 (cons
    	  k
    	  (let ((c (send (send *robot* :rleg :end-coords) :copy-worldcoords)))
    	    (send c :rotate (deg2rad -85) :y)
    	    (send c :rotate
    		  (deg2rad
    		   (* (if (eq k :rleg) 1 -1) 20))
    		  :x)
    	    (make-cascoords
    	     :pos (scale 0.5 (v+ (send *robot* k :knee-p :worldpos)
    				 (send *robot* k :ankle-p :worldpos)))
    	     :rot (copy-object (send c :worldrot))
    	     :name (read-from-string (format nil "~A-animal-endcoords" k))
    	     :parent (send *robot* k :knee-p :child-link))))
    	 )
     '(:rleg :lleg))
    (mapcar
     #'(lambda (k)
    	 (cons
    	  (read-from-string (format nil "~A-2" k))
    	  (let ((c (send (send *robot* :rleg :end-coords) :copy-worldcoords)))
    	    (send c :rotate (deg2rad -90) :y)
    	    (send c :rotate
    		  (deg2rad
    		   (* (if (eq k :rleg) 1 -1) 20))
    		  :x)
    	    (make-cascoords
    	     :pos (v+ (scale 0.3 (send *robot* k :knee-p :worldpos))
    		      (scale 0.7 (send *robot* k :ankle-p :worldpos)))
    	     :rot (copy-object (send c :worldrot))
    	     :name (read-from-string (format nil "~A-animal-endcoords" k))
    	     :parent (send *robot* k :knee-p :child-link))))
    	 )
     '(:rleg :lleg))
    )))

;(if (not (boundp '*irtviewer*)) (make-irtviewer))
;(send *irtviewer* :objects (list *robot*))
;(send *irtviewer* :look-all)

;; ---------- utility function

(defun model2real
  (&key (robot *robot*) (sleep-time *move-step-time*))
  (send *ri* :angle-vector
	(send *robot* :angle-vector) sleep-time)
  (if (find-method *ri* :hand-angle-vector)
      (send *ri* :hand-angle-vector
	    (apply #'concatenate
		   float-vector (send robot :arms :hand :angle-vector))
	    sleep-time)))

(defun model2real_
  (&optional (sl *move-step-time*))
  (if (and (boundp '*irtviewer*) *irtviewer*)
      (send *irtviewer* :draw-objects))
  (if
      (boundp '*ri*)
      (progn (model2real :sleep-time sl) (send *ri* :wait-interpolation))
    (unix:usleep (* sl 1000))
    ))


(defun simple-fullbody
  (&rest
   args
   &key
   (robot *robot*)
   (center #f(0 0 0))
   (target-centroid-pos nil)
   (torso-null-space nil)
   (balance-leg :both)
   (target nil) ;; ( ((:target . :rleg) (:move . #f(0 0 0)) (:translation-axis . :x)) ...
   &allow-other-keys)
;  (print args)
  (let* ((rest-arg
	  (labels ((itter (obj key)
			  (cond
			   ((null obj) nil)
			   ((member (car obj) key) (itter (cddr obj) key))
			   (t (cons (car obj)
				    (cons (cadr obj)
					  (itter (cddr obj) key)))))))
		  (itter args (list :robot :center :balance-leg
				    :target :target-centroid-pos
				    :torso-null-space))))
	 (axis-move-key
	  (list :rotation-axis :translation-axis :thre))
	 (target-centroid-pos
	  (if (find :target-centroid-pos args)
	      target-centroid-pos
	    (v+ center
					;(send robot :calc-target-centroid-pos
					;      balance-leg '(:lleg :rleg))
		(scale
		 (/ 1.0 (case balance-leg
			      (:both 2)
			      (t 1)))
		 (reduce
		  #'v+
		  (append
		   (list #f(0 0 0) #f(0 0 0))
		   (mapcar
		    #'(lambda (k)
			(send robot k :end-coords :worldpos))
		    (case balance-leg
			  (:rleg '(:rleg))
			  (:lleg '(:lleg))
			  (:both '(:rleg :lleg)))))))
		)))
	 (axis-move-with-leg
	  (if balance-leg
	      (let ((lleg (if (find-if
			       #'(lambda (a)
				   (or (eq (cdr (assoc :target a)) :lleg)
				       (and
					(find-method a :name)
					(reg-match
					 "lleg"
					 (format nil "~A" (send a :name))))))
			       target)
			      nil (list (list (cons :target :lleg)))))
		    (rleg (if (find-if
			       #'(lambda (a)
				   (or (eq (cdr (assoc :target a)) :rleg)
				       (and
					(find-method a :name)
					(reg-match
					 "rleg"
					 (format nil "~A" (send a :name))))))
			       target)
			      nil
			    (list (list (cons :target :rleg))))))
		(case balance-leg
		      (:lleg (append lleg target))
		      (:rleg (append rleg target))
		      (:both (append lleg rleg target))))
	    target))
					; move-target-buf
	 dissoc-buf
	 (fill-axis-move
	  (mapcar
	   #'(lambda (am)
	       (append
		(let* ((obj (cdr (assoc :target am)))
		       (move (cdr (assoc :move am)))
		       (rpy (cdr (assoc :rpy am)))
		       (worldcoords (cdr (assoc :coords am)))
		       (dissoc? nil)
		       link-list target-coords move-target)
		  (setq move-target
			(cond ((keywordp obj)
			       (send robot obj :end-coords))
			      ((eq 'cascaded-coords (send (class obj) :name))
			       obj)
			      ((or (eq 'robot-link (send (class obj) :name))
				   (eq 'bodyset-link (send (class obj) :name)))
			       (setq dissoc? t)
			       (make-cascoords :init :link-list
					       :coords (send obj :worldcoords)
					       :parent obj))
			      (t
			       (setq dissoc? t)
			       (make-cascoords :init :link-list
					       :pos
					       (copy-seq (send obj :worldpos))
					       :rot
					       (copy-object
						(send obj :worldrot))
					       :name
					       (if (find-method obj :name)
						   (send obj :name)
						 :unknown)
					       :parent (send obj :child-link)))
			      ))
		  (if dissoc?
		      (push move-target dissoc-buf))
		  (setq link-list
			(send robot :link-list (send move-target :parent)))
		  (setq target-coords
			(send (send move-target :worldcoords)
			      :copy-worldcoords))
		  (if worldcoords (setq target-coords worldcoords))
		  (if move (send target-coords :translate move :world))
		  (if rpy
		      (apply #'send (append (list target-coords :rpy) rpy)))
		  (list (cons :move-target move-target)
			(cons :target-coords target-coords)
			(cons :link-list link-list)))
		(mapcar
		 #'(lambda (key)
		     (let ((obj (assoc key am)))
		       (case key
			     (:thre (cons :thre (if obj (cdr obj) 5)))
			     (:rotation-axis (cons :rotation-axis (if obj (cdr obj) t)))
			     (:translation-axis (cons :translation-axis (if obj (cdr obj) t))))
		       ))
		 axis-move-key)
		)
	       )
	   axis-move-with-leg))
	 (get-target-link #'(lambda (key)
			      (mapcar #'(lambda (am)
					  (cdr (assoc key am))) fill-axis-move)))
	 (torso-null-space
	  (if torso-null-space
	      (let* ((dof
		      (send robot :calc-target-joint-dimension
			    (mapcar
			     #'(lambda (k)
				 (send robot :link-list
				       (send robot k :end-coords :parent)))
			     (mapcar #'(lambda (tar) (cdr (assoc :target tar)))
				     axis-move-with-leg))))
		     (tv (instantiate float-vector (+ dof 6))))
		(dotimes (i (length torso-null-space))
		  (setf (aref tv i) (aref torso-null-space i)))
		tv)))
	 ret
	 )
    (setq
     ret
     (apply #'send
	    (append
	     (list robot :fullbody-inverse-kinematics
		   (funcall get-target-link :target-coords)
		   :link-list (funcall get-target-link :link-list)
		   :move-target (funcall get-target-link :move-target)
		   :target-centroid-pos target-centroid-pos)
	     (if torso-null-space
		 (list :null-space #'(lambda nil torso-null-space))
	       nil)
	     (reduce
	      #'append
	      (mapcar #'(lambda (key)
			  (list key (funcall get-target-link key)))
		      axis-move-key))
	     rest-arg)
	    ))
    (mapcar #'(lambda (mt) (send (send mt :parent) :dissoc mt)) dissoc-buf)
    ret
    ))

(defun init-pose (&key (robot *robot*) (move 0) (rot-z 0) (debug-view nil))
  (send robot :reset-pose)
  (send robot :newcoords (make-coords))
  (simple-fullbody
   :debug-view debug-view
   :centroid-thre 5
   :target-centroid-pos
   (scale 0.5 (apply #'v+ (send robot :legs :end-coords :worldpos)))
   :target
   (list
    (list (cons :target :rleg)
	  (cons :rpy (list rot-z 0 0))
	  (cons :move (float-vector 0 move 0)))
    (list (cons :target :lleg)
	  (cons :rpy (list (* -1 rot-z) 0 0))
	  (cons :move (float-vector 0 (* -1 move) 0)))))
  (send robot :locate
	(float-vector
	 0 0  (* -1 (aref (send (send robot :lleg :end-coords) :worldpos) 2))))
  )

;; ------------------- hihi

(defun animal-fullbody
  (&key
   (move-target nil)
   (move-option)
   (rotation-axis :z)
   (translation-axis t)
   (back-centroid nil)
   (stop 30)
   (revert? nil)
   (target-keys
    (remove (read-from-string
	     (format nil "~A~A" move-target
		     (if back-centroid "-2" "")))
	    (if back-centroid
		(list :rarm-2 :rleg-2 :larm-2 :lleg-2)
	      (list :rarm :rleg :larm :lleg))))
   (move (mapcar #'(lambda (hoge) #F(0 0 0)) target-keys))
   (debug-view :no-message)
   (centroid-thre (* (length target-keys) 10))
   (end-coords *atlas-anima-endcoords*)
   (target-cascoords
    (mapcar
     #'(lambda (k) (cdr (assoc k end-coords)))
     target-keys))
   (target-centroid-pos (scale
			 (/ 1.0 (length target-cascoords))
			 (reduce #'v+ (send-all target-cascoords :worldpos))))
   )
  (if (atom rotation-axis)
      (setq rotation-axis
	    (make-list
	     (length target-cascoords)
	     :initial-element rotation-axis)))
  (if (atom translation-axis)
      (setq translation-axis
	    (make-list
	     (length target-cascoords)
	     :initial-element translation-axis)))
  (simple-fullbody
   :target
   (append
    move-option
    (mapcar
     #'(lambda (cascoords trans rotate move)
	 (list (cons :target cascoords)
	       (cons :coords (make-coords
	        	      :pos
                              (v+
                               (or move #F(0 0 0))
                               (map float-vector
                                    #'*
                                    #F(1 1 0)
                                    (send cascoords :worldpos)))))
	       (cons :rotation-axis rotate)
	       (cons :translation-axis trans)))
     target-cascoords translation-axis rotation-axis move))
   :balance-leg nil
   :target-centroid-pos target-centroid-pos
   :debug-view debug-view
   :centroid-thre centroid-thre
   :revert-if-fail revert?
   :stop stop
   :cog-gain 0.1
   ))

(defun animal-init
  (&key back-centroid)
  (init-pose)
  (send *robot*
	:angle-vector
	(scale 0 (send *robot* :angle-vector)))
  (if (not back-centroid)
      (progn
	(send *robot* :rarm :elbow-p :joint-angle -100)
	(send *robot* :larm :elbow-p :joint-angle 100)))
  (send *robot* :arms :shoulder-y :joint-angle -80)
  (send *robot* :rotate (deg2rad 45) :y)
  (send *robot* :legs :ankle-p :joint-angle 100)
  (send *robot* :torso :waist-p :joint-angle 70)
  (send *robot* :legs :knee-p :joint-angle 100)
  (send *robot* :legs :crotch-p :joint-angle -60)
  (send *robot* :rleg :crotch-y :joint-angle -20)
  (send *robot* :lleg :crotch-y :joint-angle 20)
  (send *robot* :larm :elbow-p :joint-angle 90)
  (send *robot* :rarm :elbow-p :joint-angle -90)
  (animal-fullbody
   :back-centroid back-centroid)
  (animal-fullbody
   :back-centroid back-centroid))

(defun move-step
  (key
   &key
   buf
   (move #F(200 0 0))
   (real-flag (and (boundp '*ri*) *ri*))
   (back-centroid (find key '(:rarm :larm)))
   )
  (animal-fullbody
   :move-target key
   :back-centroid back-centroid
   :move-option
   (list
    (list (cons :target
		(if (find key '(:rleg :lleg))
		    (send *robot* key :knee-p)
		  (send *robot* key :elbow-p)))
	  (cons :move #F(0 0 0))
	  (cons :translation-axis #f(0 0 1))
	  (cons :rotation-axis :z))))
  (if (and (boundp '*irtviewer*) *irtviewer*)
      (send *irtviewer* :look-all))
  (push (send *robot* :angle-vector) buf)
  (if real-flag (model2real_)); (read-line))
  (animal-fullbody
   :move-target key
   :back-centroid back-centroid
   :move-option
   (list
    (list (cons :target
		(if (find key '(:rleg :lleg))
		    (send *robot* key :knee-p)
		  (send *robot* key :elbow-p)))
	  (cons :move #F(0 0 100))
	  (cons :translation-axis #f(0 0 1))
	  (cons :rotation-axis :z))))
  (if (and (boundp '*irtviewer*) *irtviewer*)
      (send *irtviewer* :look-all))
  (push (send *robot* :angle-vector) buf)
  (if real-flag (model2real_)); (read-line))
  (animal-fullbody
   :move-target key
   :back-centroid back-centroid
   :move-option
   (list
    (list (cons :target
		(if (find key '(:rleg :lleg))
		    (send *robot* key :knee-p)
		  (send *robot* key :elbow-p)))
	  (cons :move move)
	  (cons :translation-axis #f(1 0 0))
	  (cons :rotation-axis :z))))
					;   (list
					;    (list (cons :target key)
					;  (cons :move #F(150 0 0))
					;  (cons :translation-axis #F(1 0 0))
					;  (cons :rotation-axis :z))))
  (if (and (boundp '*irtviewer*) *irtviewer*)
      (send *irtviewer* :look-all))
  (push (send *robot* :angle-vector) buf)
  (if real-flag (model2real_)); (read-line))
  (animal-fullbody
   :move-target key
   :back-centroid back-centroid
   :move-option
   (list
    (list (cons :target
		(if (find key '(:rleg :lleg))
		    (send *robot* key :knee-p)
		  (send *robot* key :elbow-p)))
	  (cons :move #F(0 0 -100))
	  (cons :translation-axis #f(0 0 1))
	  (cons :rotation-axis :z))))
  (if (and (boundp '*irtviewer*) *irtviewer*)
      (send *irtviewer* :look-all))
  (animal-fullbody :back-centroid back-centroid)
  (push (send *robot* :angle-vector) buf)
  (if real-flag (model2real_)); (read-line))
  buf
  )

(defun move-sequence
  (&key
   (real-flag (and (boundp '*ri*) *ri*))
   (mirror? nil)
   (move #F(180 0 0))
   )
					;  (animal-init :back-centroid nil)
  (animal-fullbody :back-centroid nil)
  (mapcar
   #'(lambda (key)
       (move-step key :real-flag real-flag :move move))
   (if mirror?
       '(:rleg :lleg)
     '(:lleg :rleg)))
  (send *robot* :larm :elbow-p :joint-angle 50)
  (send *robot* :rarm :elbow-p :joint-angle -50)
  (animal-fullbody :back-centroid t)
  (mapcar
   #'(lambda (key)
       (move-step key :real-flag real-flag :move move))
   (if mirror?
       '(:rarm :larm)
     '(:larm :rarm)))
  )


(defun demo-reset
  (&key
   (real-flag (and (boundp '*ri*) *ri*)))
  (let ((move-sleep-time *move-step-time*))
    (setq *move-step-time* 10)
    (move-sequence
     :real-flag real-flag
     :mirror? nil
     :move #F(0 0 0))
    (setq *move-step-time* move-sleep-time))
  )

(defun gallop-test ;; koikingu
  nil
  (do-until-key
   (setq sim-rate 0.6)

   (animal-init :back-centroid nil)
   (send *robot* :legs :ankle-p :joint-angle -40)
   (model2real :sleep-time 1000)
   (unix:usleep (round (/ (* 1000 1000) sim-rate)))

   (demo-reset)

   (send *robot* :larm :elbow-p :joint-angle 20)
   (send *robot* :rarm :elbow-p :joint-angle -20)
   (send *robot* :larm :shoulder-r :joint-angle -30)
   (send *robot* :rarm :shoulder-r :joint-angle 30)
   (animal-fullbody :back-centroid t :translation-axis '(:y :y t t))
   (model2real :sleep-time 500)
   (unix:usleep (round (/ (* 1000 1000) sim-rate)))

   (send *robot* :angle-vector (scale 0 (send *robot* :angle-vector)))
   (send *robot* :larm :shoulder-r :joint-angle 90)
   (send *robot* :rarm :shoulder-r :joint-angle -90)
   (if (and (boundp '*irtviewer*) *irtviewer*)
       (send *irtviewer* :draw-objects))
   (model2real :sleep-time 10)
   (unix:usleep (round (/ (* 10 1000) sim-rate)))

   (animal-init)
   (model2real :sleep-time 100)
   ))

(defun demo-forward
  (&key
   (real-flag (and (boundp '*ri*) *ri*))
   (mirror? nil))
  (animal-init :back-centroid nil)
  (move-sequence
   :real-flag real-flag
   :mirror? nil
   :move #F(200 0 0))
  (move-sequence
   :real-flag real-flag
   :mirror? t
   :move #F(200 0 0))
  )

(format
 t
 "(demo-forward) ; very slow and unstable hihi movement~%"
 )

#|
(setq *ri* (instance atlas-interface :init))

(init-pose)
(send *robot* :torso :waist-p :joint-angle 90)
(model2real_)

(animal-init)
(model2real_)


(do-until-key
 (demo-forward)
 (unix:sleep 1))

(send *ri* :teleport-to (make-coords :pos #f(0 -23000 980) :rpy (list pi/2 0 0)))

export ROS_MASTER_URI=http://133.11.216.38:11311

(progn
  (send *robot* :angle-vector (send *ri* :state :potentio-vector))
  (send *robot* :head :neck-p :joint-angle 30)
  )



(animal-fullbody
 :end-coords
 (append
  (mapcar #'(lambda (k) (cons k (send *robot* k :end-coords)))
          '(:rleg :lleg))
  *atlas-anima-endcoords*)
 :translation-axis :xy
 :rotation-axis :z)