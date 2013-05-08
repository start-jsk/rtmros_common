;#-:jsk (jsk)
;#-:rbrain-basic (rbrain)

(require "package://hrpsys_gazebo/euslisp/atlas-interface.l")

;;------------ initalize

(defmethod rotational-joint
  (:worldrot
   nil
   (send (send self :child-link) :worldrot))
  (:worldpos
   nil
   (send (send self :child-link) :worldpos)))

(defvar *gazebo-flag* nil)
(if *gazebo-flag*
    (atlas-init)
  (setq *atlas* (instance atlas-robot :init)))
(defvar *robot* *atlas*)
(defvar *move-step-time* 500)

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
	      :pos (scale 0.5 (v+ (send *robot* k :elbow-p :worldpos)
	    			  (send *robot* k :wrist-p :worldpos)))
	      :rot (send c :worldrot)
	      :name (read-from-string (format nil "~A-animal-endcoords" k))
	      :parent (send *robot* k :elbow-p :child-link)))))
      '(:rarm :larm))
     (mapcar
      #'(lambda (k)
	  (cons
	   (read-from-string (format nil "~A-2" k))
 	   (let ((c (send (send *robot* :rleg :end-coords) :copy-worldcoords)))
	     (send c :rotate (deg2rad 90) :z)
	     (make-cascoords
	      :pos (send *robot* k :wrist-p :worldpos)
	      :rot (send c :worldrot)
	      :name (read-from-string (format nil "~A-animal-endcoords" k))
	      :parent (send *robot* k :wrist-p :child-link)))))
      '(:rarm :larm))
     (mapcar
      #'(lambda (k)
	  (cons
	   k
 	   (let ((c (send (send *robot* :rleg :end-coords) :copy-worldcoords)))
	     (send c :rotate (deg2rad -90) :y)
	     (send c :rotate
		   (deg2rad
		    (* (if (eq k :rleg) 1 -1) 40))
		   :x)
	     (make-cascoords
	      :pos (scale 0.5 (v+ (send *robot* k :knee-p :worldpos)
				  (send *robot* k :ankle-p :worldpos)))
	      :rot (send c :worldrot)
	      :name (read-from-string (format nil "~A-animal-endcoords" k))
	      :parent (send *robot* k :knee-p :child-link))))
	  )
      '(:rleg :lleg))
     )))

(if (not (boundp '*irtviewer*)) (make-irtviewer))
(send *irtviewer* :objects (list *robot*))
(send *irtviewer* :look-all)

;; ---------- utility function

(defun model2real
  (&key (robot *robot*) (sleep-time 5000))
  (send *ri* :angle-vector
	(send *robot* :angle-vector) sleep-time)
  (if (find-method *ri* :hand-angle-vector)
      (send *ri* :hand-angle-vector
	    (apply #'concatenate
		   float-vector (send robot :arms :hand :angle-vector))
	    sleep-time)))

(defun model2real_
  (&optional (sl *move-step-time*))
  (send *irtviewer* :draw-objects)
  (if
      *ri*
      (progn (model2real :sleep-time sl) (send *ri* :wait-interpolation))
    (unix:usleep (* sl 1000))))

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
;		(send robot :calc-target-centroid-pos
;		      balance-leg '(:lleg :rleg))
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
;	 move-target-buf
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
   (target-keys
    (remove (read-from-string
	     (format nil "~A~A" move-target
		     (if back-centroid "-2" "")))
	    (if back-centroid
		(list :rarm-2 :rleg :larm-2 :lleg)
	      (list :rarm :rleg :larm :lleg))))
   (debug-view :no-message)
   (centroid-thre (* (length target-keys) 100))
   (target-cascoords
    (mapcar
     #'(lambda (k) (cdr (assoc k *atlas-anima-endcoords*)))
     target-keys)))
  (simple-fullbody
   :target
   (append
    move-option
    (mapcar
    #'(lambda (cascoords)
	(list (cons :target cascoords)
	      (cons :coords (make-coords
			     :pos (float-vector
				   (aref (send cascoords :worldpos) 0)
				   (aref (send cascoords :worldpos) 1)
				   0)))
					;(make-coords))
	      (cons :rotation-axis rotation-axis)
	      (cons :translation-axis translation-axis)))
    target-cascoords))
   :balance-leg nil
   :target-centroid-pos
   (scale
    (/ 1.0 (length target-cascoords))
    (reduce #'v+ (send-all target-cascoords :worldpos)))
   :debug-view debug-view
   :centroid-thre centroid-thre
   :cog-gain 0.01
   ))

(defun animal-init
  (&key back-centroid)
  (init-pose)
  (send *robot*
	:angle-vector
	(scale 0 (send *robot* :angle-vector)))
  (if (not back-centroid)
      (progn
	(send *robot* :rarm :elbow-p :joint-angle -90)
	(send *robot* :larm :elbow-p :joint-angle 90)))
  (send *robot* :arms :shoulder-y :joint-angle -90)
  (send *robot* :rotate (deg2rad 45) :y)
  (send *robot* :legs :ankle-p :joint-angle 90)
  (send *robot* :torso :waist-p :joint-angle 70)
  (send *robot* :legs :knee-p :joint-angle 90)
  (send *robot* :legs :crotch-p :joint-angle -60)
  (animal-fullbody
   :debug-view :no-message
   :back-centroid back-centroid
   :translation-axis :xy
   :rotation-axis :z)
  (send *irtviewer* :look-all)
  )

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
	  (cons :move #F(0 0 100))
	  (cons :translation-axis #f(0 0 1))
	  (cons :rotation-axis :z))))
  (send *irtviewer* :look-all)
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
;	  (cons :move #F(150 0 0))
;	  (cons :translation-axis #F(1 0 0))
;	  (cons :rotation-axis :z))))
  (send *irtviewer* :look-all)
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
  (send *irtviewer* :look-all)
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
  (animal-init :back-centroid nil)
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

(defun demo-forward
  (&key
   (real-flag (and (boundp '*ri*) *ri*))
   (mirror? nil))
  (move-sequence
   :real-flag real-flag
   :mirror? nil
   :move #F(150 0 0))
  (move-sequence
   :real-flag real-flag
   :mirror? t
   :move #F(150 0 0)))

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
 (unix:sleep 3))