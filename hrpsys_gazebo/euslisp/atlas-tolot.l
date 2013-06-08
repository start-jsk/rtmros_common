
(require "package://hrpsys_gazebo/euslisp/atlas-model.l")
(defvar *robot* (instance atlas-robot :init))
(if (not (boundp '*irtviewer*)) (objects *robot*))

;(require "/home/noda/ros/fuerte/jsk-ros-pkg-unreleased/jsk_rosmake_libs/euslib/demo/s-noda/atlas/robot-pose.l")
(require "atlas-hihi.l")
;(require "/home/noda/prog/euslib/demo/s-noda/atlas/movable-viewer.l")
;(require "/home/noda/prog/euslib/demo/s-noda/util/spline.l")

(defun real2model
  nil
  (send *robot* :angle-vector (send *ri* :state :potentio-vector)))

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
	    (send c :translate #F(0 0 -200))
	    (make-cascoords
	     :pos (send *robot* k :wrist-p :worldpos)
	     :rot (send c :worldrot)
	     :name (read-from-string (format nil "~A-animal-endcoords" k))
	     :parent (send *robot* k :wrist-p :child-link)))))
     '(:rarm :larm))
    (mapcar
     #'(lambda (k)
	 (cons k (send *robot* k :end-coords)))
     '(:rleg :lleg))
    (mapcar
     #'(lambda (k)
	 (cons (read-from-string (format nil "~A-2" k))
	       (send *robot* k :end-coords)))
     '(:rleg :lleg))
    )))


(defun animal-init
  nil
  (send-all (send *robot* :joint-list) :joint-angle 0)
  (send *robot* :legs :knee-p :joint-angle 100)
  (send *robot* :rleg :crotch-r :joint-angle -30)
  (send *robot* :lleg :crotch-r :joint-angle 30)
  (send *robot* :arms :shoulder-y :joint-angle 130)
  (send *robot* :larm :elbow-p :joint-angle 100)
  (send *robot* :rarm :elbow-p :joint-angle -100)
  (send *robot* :arms :elbow-y :joint-angle 100)
  (send *robot* :fix-leg-to-coords (make-coords))
  (animal-fullbody
   :debug-view :no-message
   :back-centroid nil
   :translation-axis :xy
   :rotation-axis :z))

(defvar *tolot-hash* (make-hash-table))
(send *tolot-hash* :enter :direction #F(50 0 0))
(send *tolot-hash* :enter :prev-direction nil)
(send *tolot-hash* :enter :dict #(nil nil))
(send *tolot-hash* :enter :running nil)
;(defvar *direction* #F(50 0 0))
;(defvar *prev-direction* nil)
;(defvar *tolot-dict* #(nil nil))
;(defvar *running* nil)
(defun tolot
  nil
  (if (zerop (length sys:*threads*))
      (sys:make-thread 5))
  (if (send *tolot-hash* :get :running)
      (progn
	(send *tolot-hash* :enter :running nil)
	(unix:sleep 5)))
  (send *tolot-hash* :enter :running t)
  (sys:thread
   '(lambda nil
      (while (send *tolot-hash* :get :running)
	(tolot-step)))))

(defun tolot-step
  (&optional buf)
  (animal-init)
  (if (not (eq
	    (send *tolot-hash* :get :direction)
	    (send *tolot-hash* :get :prev-direction)))
      (progn
	(print 'new-direction)
	(setq buf 'new)
	(send *tolot-hash* :enter :dict #(nil nil))
	(send *tolot-hash* :enter :prev-direction
	      (send *tolot-hash* :get :direction))))
  (if (and (aref (send *tolot-hash* :get :dict) 0)
	   (not buf))
      (send *robot* :angle-vector
	    (aref (send *tolot-hash* :get :dict) 0))
    (progn
      (print 'gen-dict1)
      (animal-fullbody
       :target-keys
       '(:rarm :larm :rleg :lleg)
       :stop 50
       :target-centroid-pos nil
       :move
       (mapcar
	#'(lambda (v) (v+ v (send *tolot-hash* :get :direction)))
	(list #F(0 0 150) #F(0 0 0) #F(0 0 0) #F(0 0 50))))
      (setf (aref (send *tolot-hash* :get :dict) 0)
	    (send *robot* :angle-vector))))
  (model2real_ 100)
  (animal-init)
  (if (and (aref (send *tolot-hash* :get :dict) 1)
	   (not buf))
      (send *robot* :angle-vector
	    (aref (send *tolot-hash* :get :dict) 1))
    (progn
      (print 'gen-dict2)
      (animal-fullbody
       :target-keys
       '(:rarm :larm :rleg :lleg)
       :stop 50
       :target-centroid-pos nil
       :move
       (mapcar
	#'(lambda (v) (v+ v (send *tolot-hash* :get :direction)))
	(list #F(0 0 0) #F(0 0 150) #F(0 0 50) #F(0 0 0))))
      (setf (aref (send *tolot-hash* :get :dict) 1)
	    (send *robot* :angle-vector))))
  (model2real_ 100))

(format t
	(concatenate
	 string
	 "[USAGE] (tolot) ; ushito muki tolot thread start. ~%"
	 "(send *tolot-hash* :enter :running nil) ; stop thread~%"
	 "(send *tolot-hash* :enter :direction #F(50 0 0)) ; change direction~%"))

#|
export ROS_MASTER_URI=http://133.11.216.38:11311/
rossetip 157.82.7.150

roseus

(load "murooka.l")
(my-init)
(load "test.l")
(animal-init)
(model2real)