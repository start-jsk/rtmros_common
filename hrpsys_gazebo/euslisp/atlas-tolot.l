
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
 *atlas-tolot-endcoords*
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
	     :pos
             (v+ #F(0 0 0)
                 (scale 0.5 (v+ (send *robot* k :elbow-p :worldpos)
                                (send *robot* k :wrist-p :worldpos))))
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
	     :pos
             (v+ #F(0 0 0) (send *robot* k :wrist-p :worldpos))
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

;; (defun tolot-init
;;   nil
;;   (send-all (send *robot* :joint-list) :joint-angle 0)
;;   (send *robot* :legs :knee-p :joint-angle 100)
;;   (send *robot* :rleg :crotch-r :joint-angle -30)
;;   (send *robot* :lleg :crotch-r :joint-angle 30)
;;   (send *robot* :arms :shoulder-y :joint-angle 130)
;;   (send *robot* :larm :elbow-p :joint-angle 100)
;;   (send *robot* :rarm :elbow-p :joint-angle -100)
;;   (send *robot* :arms :elbow-y :joint-angle 100)
;;   (send *robot* :fix-leg-to-coords (make-coords))
;;   (animal-fullbody
;;    :end-coords *atlas-tolot-endcoords*
;;    :debug-view :no-message
;;    :back-centroid nil
;;    :translation-axis :xy
;;    :rotation-axis :z))

(defun tolot-init
  (&key (reverse? t))
  (send *robot* :angle-vector
        (if reverse?
            #f(-0.003048 6.09591 -0.053802 105.109 97.2697 0.0 -26.5584 105.814 0.0 -105.089 97.0922 0.0 26.6117 105.805 0.0 99.6314 1.06171 2.72941 17.1759 -4.57762 -1.29622 99.6322 -0.980218 2.75578 -17.0183 -4.58332 1.34451 0.0)
            #f(0.001401 56.8709 -0.202539 1.1684 0.356907 63.5659 -1.32546 -80.0405 -30.2631 -1.12992 0.247962 -63.5615 1.33342 -80.0307 -30.2753 122.056 -23.8269 -96.4418 -11.2025 -43.3635 63.8859 122.091 23.8112 -96.2276 11.2151 -43.6532 -63.9565 0.084544)))
  (send *robot* :fix-leg-to-coords (make-coords))
  (send *irtviewer* :draw-objects)
  )


(defvar *tolot-hash* (make-hash-table))
(send *tolot-hash* :enter :direction #(#F(200 0 0) #F(200 0 0)))
(send *tolot-hash* :enter :prev-direction nil)
(send *tolot-hash* :enter :dict #(nil nil))
(send *tolot-hash* :enter :running nil)
;(defvar *direction* #F(50 0 0))
;(defvar *prev-direction* nil)
;(defvar *tolot-dict* #(nil nil))
;(defvar *running* nil)
(defun tolot
  (&key (reverse? t))
  (if (zerop (length sys:*threads*))
      (sys:make-thread 5))
  (if (send *tolot-hash* :get :running)
      (progn
	(send *tolot-hash* :enter :running nil)
	(unix:sleep 5)))
  (send *tolot-hash* :enter :running t)
  (sys:thread
   #'(lambda nil
      (while (send *tolot-hash* :get :running)
	(tolot-step :reverse? reverse?)))))

(defun bin-wark
  (&key
   (reverse? t)
   (sleep 100)
   (target-keys
    '(:rarm :larm :rleg :lleg))
   (scale (vector (list 0.4 0 0 1)
                  (list 0 0.4 1 0)))
   (height
    (vector
     (list #F(0 0 130) #F(0 0 0) #F(0 0 0) #F(0 0 30))
     (list #F(0 0 0) #F(0 0 130) #F(0 0 30) #F(0 0 0))))
   (revert? t)
   (callback #'(lambda (id) nil))
   (target-centroid-pos #(nil nil))
   buf)
  (labels ((inner
            (id)
            (tolot-init :reverse? reverse?)
            (if (and (aref (send *tolot-hash* :get :dict) id)
                     (not buf))
                (send *robot* :angle-vector
                      (aref (send *tolot-hash* :get :dict) id))
              (progn
                (format t "gen-dict ~A~%" id)
                (animal-fullbody
                 :end-coords
                 (append
                  (if reverse?
                      nil
                    (mapcar #'(lambda (k) (cons k (send *robot* k :end-coords)))
                            '(:rarm :larm)))
                  *atlas-tolot-endcoords*)
                 :target-keys target-keys
                 :stop 30
                 :revert? revert?
                 :target-centroid-pos (aref target-centroid-pos id)
                 :move
                 (mapcar
                  #'(lambda (a v)
                      (v+ v
                          (scale a (aref (send *tolot-hash* :get :direction) id))))
                  (aref scale id)
                  (aref height id)
                  ))
                (funcall callback id)
                (setf (aref (send *tolot-hash* :get :dict) id)
                      (send *robot* :angle-vector))))
            (model2real_ sleep)))
    (if (not (eq
            (send *tolot-hash* :get :direction)
            (send *tolot-hash* :get :prev-direction)))
        (progn
          (print 'new-direction)
          (setq buf 'new)
          (send *tolot-hash* :enter :dict #(nil nil))
          (send *tolot-hash* :enter :prev-direction
                (send *tolot-hash* :get :direction))))
    (inner 0)
    (inner 1)))

(defun tolot-step
  (&key
   (reverse? t)
   (sleep 100))
  (bin-wark :reverse? reverse? :sleep sleep))

(defun rotate-tolot
  (&optional (reverse? nil))
  (send *tolot-hash* :enter :direction
        (if reverse?
            #(#F(100 150 0) #F(-100 150 0))
          #(#f(-30.0 -70.0 0.0) #f(30.0 -70.0 0.0)))))
;          #(#F(-70 -150 0) #F(70 -150 0)))))

(defun gallop-step
  (&key
   (reverse? t)
   (sleep 100)
   )
  (bin-wark
   :reverse? reverse?
   :sleep sleep
   :target-keys
   '(:rarm :larm :rleg :lleg)
;   :target-centroid-pos
;   (vector (scale 0.5 (v+ (send *robot* :rarm :end-coords :worldpos)
;                          (send *robot* :larm :end-coords :worldpos)))
;           (scale 0.5 (v+ (send *robot* :rarm :end-coords :worldpos)
;                          (send *robot* :larm :end-coords :worldpos))))
   :revert? nil
;   :callback nil
   :scale (vector (list 0.3 0.3 0 0)
                  (list 0 0 1 1))
   :height
   (vector
    (list #F(0 0 300) #F(0 0 300) #F(0 0 0) #F(0 0 0))
    (list #F(0 0 0) #F(0 0 0) #F(0 0 100) #F(0 0 100)))))


(defun boyonboyon-step ;; aho
  nil
  (tolot-init :reverse? nil)
  (model2real_ 100)
  (unix:sleep 2)
  (send *robot* :angle-vector (scale 0 (send *robot* :angle-vector)))
  (model2real_ 10)
  (unix:sleep 1))

(format t
	(concatenate
	 string
	 "[USAGE] (tolot) ; ushito muki tolot thread start. ~%"
         "if tolot segfo, call (do-until-key (tolot-step))~%"
	 "(send *tolot-hash* :enter :running nil) ; stop thread~%"
	 "(send *tolot-hash* :enter :direction #(#F(100 0 0) #f(100 0 0))) ; change direction~%"))

#|
export ROS_MASTER_URI=http://133.11.216.38:11311/
rossetip 157.82.7.150

roseus

(load "murooka.l")
(my-init)
(load "atlas-tolot.l")
(tolot-init :reverse? nil)
(set-user)
(model2real)
(do-until-key (tolot-step :reverse? nil))

(send *tolot-hash* :enter :direction #(#F(100 0 0) #F(100 0 0)))
(do-until-key (tolot-step :reverse? nil))

(send *tolot-hash* :enter :direction #(#F(50 100 0) #F(0 0 0)))

(simple-fullbody
 :target
 (list (list (cons :target :rarm))
       (list (cons :target :larm)))
 :balance-leg :both
 :revert-if-fail nil
 :debug-view :no-message)

;; ushimawari

(tolot-init)
(animal-fullbody)
(send *robot* :torso :waist-p :joint-angle 90)
(send-all (send *robot* :larm :joint-list) :joint-angle 0)
(send-all (send *robot* :rarm :joint-list) :joint-angle 0)
(send *robot* :arms :shoulder-y :joint-angle 120)
(animal-fullbody :back-centroid t)

(send *robot* :torso :waist-p :joint-angle 90)
(send *robot* :legs :crotch-p :joint-angle 0)
(send-all (send *robot* :larm :joint-list) :joint-angle 0)
(send-all (send *robot* :rarm :joint-list) :joint-angle 0)
(send *robot* :arms :shoulder-y :joint-angle 120)
(send *robot* :larm :shoulder-r :joint-angle -50)
(send *robot* :rarm :shoulder-r :joint-angle 50)
(animal-fullbody :back-centroid t :translation-axis :xy)

(send *robot* :torso :waist-p :joint-angle 90)
(send *robot* :legs :knee-p :joint-angle 80)
(send *robot* :legs :ankle-p :joint-angle 100)
(animal-fullbody :back-centroid t :translation-axis :xy)

(tolot-init :reverse? nil)



;; mae mawari

(tolot-init :reverse? nil)

(animal-fullbody
 :end-coords
 (append
  (mapcar #'(lambda (k) (cons k (send *robot*  k :end-coords)))
          '(:rarm :larm))
  *atlas-tolot-endcoords*)
 :move-option
 (list (list (cons :target (send *robot* :torso :waist-p))
             (cons :move #F(0 0 50))
             (cons :translation-axis #f(0 0 1))
             (cons :rotation-axis nil)))
 :target-centroid-pos
 (v+
  #F(200 0 0)
  (scale 0.5
         (v+ (send *robot* :rleg :end-coords :worldpos)
             (send *robot* :lleg :end-coords :worldpos))))
 :revert? nil
 :stop 100
 :translation-axis :xy
 :rotation-axis :z)


(animal-fullbody
 :end-coords
 (append
  (mapcar #'(lambda (k) (cons k (send *robot*  k :end-coords)))
          '(:rarm :larm))
  *atlas-tolot-endcoords*)
 :target-centroid-pos
 (v+
  #F(0 0 0)
  (scale 0.5
         (v+ (send *robot* :rarm :end-coords :worldpos)
             (send *robot* :larm :end-coords :worldpos))))
 :revert? nil
 :stop 100
 :translation-axis :xy
 :rotation-axis :z)



(mapcar
 #'(lambda (k)
     (mapcar
      #'(lambda (j) (send j :joint-angle 0))
      (send *robot* k :joint-list)))
 '(:rarm :larm))


(tolot-init :reverse? t)




;;
(animal-fullbody
 :end-coords
 (append
  (mapcar #'(lambda (k) (cons k (send *robot*  k :end-coords)))
          '(:rarm :larm))
  *atlas-tolot-endcoords*)
 :target-centroid-pos
 (scale 0.5
        (v+ (send *robot* :rleg :end-coords :worldpos)
            (send *robot* :lleg :end-coords :worldpos)))
 :translation-axis :xy
 :rotation-axis :z)





(send *robot* :angle-vector #f(-1.19203 72.7803 -1.77639 40.1134 0.0 79.5125 -31.1455 -73.4036 -73.689 -40.1843 0.0 -79.3719 30.9617 -73.2118 -73.4664 111.365 -24.981 -82.8293 -17.0763 -50.2821 62.6786 111.371 24.981 -82.6903 17.055 -50.4708 -62.6916 0.084544))
(model2real_ 500)
(send *robot* :angle-vector (scale 0 (send *robot* :angle-vector)))
(model2real :sleep-time 10)
(tolot-init :reverse? nil)
(unix:usleep (* 1000 30))
(model2real :sleep-time 100)


(progn (do-until-key (tolot-step :reverse? nil)) (send *robot* :angle-vector (scale 0 (send *robot* :angle-vector))) (send *robot* :torso :waist-p :joint-angle 0) (model2real_ 500))

(send *robot* :arms :shoulder-r :joint-angle)
(send *robot* :rarm :shoulder-r :joint-angle -90)
(send *robot* :larm :shoulder-r :joint-angle 90)

(send *robot* :arms :shoulder-y :joint-angle 15)
(send *robot* :rarm :shoulder-r :joint-angle -80)
(send *robot* :larm :shoulder-r :joint-angle 80)



