
(require "package://hrpsys_gazebo/euslisp/atlas-model.l")
;(defvar *robot* (instance atlas-robot :init))
;(if (not (boundp '*irtviewer*)) (objects *robot*))

;(require "/home/noda/ros/fuerte/jsk-ros-pkg-unreleased/jsk_rosmake_libs/euslib/demo/s-noda/atlas/robot-pose.l")
(require "atlas-hihi.l")
;(require "/home/noda/prog/euslib/demo/s-noda/atlas/movable-viewer.l")
;(require "/home/noda/prog/euslib/demo/s-noda/util/spline.l")

(defvar *move-step-time* 100)

(defun real2model
  nil
  (send *robot* :angle-vector (send *ri* :state :potentio-vector)))

(defun model2real
  (&key (time *move-step-time*) (sleep-time time) (wait nil))
  (send *ri* :angle-vector (send *robot* :angle-vector) sleep-time)
  (when wait (send *ri* :wait-interpolation)))

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
  (if (and (boundp '*irtviewer*) *irtviewer*)
      (send *irtviewer* :draw-objects))
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

;; (defun tolot-step
;;   (&key
;;    (reverse? t)
;;    (sleep 100)
;;    (rotate 0))
;;   (bin-wark
;;    :reverse? reverse?
;;    :sleep sleep
;;    :callback
;;    #'(lambda (id)
;;        (if rotate
;;            (send *robot* :torso :waist-r :joint-angle rotate))
;;        (if (and reverse? (eq id 0))
;;            (progn
;;              (send *robot* :larm :shoulder-r :joint-angle 30)
;;              (send *robot* :rarm :shoulder-r :joint-angle -30))))
;;    ))

(defun tolot-step
  (&key
   (reverse? t)
   (sleep 100)
   (mirror? nil)
   (rotate 0))
  (bin-wark
   :reverse? reverse?
   :sleep sleep
   :target-keys
   (if mirror?
       '(:larm :rarm :lleg :rleg)
     '(:rarm :larm :rleg :lleg))
   :callback
   #'(lambda (id)
       (if rotate
           (send *robot* :torso :waist-r :joint-angle rotate))
       (if (and reverse? (eq id 0))
           (progn
             (send *robot* :larm :shoulder-r :joint-angle 30)
             (send *robot* :rarm :shoulder-r :joint-angle -30))))
   :scale (vector (list 0.3 0 0 1)
                  (list 0 0.3 1 0))
   :height
   (vector
    (list #F(0 0 300) #F(0 0 0) #F(0 0 0) #F(0 0 100))
    (list #F(0 0 0) #F(0 0 300) #F(0 0 100) #F(0 0 0)))
   )
  )

(defun rotate-tolot
  (&optional (reverse? nil))
  (send *tolot-hash* :enter :direction
        (if reverse?
            (copy-seq #(#F(-200 -200 0) #F(200 -200 0)))
          (copy-seq #(#f(-30.0 -70.0 0.0) #f(30.0 -70.0 0.0))))))
;          #(#F(-70 -150 0) #F(70 -150 0)))))

(defun gallop-step
  (&key
   (reverse? t)
   (rotate nil)
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
   :callback
   #'(lambda (id)
       (if rotate
           (send *robot* :torso :waist-r :joint-angle rotate))
       (if (and reverse? (eq id 0))
           (progn
             (send *robot* :larm :shoulder-r :joint-angle 60)
             (send *robot* :rarm :shoulder-r :joint-angle -60))))
   :scale (vector (list 0.3 0.3 0 0)
                  (list 0 0 1 1))
   :height
   (vector
    (list #F(0 0 300) #F(0 0 300) #F(0 0 0) #F(0 0 0))
    (list #F(0 0 0) #F(0 0 0) #F(0 0 100) #F(0 0 100)))))

(defun side-gallop-step
  (&key
   (reverse? t)
   (rotate nil)
   (sleep 100)
   (mirror nil)
   )
  (bin-wark
   :reverse? reverse?
   :sleep sleep
   :target-keys
   (if mirror
       '(:larm :rarm :lleg :rleg)
     '(:rarm :larm :rleg :lleg))
;   :target-centroid-pos
;   (vector (scale 0.5 (v+ (send *robot* :rarm :end-coords :worldpos)
;                          (send *robot* :larm :end-coords :worldpos)))
;           (scale 0.5 (v+ (send *robot* :rarm :end-coords :worldpos)
;                          (send *robot* :larm :end-coords :worldpos))))
   :revert? nil
   :callback
   #'(lambda (id)
       (if rotate
           (send *robot* :torso :waist-r :joint-angle rotate))
       (if (and reverse? (eq id 0))
           (progn
             (send *robot* :larm :shoulder-r :joint-angle 60)
             (send *robot* :rarm :shoulder-r :joint-angle -60))))
   :scale (vector (list 0.3 0.3 0 0)
                  (list 0 0 1 1))
   :height
   (vector
    (list #F(0 0 300) #F(0 0 0) #F(0 0 100) #F(0 0 0))
    (list #F(0 0 0) #F(0 0 300) #F(0 0 0) #F(0 0 100)))))

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
(tolot-init :reverse? t)
(set-user)
(model2real)
(do-until-key (tolot-step :reverse? t))

(send *tolot-hash* :enter :direction #(#F(100 0 0) #F(100 0 0)))
(do-until-key (tolot-step :reverse? t))

(send *tolot-hash* :enter :direction #(#F(50 100 0) #F(0 0 0)))

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
 :end-coords *atlas-tolot-endcoords*
 :target-centroid-pos
 (v+
  #F(0 0 0)
  (scale 0.5
         (v+ (send *robot* :rarm :end-coords :worldpos)
             (send *robot* :larm :end-coords :worldpos))))
 :revert? nil
 :stop 10
 :translation-axis :xy
 :rotation-axis :z)

(mapcar
 #'(lambda (k)
     (mapcar
      #'(lambda (j) (send j :joint-angle 0))
      (send *robot* k :joint-list)))
 '(:rarm :larm))

(tolot-init :reverse? t)


(do-until-key
 (tolot-init :reverse? nil)
 (model2real_ 500)
 (unix:sleep 1)
 ;;
 (send *robot* :angle-vector
       #f(0.0 50.0 0.0 20.0 0.0 0.0 -55.0 -80.0 0.0 -20.0 0.0 0.0 55.0 -80.0 0.0 140.0 -24.981 -100.0 28.3614 -40.0 30.0 140.0 24.981 -99.9811 -28.3614 -40.0 -30.0 0.0))
 (send *robot* :fix-leg-to-coords (make-coords))
 (model2real_ 400)
; (unix:sleep 2)
 ;;
 (tolot-init :reverse? nil)
 (send *robot* :torso :waist-p :joint-angle 0)
 (send-all (flatten (send *robot* :legs :joint-list)) :joint-angle 0)
 (send-all (flatten (send *robot* :arms :joint-list)) :joint-angle 0)
 (send *robot* :legs :ankle-p :joint-angle -40)
 (send *robot* :fix-leg-to-coords (make-coords))
 (send *irtviewer* :draw-objects)
; (send *robot* :angle-vector (scale 0 (send *robot* :angle-vector))) (send *robot* :torso :waist-p :joint-angle 0)
 (model2real_ 100)
; (unix:sleep 1)
 )

 (send *robot* :angle-vector
       #f(-0.355849 49.9885 -0.129764 41.1796 119.496 6.70627 -63.7998 67.5002 -0.054577 -54.0 120.0 -9.51683 29.5998 -92.2502 -0.030691 140.375 -24.981 -100.046 28.3614 -39.6945 30.1193 140.375 24.981 -99.9811 -28.3614 -39.7298 -29.8737 -0.438736))
 (model2real_ 5000)
 ;;


(tolot-init :reverse? nil)
(send *robot* :torso :waist-p :joint-angle 0)
(send-all (flatten (send *robot* :legs :joint-list)) :joint-angle 0)
(send *irtviewer* :draw-objects)

(tolot-init :reverse? nil)
(simple-fullbody
 :target
 (mapcar
  #'(lambda (k)
      (list (cons :target k)
            (cons :coords (make-coords))
            (cons :translation-axis :xy)
            (cons :rotation-axis nil)))
  '(:rarm :larm))
 :balance-leg :both
 :target-centroid-pos
 (scale 0.5
        (v+ (send *robot* :rleg :end-coords :worldpos)
            (send *robot* :lleg :end-coords :worldpos)))
 :centroid-thre 100
 :debug-view :no-message
 :stop 100)

(progn (rotate-tolot t) (do-until-key (gallop-step :reverse? t)))

(tolot-init)
(model2real_ 300)
(unix:sleep 1)
; for
(progn (send *tolot-hash* :enter :direction #(#f(-100 0 0) #f(-500 0 0))) (do-until-key (gallop-step :reverse? t :sleep 100)))

; right
(progn (send *tolot-hash* :enter :direction #(#f(-0 -0 0) #f(-500 -50 0))) (do-until-key (gallop-step :reverse? t :rotate -30 :sleep 100)))

;; left
(progn (send *tolot-hash* :enter :direction #(#f(-0 -0 0) #f(-500 50 0))) (do-until-key (gallop-step :reverse? t :rotate 30 :sleep 100)))

;; right2
(progn (send *tolot-hash* :enter :direction #(#F(200 200 0) #F(-350 100 0)))
       (do-until-key (tolot-step :reverse? t :rotate 60)))

; left2
(progn (send *tolot-hash* :enter :direction #(#F(350 100 0) #F(-200 200 0)))
       (do-until-key (tolot-step :reverse? t :rotate -60)))

(progn (send *tolot-hash* :enter :direction #(#F(100 100 0) #F(0 0 0)))
       (do-until-key (side-gallop-step :reverse? t :rotate -60)))

; right
(progn (send *tolot-hash* :enter :direction #(#F(-400 -200 0) #F(-100 0 0)))
       (do-until-key (side-gallop-step :reverse? t :rotate 0 :mirror t)))

; left
(progn (send *tolot-hash* :enter :direction #(#F(-400 200 0) #F(-100 0 0)))
       (do-until-key (side-gallop-step :reverse? t :rotate 0)))


; utubuse
p = -90

