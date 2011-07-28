;; #-jsk
;; (jsk)
#|
(unless (boundp '*pickview*)(pickview))
(defun da ()
(send *pickview* :draw-objects)
)
|#
;; beego model
;; Author: Tomoko Kurotobi
;; beego frame
(defun beego-org ()
  (setq co (coords))
  (send co :scale 50)

  (setq b1 (make-cube 10 60 10)
	b2 (make-cube 10 60 10)
	b3 (make-cube 60 10 10)
	b4 (make-cube 60 10 10)
	base (list b1 b2 b3 b4))

  (send b1 :translate #f(-25 0 0))
  (send b2 :translate #f(25 0 0))
  (send b3 :translate #f(0 35 0))
  (send b4 :translate #f(0 -35 0))

  (setq w1 (make-cylinder 15 10))
  ;;車輪の穴とか開けたいけど回転軸はシリンダーの中心のままだー
  ;; (setq hole (make-cylinder 3 12))
  ;; (send hole :translate #f(8 0 -1) :world)
  ;; (setq w1 (body- w1 hole))
  ;; (dotimes (i 5)
  ;;   (send hole :rotate (* 2pi (/ i 6.0) ) :z :world)
  ;;   (print i)
  ;;   (setq w1 (body- w1 hole))
  ;;   )
  (send w1 :rotate pi/2 :x)
  (send w1 :translate #f(0 -40 0) :world)
  (setq w2 (make-cylinder 15 10))
  (send w2 :rotate -pi/2 :x)
  (send w2 :translate #f(0 40 0) :world)
  (setq wheels (list w1 w2))

  (setq pole (make-cube 10 10 80)
	p1 (copy-object pole)
	p2 (copy-object pole)
	p3 (copy-object pole)
	p4 (copy-object pole))
  (setq poles (list p1 p2 p3 p4))
  (send p1 :translate #f(-25 -35 35))
  (send p2 :translate #f(25 -35 35))
  (send p3 :translate #f(-25 35 35))
  (send p4 :translate #f(25 35 35))

  (setq plate (make-cube 100 100 10))
  (send plate :translate #f(-20 0 80))

  (setq pc (make-round-corner-cube 80 100 10))
  (send pc :translate #f(-10 0 90))
  (setq kinect (make-round-corner-cube 20 80 20))
  (send kinect :translate #f(10 0 60))

  (setq bump (make-cylinder 5 80))
  (send bump :rotate pi/2 :x)
  (send bump :translate #f(40 40 0) :world)

  (setq set (flatten (list base bump wheels poles plate pc kinect)))
  (send-all set :set-color :white)
  (mapcar #'(lambda(x)(putprop x 0 :weight)) set)
  (objects (list set co))

  (setq robot (instance robot-link :init (make-cascoords) set :name "beego"))
  (setq id 0)
  (dolist (part set)
	(send part :name (format nil "link1~a" (incf id)))
	(send robot :assoc part))

  robot)

;; beego frame
(defun beego ()
  (setq co (coords))
  (send co :scale 50)

  (setq base (make-cube 200 300 20))
  (setq p1 (make-cube 20 20 300))
  (setq p2 (make-cube 20 20 300))
  (setq p3 (make-cube 20 20 300))
  (send p1 :translate (float-vector 0 0 150) :world)
  (send p1 :translate (float-vector 0 0 -10) :world)
  (send p2 :translate (float-vector -100 -150 150) :world)
  (send p2 :translate (float-vector 10 10 -10) :world)
  (send p3 :translate (float-vector -100 150 150) :world)
  (send p3 :translate (float-vector 10 -10 -10) :world)
  (setq pole (list p1 p2 p3))

  (setq w1 (make-cylinder 40 10))
  (send w1 :rotate pi/2 :x)
  (send w1 :translate (float-vector 100 -150 0) :world)
  (setq w2 (make-cylinder 40 10))
  (send w2 :rotate -pi/2 :x)
  (send w2 :translate (float-vector 100 150 0) :world)
  (setq w3 (make-cylinder 25 10))
  (send w3 :rotate -pi/2 :x)
  (send w3 :translate (float-vector -170 0 0) :world)
  (send w3 :translate (float-vector 0 -5 -15) :world)


  (setq w3-stand1 (make-cube 20 40 50))
  (setq w3-stand2 (make-cube 100 40 20))
  (setq w3-stand3 (make-cube 20 40 60))
  (send w3-stand1 :translate (float-vector 40 0 -15) :world)
  (send w3-stand3 :translate (float-vector -40 0 -20) :world)
  (setq w3stand (body+ w3-stand1 w3-stand2 w3-stand3))
  (send w3stand :translate (float-vector -130 0 40) :world)
  (setq wheels (list w1 w2 w3 w3stand))

  (setq plate (make-cube 300 300 10))
  (send plate :translate (float-vector 0 0 290) :world)

  (setq bump (make-cylinder 20 300))
  (send bump :rotate pi/2 :x)
  (send bump :translate (float-vector 200 150 0) :world)
  (setq bump-stand1 (make-cube 100 20 20))
  (setq bump-stand2 (make-cube 100 20 20))
  (send bump-stand1 :translate (float-vector 150 -100 0) :world)
  (send bump-stand2 :translate (float-vector 150 100 0) :world)
  (setq bump-asm (list bump bump-stand1 bump-stand2))

  (setq pc (make-round-corner-cube 250 300 40))
  (send pc :translate (float-vector 0 0 310) :world)
  (setq kinect (make-round-corner-cube 100 300 60))
  (send kinect :translate (float-vector 100 0 260) :world)

  (setq nakami (make-cube 150 200 100))
  (send nakami :translate (float-vector 0 0 50) :world)

  (setq beego-set (flatten (list base pole wheels plate bump-asm pc kinect nakami)))
  (send-all beego-set :set-color :white)
  (mapcar #'(lambda(x)(putprop x 100.0 :weight)) beego-set)
  (objects (list beego-set co))
  (dolist (b (cdr beego-set))
    (send (car beego-set) :assoc b))
  (setq bb (instance bodyset :init (make-cascoords) :bodies beego-set))
  ;;(read-line)

;;  (setq robot (instance robot-link :init (make-cascoords) beego-set :name "beego"))
  (putprop bb 100.0 :weight)
  ;;(setq robot (instance robot-link :init (make-cascoords) beego-set :name "beego"))
  (setq robot (instance bodyset-link :init (make-cascoords) :bodies beego-set :name "beego"))
  (setf (get robot :centroid) #f(0 0 0))
  (setf (get robot :weight) 10)
  (setf (get robot :inertia-tensor) (unit-matrix 3))
  (objects (list robot))
  (setq id 0)
  (dolist (part beego-set)
    (send part :name (format nil "link1~a" (incf id)))
    (send robot :assoc part))

  robot
  )


(defun beego-new (&key (tread 275) (wheel-radius 41)
                       (frame-width 30) (wheel-width 30)
                       (height 240))
  (setq co (coords))
  (send co :scale 50)

  (setq b1 (make-cube frame-width tread frame-width)
        b2 (make-cube frame-width tread frame-width)
        b3 (make-cube tread frame-width frame-width)
        b4 (make-cube tread frame-width frame-width))

  (send b1 :translate (float-vector (+ (/ tread -2.0) (/ frame-width 2.0)) 0 0))
  (send b2 :translate (float-vector (- (/ tread 2.0) (/ frame-width 2.0)) 0 0))
  (send b3 :translate (float-vector 0 (+ (/ tread 2.0) (/ frame-width -2.0)) 0))
  (send b4 :translate (float-vector 0 (+ (/ tread -2.0) (/ frame-width 2.0)) 0))
  (setq base (body+ (body+ b1 b3) (body+ b2 b4)))

  (setq w1 (make-cylinder wheel-radius wheel-width))
  ;;車輪の穴とか開けたいけど回転軸はシリンダーの中心のままだー
  ;; (setq hole (make-cylinder 3 12))
  ;; (send hole :translate #f(8 0 -1) :world)
  ;; (setq w1 (body- w1 hole))
  ;; (dotimes (i 5)
  ;;   (send hole :rotate (* 2pi (/ i 6.0) ) :z :world)
  ;;   (print i)
  ;;   (setq w1 (body- w1 hole))
  ;;   )
  (send w1 :rotate pi/2 :x)
  (send w1 :translate (float-vector 50 (/ tread -2.0) 0) :world)
  (setq w2 (make-cylinder wheel-radius wheel-width))
  (send w2 :rotate -pi/2 :x)
  (send w2 :translate (float-vector 50 (/ tread 2.0) 0) :world)
  (setq wheels (list w1 w2))

  (setq pole (make-cube (1- frame-width) (1- frame-width) height)
        p1 (copy-object pole)
        p2 (copy-object pole)
        p3 (copy-object pole)
        p4 (copy-object pole))
  (send p1 :translate (float-vector (+ (/ tread -2.0) (/ frame-width 2.0))
                                    (+ (/ tread -2.0) (/ frame-width 2.0))
                                    (+ (/ height 2.0) (/ frame-width -2.0))))
  (send p2 :translate (float-vector (- (/ tread 2.0) (/ frame-width 2.0))
                                    (+ (/ tread -2.0) (/ frame-width 2.0))
                                    (+ (/ height 2.0) (/ frame-width -2.0))))
  (send p3 :translate (float-vector (+ (/ tread -2.0) (/ frame-width 2.0)) 
                                    (+ (/ tread 2.0) (/ frame-width -2.0))
                                    (+ (/ height 2.0) (/ frame-width -2.0))))
  (send p4 :translate (float-vector (- (/ tread 2.0) (/ frame-width 2.0)) 
                                    (+ (/ tread 2.0) (/ frame-width -2.0))
                                    (+ (/ height 2.0) (/ frame-width -2.0))))
  (setq base (body+ (body+ (body+ (body+ base p1) p2) p3) p4))

  (setq plate (make-cube 300 300 30))
  (send plate :translate (float-vector -15 0 height))
  (setq frame (body+ base plate))

  (setq pc (make-round-corner-cube 240 300 30))
  (send pc :translate (float-vector 15 0 270))
  (setq kinect (make-round-corner-cube 60 240 60))
  (send kinect :translate (float-vector 80 0 180))

  (setq bump (make-cylinder 15 240))
  (send bump :rotate pi/2 :x)
  (send bump :translate (float-vector 160 120 0) :world)

  (setq robot-body-lst (flatten (list base wheels pc kinect bump)))
  (send-all robot-body-lst :set-color :white)
  (mapcar #'(lambda(x)(putprop x 0 :weight)) robot-body-lst)
  ;;(objects (list robot-body-lst co))

  (setq robot (instance robot-link :init (make-cascoords) robot-body-lst
                        :name "beego"))

  (setq id 0)
  (dolist (part robot-body-lst)
	(when (null (send part :name))
	  (send part :name (format nil "link~a" (incf id))))
	(send robot :assoc part))

  robot)

#|
;;(setq *beego* (beego))
(setq *beego* (beego-new))
(with-open-file
 (fd "beego.wrl" :direction :output)
; (send *beego* :dump-to-wrl :strm fd))
 (dump-object-to-wrl *beego* fd))
|#
