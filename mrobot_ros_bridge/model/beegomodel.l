#-jsk
(jsk)
#|
(unless (boundp '*pickview*)(pickview))
(defun da ()
(send *pickview* :draw-objects)
)
|#
;; beego model
;; Author: Tomoko Kurotobi
(defun set_beego nil

  (setq box (make-cube 100 100 80))
  (send box :set-color :gray)
  (setq prune1 (make-cube 41 101 81))
  (send prune1 :translate #f(31 0 -31))
  (setq waku (body- box prune1))

  (setq pc (make-cube 80 100 20))
  (send pc :translate #f(-10 0 50) :world)
  (send pc :set-color :yellow)

  (setq kinect (make-cube 20 100 20))
  (send kinect :translate #f(-45 0 30) :world)
  (send kinect :set-color :yellow)

  (setq bump (make-cylinder 5 100))
  (send bump :translate #f(-50 50 -40) :world)
  (send bump :rotate pi/2 :x :world)
  (send bump :set-color :gray)

  (setq wheel1 (make-cylinder 15 10))
  (setq wheel2 (make-cylinder 15 10))
  (send wheel1 :translate #f(-30 50 -40))
  (send wheel1 :rotate -pi/2 :x :world)
  (send wheel2 :translate #f(-30 -50 -40))
  (send wheel2 :rotate pi/2 :x :world)
  (send wheel1 :set-color :gray)
  (send wheel2 :set-color :gray)

  (setq co (coords))
  (send co :scale 50)
  ;;(objects (list co pc box bump wheel1 wheel2))
  (setq beego (list pc kinect box bump wheel1 wheel2))
  (objects (list beego co))
  beego
)
;; beego frame
(defun beego ()
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
