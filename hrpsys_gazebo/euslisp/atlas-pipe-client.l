;;(setq *fork-stream* (piped-fork "ssh" "leus@133.11.216.144" "roseus \"~/ros/fuerte/rtm-ros-robotics/rtmros_common/hrpsys_gazebo/euslisp/atlas-pipe-interface.l\""))
;;(load "package://hrpsys_gazebo/euslisp/atlas-interface.l")

(load "package://hrpsys_gazebo/euslisp/atlas-model.l")
(atlas-with-hand)
(make-irtviewer) ;; irtviewer blocked after connection ....

(defun connect-atlas (&optional (host "fc1")
                                (com "roseus \"~/ros/fuerte/rtm-ros-robotics/rtmros_common/hrpsys_gazebo/euslisp/atlas-pipe-interface.l\""))
  (setq *fork-stream* (piped-fork "ssh" host com)))

(defun rd-p (&optional (strm *fork-stream*) (timeout 0.1)) ;; readablep
  (let ((r (select-stream (list strm) timeout)))
    (car (member strm r))))

(defun rd-s (&optional (strm *fork-stream*)) ;; read-line
  (let (lines)
    (while (rd-p strm)
      (push (read-line strm) lines))
    ;;(if (= (length l) 8192) )
    (nreverse lines)))

(defun rd-r (&optional (strm *fork-stream*)) ;; read-result
  (if (rd-p strm) (read strm) :fail))

(defun command-raw (com &optional (strm *fork-stream*))
  (format strm "~A~%" com))

(defun com (com-str &optional (timeout 10.0) (strm *fork-stream*) (return-size))
  ;;(print 'com)
  (while (rd-p) (read-line strm nil nil)) ;; flush stream
  ;;(print 'finishread)
  (if return-size
      (format strm "(calc-size ~A)~%" com-str)
    (format strm "(wobj ~A)~%" com-str))
  (let ((last :timeout) read-one)
    (do-until-key
     (unless (> timeout 0) (return-from com last))
     (cond
      ((rd-p strm 0.1)
       (setq read-one t)
       (setq last (read strm)))
      (read-one
       (return-from com last)))
     (decf timeout 0.1))
    :break))

(defun com-size (com-str &optional (timeout 10.0) (strm *fork-stream*))
  (com com-str timeout strm t))

(defun com-large (com-str &optional (timeout 10.0) (strm *fork-stream*))
  (let ((size (com-size com-str timeout strm)))
    (cond
     ((numberp size)
      (when (y-or-n-p (format nil ";; Can you get object with size ~D.  " size))
        (format strm "(get-prev)~%")
        (let ((last :timeout) read-one)
          (do-until-key
           (unless (> timeout 0) (return-from com-large last))
           (cond
            ((rd-p strm 0.1)
             (setq read-one t)
             (setq last (read strm)))
            (read-one
             (return-from com-large last)))
           (decf timeout 0.1))
          :break)))
     ((null size) :no-size)
     (t size))))

(defun com-large (com-str &optional (timeout 10.0) (strm *fork-stream*))
  (let ((size (com-size com-str timeout strm)))
    (cond
     ((numberp size)
      (when (y-or-n-p (format nil ";; Can you get object with size ~D.  " size))
        (com "(get-prev)")
        ))
     ((null size) :no-size)
     (t size))))

(defun retry-large (com-str &optional (timeout 10.0) (strm *fork-stream*))
  (let ((size (com "(prev-size)" timeout strm)))
    (cond
     ((numberp size)
      (when (y-or-n-p (format nil ";; Can you get object with size ~D.  " size))
        (com "(get-prev)")
        ))
     ((null size) :no-size)
     (t size))))

(defun simple-init ()
  (com "(simple-init)"))

(defun get-angle ()
  (com "(send *ri* :state :potentio-vector)"))

(defun get-model ()
  (com "(send *atlas* :angle-vector)"))
(defun get-lhand ()
  (com "(send *atlas* :hand :larm :angle-vector)"))
(defun get-rhand ()
  (com "(send *atlas* :hand :rarm :angle-vector)"))

(defun get-imu ()
  (com "(send *ri* :state :imucoords)"))
(defun get-imu2 ()
  (com "(get-imu-data)"))

(defun get-click-point ()
  (com "(get-point)"))

(defun get-map-cloud ()
  (com-large "(get-octomap)" 180))

(defun get-cloud (&key (size 40.0) (round 2000.0))
  (com-large
   (format nil "(let ((pt (get-full-cloud))) (send pt :filter :key #'(lambda (p) (< (norm p) ~f))) (pcl::downsample pt ~f ~f ~f))"
           round size size size) 360))

(defun get-v-view ()
  (com-large
   "(let ((pt (get-full-cloud)) qq) (setq qq (send pt :filter :key #'(lambda (p) (and (< (norm p) 1000) (> (elt p 0) 0) (> (elt p 1) -100))) :create t)) (pcl::downsample qq 25.0 25.0 25.0))" 360))

(defun psize ()
  (com "(prev-size)"))
(defun getp ()
  (com "(get-prev)"))

(defun move-world (vec &optional (limb :larm))
  (let ((cds (send *atlas* limb :end-coords :copy-worldcoords)))
    (send cds :translate vec :world)
    (send *atlas* limb :inverse-kinematics cds :rotation-axis nil)
    ))
