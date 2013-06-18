;;(setq *fork-stream* (piped-fork "ssh" "leus@133.11.216.144" "roseus \"~/ros/fuerte/rtm-ros-robotics/rtmros_common/hrpsys_gazebo/euslisp/atlas-pipe-interface.l\""))

(defun connect-atlas (&optional (host "fc2")
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

(defun com (com-str &optional (timeout 10.0) (strm *fork-stream*))
  (print 'com)
  (while (rd-p)  (read-line nil nil)) ;; flush stream
  (print 'finishread)
  (format strm "(wobj ~A)~%" com-str)
  (do-until-key
   (unless (> timeout 0)
     (return-from com :timeout))
   (when (rd-p strm 0.1)
     (print 'read)
     (return-from com (read strm)))
   (decf timeout 0.1))
  :break)

(defun com2 (com-str &optional (timeout 10.0) (strm *fork-stream*))
  (while (rd-p)  (read-line nil nil)) ;; flush stream
  (format strm "(calc-size ~A)~%" com-str)
  (do-until-key
   (unless (> timeout 0)
     (return-from com2 :timeout))
   (when (rd-p strm 0.1)
     (return-from com2 (read strm)))
   (decf timeout 0.1))
  :break)

(defun com-large (com-str &optional (timeout 10.0) (strm *fork-stream*))
  (let ((size (com2 com-str)))
    (cond
     ((numberp size)
      (when (y-or-n-p (format nil ";; Can you get object with size ~D.  " size))
        (format strm "(get-prev)~%")
        (do-until-key
         (unless (> timeout 0)
           (return-from com-large :timeout))
         (when (rd-p strm 0.1)
           (return-from com-large (read strm)))
         (decf timeout 0.1)))
      :break)
     (t size))))

(defun simple-init ()
  (com "(simple-init)"))

(defun get-angle ()
  (com "(send *ri* :state :potentio-vector)"))

(defun get-click-point ()
  (com "(get-point)"))

(defun get-map-cloud ()
  (com-large "(get-octomap)"))

(defun get-cloud ()
  (com-large "(let ((pt (get-full-cloud))) (send pt :filter :key #'(lambda (p) (< (norm p) 6000))) (pcl::downsample pt 33.0 33.0 33.0))"))
