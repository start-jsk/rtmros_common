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

(defun com (com &optional (timeout 10.0) (strm *fork-stream*))
  (while (readablep)
    (read-line nil nil)) ;; flush stream
  (format strm "(wobj ~A)~%" com)
  (do-until-key
   (unless (> timeout 0)
     (return-from command :timeout))
   (when (readablep strm 0.1)
     (return-from command (read strm)))
   (decf timeout 0.1))
  :break)
