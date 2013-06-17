;;(setq *fork-stream* (piped-fork "ssh" "leus@133.11.216.144" "roseus \"~/ros/fuerte/rtm-ros-robotics/rtmros_common/hrpsys_gazebo/euslisp/atlas-interface.l\""))

(defun connect-atlas (&optional (host "fc2")
                                (com "roseus \"~/ros/fuerte/rtm-ros-robotics/rtmros_common/hrpsys_gazebo/euslisp/atlas-interface.l\""))
  (setq *fork-stream* (piped-fork "ssh" host com)))

(defun readablep (&optional (strm *fork-stream*))
  (let ((r (select-stream (list strm) 0.1)))
    (car (member strm r))))

(defun command (com &optional (strm *fork-stream*))
  (format strm "~A~%" com))
