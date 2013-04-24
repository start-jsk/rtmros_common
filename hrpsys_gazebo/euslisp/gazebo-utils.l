(load "package://eusurdf/euslisp/eusurdf.l")

(defun search-gazebo-dir (dirname)
  (let ((gpath (unix::getenv "GAZEBO_MODEL_PATH"))
        gdirs ret (prev 0))
    (dotimes (i (length gpath))
      (when (= (elt gpath i) #\:)
        (push (subseq gpath prev i) gdirs)
        (setq prev (1+ i))))
    (setq gdirs (nreverse gdirs))
    (dolist (dir gdirs)
      (when (probe-file (setq ret (format nil "~A/~A" dir dirname)))
        (return-from search-gazebo-dir ret))
      )))

(defun resolve-gazebo-model-path (fname)
  (cond
   ((substringp "model://" fname)
    (let* ((urlname (url-pathname fname))
           (package-name (send urlname :host))
           (path-name (namestring urlname))
           (package-path (search-gazebo-dir package-name)))
      (format nil "~A~A" package-path path-name)))
   (t fname)))

(unless (fboundp 'ros::resolve-ros-path-org)
  (setf (symbol-function 'ros::resolve-ros-path-org)
        (symbol-function 'ros::resolve-ros-path)))

(defun ros::resolve-ros-path (fname)
  (cond
   ((substringp "package://" fname)
    (ros::resolve-ros-path-org fname))
   (t
    (resolve-gazebo-model-path fname))))

(unless (fboundp 'load-mesh-file-org)
  (setf (symbol-function 'load-mesh-file-org)
        (symbol-function 'load-mesh-file)))

(defun load-mesh-file (fname &rest args)
  (let* ((ret (apply #'load-mesh-file-org fname args))
         (fname (send ret :filename)))
    (when fname
      (send ret :load-texture-image
            (append (butlast (pathname-directory (truename (pathname fname))))
                    (list "materials" "textures"))))
    ret))

(defun make-gazebo-scene (lst)
  (let ((cas-links))
    (dolist (l lst)
      (let ((mdl (cadr (assoc :model l)))
            (pose (cadr (assoc :pose l))))
        (let* ((dir (ros::resolve-ros-path mdl))
               (fn (format nil "~A/model.sdf" dir)))
          (when (probe-file fn)
            (let ((pf (make-string-input-stream pose))
                  ps cds)
              (let (a) (while (setq a (read pf nil nil)) (push  a ps)))
              (setq ps (nreverse ps))
              (setq cds
                    (make-coords :pos (scale 1000.0 (coerce (subseq ps 0 3) float-vector))
                                 :rpy (let ((rpy (subseq ps 3)))
                                        (list (elt rpy 2) (elt rpy 1) (elt rpy 0)))))
              (push (sdf2eus fn) cas-links)
              (send (car cas-links) :transform cds)
              )))))
    ;; gather cas links
    cas-links))
#|
(load "package://hrpsys_gazebo/euslisp/gazebo-utils.l")
(setq *parse-world-scripts* (ros::resolve-ros-path "package://hrpsys_gazebo/scripts/parse_world.py"))
(setq ll (eval (read (piped-fork
                      (format nil "~A ~A"  *parse-world-scripts*
                              "/usr/share/drcsim-2.5/worlds/qual_task_1.world")))))
(make-gazebo-scene ll)
|#
