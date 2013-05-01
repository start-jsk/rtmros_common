(load "package://eusurdf/euslisp/eusurdf.l")

(defvar *parse-world-script*
  (ros::resolve-ros-path "package://hrpsys_gazebo/scripts/parse_world.py"))

(defun find-pos (substr str)
  (dotimes (i (- (length str) (length substr)))
    (when (= (elt str i) (elt substr 0))
      (when
          (block :substr
            (dotimes (j (length substr))
              (unless (= (elt str (+ i j)) (elt substr j))
                (return-from :substr nil)))
            t)
        (return-from find-pos i))))
  -1)

(defun search-gazebo-dir (dirname
                          &optional (gpath (unix::getenv "GAZEBO_MODEL_PATH"))
                          (added-path (list "/tmp")))
  (let (gdirs ret (prev 0))
    (dotimes (i (length gpath))
      (when (= (elt gpath i) #\:)
        (push (subseq gpath prev i) gdirs)
        (setq prev (1+ i))))
    (setq gdirs (append added-path gdirs)) ;; objects in web databese may be stored in /tmp
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
   ((substringp "file://media" fname)
    (let* ((urlname (url-pathname fname))
           (package-name (send urlname :host))
           (path-name (namestring urlname))
           (package-path (search-gazebo-dir (format nil "~A/~A" package-name path-name)
                                            (unix::getenv "GAZEBO_RESOURCE_PATH") nil)))
      package-path))
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
            ;; (pprint fn)
            (let (cds)
            (when pose
              (let ((pf (when pose (make-string-input-stream pose)))
                    ps)
                (let (a) (while (setq a (read pf nil nil)) (push  a ps)))
                (setq ps (nreverse ps))
                (setq cds
                      (make-coords :pos (scale 1000.0 (coerce (subseq ps 0 3) float-vector))
                                   :rpy (let ((rpy (subseq ps 3)))
                                          (list (elt rpy 2) (elt rpy 1) (elt rpy 0)))))))
            (push (sdf2eus fn) cas-links)
            (when cds (send (car cas-links) :transform cds))
            )))))
    ;; gather cas links
    cas-links))

(defun convert-world-to-eus (world-fname)
  (let ((model-lst (eval (read
                          (piped-fork
                           (format nil "~A ~A" *parse-world-script* world-fname))))))
    ;; (pprint model-lst)
    (make-gazebo-scene model-lst)))

(defun read-material-to-list (material-file)
  (let ((fp (piped-fork
             (format nil "sed -e 's@}@)@g' ~A | sed -e 's@{@(@g' | sed -e 's/:/@/g'"
                     material-file))))
    (let (elem ret tmp)
      (while (setq elem (read fp nil nil))
        (push elem tmp)
        (when (listp elem)
          (push (nreverse tmp) ret)
          (setq tmp nil)))
      (close fp)
      ret)))

;; implemented function in eusurdf.l
(defun parse-sdf-material (obj material-sxml)
  (let* ((script (cdr (assoc 'script material-sxml)))
         (name (cadr (assoc 'name script)))
         uris)
    (block :read-uri
      (let (uri)
        (setq uri (cadr (assoc 'uri script)))
        (unless uri (return-from :read-uri))
        (push uri uris)
        (setq script (delete uri script))))
    (let (material)
      (push (list :name name) material)
      (dolist (uri uris)
        (let* ((fname (ros::resolve-ros-path uri))
               (tp (probe-file fname)) tmp)
          (case tp
            (:file
             (setq tmp (find-if #'(lambda (l) (member (read-from-string name) l))
                                (read-material-to-list fname)))
             (when tmp
               (push (list :material tmp) material)
               (push (list :filename fname) material)))
            (:directory
             (push (list :dirname fname) material)))))
      (setq material (append material (parse-gazebo-material material)))
      ;;(pprint material)
      (block :add-texture
        (let ((texfile (cadr (assoc :texture-file material))))
          ;;
          (when (probe-file texfile)
            (let ((img (read-image-file texfile)))
              (dolist (f (send obj :faces))
                (send obj :paste-texture-to-face f :image img)))
            (return-from :add-texture))
          ;;
          (when texfile
            (let* ((pname (pathname texfile))
                   (fname (if (send pname :type)
                              (format nil "~A.~A" (send pname :name) (send pname :type))
                            (send pname :name)))
                   (gname (format nil "file://media/materials/textures/~A" fname)))
              (setq gname (ros::resolve-ros-path gname))
              (when (probe-file gname)
                (let ((img (read-image-file gname)))
                  (dolist (f (send obj :faces))
                    (send obj :paste-texture-to-face f :image img)))
                (return-from :add-texture))
              ))
          ;;
          (let ((amb (cadr (assoc :ambient material)))
                (dif (cadr (assoc :diffuse material)))
                (spec (cadr (assoc :specular material))))
            (when dif
              (setf (get obj :face-color) (coerce dif float-vector))))
          ))
      obj
      )))

(defun assoc-read-value (alist key &optional (len 3))
  (let ((pos (position key alist))
        ret)
    (when pos (incf pos))
    (when (and pos
               (<= (+ pos len)
                   (length alist)))
      (dotimes (i len)
        (push (elt alist (+ pos i)) ret))
      (nreverse ret))))

(defun parse-gazebo-material (material)
  (let ((fname (cadr (assoc :filename material)))
        (mat (cadr (assoc :material material)))
        texture-dir texture-file ret)
    (let ((pos (find-pos "scripts" fname)))
      (when (/= pos -1)
        (setq texture-dir (format nil "~Atextures" (subseq fname 0 pos)))))
    (let (;;(alist (cadr (massoc mat 'technique 'pass)))
          (alist (flatten mat))
          )
      ;;(setq texture-file (cadr (assoc 'texture alist)))
      (let ((p (position 'texture alist)))
        (when p (setq texture-file (elt alist (1+ p)))))
      (dolist (key (list (list 'ambient 4 :ambient) (list 'diffuse 4 :diffuse)
                         (list 'specular 5 :specular)))
        (let ((r (assoc-read-value alist (car key) (cadr key))))
          (when r (push (list (caddr key) r) ret)))))
    (when texture-dir
      (push (list :texture-dir texture-dir) ret))
    (when texture-file
      (push
       (if texture-dir (list :texture-file (format nil "~A/~A" texture-dir texture-file))
         (list :texture-file texture-file))
       ret))
    ret
    ))

#|
(convert-world-to-eus "/usr/share/drcsim-2.5/worlds/qual_task_1.world")
(read-material-to-list "/usr/share/gazebo-1.7/media/materials/scripts/gazebo.material")
(read-material-to-list "/usr/share/drcsim-2.5/media/materials/scripts/drc_qual.material")
|#
