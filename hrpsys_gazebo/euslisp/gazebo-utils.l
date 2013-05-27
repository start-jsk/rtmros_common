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
    (when (substringp ".dae" fname)
      (let ((scl (check-unit-dae fname)))
        ;;(pprint (list :fname fname :unit scl))
        (unless (= scl 1.0)
          (let ((mlst (ret . gl::mesh-list)))
            (dolist (m mlst)
              (let ((mat (cadr (assoc :vertices m))))
                (scale scl (array-entity mat) (array-entity mat))))))))
    ;;
    (when fname
      (send ret :load-texture-image
            (append (butlast (pathname-directory (truename (pathname fname))))
                    (list "materials" "textures"))))
    ret))

(unless (fboundp 'read-image-file-org)
  (setf (symbol-function 'read-image-file-org)
        (symbol-function 'read-image-file)))

(defun read-image-file (fname)
  (let ((r (read-image-file-org fname)))
    (unless r
      (setq r (assimp-read-image-file fname)))
    r))

(defun make-gazebo-scene (lst)
  (let ((cas-links))
    (dolist (l lst)
      (let ((mdl (cadr (assoc :model l)))
            (pose (cadr (assoc :pose l)))
            (name (cadr (assoc :name l))))
        (let* ((dir (ros::resolve-ros-path mdl))
               (fn (format nil "~A/model.sdf" dir))
               (fn-config (format nil "~A/model.config" dir)))
          (when (probe-file fn-config)
            (let ((sdf-lst (sxml-find-element-by-tagname 'sdf (read-sxml fn-config)))
                  nfn)
              (dolist (sdfl sdf-lst)
                (when (string= (cadr (assoc 'version (assoc '@ sdfl))) "1.3")
                  (setq nfn (car (last sdfl)))))
              (unless nfn ;; defualt sdf
                (setq nfn (car (last (car sdf-lst)))))
              (when (stringp nfn)
                (setq nfn (format nil "~A/~A" dir nfn))
                (when (probe-file nfn) (setq fn nfn)))
              ))
          (when (probe-file fn)
            (warn ";; start converting model ~A~%" fn)
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
            (cond
             ((substringp ".urdf" fn)
              (push (urdf2eus fn) cas-links))
             (t (push (sdf2eus fn) cas-links)))
            (when cds (send (car cas-links) :transform cds))
            )))))
    ;; gather cas links
    cas-links))

(defun parse-world-to-list (world-fname)
  (eval (read
         (piped-fork
          (format nil "~A ~A" *parse-world-script* world-fname)))))
(defun convert-world-to-eus (world-fname)
  (let ((model-lst (parse-world-to-list world-fname)))
    (setq model-lst (delete-if #'(lambda (x) (substringp "atlas_sandia_hands" (cadr (assoc :model x)))) model-lst))
    (make-gazebo-scene model-lst)))

(defun read-material-to-list (material-file)
  (let ((fp (piped-fork
             (format nil "sed -e 's/texture \\+\\(.*\\)/texture \"\\1\"/g' ~A | sed -e 's@}@)@g' | sed -e 's@{@(@g' | sed -e 's/:/@/g'"
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
      (while t
        (let (uri)
          (setq uri (assoc 'uri script))
          (unless uri (return-from :read-uri))
          (push (cadr uri) uris)
          (setq script (delete uri script)))))
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
             (cond
              ((substringp "scripts" fname)
               (let ((m (find-if #'(lambda (x) (substringp ".material" x)) (directory fname))))
                 (when m
                   (setq m (format nil "~A/~A" fname m))
                   (setq tmp (find-if #'(lambda (l) (member (read-from-string name) l))
                                      (read-material-to-list m)))
                   (when tmp
                     (push (list :material tmp) material)
                     (push (list :filename m) material)))))
              (t (push (list :dirname fname) material))))
            )))
      (setq material (append material (parse-gazebo-material material)))
      (pprint material)
      (block :add-texture
        (let ((texfile (cadr (assoc :texture-file material))))
          ;;
          (when (probe-file texfile)
            (let ((img (read-image-file texfile)))
              (add-texture-to-obj obj img))
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
                  (add-texture-to-obj obj img))
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

(defun add-texture-to-obj (obj img)
  (cond
   ((derivedp obj gl::glvertices)
    (let ((ml (obj . gl::mesh-list)))
      (dolist (m ml)
        (let ((mt (cadr (assoc :material m))))
          (unless (assoc mt :teximage)
            (nconc mt (list (list :teximage img))))))))
   (t
    (dolist (f (send obj :faces))
      (send obj :paste-texture-to-face f :image img)))
   ))

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

(defun check-unit-dae (fname)
  ""
  (let ((pf (piped-fork (format nil "python -c \"import xml.dom.minidom; import sys; doc = xml.dom.minidom.parse('~A'); uni_elem = doc.getElementsByTagName('unit'); uni_elem != [] and uni_elem[0].hasAttribute('meter') and [sys.stdout.write(uni_elem[0].getAttribute('meter'))] or [sys.stdout.write('1.0')]\"" fname))))
    (prog1 (read pf)
      (close pf))))

(defun make-heightmap-faces (img &key ((:scale scl) (float-vector 1 1 1))
                                 (pos (float-vector 0 0 0))
                                 (image-scale (float-vector 1 1)) (texture))
  (let* ((w (send img :width))
         (h (send img :height))
         (buf (array-entity img))
         (sclx (elt scl 0))
         (scly (elt scl 1))
         (sclz (elt scl 2))
         (offx (elt pos 0))
         (offy (elt pos 1))
         (offz (elt pos 2))
         (mat (make-matrix (* 2 (1- h) w) 3))
         (tcoords (instantiate float-vector (* 2 (* 2 (1- h) w))))
         (idx (instantiate integer-vector (* 6 (1- h) (- w 1))))
         (cntr 0)
         (ifw (/ 1.0 (float w)))
         (ifh (/ 1.0 (float h)))
         ret)
    (dotimes (i (1- h))
      (dotimes (j w)
        (c-matrix-row mat cntr
                      (float-vector
                       (+ (* sclx j) offx)
                       (+ (* scly i) offy)
                       (+ (* sclz (/ (elt buf (+ (* i w) j)) 255.0)) offz)) t)
        (setf (elt tcoords (* 2 cntr)) (* j ifw))
        (setf (elt tcoords (1+ (* 2 cntr))) (* i ifh))
        (incf cntr)
        (c-matrix-row mat cntr
                      (float-vector
                       (+ (* sclx j) offx)
                       (+ (* scly (1+ i)) offy)
                       (+ (* sclz (/ (elt buf (+ (* (1+ i) w) j)) 255.0)) offz)) t)
        (setf (elt tcoords (* 2 cntr)) (* j ifw))
        (setf (elt tcoords (1+ (* 2 cntr))) (* (1+ i) ifh))
        (incf cntr)
        ))
    (setq cntr 0)
    (dotimes (i (1- h))
      (dotimes (j (- w 1))
        (let ((st (* 2 (+ (* w i) j))))
          (setf (elt idx cntr) st) (incf cntr)
          (setf (elt idx cntr) (+ st 1)) (incf cntr)
          (setf (elt idx cntr) (+ st 2)) (incf cntr)

          (setf (elt idx cntr) (+ st 3)) (incf cntr)
          (setf (elt idx cntr) (+ st 2)) (incf cntr)
          (setf (elt idx cntr) (+ st 1)) (incf cntr)
          )))
    (let (ret)
      (when texture
        (push (list :material (list (list :teximage texture))) ret))
      (push (list :type :triangles) ret)
      (push (list :vertices mat) ret)
      (push (list :indices idx) ret)
      (push (list :texcoords tcoords) ret)
      (instance gl::glvertices :init (list ret)))
    ))
;; making meshes of hand
;;(load "package://eusurdf/euslisp/eusurdf.l")
;;(setq ll (read-csg-bodies (dump-csg-bodies (cdr (send (car (urdf2eus "sandia_left.urdf")) :bodies)))))
;;(setq gv (convert-gl-vertices-bodies ll))
;;(send gv :rotate -pi/2 :x)
;;(send gv :translate #f(0 0 -500))
;;(send gv :reset-offset-from-parent)
;;(store-glvertices "sandia_left.dae" gv :scale 0.001)
(defun dump-csg-bodies (ll)
  (let (ret)
    (dolist (l ll)
      (let ((csg-cf (send l :csg)))
        (let ((cb (assoc :cube csg-cf)))
          (when (or (and cb
                         (or (> (elt (cdr cb) 0) 5.1)
                             (> (elt (cdr cb) 1) 5.1)
                             (> (elt (cdr cb) 2) 5.1)))
                    (null cb))
            (push
             (list (cons :csg csg-cf)
                   (cons :coords (send l :copy-worldcoords))) ret)))))
    ret))

(defun parse-csg (cf)
  (let ((bx (cdr (assoc :cube cf)))
        (cl (cdr (assoc :cylinder cf)))
        (tv (cadr (assoc :translate-vertices cf)))
        bd)
    (cond
     (bx (setq bd (apply #'make-cube bx)))
     (cl (setq bd (apply #'make-cylinder (subseq cl 0 2)))))
    (when tv
      (send bd :translate-vertices tv))
    bd
    ))

(defun read-csg-bodies (ret)
  (mapcar #'(lambda (l)
              (let ((cds (cdr (assoc :coords l)))
                    (csg-cf (cdr (assoc :csg l)))
                    bd)
                (setq bd (parse-csg csg-cf))
                (when cds (send bd :transform cds))
                bd)) ret))

(defun convert-gl-vertices-bodies (bds) ;; fix origin
  (let ((gv
         (mapcar #'(lambda (l)
                     (let ((glv (make-glvertices-from-faces (send l :faces))))
                       ;;(send glv :transform (send l :worldcoords))
                       glv)) bds)))
    (send (car gv) :append-glvertices (cdr gv))
    ))

#|
(setq teximg (read-image-file "/usr/share/gazebo-1.7/media//materials/textures/dirt_diffusespecular.png"))
(setq img (read-image-file "/usr/share/drcsim-2.5/gazebo_models/environments/vrc_heightmap_1/materials/textures/heightmap.png"))
(make-heightmap-faces img :scale #f(10 10 1000) :texture teximg)

(convert-world-to-eus "/usr/share/drcsim-2.5/worlds/qual_task_1.world")
(convert-world-to-eus "/usr/share/drcsim-2.5/worlds/vrc_task_1.world")
(read-material-to-list "/usr/share/gazebo-1.7/media/materials/scripts/gazebo.material")
(read-material-to-list "/usr/share/drcsim-2.5/media/materials/scripts/drc_qual.material")

drill should be Z_UP ???
#|
