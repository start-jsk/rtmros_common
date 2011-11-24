(load "vrmlParser.l")
(load "vrmlNodeSpec.l")

(in-package "VRML")

;;;;;;;;;;;;;;;;;;;;;;;;
;; VRML PROTO Parser
;;;;;;;;;;;;;;;;;;;;;;;;

;; Vrml Reader
;;   We can directly read vrml file substituting '{', '}', and ',' -> '(', ')', and '?'
;;   For example,
;;   1. original vrml file
;;      DEF AAA BB {
;;        cc [1.0, 2.0]
;;      }
;;    ->
;;   2. fixed vrml file
;;      DEF AAA BB (
;;        cc [1.0? 2.0]
;;      )
;;   ->
;;   3. read s expression
;;      '(DEF AA BB (CC (aref 1.0? 2.0)))
(defun read-vrml-file-with-vrml-fix (vrml-fname)
  (let ((fixed-vrml-fname (format nil "/tmp/~A.readvrmltmp" (pathname-name vrml-fname))) (ret) (str))
    (unix:system (format nil "cp ~A ~A" vrml-fname fixed-vrml-fname))
    (sed-vrml-file-using-s-expression fixed-vrml-fname)
    (with-replace-read-macros
     (with-open-file
      (f fixed-vrml-fname :direction :input)
      (while (setq str (read f nil nil)) (push str ret)))
     (unix:system (format nil "rm -f ~A" fixed-vrml-fname))
     (reverse ret))))

(defmacro with-replace-read-macros
  (&rest forms)
  (let ((tmp-readtable (gensym)))
    `(let ((,tmp-readtable (copy-readtable)))
       ;; cannot use ( and ) ??
       ;;(set-macro-character #\{ #'(lambda (strm char) '|(|) nil ,tmp-readtable)
       ;;(set-macro-character #\} #'(lambda (strm char) '|)|) nil ,tmp-readtable)
       (set-macro-character #\, #'(lambda (strm char) '|?|) nil ,tmp-readtable)
       (let ((*readtable* ,tmp-readtable))
	 ,@forms
	 ))))

;; Define vrml node spec from read vrml data
;;   'data' must be '(proto aa (field-definition-for-aa) (implementation-for-aa) proto bb (field-definition-for-bb) (implementation-for-bb) ...)
(defun define-vrml-node-spec-from-read-vrml-data (data &optional (debug nil))
  (cond
   ((null data))
   ((eq (car data) 'proto)
    (let ((proto-name (cadr data))
	  (field-defs (cdr (caddr data)))
	  (impl-defs (cadr (cddr data))))
      (when debug
	(print "in define-vrml-node-spec-from-read-vrml-data")
	(print (list 'proto proto-name))
	(print (list 'field field-defs))
	(print (list 'impl impl-defs)))
      (eval `(define-vrml-node-spec ,proto-name
	       nil
	       ,@(store-vrml-fields field-defs)))
      (define-vrml-node-spec-from-read-vrml-data (cddddr data) debug)
      ))
   (t (define-vrml-node-spec-from-read-vrml-data (cdr data) debug))
   ))

(defun store-vrml-fields (field-defs)
  (let ((sret) (ret))
    (labels ((_store-fields
              (fd)
              (cond
               ((memq (car fd) '(field exposedField))
		(when sret
		  (if (> (length sret) 4)
		      (let ((tmpret (reverse sret)))
			(push (append (subseq tmpret 0 3) (list (subseq tmpret 3))) ret))
		    (push (reverse sret) ret)))
		(setq sret (list (cadr fd) (car fd)))
		(_store-fields (cddr fd)))
               ((null fd))
               (t
                (push (car fd) sret)
                (_store-fields (cdr fd))))))
      (_store-fields field-defs)
      (when sret (push (reverse sret) ret))
      ret)))

(defun store-vrml-impls (impl-defs field-defs)
  (let ((sret) (ret))
    (labels ((_store-impls
              (fd)
              (cond
               ((eq (car fd) 'is)
                (when sret (push (reverse sret) ret))
                (push (car (last (find-if #'(lambda (x) (eq (cadr x) (cadr fd)))
                                          (store-fields field-defs))))
                      sret)
                (_store-impls (cddr fd)))
               ((null fd))
               (t
                (push (car fd) sret)
                (_store-impls (cdr fd))))))
      (_store-impls impl-defs)
      (when sret (push (reverse sret) ret))
      ret)))

;;;;;;;;;;;;;;;;;;;;;;;;
;; Define VRML PROTO for webots
;;  Although these should be automatically generated from PROTO definition, current webots archive does not include PROTO definition.
;;;;;;;;;;;;;;;;;;;;;;;;
(define-vrml-node-spec Robot
  nil
  (field SFVec3f    translation (0 0 0))
  (field SFRotation rotation  (0 1 0 0))
  (field SFString   controller "void")
  (field SFString   controllerArgs "")
  (field SFString   name "DARwIn-OP")
  (field SFString   model "DARwIn-OP")
  (field SFString   description "DARwIn-OP")
  (field SFColor    plasticColor 0.301961 0.301961 0.301961)
  (field SFVec2f    camera_windowPosition 0 0)
  (field SFFloat    camera_pixelSize 1.0)
  (field SFInt32    camera_width 160)
  (field SFInt32    camera_height 120)
  (exposedfield MFNode   boundingobject        ())
  (exposedfield MFNode   children        ())
  (exposedfield MFNode Physics ())
  )

(define-vrml-node-spec Accelerometer
  nil
  (field SFInt32    sensorid -1)
  (field SFRotation rotation 0)
  (field SFVec3f translation 0)
  (field SFString name "")
  (field MFFloat lookupTable ())
  )

(define-vrml-node-spec Gyro2
  nil
  (field SFInt32    sensorid -1)
  (field SFRotation rotation 0)
  (field SFVec3f translation 0)
  (field SFString name "")
  (field MFFloat lookupTable ())
  )

(define-vrml-node-spec Servo
  nil
  (field SFRotation rotation (0 1 0 0))
  (field SFVec3f translation 0)
  (field SFString name "")
  (field SFFloat maxForce 0.0)
  (exposedfield MFNode   children        ())
  (exposedfield MFNode boundingObject ())
  (exposedfield MFNode Physics())
  )

(define-vrml-node-spec Physics
  nil
  (field SFFloat mass 0.0)
  (field SFVec3f centerOfMass (0 0 0))
  (field SFInt32 density -1)
  )

(define-vrml-node-spec Capsule
  nil
  (field SFFloat height 0.0)
  (field SFFloat radius 0.0)
  )

(define-vrml-node-spec LED
  nil
  (field SFRotation rotation 0)
  (field SFVec3f translation 0)
  (field SFString name "")
  (exposedfield MFNode   children        ())
  (field SFColor color ())
  )

(define-vrml-node-spec Camera
  nil
  (field SFRotation rotation 0)
  (field SFVec3f translation 0)
  (field SFString name "")
  (field SFInt32 width 0)
  (field SFInt32 height 0)
  (field SFFloat pixelSize 0)
  (exposedfield MFNode   children        ())
  (exposedfield SFVec2f windowPosition ())
  )

;;;;;;;;;;;;;;;;;;;;;;;;
;; VRML Parser
;;   Read VRML file and convert s-expression.
;;   For example,
;;   DEF node-name node-type (
;;     field1 [1.0? 2.0]
;;     field2 3.0
;;   )
;;   ->
;;   '(def node-name node-type
;;      (field1 (aref 1.0? 2.0))
;;      (field2 3.0)
;;     )
;;   Only parse nodes and fields defined by PROTO.
;;;;;;;;;;;;;;;;;;;;;;;;
(defun is-node (l)
  (or (consp (cadddr l)) (consp (cadr l))))

;; get field names which uses MFNode
(defun get-field-names-using-MFNode ()
  (remove-duplicates
   (apply #'append
	  (mapcar #'(lambda (y)
		      (mapcar #'car (remove-if-not #'(lambda (x) (eq (elt x 1) 'mfnode)) (cdr (assoc 'field (send y :get :vrml-node-spec))))))
		  *nodetypes*))))

;; Split field
;;   '(def node-name node-type (field1 value1 field2 value2))
;;   ->
;;   '(def node-name node-type (field1 value1) (field2 value2))
(defun split-fields (lis node-type &optional (debug t))
  (if debug (print (list "in split-fields" lis node-type)))
  (let ((sret) (ret)
        (node-fields (mapcar 'car (cdr (assoc 'field (send node-type :get :vrml-node-spec))))))
    (labels ((_split-fields
	      (l)
	      (cond
	       ((null l))
	       ((memq (car l) node-fields)
		(when sret (push (reverse sret) ret))
		(cond
		 ((eq (cadr l) 'is)
		  (let ((default-proto-argument
			  (caddr (assoc (caddr l) (cdr (assoc 'field (get 'darwinop :vrml-node-spec)))))))
		    (setq sret (append (if (consp default-proto-argument) default-proto-argument (list default-proto-argument))
				       (list (car l)))))
		  (_split-fields (cdddr l)))
		 ((eq (cadr l) 'def)
		  (setq sret (reverse (subseq l 0 4)))
		  (_split-fields (cddddr l)))
		 (t
		  (setq sret (reverse (subseq l 0 2)))
		  (_split-fields (cddr l)))
		 ))
	       (t
		(if (eq (car l) 'is)
		    (push (cddr (assoc (cadr l) (cdr (assoc 'field (get 'darwinop :vrml-node-spec))))) sret)
		  (push (car l) sret))
		(if (eq (car l) 'is)
		    (_split-fields (cddr l))
		  (_split-fields (cdr l)))))))
      (_split-fields lis)
      (when sret (push (reverse sret) ret))
      (if debug (print (list 'ret-fields (reverse ret))))
      (mapcar #'(lambda (x)
		  (if debug (print (list 'fuga (car x) (cadr x) x (is-node (cdr x)))))
		  (cond 
		   ((is-node (cdr x))
		    (append (list (car x)) (split-nodes (cdr x) debug)))
		   ((memq (car x) (get-field-names-using-MFNode))
		    (append (list (car x)) (list (split-nodes (cadr x) debug))))
		   (t x)))
	      (reverse ret))
      )))

;;  Split node
;;   '(def node-name1 node-type1 (fields1..) def node-name2 node-type2 (fields2..))
;;   ->
;;   '((def node-name1 node-type1 (fields1..)) (def node-name2 node-type2 (fields2..)))
(defun split-nodes (lis &optional (debug t))
  (if debug (print (list "in split-nodes" lis)))
  (let ((sret) (ret))
    (labels ((_split-nodes
	      (l)
	      (cond
	       ((null l))
	       ((and (or (eq 'def (car l)) (memq (car l) *nodetypes*))
		     (find-if #'consp l))
		(when sret (push (reverse sret) ret))
		(if (eq 'def (car l))
		    (setq sret (reverse (subseq l 0 3)))
		  (setq sret (list (car l))))
		(if (eq 'def (car l))
		    (_split-nodes (cdddr l))
		  (_split-nodes (cdr l))))
	       (t
		(push (car l) sret)
		(_split-nodes (cdr l))))))
      (_split-nodes lis)
      (when sret (push (reverse sret) ret))
      (if debug (print (list 'ret-node (reverse ret))))
      (mapcar #'(lambda (x)
		  (if debug (print (list 'hoge (car x))))
		  (if (eq (car x) 'def)
		      (append (subseq x 0 3)
			      (split-fields (elt x 3) (elt x 2) debug))
		    (append (list (car x))
			    (split-fields (cadr x) (car x) debug))))
	      (reverse ret))
      )))

;;;;;;;;;;;;;;;;;;;;;;;;
;; Preprocess for VRML convert
;;;;;;;;;;;;;;;;;;;;;;;;
(defun get-vrml-convert-script-dir ()
  (let* ((strm (piped-fork "rospack find hrpsys"))
	 (str (format nil "~A/scripts" (read-line strm nil nil))))
    (close strm)
    str))

(defun preprocess-for-vrml-convert (robot-vrml-fname)
  ;; define OpenHRP3 PROTOs
  (define-vrml-node-spec-from-read-vrml-data (read-vrml-file-with-vrml-fix (format nil "~A/openhrp3_protos.wrl" (get-vrml-convert-script-dir))))
  (define-vrml-node-spec-from-read-vrml-data (read-vrml-file-with-vrml-fix robot-vrml-fname))
  (def-all-print-name)
  t)

(defun preprocess-for-vrml-convert-for-darwin ()
  (preprocess-for-vrml-convert "$(rospack find webots)/build/webots/projects/robots/darwin-op/protos/DARwInOP.proto")
  ;; rename webots Gyro -> Gyro2 because the name 'Gyro' is duplicate name
  (unix:system "cp $(rospack find webots)/build/webots/projects/robots/darwin-op/protos/DARwInOP.proto /tmp/DARwInOP.proto")
  (unix:system "sed -i -e s/Gyro/Gyro2/g /tmp/DARwInOP.proto")
  t)

(defun post-process-for-vrml-convert (robot-vrml-fname)
  (sed-s-expression-file-using-vrml robot-vrml-fname))

(defun print-comment-header-for-vrml-file (fstrm)
  (format fstrm "# DO NOT EDIT THIS FILE~%")
  (format fstrm "# This file is automatically generated from euslisp~%"))

;; concatenate OpenHRP3 PROTO definition
(defun print-openhrp3-vrml-protos (fstrm)
  (with-open-file
   (ifstrm (format nil "~A/openhrp3_protos.wrl" (get-vrml-convert-script-dir)) :direction :input)
   (let ((str))
     (while (setq str (read-line ifstrm nil nil))
       (format fstrm "~A~%" str)
       ))))

;; vrml -> fixed vrml
(defun sed-vrml-file-using-s-expression (fname)
  (unix:system (format nil "sed -i -e s/\{/\\(/g ~A" fname))
  (unix:system (format nil "sed -i -e s/\}/\\)/g ~A" fname))
  ;; remove lines with #
  (unix:system (format nil "grep -v \"\#\" ~A > /tmp/tmp.wrl && cp /tmp/tmp.wrl ~A && rm -f /tmp/tmp.wrl" fname fname))
  )

;; fixed vrml -> vrml
(defun sed-s-expression-file-using-vrml (fname)
  (unix:system (format nil "sed -i -e s/\\(/\{/g ~A" fname))
  (unix:system (format nil "sed -i -e s/\\)/\}/g ~A" fname))
  )

;;;;;;;;;;;;;;;;;;;;;;;;
;; Printer for splitted s-expression vrml data
;;;;;;;;;;;;;;;;;;;;;;;;
(defun print-all-vrml-nodes-fields
  (fname splitted-vrml-data &optional (print-func))
  (with-open-file
      (fstrm fname :direction :output)
      (if print-func (funcall print-func fstrm))
      (print-vrml-nodes fstrm splitted-vrml-data))
  )

(defun gen-indent-string-from-size (size &optional (default-width 2))
  (let ((ret ""))
    (dotimes (i (* default-width size))
      (setq ret (format nil " ~A" ret)))
    ret))

(defun print-vrml-nodes (fstrm lis &optional (indent-size 0) (use-indent t))
  (cond
   ((and (consp (car lis)) (eq (caar lis) 'aref))
    (format fstrm "~A[~%" (if use-indent (gen-indent-string-from-size indent-size) ""))
    (mapcar #'(lambda (x)
		(print-vrml-nodes fstrm x indent-size))
	    (cdr lis))
    (format fstrm "~A]~%" (gen-indent-string-from-size indent-size)))
   ((consp (car lis))
    (mapcar #'(lambda (x)
		(print-vrml-nodes fstrm x indent-size))
	    lis))
   ((eq (car lis) 'def)
    (format fstrm "~ADEF ~A ~A {~%"
	    (if use-indent (gen-indent-string-from-size indent-size) "")
	    (if (send (caddr lis) :get :print-field-varname-upcase)
		(string-upcase (cadr lis))
	      (cadr lis))
	    (send (caddr lis) :get :print-object-string))
    (print-vrml-fields fstrm (cdddr lis) (caddr lis) (+ indent-size 2))
    (format fstrm "~A}~%" (gen-indent-string-from-size indent-size)))
   (t
    (format fstrm "~A~A {~%"
	    (if use-indent (gen-indent-string-from-size indent-size) "")
	    (send (car lis) :get :print-object-string))
    (print-vrml-fields fstrm (cdr lis) (car lis) (+ indent-size 2))
    (format fstrm "~A}~%" (gen-indent-string-from-size indent-size))
    ))
  )

(defun print-vrml-fields (fstrm lis node-type &optional (indent-size 0))
  (labels ((check-string-or-not
	    (l)
	    (cond
	     ((stringp l) (format nil "\"~A\"" l))
	     ((eq l 'use) "USE")
	     ((eq l '|?|) ",")
	     (t l))))
    (dolist (ll lis)
      (format fstrm "~A~A " (gen-indent-string-from-size indent-size)
	      (if (cadr (assoc (car ll) (get node-type :print-field-strings)))
		  (cadr (assoc (car ll) (get node-type :print-field-strings)))
		(car ll)))
      (let ((upcasep
	     (memq (car ll) (get node-type :print-field-upcase))))
	(cond
	 ((and (consp (car (cdr ll))) (eq (caar (cdr ll)) 'aref))
	  (format fstrm "[~%")
	  (dolist (l (cdar (cdr ll)))
	    (format fstrm "~A " (if upcasep (string-upcase (check-string-or-not l)) (check-string-or-not l))))
	  (format fstrm "]~%"))
	 ((consp (car (cdr ll)))
	  (print-vrml-nodes fstrm (car (cdr ll)) (+ indent-size 2) nil))
	 (t
	  (dolist (l (cdr ll))
	    (format fstrm "~A " (if upcasep (string-upcase (check-string-or-not l)) (check-string-or-not l)))
	    )))
	(format fstrm "~%")
	))
    ))

;; Define print strings for nodes, fieldes, and field values
(defun def-all-print-name ()
  (dolist (l *nodetypes*)
    (send l :put :print-object-string (string-capitalization (format nil "~A" l))))
  (send 'indexedfaceset :put :print-object-string "IndexedFaceSet")
  (send 'accelerationsensor :put :print-object-string "AccelerationSensor")
  (send 'visionsensor :put :print-object-string "VisionSensor")
  (send 'joint :put :print-field-strings (list))
  (push (list 'jointaxis "jointAxis") (get 'joint :print-field-strings))
  (push (list 'jointid "jointId") (get 'joint :print-field-strings))
  (push (list 'jointtype "jointType") (get 'joint :print-field-strings))
  (send 'humanoid :put :print-field-strings (list))
  (push (list 'humanoidbody "humanoidBody") (get 'humanoid :print-field-strings))
  (send 'material :put :print-field-strings (list))
  (push (list 'specularcolor "specularColor") (get 'material :print-field-strings))
  (push (list 'diffusecolor "diffuseColor") (get 'material :print-field-strings))
  (push (list 'ambientintensity "ambientIntensity") (get 'material :print-field-strings))
  (send 'indexedfaceset :put :print-field-strings (list))
  (push (list 'coordindex "coordIndex") (get 'indexedfaceset :print-field-strings))
  (push (list 'creaseangle "creaseAngle") (get 'indexedfaceset :print-field-strings))
  (send 'segment :put :print-field-strings (list))
  (push (list 'centerOfMass "centerOfMass") (get 'segment :print-field-strings))
  (push (list 'momentsofinertia "momentsOfInertia") (get 'segment :print-field-strings))
  (send 'AccelerationSensor :put :print-field-strings (list))
  (push (list 'sensorid "sensorId") (get 'accelerationsensor :print-field-strings))
  (send 'Gyro :put :print-field-strings (list))
  (push (list 'sensorid "sensorId") (get 'gyro :print-field-strings))
  (send 'VisionSensor :put :print-field-strings (list))
  (push (list 'sensorId "sensorId") (get 'VisionSensor :print-field-strings))
  (send 'humanoid :put :print-field-upcase (list 'joints 'segments))
  (send 'segment :put :print-field-varname-upcase t)
  (send 'joint :put :print-field-varname-upcase t)
  )

;; convert "string" -> "String"
(defun string-capitalization (str)
  (labels ((_range
	    (len)
	    (let ((ret))
	      (dotimes (i len) (push i ret))
	      (reverse ret))))
    (let ((ret2 (mapcar #'(lambda (i) (char str i))
			(_range (length str)))))
      (concatenate string (append (list (char-upcase (car ret2))) (mapcar #'char-downcase (cdr ret2))))
      )))
;; (print-all-vrml-nodes-fields "hoge.wrl" (split-nodes (append (list 'aref) cc) nil))
;; (print-all-vrml-nodes-fields "hoge.wrl" (split-nodes (append (list 'aref) data2) nil))

;;;;;;;;;;;;;;;;;;;;;;;;
;; Webots VRML -> OpenHRP3 VRML Converter
;;;;;;;;;;;;;;;;;;;;;;;;
(defun convert-webots-vrml->openhrp3-vrml-for-darwin
  (&optional (openhrp3-vrml-fname "DARWIN.wrl"))
  (in-package "VRML") ;; execute all conversion in "VRML" package because of using vrml:: grobal variables in vrmlNodeSpec.l
  (preprocess-for-vrml-convert-for-darwin)
  (let* ((webots-vrml-fname "DARwInOP.proto")
	 (openhrp-vrml-data
	  (append-openhrp3-humanoid
	   (replace-all-objects-for-Webots->OpenHRP3
	    (insert-segment
	     (append-openhrp3-waist-joint
	      (split-nodes
	       (cadddr (read-vrml-file-with-vrml-fix (format nil "/tmp/~A" webots-vrml-fname)))
	       nil) ;; debug print nil
	      (list 0 0 0.33951) (list 0.57735 0.57735 0.57735 2.09439) ;; WAIST translation and rotation
	      )
	     (darwin-inertia-lists) ;; inertia-maps for insert-segment
	     ))
	   "DARWIN")))
    (print-all-vrml-nodes-fields
     openhrp3-vrml-fname openhrp-vrml-data
     #'(lambda (fstrm)
	 (print-comment-header-for-vrml-file fstrm)
	 (print-openhrp3-vrml-protos fstrm)))
    (post-process-for-vrml-convert openhrp3-vrml-fname)
    (unix:system "rm -f mp_*.jpg mp_*.txt DARwIn-OP_Dynamics.zip")
    ;;(unix:system "rm -f mp_*.jpg mp_*.txt")
    (in-package "USER")
    t))
(export '(convert-webots-vrml->openhrp3-vrml-for-darwin))
(warn "(convert-webots-vrml->openhrp3-vrml-for-darwin)~%")

;; Difference between Webots VRML <-> OpenHRP3 VRML
;;   1. WAIST joint exist only in OpenHRP3 VRML
;;      -> Append WAIST joint to Webots VRML in (append-waist-joint) function
;;   2. Segments exist only in OpenHRP3 VRML
;;      -> Enclose Webot's Shape nodes with Segment in (insert-segment) function
;;   3. Servo (Webots) -> Joint (OpenHRP3)
;;      -> Rename Servo -> Joint and define jointType, jointAxis, and jointId in (convert-servo->joint) function
;;         Joint axis definition ;; webots -> (almost) all joints asymmetorical ;; OpenHRP3 and euslisp -> pitch is ssymnetrical
;;   4. Difference in PROTO
;;      -> Convert Accelerometer -> Accelerationsenso
;;         Convert Gyro2 -> Gyro
;;         Convert Camera -> VisionSensor
;;         Convert Led -> VisionSensor (<=correct??)
;;   5. Humanoid node exist only in OpenHRP3 VRML
;;      -> Append Humanoid in (append-humanoid) function
(defun append-openhrp3-waist-joint
  (rb-lis &optional (waist-translation (list 0 0 0.33951)) (waist-rotation (list 0.57735 0.57735 0.57735 2.09439)))
  (list
   (append
    (list 'def 'waist 'servo)
    (append
     (list
      (append (list 'translation) waist-translation)
      (append (list 'rotation) waist-rotation))
     (remove-if-not
      #'(lambda (x)
	  (and
	   (memq (car x)
		 (mapcar 'car (cdr (assoc 'field (send 'servo :get :vrml-node-spec)))))
	   (not (eq (car x) 'rotation))
	   (not (eq (car x) 'translation))
	   (not (eq (car x) 'name))))
      (cdar rb-lis)
      )))))
;; (append-openhrp3-waist-joint (split-nodes c3 nil))

(defun insert-segment (rb-lis &optional (inertia-maps (darwin-inertia-lists)))
  (let ((ret (copy-object rb-lis)))
    (labels ((find-servo
	      (l)
	      (let ((sv-l
		     (remove-if-not #'(lambda (x) (and (> (length x) 2) (eq (elt x 2) 'servo))) l)))
		(when sv-l
		  (dolist (sv sv-l)
		    (dolist (sym '(shape group))
		      (let ((sp0 (cadr (assoc 'children sv))))
			(dotimes (i (length sp0))
			  (when (and (> (length (elt sp0 i)) 2) (eq (elt (elt sp0 i) 2) sym))
			    (let ((segment-name
				   ;; naming rule of segment node
				   (read-from-string (format nil "~A_link"
							     (string-right-trim "shape"  (format nil "~A" (elt (elt sp0 i) 1)))))))
			      (setf (elt sp0 i)
				    (append
				     (list 'def segment-name 'segment)
				     (list (list 'children (list (list 'aref) (copy-object (elt sp0 i)))))
				     (list (list 'mass (cadr (assoc 'mass (cadr (assoc 'physics sv))))))
				     (list (append (list 'centerOfMass) (cdr (assoc 'centerofmass (cadr (assoc 'physics sv))))))
				     (list (append (list 'momentsOfInertia)
						   (list (append (list 'aref)
								 (concatenate cons (array-entity
										    (car (elt (find-if #'(lambda (x) (memq segment-name (elt x 1))) inertia-maps) 2))))))))
				     )))))))
		    (find-servo (cadr (assoc 'children sv)))))
		)))
      (find-servo ret)
      ret)))

(defun darwin-inertia-lists (&optional (force-update nil))
  (if force-update (unix:system "rm -f DARwIn-OP_Dynamics.zip"))
  (unix:system "if [ ! -e DARwIn-OP_Dynamics.zip ]; then wget http://sourceforge.net/projects/darwinop/files/Hardware/Mechanics/Physical%20Information/DARwIn-OP_Dynamics.zip;fi")
  (unix:system "if [ ! -e mp_ankle_l.jpg ] ;then unzip DARwIn-OP_Dynamics.zip;fi")
  (let ((ff (remove-if-not #'(lambda (x) (and (substringp "mp" x) (substringp "txt" x)))
			   (directory ".")))
	(file-names<->webots-segment-name-map
	 '((mp_body.txt (DBODY_LINK) ())
	   (mp_neck.txt (DNECK_LINK) ())
	   (mp_head.txt (DHEAD_LINK) ())
	   (mp_pelv_y_l.txt (DPELVYL_LINK DPELVYR_LINK) ())
	   (mp_pelv_l.txt (DPELVL_LINK DPELVR_LINK) ())
	   (mp_thigh_l.txt (DLEGUPPERL_LINK DLEGUPPERR_LINK) ())
	   (mp_tibia_l.txt (DLEGLOWERL_LINK DLEGLOWERR_LINK) ())
	   (mp_ankle_l.txt (DANKLEL_LINK DANKLER_LINK) ())
	   (mp_foot_l.txt (DFOOTL_LINK DFOOTR_LINK) ())
	   (mp_shoulder_l.txt (DSHOULDERL_LINK DSHOULDERR_LINK) ())
	   (mp_arm_upper_l.txt (DARMUPPERL_LINK DARMUPPERR_LINK) ())
	   (mp_arm_lower_l.txt (DARMLOWERL_LINK DARMLOWERR_LINK) ())
	   )))
    (dolist (f ff)
      (with-open-file
       (fstrm f :direction :input)
       (let ((str))
	 (while (and (setq str (read-line fstrm nil nil))
		     (not (substringp "INERTIA TENSOR" str))))
	 (let ((mat
		(make-matrix
		 3 3
		 (mapcar #'(lambda (trim-str)
			     (let ((str
				    (make-string-input-stream
				     (string-left-trim trim-str (read-line fstrm nil nil)))))
			       (float-vector (read str nil nil) (read str nil nil) (read str nil nil))
			       ))
			 (list "Ixx Ixy Ixz " "Iyx Iyy Iyz " "Izx Izy Izz ")))))
	   (setf (elt (assoc (read-from-string f) file-names<->webots-segment-name-map) 2)
		 (list (scale-matrix 1e-9 mat))) ;; [g mm^2] -> [kg m^2]
	   )))
      )
    file-names<->webots-segment-name-map))

(defun append-openhrp3-humanoid (rb-lis robot-name)
  (list
   (append
    (list 'def (string-upcase (read-from-string robot-name)) 'humanoid)
    (list
     (list 'name robot-name)
     (list 'humanoidbody (append (list (list 'aref)) rb-lis))
     (list 'joints
	   (apply #'append (list 'aref) (mapcar #'(lambda (x) (list 'USE x '|?|)) (gather-all-objecet rb-lis 'joint))))
     (list 'segments
	   (apply #'append (list 'aref) (mapcar #'(lambda (x) (list 'USE x '|?|)) (gather-all-objecet rb-lis 'segment))))
     )
    )
   )
  )

(defun replace-all-objects-for-Webots->OpenHRP3 (rb-lis)
  (replace-Camera->VisionSensor
   (replace-Led->VisionSensor
    (replace-gyro2->gyro
     (replace-Accelerometer->AccelerationSensor
      (replace-servo->joint rb-lis))))))

(defun find-vrml-node-from-node-list (lis node-name)
  (remove-if-not #'(lambda (x)
		     (cond
		      ((eq (elt x 0) 'def) (eq (elt x 2) node-name))
		      (t (eq (elt x 0) node-name))))
		 lis))

(defun replace-servo->joint (rb-lis)
  (let ((node-fields (mapcar 'car (cdr (assoc 'field (send 'joint :get :vrml-node-spec)))))
	(default-roation-value (caddr (cadr (assoc 'field (get 'servo :vrml-node-spec)))))
	(jointid -1))
    (labels ((find-servo
	      (l)
	      (let ((sv-l (find-vrml-node-from-node-list l 'servo)))
		(when sv-l
		  (dolist (sv sv-l)
		    (delete-if #'(lambda (x) (not (memq (car x) node-fields)))
			       (cdddr sv))
		    (push (if (= jointid -1)
			      (list 'jointType "free")
			    (list 'jointType "rotate"))
			  (cdddr sv))
		    (push (append (list 'jointaxis)
				  (cond
				   ((and (assoc 'rotation sv) (/= jointid -1))
				    (mapcar #'abs (subseq (assoc 'rotation sv) 1 4)) ;; OpenHRP3, eus
				    ;;(subseq (assoc 'rotation sv) 1 4) ;; default
				    )
				   (t (subseq default-roation-value 0 3))))
			  (cdddr sv))
		    (push (list 'jointid jointid) (cdddr sv))
		    (incf jointid)
		    (setf (elt sv 2) 'joint)
		    (find-servo (cadr (assoc 'children sv)))
		    )))))
      (find-servo rb-lis)
      rb-lis)))

(defun replace-webots-node->openhrp3-node
  (rb-lis webots-node-name openhrp-node-name
	  &optional (new-node-name) (add-sensor-id t))
  (let ((node-fields (mapcar 'car (cdr (assoc 'field (send openhrp-node-name :get :vrml-node-spec)))))
	(sensorid 0))
    (labels ((find-servo
	      (l)
	      (let* ((sv-l (find-vrml-node-from-node-list l 'joint)))
		(dolist (sv sv-l)
		  (let ((sv2-l (find-vrml-node-from-node-list (cadr (assoc 'children sv)) webots-node-name)))
		    (dolist (sv2 sv2-l)
		      ;; update node's fields
		      (let ((tmp-node (if (eq (elt sv2 0) 'def) (cddr sv2) sv2))) ;; tmp-node -> '(node-name (field1 value1) (field2 value2) ..)
			(delete-if #'(lambda (x) (not (memq (car x) node-fields)))
				   (cdr tmp-node))
			(when add-sensor-id
			  (push (list 'sensorid sensorid) (cdr tmp-node))
			  (incf sensorid)))
		      ;; update node's names
		      (if (eq (elt sv2 0) 'def)
			  (setf (elt sv2 2) openhrp-node-name)
			(progn
			  (setf (elt sv2 0) 'def)
			  (push openhrp-node-name (cdr sv2))
			  (push (read-from-string (format nil "~A~A" (if new-node-name new-node-name openhrp-node-name) sensorid)) (cdr sv2))
			  )))
		    (find-servo (cadr (assoc 'children sv)))
		    )))))
      (find-servo rb-lis)
    rb-lis)))

(defun replace-Accelerometer->AccelerationSensor (rb-lis)
  (replace-webots-node->openhrp3-node
   rb-lis
   'accelerometer
   'accelerationsensor
   'gsensor
   ))

(defun replace-Gyro2->Gyro (rb-lis)
  (replace-webots-node->openhrp3-node
   rb-lis
   'gyro2
   'gyro
   'gyrometer
   ))

(defun replace-Led->VisionSensor (rb-lis)
  (replace-webots-node->openhrp3-node
   rb-lis
   'Led
   'visionsensor
   nil
   nil
   ))

(defun replace-Camera->VisionSensor (rb-lis)
  (replace-webots-node->openhrp3-node
   rb-lis
   'camera
   'visionsensor
   'camera
   ))

(defun gather-all-objecet (rb-lis sym)
  (let ((ret))
    (labels ((_gather
	      (l)
	      (let ((l2 (memq 'def l)))
		(when l2
		  (if (eq (elt l2 2) sym)
		      (push (cadr l2) ret))
		  (_gather (cdddr l2)))
		)))
      (_gather (flatten rb-lis))
      (reverse ret))))
;; (pprint (append-openhrp3-humanoid (replace-servo->joint (insert-segment (append-openhrp3-waist-joint (split-nodes c3 nil))))))
;; (append (list '(aref)) (append-openhrp3-humanoid (replace-servo->joint (insert-segment (append-openhrp3-waist-joint (split-nodes c3 nil))))))
;; (print-all-vrml-nodes-fields "hoge.wrl" (append (list '(aref)) (append-openhrp3-humanoid (replace-servo->joint (insert-segment (append-openhrp3-waist-joint (split-nodes c3 nil)))))))
;; (print-all-vrml-nodes-fields "hoge.wrl" (append (list '(aref)) (append-openhrp3-humanoid (replace-servo->joint (insert-segment (append-openhrp3-waist-joint (split-nodes (append (list 'aref) cc) nil)))))))

(in-package "USER")
(import '(vrml::convert-webots-vrml->openhrp3-vrml-for-darwin))