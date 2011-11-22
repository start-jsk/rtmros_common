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
;;      [
;;        DEF AAA BB (
;;          cc [1.0? 2.0]
;;        )
;;      ]
;;   ->
;;   3. read s expression
;;      '(aref DEF AA BB (CC (aref 1.0? 2.0)))
(defun read-fixed-vrml-file (vrml-fname)
  (let ((fixed-vrml-fname (format nil "/tmp/~A.readvrmltmp" (pathname-name vrml-fname))) (ret) (str))
    (unix:system (format nil "cp ~A ~A" vrml-fname fixed-vrml-fname))
    (sed-vrml-file-using-s-expression fixed-vrml-fname)
    (with-open-file
     (f fixed-vrml-fname :direction :input)
     (while (setq str (read f nil nil)) (push str ret)))
    (unix:system (format nil "rm -f ~A" fixed-vrml-fname))
    (reverse ret)))

;; Define vrml node spec from read vrml data
;;   'data' must be '(proto aa (field-definition-for-aa) (implementation-for-aa) proto bb (field-definition-for-bb) (implementation-for-bb) ...)
(defun define-vrml-node-spec-from-read-vrml-data (data)
  (unless data (return-from define-vrml-node-spec-from-read-vrml-data nil))
  (when (eq (car data) 'proto)
    (let ((proto-name (cadr data))
	  (field-defs (cdr (caddr data)))
	  (impl-defs (cadr (cddr data))))
      ;; (print (list 'proto proto-name))
      ;; (print (list 'field field-defs))
      ;; (print (list 'impl impl-defs))
      (eval `(define-vrml-node-spec ,proto-name
	       nil
	       ,@(store-vrml-fields field-defs)))
      (define-vrml-node-spec-from-read-vrml-data (cddddr data))
      )))

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
  (field SFRotation rotation 0)
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
;;   1. Neglect lines with #
;;   2. All tempolary files are named as *.bak and tmp.wrl
;;;;;;;;;;;;;;;;;;;;;;;;
(defun get-vrml-convert-script-dir ()
  (let* ((strm (piped-fork "rospack find hrpsys"))
	 (str (format nil "~A/scripts" (read-line strm nil nil))))
    (close strm)
    str))

(defun preprocess-for-vrml-convert (robot-vrml-fname)
  ;; define OpenHRP3 PROTOs
  (define-vrml-node-spec-from-read-vrml-data (read-fixed-vrml-file (format nil "~A/openhrp3_protos.wrl" (get-vrml-convert-script-dir))))
  (define-vrml-node-spec-from-read-vrml-data (read-fixed-vrml-file robot-vrml-fname))
  (def-all-print-name)
  t)

(defun preprocess-for-vrml-convert-for-darwin ()
  (preprocess-for-vrml-convert "$(rospack find webots)/build/webots/projects/robots/darwin-op/protos/DARwInOP.proto")
  ;; rename webots Gyro -> Gyro2 because the name 'Gyro' is duplicate name
  (unix:system "cp $(rospack find webots)/build/webots/projects/robots/darwin-op/protos/DARwInOP.proto /tmp/DARwInOP.proto")
  (unix:system "sed -i -e s/Gyro/Gyro2/g /tmp/DARwInOP.proto")
  t)

(defun post-process-for-vrml-convert (robot-vrml-fname)
  (sed-s-expression-file-using-vrml robot-vrml-fname)
  ;; concatenate OpenHRP3 PROTO definition
  (unix:system (format nil "cat ~A/openhrp3_protos.wrl ~A > /tmp/~A && mv /tmp/~A ~A"
		       (get-vrml-convert-script-dir)
		       robot-vrml-fname (pathname-name robot-vrml-fname) (pathname-name robot-vrml-fname) robot-vrml-fname))
  )

;; vrml -> fixed vrml
(defun sed-vrml-file-using-s-expression (fname)
  (unix:system (format nil "sed -i -e s/\{/\\(/g ~A" fname))
  (unix:system (format nil "sed -i -e s/\}/\\)/g ~A" fname))
  (unix:system (format nil "sed -i -e s/\,/\?/g ~A" fname))
  ;; remove lines with #
  (unix:system (format nil "grep -v \"\#\" ~A > /tmp/tmp.wrl && cp /tmp/tmp.wrl ~A && rm -f /tmp/tmp.wrl" fname fname))
  )

;; fixed vrml -> vrml
(defun sed-s-expression-file-using-vrml (fname)
  (unix:system (format nil "sed -i -e s/\\(/\{/g ~A" fname))
  (unix:system (format nil "sed -i -e s/\\)/\}/g ~A" fname))
  (unix:system (format nil "sed -i -e s/\?/\,/g ~A" fname))
  )

;;;;;;;;;;;;;;;;;;;;;;;;
;; Printer for splitted s-expression vrml data
;;;;;;;;;;;;;;;;;;;;;;;;
(defun print-all-vrml-nodes-fields (fname splitted-vrml-data)
  (with-open-file
      (fstrm fname :direction :output)
      (print-vrml-nodes fstrm splitted-vrml-data))
  )

(defun gen-indent-string-from-size (size &optional (default-width 2))
  (let ((ret ""))
    (dotimes (i (* default-width size))
      (setq ret (format nil " ~A" ret)))
    ret))

(defun print-vrml-nodes (fstrm lis &optional (indent-size 0))
  (cond
   ((and (consp (car lis)) (eq (caar lis) 'aref))
    (format fstrm "~A[~%" (gen-indent-string-from-size indent-size))
    (mapcar #'(lambda (x)
		(print-vrml-nodes fstrm x indent-size))
	    (cdr lis))
    (format fstrm "~A]~%" (gen-indent-string-from-size indent-size)))
   ((consp (car lis))
    (mapcar #'(lambda (x)
		(print-vrml-nodes fstrm x indent-size))
	    lis))
   ((eq (car lis) 'def)
    (format fstrm "DEF ~A ~A {~%"
	    (if (send (caddr lis) :get :print-field-varname-upcase)
		(string-upcase (cadr lis))
	      (cadr lis))
	    (send (caddr lis) :get :print-object-string))
    (print-vrml-fields fstrm (cdddr lis) (caddr lis) (+ indent-size 2))
    (format fstrm "~A}~%" (gen-indent-string-from-size indent-size)))
   (t
    (format fstrm "~A {~%" (send (car lis) :get :print-object-string))
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
	  (format fstrm "~A]~%" (gen-indent-string-from-size indent-size)))
	 ((consp (car (cdr ll)))
	  (print-vrml-nodes fstrm (car (cdr ll)) (+ indent-size 2)))
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
  (let ((webots-vrml-fname "DARwInOP.proto"))
    (in-package "VRML") ;; execute all conversion in "VRML" package because of using vrml:: grobal variables in vrmlNodeSpec.l
    (preprocess-for-vrml-convert-for-darwin)
    (print-all-vrml-nodes-fields
     openhrp3-vrml-fname
     (append-openhrp3-humanoid
      (replace-all-objects-for-Webots->OpenHRP3
       (insert-segment
	(append-openhrp3-waist-joint
	 (split-nodes
	  (cadddr (read-fixed-vrml-file (format nil "/tmp/~A" webots-vrml-fname)))
	  nil) ;; debug print nil
	 (list 0 0 0.33951) (list 0.57735 0.57735 0.57735 2.09439) ;; WAIST translation and rotation
	 )
	(darwin-inertia-lists) ;; inertia-maps for insert-segment
	))
      "DARWIN"))
    (post-process-for-vrml-convert openhrp3-vrml-fname)
    (unix:system "rm -f mp_*.jpg mp_*.txt DARwIn-OP_Dynamics.zip")
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
  (unix:system "if [ ! -e mp_*.* ] ;then unzip DARwIn-OP_Dynamics.zip;fi")
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
	   (apply #'append (list 'aref) (mapcar #'(lambda (x) (list 'USE (read-from-string (format nil "~A\?" x)))) (gather-all-objecet rb-lis 'joint))))
     (list 'segments
	   (apply #'append (list 'aref) (mapcar #'(lambda (x) (list 'USE (read-from-string (format nil "~A\?" x)))) (gather-all-objecet rb-lis 'segment))))
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

(defun replace-servo->joint (rb-lis)
  (let ((node-fields (mapcar 'car (cdr (assoc 'field (send 'joint :get :vrml-node-spec)))))
	(jointid -1))
    (labels ((find-servo
	      (l)
	      (let ((sv-l
		     (remove-if-not #'(lambda (x) (and (> (length x) 2) (eq (elt x 2) 'servo))) l)))
		(when sv-l
		  (dolist (sv sv-l)
		    ;; here is only for DARwInOP.proto !! -->
		    ;; origin of elbow joints in Webots model are incorrect??
		    (if (memq (elt sv 1) '(darmlowerl darmlowerr))
			(delete-if #'(lambda (x) (eq (car x) 'rotation))
				   (cdddr sv)))
		    ;; <-- here is only for DARwInOP.proto !!
		    (delete-if #'(lambda (x) (not (memq (car x) node-fields)))
			       (cdddr sv))
		    (push (if (= jointid -1)
			      (list 'jointType "free")
			    (list 'jointType "rotate"))
			  (cdddr sv))
		    (push (append (list 'jointaxis)
				  (if (and (assoc 'rotation sv) (/= jointid -1))
				      (mapcar #'abs (subseq (assoc 'rotation sv) 1 4))
				    (list 1 0 0))) (cdddr sv))
		    (push (list 'jointid jointid) (cdddr sv))
		    (incf jointid)
		    (setf (elt sv 2) 'joint)
		    (find-servo (cadr (assoc 'children sv)))
		    )))))
      (find-servo rb-lis)
      rb-lis)))

(defun replace-Accelerometer->AccelerationSensor (rb-lis)
  (let ((node-fields (mapcar 'car (cdr (assoc 'field (send 'accelerationsensor :get :vrml-node-spec))))))
    (labels ((find-servo
	      (l)
	      (let* ((sv
		      (find-if #'(lambda (x) (and (> (length x) 2) (eq (elt x 2) 'joint))) l))
		     (sv2 (find-if #'(lambda (x) (eq (elt x 0) 'accelerometer)) (cadr (assoc 'children sv)))))
		(when sv2
		  (delete-if #'(lambda (x) (not (memq (car x) node-fields)))
			     (cdr sv2))
		  (setf (elt sv2 0) 'accelerationsensor)
		  (push (list 'sensorid 0) (cdr sv2))
		  (find-servo (cadr (assoc 'children sv2)))
		  ))))
      (find-servo rb-lis)
    rb-lis)))

(defun replace-Gyro2->Gyro (rb-lis)
  (let ((node-fields (mapcar 'car (cdr (assoc 'field (send 'gyro :get :vrml-node-spec))))))
    (labels ((find-servo
	      (l)
	      (let* ((sv
		      (find-if #'(lambda (x) (and (> (length x) 2) (eq (elt x 2) 'joint))) l))
		     (sv2 (find-if #'(lambda (x) (eq (elt x 0) 'gyro2)) (cadr (assoc 'children sv)))))
		(when sv2
		  (delete-if #'(lambda (x) (not (memq (car x) node-fields)))
			     (cdr sv2))
		  (setf (elt sv2 0) 'gyro)
		  (push (list 'sensorid 0) (cdr sv2))
		  (find-servo (cadr (assoc 'children sv2)))
		  ))))
      (find-servo rb-lis)
    rb-lis)))

(defun replace-Led->VisionSensor (rb-lis)
  (let ((node-fields (mapcar 'car (cdr (assoc 'field (send 'visionsensor :get :vrml-node-spec))))))
    (labels ((find-servo
	      (l)
	      (let* ((sv
		      (find-if #'(lambda (x) (and (> (length x) 2) (eq (elt x 2) 'joint))) l))
		     (sv2 (remove-if-not #'(lambda (x) (and (> (length x) 2) (eq (elt x 2) 'Led))) (cadr (assoc 'children sv)))))
		(when sv
		  (when sv2
		    (dolist (sv3 sv2)
		      (delete-if #'(lambda (x) (not (memq (car x) node-fields)))
				 (cdddr sv3))
		      (setf (elt sv3 2) 'visionsensor)))
		  (find-servo (cadr (assoc 'children sv))))
		)))
      (find-servo rb-lis)
    rb-lis)))

(defun replace-Camera->VisionSensor (rb-lis)
  (let ((node-fields (mapcar 'car (cdr (assoc 'field (send 'visionsensor :get :vrml-node-spec))))))
    (labels ((find-servo
	      (l)
	      (let* ((sv
		      (find-if #'(lambda (x) (and (> (length x) 2) (eq (elt x 2) 'joint))) l))
		     (sv2 (remove-if-not #'(lambda (x) (eq (elt x 0) 'Camera)) (cadr (assoc 'children sv)))))
		(when sv
		  (when sv2
		    (dolist (sv3 sv2)
		      (delete-if #'(lambda (x) (not (memq (car x) node-fields)))
				 (cdr sv3))
		      (push (list 'sensorid 0) (cdr sv3))
		      (setf (elt sv3 0) 'visionsensor)))
		  (find-servo (cadr (assoc 'children sv))))
		)))
      (find-servo rb-lis)
    rb-lis)))

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

;; tempolary
(defun vrml () (in-package "VRML"))


;;;;;;;;;;;;;;;;
;; for test
;;;;;;;;;;;;;;;;
(when (probe-file "openhrp3_protos.wrl.bak")
(setq data
      (with-open-file
          (f "openhrp3_protos.wrl.bak" :direction :input)
        (let (str ret)
          (while (setq str (read f nil nil)) (push str ret))
          (reverse ret)))))

(when (probe-file "DARWINmain.wrl")
(setq data2
      (with-open-file
          (f "DARWINmain.wrl" :direction :input)
        (let (str ret)
          (while (setq str (read f nil nil)) (push str ret))
          (reverse ret)))))

;; (setq bb (with-open-file
;;           (f "DARwInOP.proto" :direction :input)
;; 	  (let (str ret)
;;           (while (setq str (read f nil nil)) (push str ret))
;;           (reverse ret))))
;;(setq cc (cadddr bb))
(setq c2 (append (list 'aref) '(robot (translation 0 0 0 rotation 0 1 0 0 children (aref ) name "DARwIn-OP" model "DARwIn-OP" boundingobject group (children (aref transform (translation 0 -0.059 -0.025 children (aref box (size 0.1 0.058 0.091))) transform (translation 0 -0.029 -0.038 children (aref box (size 0.126 0.045 0.076))) transform (translation 0 -0.006 -0.027 rotation 1 0 0 0.1 children (aref box (size 0.114 0.058 0.076))) transform (translation 0 -0.042 0.028 children (aref box (size 0.114 0.046 0.016))))) name darwin-op model "DARwIn-OP" description "Robotis' DARwIn-OP modeled by Fabien Rohrer" physics physics (density -1 mass 0.975599 centerofmass -0.003116 -0.039444 -0.019663) controller void controllerargs voidargs))))
(setq c3 '(aref robot (translation 0 0 0 rotation 0 1 0 0 children (aref def dbodyshape shape (appearance appearance (material def dplasticmaterial material (diffusecolor 0.301961 0.301961 0.301961 specularcolor 0.401201 0.401201 0.401201)) geometry indexedfaceset (coord coordinate (point (aref )) coordindex (aref ) creaseangle 0.2)) accelerometer (translation 0 -0.068406 -0.01 rotation 1 0 0 1.5708 name "Accelerometer" lookuptable (aref -39.24 0 0 39.24 1024 0)) gyro2 (translation 0 -0.068406 0.01 rotation -0.57735 0.57735 0.57735 2.0944 name "Gyro" lookuptable (aref -27.925 0 0 27.925 1024 0)) def dneck servo (translation 0 0.051 0 rotation 0 -1 0 0 children (aref def dneckshape shape (appearance appearance (material def dmetalmaterial material (diffusecolor 0.7 0.7 0.7 specularcolor 0.401201 0.401201 0.401201)) geometry indexedfaceset (coord coordinate (point (aref )) coordindex (aref ) creaseangle 0.2)) def dhead servo (rotation 1 0 0 0 children (aref def dheadshape group (children (aref shape (appearance appearance (material use dplasticmaterial) geometry indexedfaceset (coord def dheadcoord coordinate (point (aref )) coordindex (aref ) creaseangle 0.52)) shape (appearance appearance (material def deyematerial material (diffusecolor 0.14 0.14 0.14 specularcolor 0.401201 0.401201 0.401201)) geometry indexedfaceset (coord use dheadcoord coordindex (aref ) creaseangle 0.52)))) camera (translation 0 0.032907 0.035982 rotation 0 -1 0 3.14159 children (aref def dcamerashape shape (appearance appearance (material def dcameramaterial material (diffusecolor 0 0 0 specularcolor 0.401201 0.401201 0.401201)) geometry indexedfaceset (coord coordinate (point (aref )) coordindex (aref ) creaseangle 0.2))) name "Camera" windowposition 0 0 width 160 height 120 pixelsize 1.0size) def deyeled led (rotation 1 0 0 0 children (aref def deyeledshape shape (appearance appearance (material def dledmaterial material (ambientintensity 0 diffusecolor 0.7 0.7 0.7)) geometry indexedfaceset (coord coordinate (point (aref )) coordindex (aref ) creaseangle 0.2))) name "EyeLed" color (aref) gradual true) def dheadled led (rotation 1 0 0 0 children (aref def dheadledshape shape (appearance appearance (material use dledmaterial) geometry indexedfaceset (coord coordinate (point (aref )) coordindex (aref ) creaseangle 0.2))) name "HeadLed" color (aref 0 1 0))) name "Head" boundingobject transform (translation 0 0.022 -0.012 rotation 1 0 0 1.5708 children (aref capsule (height 0.018 radius 0.042))) physics physics (density -1 mass 0.158042 centerofmass 6.391978e-05 0.018564 0.007667) maxforce 2.35)) name "Neck" boundingobject transform (translation 0 -0.009 0 children (aref box (size 0.044 0.043 0.022))) physics physics (density -1 mass 0.024358 centerofmass 0.001424 -0.016568 -0.000713) maxforce 2.35) ) name "DARwIn-OP" model "DARwIn-OP" boundingobject group (children (aref transform (translation 0 -0.059 -0.025 children (aref box (size 0.1 0.058 0.091))) transform (translation 0 -0.029 -0.038 children (aref box (size 0.126 0.045 0.076))) transform (translation 0 -0.006 -0.027 rotation 1 0 0 0.1 children (aref box (size 0.114 0.058 0.076))) transform (translation 0 -0.042 0.028 children (aref box (size 0.114 0.046 0.016))))) name darwin-op model "DARwIn-OP" description "Robotis' DARwIn-OP modeled by Fabien Rohrer" physics physics (density -1 mass 0.975599 centerofmass -0.003116 -0.039444 -0.019663) controller void controllerargs voidargs)))
(setq dd '(aref def dbodyshape shape (appearance appearance (material def dplasticmaterial material (diffusecolor is plasticcolor specularcolor 0.401201 0.401201 0.401201)) geometry indexedfaceset (coord coordinate (point (aref )) coordindex (aref ) creaseangle 0.2)) accelerometer (translation 0 -0.068406 -0.01 rotation 1 0 0 1.5708 name "Accelerometer" lookuptable (aref -39.24 0 0 39.24 1024 0)) gyro2 (translation 0 -0.068406 0.01 rotation -0.57735 0.57735 0.57735 2.0944 name "Gyro" lookuptable (aref -27.925 0 0 27.925 1024 0)) def dneck servo (translation 0 0.051 0 rotation 0 -1 0 0 children (aref def dneckshape shape (appearance appearance (material def dmetalmaterial material (diffusecolor 0.7 0.7 0.7 specularcolor 0.401201 0.401201 0.401201)) geometry indexedfaceset (coord coordinate (point (aref )) coordindex (aref ) creaseangle 0.2)) def dhead servo (rotation 1 0 0 0 children (aref def dheadshape group (children (aref shape (appearance appearance (material use dplasticmaterial) geometry indexedfaceset (coord def dheadcoord coordinate (point (aref )) coordindex (aref ) creaseangle 0.52)) shape (appearance appearance (material def deyematerial material (diffusecolor 0.14 0.14 0.14 specularcolor 0.401201 0.401201 0.401201)) geometry indexedfaceset (coord use dheadcoord coordindex (aref ) creaseangle 0.52)))) camera (translation 0 0.032907 0.035982 rotation 0 -1 0 3.14159 children (aref def dcamerashape shape (appearance appearance (material def dcameramaterial material (diffusecolor 0 0 0 specularcolor 0.401201 0.401201 0.401201)) geometry indexedfaceset (coord coordinate (point (aref )) coordindex (aref ) creaseangle 0.2))) name "Camera" windowposition is camera_windowposition width is camera_width height is camera_height pixelsize is camera_pixelsize) def deyeled led (rotation 1 0 0 0 children (aref def deyeledshape shape (appearance appearance (material def dledmaterial material (ambientintensity 0 diffusecolor 0.7 0.7 0.7)) geometry indexedfaceset (coord coordinate (point (aref )) coordindex (aref ) creaseangle 0.2))) name "EyeLed" color (aref) gradual true) def dheadled led (rotation 1 0 0 0 children (aref def dheadledshape shape (appearance appearance (material use dledmaterial) geometry indexedfaceset (coord coordinate (point (aref )) coordindex (aref ) creaseangle 0.2))) name "HeadLed" color (aref 0 1 0))) name "Head" boundingobject transform (translation 0 0.022 -0.012 rotation 1 0 0 1.5708 children (aref capsule (height 0.018 radius 0.042))) physics physics (density -1 mass 0.158042 centerofmass 6.391978e-05 0.018564 0.007667) maxforce 2.35)) name "Neck" boundingobject transform (translation 0 -0.009 0 children (aref box (size 0.044 0.043 0.022))) physics physics (density -1 mass 0.024358 centerofmass 0.001424 -0.016568 -0.000713) maxforce 2.35)))
(setq ee '(appearance appearance (material def dplasticmaterial material (diffusecolor is plasticcolor specularcolor 0.401201 0.401201 0.401201)) geometry indexedfaceset (coord coordinate (point (aref)) coordindex (aref) creaseangle 0.2)))
(setq c4 '(robot (translation 0 0 0 rotation 0 1 0 0 children (aref def dbodyshape shape (appearance appearance (material def dplasticmaterial material (diffusecolor 0.301961 0.301961 0.301961 specularcolor 0.401201 0.401201 0.401201)) geometry indexedfaceset (coord coordinate (point (aref )) coordindex (aref ) creaseangle 0.2)) accelerometer (translation 0 -0.068406 -0.01 rotation 1 0 0 1.5708 name "Accelerometer" lookuptable (aref -39.24 0 0 39.24 1024 0)) gyro2 (translation 0 -0.068406 0.01 rotation -0.57735 0.57735 0.57735 2.0944 name "Gyro" lookuptable (aref -27.925 0 0 27.925 1024 0)) def dneck servo (translation 0 0.051 0 rotation 0 -1 0 0 children (aref def dneckshape shape (appearance appearance (material def dmetalmaterial material (diffusecolor 0.7 0.7 0.7 specularcolor 0.401201 0.401201 0.401201)) geometry indexedfaceset (coord coordinate (point (aref )) coordindex (aref ) creaseangle 0.2)) def dhead servo (rotation 1 0 0 0 children (aref def dheadshape group (children (aref shape (appearance appearance (material use dplasticmaterial) geometry indexedfaceset (coord def dheadcoord coordinate (point (aref )) coordindex (aref ) creaseangle 0.52)) shape (appearance appearance (material def deyematerial material (diffusecolor 0.14 0.14 0.14 specularcolor 0.401201 0.401201 0.401201)) geometry indexedfaceset (coord use dheadcoord coordindex (aref ) creaseangle 0.52)))) camera (translation 0 0.032907 0.035982 rotation 0 -1 0 3.14159 children (aref def dcamerashape shape (appearance appearance (material def dcameramaterial material (diffusecolor 0 0 0 specularcolor 0.401201 0.401201 0.401201)) geometry indexedfaceset (coord coordinate (point (aref )) coordindex (aref ) creaseangle 0.2))) name "Camera" windowposition 0 0 width 160 height 120 pixelsize 1.0size) def deyeled led (rotation 1 0 0 0 children (aref def deyeledshape shape (appearance appearance (material def dledmaterial material (ambientintensity 0 diffusecolor 0.7 0.7 0.7)) geometry indexedfaceset (coord coordinate (point (aref )) coordindex (aref ) creaseangle 0.2))) name "EyeLed" color (aref) gradual true) def dheadled led (rotation 1 0 0 0 children (aref def dheadledshape shape (appearance appearance (material use dledmaterial) geometry indexedfaceset (coord coordinate (point (aref )) coordindex (aref ) creaseangle 0.2))) name "HeadLed" color (aref 0 1 0))) name "Head" boundingobject transform (translation 0 0.022 -0.012 rotation 1 0 0 1.5708 children (aref capsule (height 0.018 radius 0.042))) physics physics (density -1 mass 0.158042 centerofmass 6.391978e-05 0.018564 0.007667) maxforce 2.35)) name "Neck" boundingobject transform (translation 0 -0.009 0 children (aref box (size 0.044 0.043 0.022))) physics physics (density -1 mass 0.024358 centerofmass 0.001424 -0.016568 -0.000713) maxforce 2.35) def dpelvyl servo (translation 0.037 -0.1222 -0.005 children (aref def dpelvylshape shape (appearance appearance (material use dmetalmaterial) geometry indexedfaceset (coord coordinate (point (aref )) coordindex (aref ) creaseangle 0.2)) def dpelvl servo (rotation 0 0 -1 0 children (aref def dpelvlshape shape (appearance appearance (material def dservomaterial material (diffusecolor 0.14 0.14 0.14 specularcolor 0.168737 0.168737 0.168737)) geometry indexedfaceset (coord coordinate (point (aref )) coordindex (aref ) creaseangle 0.2)) def dlegupperl servo (rotation -1 0 0 0 children (aref def dlegupperlshape group (children (aref shape (appearance appearance (material use dplasticmaterial) geometry indexedfaceset (coord def dlegupperlcoord coordinate (point (aref )) coordindex (aref ) creaseangle 0.2)) shape (appearance appearance (material use dservomaterial) geometry indexedfaceset (coord use dlegupperlcoord coordindex (aref ) creaseangle 0.2)))) def dleglowerl servo (translation 0 -0.093 0 rotation -1 0 0 0 children (aref def dleglowerlshape shape (appearance appearance (material use dplasticmaterial) geometry indexedfaceset (coord def dleglowerlcoord coordinate (point (aref )) coordindex (aref ) creaseangle 0.2)) def danklel servo (translation 0 -0.093 0 rotation 1 0 0 0 children (aref def danklelshape shape (appearance appearance (material use dservomaterial) geometry indexedfaceset (coord def dcoord_anklel coordinate (point (aref )) coordindex (aref ) creaseangle 0.2)) def dfootl servo (rotation 0 0 1 0 children (aref def dfootlshape group (children (aref shape (appearance appearance (material use dplasticmaterial) geometry indexedfaceset (coord def dfootlcoord coordinate (point (aref )) coordindex (aref ) creaseangle 0.2)) shape (appearance appearance (material use dmetalmaterial) geometry indexedfaceset (coord use dfootlcoord coordindex (aref ) creaseangle 0.2))))) name "FootL" boundingobject transform (translation 0.011 -0.026 -0.001 children (aref box (size 0.066 0.016 0.105))) physics physics (density -1 mass 0.079446 centerofmass 0.009506 -0.025995 -0.000503) maxforce 2.35)) name "AnkleL" boundingobject transform (translation 0 0.01 -0.0205 children (aref box (size 0.036 0.045 0.083))) physics physics (density -1 mass 0.167108 centerofmass -0.000214 0.013873 -0.018536) maxforce 2.35)) name "LegLowerL" boundingobject group (children (aref transform (translation 0 -0.058 -0.008 children (aref box (size 0.064 0.038 0.057))) transform (translation 0 -0.02 0.016 rotation 1 0 0 0.04 children (aref box (size 0.062 0.073 0.027))))) physics physics (density -1 mass 0.07031 centerofmass -0.000592 -0.039045 0.006548) maxforce 2.35)) name "LegUpperL" boundingobject transform (translation 0 -0.048 0.003 children (aref box (size 0.054 0.12 0.045))) physics physics (density -1 mass 0.119043 centerofmass -0.000323 -0.062965 0.000692) maxforce 2.35)) name "PelvL" boundingobject transform (translation 0 -0.0105 -0.0205 children (aref box (size 0.036 0.045 0.083))) physics physics (density -1 mass 0.167108 centerofmass 7.998280e-05 -0.013873 -0.018242) maxforce 2.35)) name "PelvYL" boundingobject transform (translation 0 0.0095 0.001 children (aref box (size 0.022 0.043 0.05))) physics physics (density -1 mass 0.027069 centerofmass 0 0.018437 0.00048) maxforce 2.35) def dpelvyr servo (translation -0.037 -0.1222 -0.005 children (aref def dpelvyrshape shape (appearance appearance (material use dmetalmaterial) geometry indexedfaceset (coord coordinate (point (aref )) coordindex (aref ) creaseangle 0.2)) def dpelvr servo (rotation 0 0 1 0 children (aref def dpelvrshape shape (appearance appearance (material use dservomaterial) geometry indexedfaceset (coord coordinate (point (aref )) coordindex (aref ) creaseangle 0.2)) def dlegupperr servo (rotation 1 0 0 0 children (aref def dlegupperrshape group (children (aref shape (appearance appearance (material use dplasticmaterial) geometry indexedfaceset (coord def dlegupperrcoord coordinate (point (aref )) coordindex (aref ) creaseangle 0.2)) shape (appearance appearance (material use dservomaterial) geometry indexedfaceset (coord use dlegupperrcoord coordindex (aref ) creaseangle 0.2)))) def dleglowerr servo (translation 0 -0.093 0 rotation 1 0 0 0 children (aref def dleglowerrshape shape (appearance appearance (material use dplasticmaterial) geometry indexedfaceset (coord coordinate (point (aref )) coordindex (aref ) creaseangle 0.2)) def dankler servo (translation 0 -0.093 0 rotation -1 0 0 0 children (aref def danklershape shape (appearance appearance (material use dservomaterial) geometry indexedfaceset (coord coordinate (point (aref )) coordindex (aref ) creaseangle 0.2)) def dfootr servo (rotation 0 0 1 0 children (aref def dfootrshape group (children (aref shape (appearance appearance (material use dplasticmaterial) geometry indexedfaceset (coord def dfootrcoord coordinate (point (aref )) coordindex (aref ) creaseangle 0.2)) shape (appearance appearance (material use dmetalmaterial) geometry indexedfaceset (coord use dfootrcoord coordindex (aref ) creaseangle 0.2))))) name "FootR" boundingobject transform (translation -0.011 -0.026 -0.001 children (aref box (size 0.066 0.016 0.105))) physics physics (density -1 mass 0.079446 centerofmass -0.009506 -0.025995 -0.000503) maxforce 2.35)) name "AnkleR" boundingobject transform (translation 0 0.01 -0.0205 children (aref box (size 0.036 0.045 0.083))) physics physics (density -1 mass 0.167108 centerofmass 0.000214 0.013873 -0.018536) maxforce 2.35)) name "LegLowerR" boundingobject group (children (aref transform (translation 0 -0.058 -0.008 children (aref box (size 0.064 0.038 0.057))) transform (translation 0 -0.02 0.016 rotation 1 0 0 0.04 children (aref box (size 0.062 0.073 0.027))))) physics physics (density -1 mass 0.07031 centerofmass 0.000592 -0.039045 0.006548) maxforce 2.35)) name "LegUpperR" boundingobject transform (translation 0 -0.048 0.003 children (aref box (size 0.054 0.12 0.045))) physics physics (density -1 mass 0.119043 centerofmass 0.000323 -0.062965 0.000692) maxforce 2.35)) name "PelvR" boundingobject transform (translation 0 -0.0105 -0.0205 children (aref box (size 0.036 0.045 0.083))) physics physics (density -1 mass 0.167108 centerofmass -7.998280e-05 -0.013873 -0.018242) maxforce 2.35)) name "PelvYR" boundingobject transform (translation 0 0.0095 0.001 children (aref box (size 0.022 0.043 0.05))) physics physics (density -1 mass 0.027069 centerofmass 0 0.018437 0.00048) maxforce 2.35) def dshoulderl servo (translation 0.082 0 0 rotation 1 0 0 0 children (aref def dshoulderlshape shape (appearance appearance (material use dmetalmaterial) geometry indexedfaceset (coord coordinate (point (aref )) coordindex (aref ) creaseangle 0.2)) def darmupperl servo (translation 0 -0.016 0 rotation 0 0 1 0 children (aref def darmupperlshape group (children (aref shape (appearance appearance (material use dplasticmaterial) geometry indexedfaceset (coord def darmupperlcoord coordinate (point (aref )) coordindex (aref ) creaseangle 0.2)) shape (appearance appearance (material use dservomaterial) geometry indexedfaceset (coord use darmupperlcoord coordindex (aref ) creaseangle 0.2)))) def darmlowerl servo (translation 0 -0.06 0.016 rotation 1 0 0 -1.5708 children (aref def darmlowerlshape group (children (aref shape (appearance appearance (material use dplasticmaterial) geometry indexedfaceset (coord def darmlowerlcoord coordinate (point (aref )) coordindex (aref ) creaseangle 0.2)) shape (appearance appearance (material use dmetalmaterial) geometry indexedfaceset (coord use darmlowerlcoord coordindex (aref ) creaseangle 0.2))))) name "ArmLowerL" boundingobject transform (translation 0.0075 -0.073 -0.016 children (aref box (size 0.027 0.113 0.036))) physics physics (density -1 mass 0.059289 centerofmass 0.006666 -0.045838 -0.01349) maxforce 2.35)) name "ArmUpperL" boundingobject group (children (aref transform (translation 0 -0.013 0 children (aref box (size 0.036 0.055 0.04))) transform (translation 0 -0.06 0.003 children (aref box (size 0.036 0.036 0.051))))) physics physics (density -1 mass 0.168377 centerofmass 0.00066 -0.036239 0.000734) maxforce 2.35)) name "ShoulderL" boundingobject transform (translation -0.009 -0.0065 0 rotation 0 0 1 -0.785398 children (aref box (size 0.027 0.022 0.044))) physics physics (density -1 mass 0.025913 centerofmass -0.013523 -0.005736 0.001394) maxforce 2.35) def dshoulderr servo (translation -0.082 0 0 rotation -1 0 0 0 children (aref def dshoulderrshape shape (appearance appearance (material use dmetalmaterial) geometry indexedfaceset (coord coordinate (point (aref )) coordindex (aref ) creaseangle 0.2)) def darmupperr servo (translation 0 -0.016 0 rotation 0 0 1 0 children (aref def darmupperrshape group (children (aref shape (appearance appearance (material use dplasticmaterial) geometry indexedfaceset (coord def darmupperrcoord coordinate (point (aref )) coordindex (aref ) creaseangle 0.2)) shape (appearance appearance (material use dservomaterial) geometry indexedfaceset (coord use darmupperrcoord coordindex (aref ) creaseangle 0.2)))) def darmlowerr servo (translation 0 -0.06 0.016 rotation -1 0 0 1.5708 children (aref def darmlowerrshape group (children (aref shape (appearance appearance (material use dplasticmaterial) geometry indexedfaceset (coord def darmlowerrcoord coordinate (point (aref )) coordindex (aref ) creaseangle 0.2)) shape (appearance appearance (material use dmetalmaterial) geometry indexedfaceset (coord use darmlowerrcoord coordindex (aref ) creaseangle 0.2))))) name "ArmLowerR" boundingobject transform (translation -0.0075 -0.073 -0.016 children (aref box (size 0.027 0.113 0.036))) physics physics (density -1 mass 0.059289 centerofmass -0.006666 -0.045838 -0.01349) maxforce 2.35)) name "ArmUpperR" boundingobject group (children (aref transform (translation 0 -0.013 0 children (aref box (size 0.036 0.055 0.04))) transform (translation 0 -0.06 0.003 children (aref box (size 0.036 0.036 0.051))))) physics physics (density -1 mass 0.168377 centerofmass -0.00066 -0.036239 0.000734) maxforce 2.35)) name "ShoulderR" boundingobject transform (translation 0.009 -0.0065 0 rotation 0 0 1 0.785398 children (aref box (size 0.027 0.022 0.044))) physics physics (density -1 mass 0.025913 centerofmass 0.013523 -0.005736 0.001394) maxforce 2.35)) name "DARwIn-OP" model "DARwIn-OP" boundingobject group (children (aref transform (translation 0 -0.059 -0.025 children (aref box (size 0.1 0.058 0.091))) transform (translation 0 -0.029 -0.038 children (aref box (size 0.126 0.045 0.076))) transform (translation 0 -0.006 -0.027 rotation 1 0 0 0.1 children (aref box (size 0.114 0.058 0.076))) transform (translation 0 -0.042 0.028 children (aref box (size 0.114 0.046 0.016))))) name darwin-op model "DARwIn-OP" description "Robotis' DARwIn-OP modeled by Fabien Rohrer" physics physics (density -1 mass 0.975599 centerofmass -0.003116 -0.039444 -0.019663) controller void controllerargs voidargs)))

(Defun hoge ()
  (let* ((zaxis (normalize-vector (v- (send *darwin* :head :neck-p :child-link :worldpos) (send *darwin* :worldpos))))
	 (yaxis (normalize-vector (v- (send *darwin* :larm :shoulder-p :child-link :worldpos) (send *darwin* :worldpos))))
	 (xaxis (v* yaxis zaxis))
	 (dif
	  (send (make-coords :rot (transpose (make-matrix 3 3 (list xaxis yaxis zaxis))))
		:transformation (make-coords))))
    (append (concatenate cons (normalize-vector (matrix-log (send dif :worldrot))))
	    (list (norm (matrix-log (send dif :worldrot)))))
    ))

(in-package "USER")
(import '(vrml::convert-webots-vrml->openhrp3-vrml-for-darwin))