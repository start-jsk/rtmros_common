#!/usr/bin/env roseus

(require :unittest "lib/llib/unittest.l")
(init-unit-test)

;; Wait until Ros bridges are prepared.
(ros::roseus "unittest")
(unless (boundp '*tfl*)
  (defvar *tfl* (instance ros::transform-listener :init)))
(send *tfl* :wait-for-transform "WAIST_LINK0" "odom" (ros::time) 20.0)
;; Load demo file which is specified next to this file in argument
(let ((name (elt lisp::*eustop-argument* (1+ (position-if #'(lambda (x) (substringp "samplerobot-unittest.l" x)) lisp::*eustop-argument*)))))
  (defvar *test-prefix* (pathname-name name))
  (load name))

;; Test init
(let ((func (find-if #'(lambda (x) ;; find init function
                         (substringp (format nil "~A-init" *test-prefix*) (string-downcase x)))
                     (functions))))
 (eval `(deftest ,(read-from-string (format nil "test-~A" func))
          (assert (,func)))))

;; Test demos
(dolist (funcs (remove-if-not #'(lambda (x) ;; find demo functions
                                  (and (substringp (format nil "~A-demo" *test-prefix*) (string-downcase x))
                                       (not (string= (format nil "~A-demo" *test-prefix*) (string-downcase x)))))
                              (functions)))
  (eval `(deftest ,(read-from-string (format nil "test-~A" funcs))
           (assert (,funcs)))))

(run-all-tests)
(exit 0)