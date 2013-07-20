(defun print-link (f ls)
   (dolist (l ls)
     (let ((p (scale 0.001 (send *robot* l :worldpos))) (r (send *robot* l :worldrot)))
       (format f "        self.check_link(\"~A\", [~A, ~A, ~A], [~A, ~A, ~A,  ~A, ~A, ~A, ~A, ~A, ~A])~%"
               (send *robot* l :joint :name)
               (elt p 0) (elt p 1) (elt p 2)
               (aref r 0 0) (aref r 0 1) (aref r 0 2) (aref r 1 0) (aref r 1 1) (aref r 1 2) (aref r 2 0) (aref r 2 1) (aref r 2 2)))))
(defun change-joint (f ls &optional a)
  (let (angles)
    (dolist (l ls)
      (let ((j (send *robot* l :joint)) angle)
        (setq angle (if a a (* 0.5 (+ (send j :min-angle) (send j :max-angle)))))
        (if (and (< angle 0) (> angle 0)) ;; isnan
            (setq angle 125))
        (send j :joint-angle angle)
        (push angle angles)))
    (setq angles (reverse angles))
    (format f "        self.angle_vector([")
    (dolist (a angles) (format f "~A, " (deg2rad a)))
    (format f "])~%")
    ))

(defun print-header (f pkg filepath)
    (format f "#!/usr/bin/env python

import sys,math,rospkg
sys.path.append(rospkg.RosPack().get_path(\"openhrp3\")+\"/test\")

from  test_modelloader import testModelLoader


class Test~A(testModelLoader):

    def setUp(self):
        self.loadModel(rospkg.RosPack().get_path(\"~A\")+\"/~A\")
"
            *robot-name* pkg filepath))

(defun print-footer (f)
    (format f "
if __name__ == '__main__':
    import rostest
    rostest.rosrun('openhrp3', 'test_~A', Test~A)
"
            *file-name* *robot-name*))

(defun gen-testcode (robot links &optional pkg filepath) ;; order of links must be JointID order
  (setq *robot* robot)
  (setq *robot-name* (send *robot* :name))
  (setq *file-name* (pathname-name (cadr (assoc (intern (string-upcase (send *robot* :name)) "KEYWORD") *modules*))))
  (warning-message 2 ";; generate test code for ~A to test_~A.py~%" *robot-name* *file-name*)
  (with-open-file (f (format nil "test_~A.py" *file-name*) :direction :output)
    (print-header f pkg filepath)
    (format f "~%    def test_av0(self):~%")
    (print-link f links)
    (format f "~%    def test_av_mid(self):~%")
    (change-joint f links)
    (print-link f links)
    (print-footer f)
  ))
