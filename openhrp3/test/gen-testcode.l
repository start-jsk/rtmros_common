(defun print-link (f ls)
   (dolist (l ls)
     (let ((p (scale 0.001 (send *robot* l :worldpos))) (r (send *robot* l :worldrot)))
       (format f "        self.check_link(\"~A\", [~A, ~A, ~A], [~A, ~A, ~A,  ~A, ~A, ~A, ~A, ~A, ~A])~%"
               (send *robot* l :joint :name)
               (elt p 0) (elt p 1) (elt p 2)
               (aref r 0 0) (aref r 0 1) (aref r 0 2) (aref r 1 0) (aref r 1 1) (aref r 1 2) (aref r 2 0) (aref r 2 1) (aref r 2 2)))))
(defun change-joint (f ls a)
  (format f "        self.angle_vector([math.pi * ~A / 180.0 for i in range(~A)])~%" a (length ls))
  (dolist (l ls) (send *robot* l :joint :joint-angle a)))

(defun print-header (f pkg filepath)
    (format f "#!/usr/bin/env python

from  test_modelloader import testModelLoader
import rospkg, math

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

(defun gen-testcode (robot links &optional pkg filepath)
  (setq *robot* robot)
  (setq *robot-name* (send *robot* :name))
  (setq *file-name* (pathname-name (cadr (assoc (intern (string-upcase (send *robot* :name)) "KEYWORD") *modules*))))
  (warning-message 2 ";; generate test code for ~A to test_~A.py~%" *robot-name* *file-name*)
  (with-open-file (f (format nil "test_~A.py" *file-name*) :direction :output)
    (print-header f pkg filepath)
    (format f "~%    def test_av0(self):~%")
    (print-link f links)
    (format f "~%    def test_av30(self):~%")
    (change-joint f links 30)
    (print-link f links)
    (print-footer f)
  ))
