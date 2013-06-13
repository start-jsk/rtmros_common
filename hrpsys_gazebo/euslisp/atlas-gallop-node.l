;

(defclass dummy-viewer
  :super object
  :slots nil)
(defmethod dummy-viewer
  (:init (&rest args) nil)
  (:draw-objects (&rest args) nil))
(defun objects (&rest args) nil)

(defvar *irtviewer* (instance dummy-viewer :init))


(require "murooka.l")
(my-init)

(require "atlas-tolot.l")
(require "atlas-standup.l")
(tolot-init :reverse? t)
(set-user)
(model2real)

(defun gen-dict
  nil
  (list
   (cons :stop
         (list
          (cons :av-list
                (progn
                  (tolot-init)
                  (model2real_ 300)
                  (unix:sleep 1)
                  (let ((buf (send *robot* :angle-vector)))
                    (vector buf buf))))
          (cons :sleep #(300 300))))
   (cons :forward
         (list
          (cons :av-list
                (progn
                  (send *tolot-hash* :enter :direction #(#f(-100 0 0) #f(-500 0 0)))
                  (gallop-step :reverse? t :sleep 100)
                  (copy-object (send *tolot-hash* :get :dict))))
          (cons :sleep #(100 100))))
   (cons :right
         (list
          (cons :av-list
                (progn
                  (send *tolot-hash* :enter :direction #(#f(-0 -0 0) #f(-500 -50 0)))
                  (gallop-step :reverse? t :rotate -30 :sleep 100)
                  (copy-object (send *tolot-hash* :get :dict))))
          (cons :sleep #(100 100))))
   (cons :left
         (list
          (cons :av-list
                (progn
                  (send *tolot-hash* :enter :direction #(#f(-0 -0 0) #f(-500 50 0)))
                  (gallop-step :reverse? t :rotate 30 :sleep 100)
                  (copy-object (send *tolot-hash* :get :dict))))
          (cons :sleep #(100 100))))))

(defun negaeri-once
  nil
  (negaeri)
  (setq *gallop-mode* :stop))

(setq *gallop-dict*
      '((:stop (:av-list . #(#f(-0.003048 6.09591 -0.053802 105.109 97.2697 0.0 -26.5584 105.814 0.0 -105.089 97.0922 0.0 26.6117 105.805 0.0 99.6314 1.06171 2.72941 17.1759 -4.57762 -1.29622 99.6322 -0.980218 2.75578 -17.0183 -4.58332 1.34451 0.0) #f(-0.003048 6.09591 -0.053802 105.109 97.2697 0.0 -26.5584 105.814 0.0 -105.089 97.0922 0.0 26.6117 105.805 0.0 99.6314 1.06171 2.72941 17.1759 -4.57762 -1.29622 99.6322 -0.980218 2.75578 -17.0183 -4.58332 1.34451 0.0))) (:sleep . #(300 300))) (:forward (:av-list . #(#f(0.02106 19.9715 -0.063195 108.512 72.4522 0.0 60.0 99.8673 0.0 -108.271 72.0259 0.0 -60.0 99.6771 0.0 89.4923 1.78779 6.2236 17.7434 -3.04163 0.879951 89.4598 -1.65378 6.27114 -17.6136 -3.01217 -0.762856 0.0) #f(-0.000869 -8.5676 -0.059016 103.058 88.9548 0.0 -15.4401 112.063 0.0 -103.068 88.9449 0.0 15.4401 112.069 0.0 130.294 -12.5845 30.023 27.9989 -51.5662 -18.3346 130.29 12.6254 30.023 -27.9531 -51.5662 18.3346 0.0))) (:sleep . #(100 100))) (:right (:av-list . #(#f(-0.013919 23.9251 -30.0 107.668 92.2545 0.0 60.0 98.3323 0.0 -107.538 92.0215 0.0 -60.0 98.2446 0.0 93.1815 1.3889 4.89525 17.6166 -4.30003 0.174148 93.177 -1.30705 4.92571 -17.4672 -4.29974 -0.11187 0.0) #f(1.14502 -7.98182 -30.0 105.159 63.8736 0.0 -21.3554 111.804 0.0 -102.146 104.352 0.0 12.5502 111.655 0.0 130.881 -11.1906 30.023 28.3614 -51.5662 -18.3346 130.188 14.1449 30.023 -24.6546 -51.5662 18.3346 0.0))) (:sleep . #(100 100))) (:left (:av-list . #(#f(-0.013919 23.9251 30.0 107.668 92.2545 0.0 60.0 98.3323 0.0 -107.538 92.0215 0.0 -60.0 98.2446 0.0 93.1815 1.3889 4.89525 17.6166 -4.30003 0.174148 93.177 -1.30705 4.92571 -17.4672 -4.29974 -0.11187 0.0) #f(-1.1423 -7.99684 30.0 102.134 104.378 0.0 -12.5475 111.646 0.0 -105.164 63.8791 0.0 21.3513 111.808 0.0 130.198 -14.0979 30.023 24.742 -51.5662 -18.3346 130.877 11.2193 30.023 -28.3614 -51.5662 18.3346 0.0))) (:sleep . #(100 100))) (:negaeri . negaeri-once)))

(defvar *gallop-mode* :stop)
(defvar *neck-angle* -40)

(defun play
  (key)
  (let* ((target (cdr (assoc key *gallop-dict*))))
    (if (functionp target)
        (funcall target)
      (map cons
           #'(lambda (av time)
               (send *robot* :angle-vector av)
               (send *robot* :head :neck-p :joint-angle *neck-angle*)
               (model2real_ time))
           (cdr (assoc :av-list target))
           (cdr (assoc :sleep target))))))

(defun gallop-callback
  (msg)
  (describe msg)
  (setq
   *gallop-mode*
   (cdr
    (find-if #'(lambda (a) (eq (aref (send msg :data) 0)
                               (car a)))
             (list
              (cons #\n :negaeri)
              (cons #\f :forward)
              (cons #\s :stop)
              (cons #\r :right)
              (cons #\l :left))))))

(ros::roseus-add-msgs "std_msgs")
(ros::subscribe "/atlas_gallop_controller/command"
                std_msgs::string
                #'gallop-callback)

(do-until-key
 (play *gallop-mode*)
 (ros::spin-once))
#|

(instance std_msgs::String :init :data "stop")