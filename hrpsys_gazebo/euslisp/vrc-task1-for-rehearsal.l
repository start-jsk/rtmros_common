;; 10.5 3 

;; [0]  start roseus
roseus atlas-vehicle-motion.l

;; [1]  initialize (make *ri*)
(init-task1)

;; [2]  prepare for walking
(start-walk)

;; [3]  walk to near side of edge by following functions
;; (walk-clickpos)
;; (turn-clickpos)
;; (walk 0)
;; (turn 0)
;; (go-pos x y theta)
;; (rotate-with-imu 90)

;; [3.1] when falling down, stand up
;; (finish-walk)
;; (stand-up2)

;; [3.2] when you stop walking, finalize
;; (finish-walk)

;; [3.3] you can get snapshot of to see the relative pos of edge and foot
;; (snapshot-hand)

;; [4]  walk over edge
(walk-over-edge)

;; [5]  walk with first pose of getting-in-vehicle 
(start-walk2)
;; (walk-clickpos)
;; (turn-clickpos)
;; (walk 0)
;; (turn 0)
;; (go-pos x y theta)
;; (rotate-with-imu 90)

;; [6]  move waist and neck for seeing the vehicle. revert angle when walking again
;; (move-waist -10)
;; (move-neck 10)
;; (move-waist :zero)
;; (move-neck :zero)

;; [7]  localization with pointclolud
(get-diff-vehicle-pos)

;; [8]  walk according to localization, and finalzie walking and start manip
(finish-walk2)

;; [9]  get in vehicle
(lr-rev)
(get-in-for-task1 :real t :after-walk t)
(drive-for-task1 :real t :wait t)
