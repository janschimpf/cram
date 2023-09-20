(in-package :cashier)

;;@author Jan Schimpf

(defparameter *z-across-x-grasp-rotation*
  '((0 1  0)
    (1  0  0)
    (0  0  1)))

(defparameter *lift-z-offset* 0.1 "in meters")
(defparameter *lift-offset* `(0.0 0.0 ,*lift-z-offset*))

(defmethod man-int:get-action-gripping-effort :heuristics 20
    ((object-type (eql :fruit-juice))) 50)
    
(defmethod man-int:get-action-gripping-effort :heuristics 20
    ((object-type (eql :small-book))) 50)

(defmethod man-int:get-action-gripper-opening :heuristics 20
    ((object-type (eql :small-book))) 0.10)
    
(defmethod man-int:get-action-gripper-opening :heuristics 20
    ((object-type (eql :fruit-juice))) 0.10)

(defmethod man-int:get-object-type-carry-config :heuristics 20
    ((object-type (eql :small-book)) grasp)
  :carry)
  
(defmethod man-int:get-object-type-carry-config :heuristics 20
    ((object-type (eql :fruit-juice )) grasp)
  :carry)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; fruit-juice ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defparameter *fruit-juice-pregrasp-xy-offset* 0.05)
(defparameter *fruit-juice-grasp-xy-offset* 0.01 "in meters")
(defparameter *fruit-juice-grasp-z-offset* 0.02 "in meters")
(defparameter *fruit-juice-top-grasp-x-offset* 0.02 "in meters")
(defparameter *fruit-juice-top-grasp-z-offset* 0.02 "in meters")

;; Side grasp

(man-int:def-object-type-to-gripper-transforms :fruit-juice '(:left :right) :left-side
  :grasp-translation `(0.0d0 ,(- *fruit-juice-grasp-xy-offset*) ,*fruit-juice-grasp-z-offset*)
  :grasp-rot-matrix man-int:*y-across-z-grasp-rotation*
  :pregrasp-offsets `(0.0 ,*fruit-juice-pregrasp-xy-offset* ,0)
  :2nd-pregrasp-offsets `(0.0 ,*fruit-juice-pregrasp-xy-offset* 0.0)
  :lift-translation *lift-offset*
  :2nd-lift-translation *lift-offset*)

(man-int:def-object-type-to-gripper-transforms :fruit-juice '(:left :right) :right-side
  :grasp-translation `(0.0d0 ,*fruit-juice-grasp-xy-offset* ,*fruit-juice-grasp-z-offset*)
  :grasp-rot-matrix man-int:*-y-across-z-grasp-rotation*
  :pregrasp-offsets `(0.0 ,(- *fruit-juice-pregrasp-xy-offset*) ,0)
  :2nd-pregrasp-offsets `(0.0 ,(- *fruit-juice-pregrasp-xy-offset*) 0.0)
  :lift-translation *lift-offset*
  :2nd-lift-translation *lift-offset*)
  
;; Back grasp
(man-int:def-object-type-to-gripper-transforms :fruit-juice '(:left :right) :back
  :grasp-translation `(,*fruit-juice-grasp-xy-offset* 0.0d0 ,*fruit-juice-grasp-z-offset*)
  :grasp-rot-matrix man-int:*-x-across-z-grasp-rotation-2*
  :pregrasp-offsets `(,(- *fruit-juice-pregrasp-xy-offset*) 0.0 ,0)
  :2nd-pregrasp-offsets `(,(- *fruit-juice-pregrasp-xy-offset*) 0.0 0.0)
  :lift-translation *lift-offset*
  :2nd-lift-translation *lift-offset*)

;; front grasp
(man-int:def-object-type-to-gripper-transforms :small-book '(:left :right) :front
  :grasp-translation `(,*fruit-juice-grasp-xy-offset* 0.0d0 ,*fruit-juice-grasp-z-offset*)
  :grasp-rot-matrix man-int:*x-across-z-grasp-rotation*
  :pregrasp-offsets `(,(- *fruit-juice-pregrasp-xy-offset* 0) 0.0 ,0)
  :2nd-pregrasp-offsets `(,(- *fruit-juice-pregrasp-xy-offset*) 0.0 0.0)
  :lift-translation *lift-offset*
  :2nd-lift-translation *lift-offset*)
  
;; Bottom grasp
(man-int:def-object-type-to-gripper-transforms :fruit-juice '(:left :right) :bottom
  :grasp-translation `(0.0d0 0.0d0 ,(- *fruit-juice-grasp-z-offset*))
  :grasp-rot-matrix man-int:*-z-across-x-grasp-rotation*
  :pregrasp-offsets `(0.0 0.0 ,(- *lift-z-offset*))
  :2nd-pregrasp-offsets `(0.0 0 ,(- *lift-z-offset*))
  :lift-translation *lift-offset*
  :2nd-lift-translation *lift-offset*)
  
;; Top grasp
(man-int:def-object-type-to-gripper-transforms :fruit-juice '(:left :right) :top
  :grasp-translation `(0 0.0d0 ,*fruit-juice-top-grasp-z-offset*)
  :grasp-rot-matrix man-int:*z-across-y-grasp-rotation*
  :pregrasp-offsets *lift-offset*
  :2nd-pregrasp-offsets *lift-offset*
  :lift-translation *lift-offset*
  :2nd-lift-translation *lift-offset*)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; small-book ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defparameter *small-book-pregrasp-xy-offset* 0.05)
(defparameter *small-book-grasp-xy-offset* 0.03 "in meters")
(defparameter *small-book-grasp-z-offset* 0.05 "in meters")
(defparameter *small-book-top-grasp-x-offset* 0.05 "in meters")
(defparameter *small-book-top-grasp-z-offset* 0.05 "in meters")

;; Side grasp

(man-int:def-object-type-to-gripper-transforms :small-book '(:left :right) :left-side
  :grasp-translation `(0.0d0 ,(- *small-book-grasp-xy-offset*) 0)
  :grasp-rot-matrix man-int:*y-across-z-grasp-rotation*
  :pregrasp-offsets `(0.0 ,*small-book-grasp-xy-offset* ,*lift-z-offset*)
  :2nd-pregrasp-offsets `(0.0 ,*small-book-grasp-xy-offset* 0.0)
  :lift-translation *lift-offset*
  :2nd-lift-translation *lift-offset*)

(man-int:def-object-type-to-gripper-transforms :small-book '(:left :right) :right-side
  :grasp-translation `(0.0d0 ,*small-book-grasp-xy-offset* 0)
  :grasp-rot-matrix man-int:*-y-across-z-grasp-rotation*
  :pregrasp-offsets `(0.0 ,(- *small-book-grasp-xy-offset*) ,*lift-z-offset*)
  :2nd-pregrasp-offsets `(0.0 ,(- *small-book-grasp-xy-offset*) 0.0)
  :lift-translation *lift-offset*
  :2nd-lift-translation *lift-offset*)

;; Bottom grasp
(man-int:def-object-type-to-gripper-transforms :small-book '(:left :right) :bottom
  :grasp-translation `(0 0 ,(- *small-book-grasp-z-offset*))
  :grasp-rot-matrix *z-across-x-grasp-rotation*
  :pregrasp-offsets `(0.0 0.0 ,(- *lift-z-offset*))
  :2nd-pregrasp-offsets `(0.0 0 ,(- *lift-z-offset*))
  :lift-translation *lift-offset*
  :2nd-lift-translation *lift-offset*)
  
;; Top grasp
(man-int:def-object-type-to-gripper-transforms :small-book '(:left :right) :top
  :grasp-translation `(0 0.0d0 ,*small-book-top-grasp-z-offset*)
  :grasp-rot-matrix man-int:*z-across-y-grasp-rotation*
  :pregrasp-offsets *lift-offset*
  :2nd-pregrasp-offsets *lift-offset*
  :lift-translation *lift-offset*
  :2nd-lift-translation *lift-offset*)
  
