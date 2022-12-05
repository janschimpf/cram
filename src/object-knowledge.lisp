;;;
;;; Copyright (c) 2019, Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
;;                      Vanessa Hassouna <hassouna@uni-bremen.de>
;;; All rights reserved.
;;;
;;; Redistribution and use in source and binary forms, with or without
;;; modification, are permitted provided that the following conditions are met:
;;;
;;;     * Redistributions of source code must retain the above copyright
;;;       notice, this list of conditions and the following disclaimer.
;;;     * Redistributions in binary form must reproduce the above copyright
;;;       notice, this list of conditions and the following disclaimer in the
;;;       documentation and/or other materials provided with the distribution.
;;;     * Neither the name of the Intelligent Autonomous Systems Group/
;;;       Technische Universitaet Muenchen nor the names of its contributors
;;;       may be used to endorse or promote products derived from this software
;;;       without specific prior written permission.
;;;
;;; THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
;;; AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
;;; IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
;;; ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
;;; LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
;;; CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
;;; SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
;;; INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
;;; CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
;;; ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
;;; POSSIBILITY OF SUCH DAMAGE.

(in-package :cashier)

(defparameter *lift-z-offset* 0.05 "in meters")
(defparameter *lift-offset* `(0.0 0.0 ,*lift-z-offset*))

(defmethod man-int:get-action-gripping-effort :heuristics 20
    ((object-type (eql :pringles))) 50)

(defmethod man-int:get-action-gripping-effort :heuristics 20
    ((object-type (eql :small-cube))) 50)
    
(defmethod man-int:get-action-gripping-effort :heuristics 20
    ((object-type (eql :fruit-juice))) 50)
    
(defmethod man-int:get-action-gripping-effort :heuristics 20
    ((object-type (eql :snackbar))) 50)
    
(defmethod man-int:get-action-gripping-effort :heuristics 20
    ((object-type (eql :small-book))) 50)

(defmethod man-int:get-action-gripper-opening :heuristics 20
    ((object-type (eql :pringles))) 0.10)


(defmethod man-int:get-action-gripper-opening :heuristics 20
    ((object-type (eql :small-book))) 0.10)
    

(defmethod man-int:get-action-gripper-opening :heuristics 20
    ((object-type (eql :fruit-juice))) 0.10)
    
    
(defmethod man-int:get-action-gripper-opening :heuristics 20
    ((object-type (eql  :snackbar))) 0.10)


(defmethod man-int:get-action-gripper-opening :heuristics 20
    ((object-type (eql :small-cube))) 0.10)

(defmethod man-int:get-object-type-carry-config :heuristics 20
    ((object-type (eql :pringles)) grasp)
  :carry)
  
(defmethod man-int:get-object-type-carry-config :heuristics 20
    ((object-type (eql :small-book)) grasp)
  :carry)
  
(defmethod man-int:get-object-type-carry-config :heuristics 20
    ((object-type (eql :fruit-juice )) grasp)
  :carry)
  
(defmethod man-int:get-object-type-carry-config :heuristics 20
    ((object-type (eql :snackbar )) grasp)
  :carry)
  
(defmethod man-int:get-object-type-carry-config :heuristics 20
    ((object-type (eql  :small-cube)) grasp)
  :carry)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; pringles ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;; pregrasp offset for pringles size '((0.7 0.0777 0.65))
(defparameter *pringles-pregrasp-xy-offset* 0.03)
(defparameter *pringles-grasp-xy-offset* 0.01 "in meters")
(defparameter *pringles-grasp-z-offset* 0.005 "in meters")

;; SIDE grasp
(man-int:def-object-type-to-gripper-transforms :pringles :left :left-side
  :grasp-translation `(0.0d0 ,(- *pringles-grasp-xy-offset*) ,*pringles-grasp-z-offset*)
  :grasp-rot-matrix man-int:*y-across-z-grasp-rotation*
  :pregrasp-offsets `(0.0 ,*pringles-pregrasp-xy-offset* 0.01)
  :2nd-pregrasp-offsets `(0.0 ,*pringles-pregrasp-xy-offset* 0.01)
  :lift-translation `(0.0 0.0 ,*lift-z-offset*)
  :2nd-lift-translation `(0.0 0.0 ,*lift-z-offset*))

(man-int:def-object-type-to-gripper-transforms :pringles :left :right-side
 :grasp-translation `(0.0d0 ,*pringles-grasp-xy-offset* ,*pringles-grasp-z-offset*)
  :grasp-rot-matrix man-int:*-y-across-z-grasp-rotation*
  :pregrasp-offsets `(0.0 ,(- *pringles-pregrasp-xy-offset*) 0.01)
  :2nd-pregrasp-offsets `(0.0 ,(- *pringles-pregrasp-xy-offset*) 0.01)
  :lift-translation `(0.0 0.0 ,*lift-z-offset*)
  :2nd-lift-translation `(0.0 0.0 ,*lift-z-offset*))

;; BACK grasp
(man-int:def-object-type-to-gripper-transforms :pringles :left :back
  :grasp-translation `(,*pringles-grasp-xy-offset* 0.0d0 ,*pringles-grasp-z-offset*)
  :grasp-rot-matrix man-int:*-x-across-z-grasp-rotation*
  :pregrasp-offsets `(,(- *pringles-pregrasp-xy-offset*) 0.0 0.01)
  :2nd-pregrasp-offsets `(,(- *pringles-pregrasp-xy-offset*) 0.0 0.01)
  :lift-translation `(0.0 0.0 ,*lift-z-offset*)
  :2nd-lift-translation `(0.0 0.0 ,*lift-z-offset*))

;; FRONT grasp
(man-int:def-object-type-to-gripper-transforms :pringles :left :front
  :grasp-translation `(,*pringles-grasp-xy-offset* 0.0d0 ,*pringles-grasp-z-offset*)
  :grasp-rot-matrix man-int:*x-across-z-grasp-rotation*
  :pregrasp-offsets `(,(+ *pringles-pregrasp-xy-offset*) 0.0 0.01)
  :2nd-pregrasp-offsets `(,(+ *pringles-pregrasp-xy-offset*) 0.0 0.01)
  :lift-translation `(0.0 0.0 ,*lift-z-offset*)
  :2nd-lift-translation `(0.0 0.0 ,*lift-z-offset*))

;; Front grasp robot
(man-int:def-object-type-to-gripper-transforms :pringles :left :front
  :location-type :robot
  :grasp-translation `(,*pringles-grasp-xy-offset* 0.0d0 ,*pringles-grasp-z-offset*)
  :grasp-rot-matrix man-int:*x-across-z-grasp-rotation*
  :pregrasp-offsets `(,(+ *pringles-pregrasp-xy-offset*) 0.0 0.01)
  :2nd-pregrasp-offsets `(,(+ *pringles-pregrasp-xy-offset*) 0.0 0.01)
  :lift-translation `(0.0 0.0 ,*lift-z-offset*)
  :2nd-lift-translation `(0.0 0.0 ,*lift-z-offset*))
  
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; small-cube ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defparameter *small-cube-pregrasp-xy-offset* 0.15)
(defparameter *small-cube-grasp-xy-offset* 0.01 "in meters")
(defparameter *small-cube-grasp-z-offset* 0.001 "in meters")
(defparameter *small-cube-top-grasp-x-offset* 0.001 "in meters")
(defparameter *small-cube-top-grasp-z-offset* 0.001 "in meters")

;; Side grasp

(man-int:def-object-type-to-gripper-transforms :small-cube'(:left :right) :left-side
  :grasp-translation `(0.0d0 ,(- *small-cube-grasp-xy-offset*) ,*small-cube-grasp-z-offset*)
  :grasp-rot-matrix man-int:*y-across-z-flipped-grasp-rotation*
  :pregrasp-offsets `(0.0 ,*small-cube-pregrasp-xy-offset* ,*lift-z-offset*)
  :2nd-pregrasp-offsets `(0.0 ,*small-cube-pregrasp-xy-offset* 0.0)
  :lift-translation *lift-offset*
  :2nd-lift-translation *lift-offset*)

(man-int:def-object-type-to-gripper-transforms :small-cube '(:left :right) :right-side
  :grasp-translation `(0.0d0 ,*small-cube-grasp-xy-offset* ,*small-cube-grasp-z-offset*)
  :grasp-rot-matrix man-int:*-y-across-z-flipped-grasp-rotation*
  :pregrasp-offsets `(0.0 ,(- *small-cube-pregrasp-xy-offset*) ,*lift-z-offset*)
  :2nd-pregrasp-offsets `(0.0 ,(- *small-cube-pregrasp-xy-offset*) 0.0)
  :lift-translation *lift-offset*
  :2nd-lift-translation *lift-offset*)
  
;; Back grasp

(man-int:def-object-type-to-gripper-transforms :small-cube '(:left :right) :back
  :grasp-translation `(,*small-cube-grasp-xy-offset* 0.0d0 ,*small-cube-grasp-z-offset*)
  :grasp-rot-matrix man-int:*-x-across-z-grasp-rotation-2*
  :pregrasp-offsets `(,(- *small-cube-pregrasp-xy-offset*) 0.0 ,*lift-z-offset*)
  :2nd-pregrasp-offsets `(,(- *small-cube-pregrasp-xy-offset*) 0.0 0.0)
  :lift-translation *lift-offset*
  :2nd-lift-translation *lift-offset*)

;; Front grasp

(man-int:def-object-type-to-gripper-transforms :small-cube '(:left :right) :front
  :grasp-translation `(,(- *small-cube-grasp-xy-offset*) 0.0d0 ,*small-cube-grasp-z-offset*)
  :grasp-rot-matrix man-int:*x-across-z-grasp-rotation*
  :pregrasp-offsets `(,*small-cube-pregrasp-xy-offset* 0.0 ,*lift-z-offset*)
  :2nd-pregrasp-offsets `(,*small-cube-pregrasp-xy-offset* 0.0 0.0)
  :lift-translation *lift-offset*
  :2nd-lift-translation *lift-offset*)
;; Bottom grasp
(man-int:def-object-type-to-gripper-transforms :small-cube '(:left :right) :bottom
  :grasp-translation `(0.0d0 0.0d0 ,(- *small-cube-grasp-z-offset*))
  :grasp-rot-matrix man-int:*-z-across-x-grasp-rotation*
  :pregrasp-offsets `(0.0 0.0 ,(- *lift-z-offset*))
  :2nd-pregrasp-offsets `(0.0 0 ,(- *lift-z-offset*))
  :lift-translation *lift-offset*
  :2nd-lift-translation *lift-offset*)
  
;; Top grasp
(man-int:def-object-type-to-gripper-transforms :small-cube '(:left :right) :top
  :grasp-translation `(,(- *small-cube-top-grasp-x-offset*) 0.0d0 ,*small-cube-top-grasp-z-offset*)
  :grasp-rot-matrix man-int:*z-across-y-grasp-rotation*
  :pregrasp-offsets *lift-offset*
  :2nd-pregrasp-offsets *lift-offset*
  :lift-translation *lift-offset*
  :2nd-lift-translation *lift-offset*)


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; snackbar ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defparameter *snackbar-pregrasp-xy-offset* 0.15)
(defparameter *snackbar-grasp-xy-offset* 0.01 "in meters")
(defparameter *snackbar-grasp-z-offset* 0.001 "in meters")
(defparameter *snackbar-top-grasp-x-offset* 0.001 "in meters")
(defparameter *snackbar-top-grasp-z-offset* 0.001 "in meters")

;; Side grasp

(man-int:def-object-type-to-gripper-transforms :snackbar '(:left :right) :left-side
  :grasp-translation `(0.0d0 ,(- *snackbar-grasp-xy-offset*) ,*snackbar-grasp-z-offset*)
  :grasp-rot-matrix man-int:*y-across-z-flipped-grasp-rotation*
  :pregrasp-offsets `(0.0 ,*snackbar-pregrasp-xy-offset* ,*lift-z-offset*)
  :2nd-pregrasp-offsets `(0.0 ,*snackbar-pregrasp-xy-offset* 0.0)
  :lift-translation *lift-offset*
  :2nd-lift-translation *lift-offset*)

(man-int:def-object-type-to-gripper-transforms :snackbar '(:left :right) :right-side
  :grasp-translation `(0.0d0 ,*snackbar-grasp-xy-offset* ,*snackbar-grasp-z-offset*)
  :grasp-rot-matrix man-int:*-y-across-z-flipped-grasp-rotation*
  :pregrasp-offsets `(0.0 ,(- *snackbar-pregrasp-xy-offset*) ,*lift-z-offset*)
  :2nd-pregrasp-offsets `(0.0 ,(- *snackbar-pregrasp-xy-offset*) 0.0)
  :lift-translation *lift-offset*
  :2nd-lift-translation *lift-offset*)
  
;; Back grasp

(man-int:def-object-type-to-gripper-transforms :snackbar '(:left :right) :back
  :grasp-translation `(,*snackbar-grasp-xy-offset* 0.0d0 ,*snackbar-grasp-z-offset*)
  :grasp-rot-matrix man-int:*-x-across-z-grasp-rotation-2*
  :pregrasp-offsets `(,(- *snackbar-pregrasp-xy-offset*) 0.0 ,*lift-z-offset*)
  :2nd-pregrasp-offsets `(,(- *snackbar-pregrasp-xy-offset*) 0.0 0.0)
  :lift-translation *lift-offset*
  :2nd-lift-translation *lift-offset*)

;; Front grasp

(man-int:def-object-type-to-gripper-transforms :snackbar '(:left :right) :front
  :grasp-translation `(,(- *snackbar-grasp-xy-offset*) 0.0d0 ,*snackbar-grasp-z-offset*)
  :grasp-rot-matrix man-int:*x-across-z-grasp-rotation*
  :pregrasp-offsets `(,*snackbar-pregrasp-xy-offset* 0.0 ,*lift-z-offset*)
  :2nd-pregrasp-offsets `(,*snackbar-pregrasp-xy-offset* 0.0 0.0)
  :lift-translation *lift-offset*
  :2nd-lift-translation *lift-offset*)
;; Bottom grasp
(man-int:def-object-type-to-gripper-transforms :snackbar '(:left :right) :bottom
  :grasp-translation `(0.0d0 0.0d0 ,(- *snackbar-grasp-z-offset*))
  :grasp-rot-matrix man-int:*-z-across-x-grasp-rotation*
  :pregrasp-offsets `(0.0 0.0 ,(- *lift-z-offset*))
  :2nd-pregrasp-offsets `(0.0 0 ,(- *lift-z-offset*))
  :lift-translation *lift-offset*
  :2nd-lift-translation *lift-offset*)
  
;; Top grasp
(man-int:def-object-type-to-gripper-transforms :snackbar '(:left :right) :top
  :grasp-translation `(,(- *snackbar-top-grasp-x-offset*) 0.0d0 ,*snackbar-top-grasp-z-offset*)
  :grasp-rot-matrix man-int:*z-across-y-grasp-rotation*
  :pregrasp-offsets *lift-offset*
  :2nd-pregrasp-offsets *lift-offset*
  :lift-translation *lift-offset*
  :2nd-lift-translation *lift-offset*)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; fruit-juice ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defparameter *fruit-juice-pregrasp-xy-offset* 0.15)
(defparameter *fruit-juice-grasp-xy-offset* 0.01 "in meters")
(defparameter *fruit-juice-grasp-z-offset* 0.005 "in meters")
(defparameter *fruit-juice-top-grasp-x-offset* 0.05 "in meters")
(defparameter *fruit-juice-top-grasp-z-offset* 0.05 "in meters")

;; Side grasp

(man-int:def-object-type-to-gripper-transforms :fruit-juice '(:left :right) :left-side
  :grasp-translation `(0.0d0 ,(- *fruit-juice-grasp-xy-offset*) ,*fruit-juice-grasp-z-offset*)
  :grasp-rot-matrix man-int:*y-across-z-flipped-grasp-rotation*
  :pregrasp-offsets `(0.0 ,*fruit-juice-pregrasp-xy-offset* ,*lift-z-offset*)
  :2nd-pregrasp-offsets `(0.0 ,*fruit-juice-pregrasp-xy-offset* 0.0)
  :lift-translation *lift-offset*
  :2nd-lift-translation *lift-offset*)

(man-int:def-object-type-to-gripper-transforms :fruit-juice '(:left :right) :right-side
  :grasp-translation `(0.0d0 ,*fruit-juice-grasp-xy-offset* ,*fruit-juice-grasp-z-offset*)
  :grasp-rot-matrix man-int:*-y-across-z-flipped-grasp-rotation*
  :pregrasp-offsets `(0.0 ,(- *fruit-juice-pregrasp-xy-offset*) ,*lift-z-offset*)
  :2nd-pregrasp-offsets `(0.0 ,(- *fruit-juice-pregrasp-xy-offset*) 0.0)
  :lift-translation *lift-offset*
  :2nd-lift-translation *lift-offset*)
  
;; Back grasp

(man-int:def-object-type-to-gripper-transforms :fruit-juice '(:left :right) :back
  :grasp-translation `(,*fruit-juice-grasp-xy-offset* 0.0d0 ,*fruit-juice-grasp-z-offset*)
  :grasp-rot-matrix man-int:*-x-across-z-grasp-rotation-2*
  :pregrasp-offsets `(,(- *fruit-juice-pregrasp-xy-offset*) 0.0 ,*lift-z-offset*)
  :2nd-pregrasp-offsets `(,(- *fruit-juice-pregrasp-xy-offset*) 0.0 0.0)
  :lift-translation *lift-offset*
  :2nd-lift-translation *lift-offset*)

;; Front grasp

(man-int:def-object-type-to-gripper-transforms :fruit-juice '(:left :right) :front
  :grasp-translation `(,(- *fruit-juice-grasp-xy-offset*) 0.0d0 ,*fruit-juice-grasp-z-offset*)
  :grasp-rot-matrix man-int:*x-across-z-grasp-rotation*
  :pregrasp-offsets `(,*fruit-juice-pregrasp-xy-offset* 0.0 ,*lift-z-offset*)
  :2nd-pregrasp-offsets `(,*fruit-juice-pregrasp-xy-offset* 0.0 0.0)
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
  :grasp-translation `(,(- *fruit-juice-top-grasp-x-offset*) 0.0d0 ,*fruit-juice-top-grasp-z-offset*)
  :grasp-rot-matrix man-int:*z-across-y-grasp-rotation*
  :pregrasp-offsets *lift-offset*
  :2nd-pregrasp-offsets *lift-offset*
  :lift-translation *lift-offset*
  :2nd-lift-translation *lift-offset*)
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; small-book ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defparameter *small-book-pregrasp-xy-offset* 0.15)
(defparameter *small-book-grasp-xy-offset* 0.01 "in meters")
(defparameter *small-book-grasp-z-offset* 0.005 "in meters")
(defparameter *small-book-top-grasp-x-offset* 0.05 "in meters")
(defparameter *small-book-top-grasp-z-offset* 0.05 "in meters")

;; Side grasp

(man-int:def-object-type-to-gripper-transforms :small-book '(:left :right) :left-side
  :grasp-translation `(0.0d0 ,(- *small-book-grasp-xy-offset*) ,*small-book-grasp-z-offset*)
  :grasp-rot-matrix man-int:*y-across-z-flipped-grasp-rotation*
  :pregrasp-offsets `(0.0 ,*small-book-pregrasp-xy-offset* ,*lift-z-offset*)
  :2nd-pregrasp-offsets `(0.0 ,*small-book-pregrasp-xy-offset* 0.0)
  :lift-translation *lift-offset*
  :2nd-lift-translation *lift-offset*)

(man-int:def-object-type-to-gripper-transforms :small-book '(:left :right) :right-side
  :grasp-translation `(0.0d0 ,*small-book-grasp-xy-offset* ,*small-book-grasp-z-offset*)
  :grasp-rot-matrix man-int:*-y-across-z-flipped-grasp-rotation*
  :pregrasp-offsets `(0.0 ,(- *small-book-pregrasp-xy-offset*) ,*lift-z-offset*)
  :2nd-pregrasp-offsets `(0.0 ,(- *small-book-pregrasp-xy-offset*) 0.0)
  :lift-translation *lift-offset*
  :2nd-lift-translation *lift-offset*)
  
;; Back grasp

(man-int:def-object-type-to-gripper-transforms :small-book '(:left :right) :back
  :grasp-translation `(,*small-book-grasp-xy-offset* 0.0d0 ,*small-book-grasp-z-offset*)
  :grasp-rot-matrix man-int:*-x-across-z-grasp-rotation-2*
  :pregrasp-offsets `(,(- *small-book-pregrasp-xy-offset*) 0.0 ,*lift-z-offset*)
  :2nd-pregrasp-offsets `(,(- *small-book-pregrasp-xy-offset*) 0.0 0.0)
  :lift-translation *lift-offset*
  :2nd-lift-translation *lift-offset*)

;; Front grasp

(man-int:def-object-type-to-gripper-transforms :small-book '(:left :right) :front
  :grasp-translation `(,(- *small-book-grasp-xy-offset*) 0.0d0 ,*small-book-grasp-z-offset*)
  :grasp-rot-matrix man-int:*x-across-z-grasp-rotation*
  :pregrasp-offsets `(,*small-book-pregrasp-xy-offset* 0.0 ,*lift-z-offset*)
  :2nd-pregrasp-offsets `(,*small-book-pregrasp-xy-offset* 0.0 0.0)
  :lift-translation *lift-offset*
  :2nd-lift-translation *lift-offset*)
;; Bottom grasp
(man-int:def-object-type-to-gripper-transforms :small-book '(:left :right) :bottom
  :grasp-translation `(0.0d0 0.0d0 ,(- *small-book-grasp-z-offset*))
  :grasp-rot-matrix man-int:*-z-across-x-grasp-rotation*
  :pregrasp-offsets `(0.0 0.0 ,(- *lift-z-offset*))
  :2nd-pregrasp-offsets `(0.0 0 ,(- *lift-z-offset*))
  :lift-translation *lift-offset*
  :2nd-lift-translation *lift-offset*)
  
;; Top grasp
(man-int:def-object-type-to-gripper-transforms :small-book '(:left :right) :top
  :grasp-translation `(,(- *small-book-top-grasp-x-offset*) 0.0d0 ,*small-book-top-grasp-z-offset*)
  :grasp-rot-matrix man-int:*z-across-y-grasp-rotation*
  :pregrasp-offsets *lift-offset*
  :2nd-pregrasp-offsets *lift-offset*
  :lift-translation *lift-offset*
  :2nd-lift-translation *lift-offset*)
