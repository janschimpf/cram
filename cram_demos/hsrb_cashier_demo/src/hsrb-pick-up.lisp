;;;
;;; Copyright (c) 2020, Vanessa Hassouna <hassouna@uni-bremen.de>
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

(in-package :demo)
(defvar object-design NIL)

(defun pick-up-object (object-name object-type)
  "move hsr and grasp the object"
  (mirror-object-pose object-name)
  (grasp-object object-name object-type))

(defun mirror-object-pose (object-name)
  "moves the robot as the object while ignoring the enviroment"
  (let* ((?object-pose (btr:object-pose object-name)))
    (roslisp:with-fields (x y) (cl-transforms:origin ?object-pose)
      ;; TODO This does not work on the real robot
      ;; to make it work multiply the transforms instead of teleporting the robot
      (btr-utils:move-robot
       (cl-transforms:make-pose
        (cl-transforms:make-3d-vector x y 0)
        (cl-transforms:orientation ?object-pose)))))
  ;; update that the robot-state changed otherwise the transformer dosen't update
  (coe:on-event (make-instance 'cpoe:robot-state-changed)))

(defun grasp-object (object-name object-type)
  "places the robot in front-/left-/right-/back-side of the object and tries
to grasp the object, if that fails the next position in list will be tried"
  (let* ((nav-pose
           ;; TODO shuffle the list?
           ;; applying append function to the successive subsets of the list
           (reduce
            #'append
            ;; goes through each element in the sequence, and returns a new list
            (mapcar
             (lambda (list-poses)
               (let ((tmp-list nil))
                 (loop for a from 0.0 to 0.4 by 0.01
                       do
                          ;; push element to list
                          (push
                           (roslisp:with-fields (x y)
                               (cl-transforms:origin (car list-poses))
                             ;; check if the offset needs to be summed or subtracted
                             (if (or (eq (car (cdr list-poses)) :front)
                                     (eq (car (cdr list-poses)) :back))
                                 (setf x (+ x
                                            (if (> x 0)
                                                a
                                                (- a))))
                                 (setf y (+ y
                                            (if (> y 0)
                                                a
                                                (- a)))))
                             ;; create new list with a stamped-pose and x/y w
                             ;; with edited offset and the grasping-side
                             (list
                              (cl-transforms-stamped:transform-pose-stamped
                               cram-tf:*transformer*
                               :pose
                               (cl-transforms-stamped:make-pose-stamped
                                "base_footprint" 0
                                (cl-transforms:make-3d-vector x y 0)
                                (cl-transforms:orientation (car list-poses)))
                               :target-frame "map")

                              ;; get grasping-side
                              (cdr list-poses)))
                           ;; push to my-list
                           tmp-list))
                 ;; loop done and reverse tmp-list such that the lower value is first
                 (reverse tmp-list)))
             ;;the basic list that goes through the mapcar
             (list
              ;;(list (cl-transforms:make-pose
              ;;       (cl-transforms:make-3d-vector 0.40  0.07769999504089356d0 0)
              ;;       (cl-transforms:euler->quaternion :az pi))
              ;;      :front)
              ;;(list (cl-transforms:make-pose
              ;;       (cl-transforms:make-3d-vector -0.40  -0.07769999504089356d0 0)
              ;;       (cl-transforms:make-identity-rotation))
              ;;      :back)
              ;;(list (cl-transforms:make-pose
              ;;       (cl-transforms:make-3d-vector -0.07769999504089356d0 0.40 0)
              ;;       (cl-transforms:euler->quaternion :az (- (/ pi 2))))
              ;;      :left-side)
              (list (cl-transforms:make-pose
                     (cl-transforms:make-3d-vector +0.07769999504089356d0 -0.4 0)
                     (cl-transforms:euler->quaternion :az  (/ pi 2)))
                    :right-side)
              ))))
         ;; sets object-type to prolog variable
         (?object-type object-type))

    (cpl:with-retry-counters ((going-retry 1000))
      ;; TODO if it takes to long to go through the whole list implement a stop
      (cpl:with-failure-handling
          (((or common-fail:low-level-failure
                cl::simple-error
                cl::simple-type-error) (e)
             (roslisp:ros-warn (grasp-object fail)
                               "~%Failed with given msgs ~a~%" e)
             ;; get rid of head of nav-pose by setting only the rest
             (unless (eq nil (cdr nav-pose))
               (setf nav-pose (cdr nav-pose))

               (cpl:do-retry going-retry
                 (roslisp:ros-warn (grasp-object fail)
                                   "~%Failed with given msgs ~a~%" e)
                 (cpl:retry)))
             (roslisp:ros-warn (grasp-object fail)
                               "~%No more retries~%")))
        ;; park-robot (percieve pose)
        (btr-utils:park-robot)
        (let* ((?nav-pose
                 (car (car nav-pose)))
               (?object-pose
                 (cl-transforms-stamped:pose->pose-stamped
                  "map" 0 (btr:object-pose object-name))))
          ;; perform a action type going to first pose
          (exe:perform
           (desig:an action
                     (type going)
                     (target (desig:a location (pose ?nav-pose)))))

          ;;perform a action type looking towards the object
          (exe:perform
           (desig:an action
                     (type looking)
                     (target (desig:a location (pose ?object-pose))))))
        
        (setf object-design (exe:perform (desig:a motion
                                       (type detecting)
                                       (object (desig:an object
                                                         (type ?object-type))))))

        (let* ((?object-desig
                 (exe:perform (desig:a motion
                                       (type detecting)
                                       (object (desig:an object
                                                         (type ?object-type))))))
               (?grasp (caadar nav-pose)))

          ;; perform a action type picking-up with given grasp and object
          (exe:perform (desig:an action
                                 (type picking-up)
                                 (arm :left)
                                 (grasp ?grasp)
                                 (object ?object-desig)))
          
          ;;(place-object ?object-desig)
          (print ?grasp))))))




(defun place-object(object-desig turn 3d-vector)
 ;; (let ((3d-vector (cl-transforms:make-3d-vector -1.5 -0.1 0.75)))
  (let (
        (place-vector 3d-vector)
    (?object-desig object-desig)) 
    (let ((?placing
                (cl-transforms-stamped:make-pose-stamped
                "map" 0
                place-vector
                (if turn
                (cl-transforms:q*    
                (cl-transforms:make-quaternion 0.0d0 0.0d0 0.7071067690849304d0 0.7071067690849304d0)
                (cl-transforms:euler->quaternion :ax 0 :ay 0 :az  (-(/ pi 2))))
                (cl-transforms:make-quaternion 0.0d0 0.0d0 0.7071067690849304d0 0.7071067690849304d0)))))

  "place the object on a predefined place on the counter"
        (exe:perform (desig:an action
                               (type placing)
                               (arm :left)      
                               (object ?object-desig)
                               (target (desig:a location (pose ?placing))))))))

(defun move-place-right (stamped-pose)
  (let ((nav-pose (cl-transforms-stamped:make-pose-stamped
                    "map" 0
                    (cl-transforms-stamped:make-3d-vector
                     (+ (first (cram-tf:3d-vector->list
                                stamped-pose))
                        0.8)
                     (+ (second (cram-tf:3d-vector->list
                                 stamped-pose))
                        0.08)
                    0)
                    (cl-transforms:euler->quaternion :az pi))))
   nav-pose))

(defun move-place-left (3d-vector)
    (let ((nav-pose (cl-transforms-stamped:make-pose-stamped
                    "map" 0
                    (cl-transforms-stamped:make-3d-vector
                     (+ (first (cram-tf:3d-vector->list
                                3d-vector))
                        0.08)
                     (- (second (cram-tf:3d-vector->list
                                 3d-vector))
                        0.8)
                    0)
                    (cl-transforms:euler->quaternion :az (/ pi 2)))))
    nav-pose))


(defun create-pose-with-3d-vector (3d-vector)
      (let ((nav-pose (cl-transforms-stamped:make-pose-stamped
                    "map" 0
                    (cl-transforms-stamped:make-3d-vector
                     (first (cram-tf:3d-vector->list
                                3d-vector))
                     (second (cram-tf:3d-vector->list
                                 3d-vector))
                    (third (cram-tf:3d-vector->list
                                 3d-vector)))
                    (cl-transforms:euler->quaternion :az (/ pi 2)))))
    nav-pose))
  



(defun create-offset-list ()
  (loop for a from -0.05 to 0.05 by 0.01 collect a))

(defun create-3d-vector-list (3d-vector)
  ;;(mapcar (lambda (a)
  ;;          (cl-transforms-stamped:make-pose-stamped "map" 0 a
  ;;                                                   (cl-transforms:euler->quaternion :az (/ pi 2))))
          (mapcar (lambda (a)
  (cl-transforms-stamped:make-3d-vector
    (first (cram-tf:3d-vector->list 3d-vector))
     (+ (second (cram-tf:3d-vector->list 3d-vector)) a)
     0)) (create-offset-list)))
;;)

(defun move (nav-pose 3d-vector)
    (cpl:with-retry-counters ((going-retry 10))
      ;; TODO if it takes to long to go through the whole list implement a stop
      (cpl:with-failure-handling
          (((or common-fail:low-level-failure
                cl::simple-error
                cl::simple-type-error) (e)
             (roslisp:ros-warn (grasp-object fail)
                               "~%Failed with given msgs ~a~%" e)
             ;; get rid of head of nav-pose by setting only the rest
             (unless (eq nil (cdr nav-pose))
               (setf nav-pose (cdr nav-pose))

               (cpl:do-retry going-retry
                 (roslisp:ros-warn (grasp-object fail)
                                   "~%Failed with given msgs ~a~%" e)
                 (cpl:retry)))
             (roslisp:ros-warn (grasp-object fail)
                               "~%No more retries~%")))
        (let ((?nav-pose (demo::move-place-right (car nav-pose)))
              (?object-pose (demo::create-pose-with-3d-vector 3d-vector)))
          (exe:perform (desig:an action
                       (type going)
                       (target (desig:a location (pose ?nav-pose)))))
          
          (exe:perform (desig:an action
                       (type looking)
                       (target (desig:a location (pose ?object-pose)))))
          (let ((?object-desig
                 (exe:perform (desig:a motion
                                       (type detecting)
                                       (object (desig:an object
                                                         (type :pringles)))))))
              (exe:perform (desig:an action
                                (type picking-up)
                                (arm :left)
                                (grasp :right-side)
                                (object ?object-desig)))
            ?object-desig)))))


(defun test-reaching  (object-desig)
  (let ((?object-desig object-desig)
        (?test (cl-transforms-stamped:make-pose-stamped
                "map" 0
                (cl-transforms:make-3d-vector -1.5 -0.1 0.75)
                (cl-transforms:euler->quaternion :az pi))))
        ;;(let (
        ;;(?left-reach-poses (man-int:get-traj-poses-by-label (hsrb-get-traj :left ?object-desig ?test) :reaching)))
        ;;(let 
        ;;((?goal `(cpoe:tool-frames-at ?left-reach-poses)))
    
    (exe:perform (desig:an action
                          (type hsrb-reach)
                          (arm :left)
                          (object ?object-desig)
                          (target (desig:a location (pose ?test)))
    
    ))))

(defun test-retracting (?left-retract-poses)
  (let ((?test (cl-transforms-stamped:make-pose-stamped
                "map" 0
                (cl-transforms:make-3d-vector -1.5 -0.1 0.75)
                (cl-transforms:euler->quaternion :az pi)))
        (?goal `(cpoe:tool-frames-at ?left-retract-poses)))
    
    (exe:perform (desig:an action
                          (type retracting)
                          (location (desig:a location (pose ?test)))
                          (left-poses ?left-retract-poses)
                          (goal ?goal))
  )))

(defun hsrb-get-traj (?arm object-desig location)
  (roslisp:ros-warn (traj) "object: ~a" object-desig)
  (man-int:get-action-trajectory :picking-up
                       ?arm :right-side location nil)
  ;;(man-int:get-traj-poses-by-label ?left-trajectory type)
  )
