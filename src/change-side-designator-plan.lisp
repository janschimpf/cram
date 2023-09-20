(in-package :cashier)
;;@author Jan Schimpf

(defun change-side-plan (&key
                           ((:object-type ?object-type))
                           ((:object-size ?object-size))
                           ((:arm ?arm))
                           ((:base-sides ?b-sides))
                           ((:scan-pose ?scan-pose))
                           ((:non-graspable ?non-graspable))
                           ((:side-goal ?side-goal))
                           ((:plan ?plan))
                         &allow-other-keys)
  (declare (type list ?plan ?object-size ?b-sides ?arm ?non-graspable)
           (type keyword ?object-type))
  
  (loop for move in (remove nil ?plan)
        do
           (let* ((adjusted-pose-vector (vector-change
                            (cram-tf:3d-vector->list (cl-tf2:origin ?scan-pose))
                             ?side-goal ?object-size))
                  (perceived-object (perceive-object ?scan-pose ?object-type))
                  (located-sides (side-location perceived-object ?b-sides))
                  (orientation-change (orientation-change-with-move move located-sides))
                  (new-orientation (change-orientation-and-transform
                                    (man-int:get-object-pose perceived-object)
                                    orientation-change (cram-tf:robot-current-transform)))
                  (?grasp (which-sides located-sides move ?non-graspable))
                  (target (place-pose-stability-adjustment
                            adjusted-pose-vector new-orientation ?object-type)))
             (execute-change-side ?object-type ?arm ?grasp target ?scan-pose))))

(defun change-orientation-and-transform (pose orientation transform)
  (let* ((origin (cl-tf2:origin pose))
         (pose-orientation (cl-tf2:orientation pose))
         (new-orientation (cl-tf2:q*
                           pose-orientation
                           orientation))
         (new-pose (cl-transforms-stamped:transform transform
                                                    (cl-tf2:make-pose origin new-orientation))))
    (cl-tf2:orientation new-pose)))


(defun place-pose-stability-adjustment (origin-list orientation object-type)
  (let ((offset 0))
    (cpl:with-retry-counters ((pose-adjustment-retries 15))
    (cpl:with-failure-handling 
        ((common-fail:high-level-failure (e)
       (roslisp:ros-warn (cashier-demo) "Falure happend: ~a~% adjusting place postion" e)
       (cpl:do-retry pose-adjustment-retries
         (setf offset (- offset 0.001))
         (cpl:retry))
       (cpl:fail 'common-fail:high-level-failure)))
      (let* ((target-pose
               (cl-tf2:make-pose-stamped "map" 0 
                                         (cram-tf:list->3d-vector
                                          (vector-offset origin-list (list 0 0 offset)))
                                         orientation )))
        
         (let ((?target-pose-test (btr:ensure-pose target-pose)))
           (print "test return before target-pose")
           (shortcut-pose-stability (object-desig-shortcut object-type)
                                    (a location (pose ?target-pose-test)))
           ?target-pose-test))))))

(defun shortcut-pose-stability (object-desig placing-location)
  (proj-reasoning:check-placing-pose-stability
   object-desig
   placing-location))

(defun execute-change-side (?object-typ ?arm ?grasp target-pose ?scan-pose)
  (multiple-value-bind (?perceived-object)
      (perceive-object ?scan-pose ?object-typ)
    
    (let* ((?current-grasp
           (grasp-object-with-handling
             ?arm
             ?grasp
             ?perceived-object)))
      
      (move (desig:reference (desig:a location (locate ?scan-pose) (arm (first ?arm)))))
    (cpl:with-retry-counters ((place-retry 3))
    (cpl:with-failure-handling
        ((common-fail:manipulation-low-level-failure (e)
           (roslisp:ros-warn (cashier-demo) "failure happened: ~a~% changing grasp" e)
           (cpl:do-retry place-retry
           (cpl:retry))
           (cpl:fail 'high-level-grasp-failure)))
      
      (place-object-with-handling
       target-pose
       ?arm
       ?current-grasp))
    (btr:simulate btr:*current-bullet-world* 100)))))


;;; ===== pose changes for placing the object with a different orientation / side ======


(defun orientation-change-with-move (move located-sides)
  (print move)
  (let* ((90-turn (/ pi 2))
         (turn
           (cond
             ((or (string-equal "front-turn" move)
                  (string-equal "left-turn" move)
                  (string-equal "left-rotation" move))
              (- 90-turn))
             ((or (string-equal "back-turn" move)
                  (string-equal "right-turn" move)
                  (string-equal "right-rotation" move))
              90-turn)
             ((string-equal "flip" move)
              pi)
             (t (print move))))
         (result (or (if (or (string-equal "front-turn" move)
                             (string-equal "back-turn" move))
                         (finding-axis-front/back located-sides turn move))
                     (if (or (string-equal "left-turn" move)
                             (string-equal "right-turn" move)
                             (string-equal "flip" move))
                         (finding-axis-left/right located-sides turn move))
                     (if (or (string-equal "left-rotation" move)
                             (string-equal "right-rotation" move))
                         (finding-axis-left/right-rotation located-sides turn move)))))
                 
    (cl-tf:euler->quaternion
     :ax (first result)
     :ay (second result)
     :az (third result))))

(defun finding-axis-front/back (located-sides turn move)
  (let* ((axis-bottom (axis-short (first located-sides)))
         (axis-right (axis-short (second located-sides)))
         (axis-front (axis-short (third located-sides)))
         (result (cond
                   ((string-equal "-Y" axis-right)
                    (list 0 (- turn) 0))
                   ((string-equal "Y" axis-right)
                    (list 0 (- turn) 0))
                   ((string-equal "X" axis-right)
                    (list (- turn) 0 0))
                   ((string-equal "-X" axis-right)
                    (list turn 0 0))
                   ((string-equal "Z" axis-right)
                    (list 0 0 turn))
                   ((string-equal "-Z" axis-right)
                    (list 0 0 (- turn)))
                   (t "test"))))
    result))

(defun finding-axis-left/right (located-sides turn move)
  (let* ((axis-bottom (axis-short (first located-sides)))
         (axis-right (axis-short (second located-sides)))
         (axis-front (axis-short (third located-sides)))
         (result (cond
                   ((string-equal "-Y" axis-front)
                    (list 0 (- turn) 0))
                   ((string-equal "Y" axis-front)
                    (list 0 turn 0))
                   ((string-equal "X" axis-front)
                    (list (- turn) 0 0))
                   ((string-equal "-X" axis-front)
                    (list turn 0 0))
                   ((string-equal "Z" axis-front)
                    (list 0 0 turn))
                   ((string-equal "-Z" axis-front)
                    (list 0 0 (- turn)))
                   (t "test"))))
    result))

(defun finding-axis-left/right-rotation (located-sides turn move)
    (let* ((axis-bottom (axis-short (first located-sides)))
           (axis-right (axis-short (second located-sides)))
           (axis-front (axis-short (third located-sides)))
           (result (cond
                   ((string-equal "-Y" axis-bottom)
                    (list 0 0 turn))
                   ((string-equal "Y" axis-bottom)
                    (list 0 0 (- turn)))
                   ((string-equal "X" axis-bottom)
                    (list (- turn) 0 0))
                   ((string-equal "-X" axis-bottom)
                    (list turn 0 0))
                   ((string-equal "Z" axis-bottom)
                    (list 0 0 turn))
                   ((string-equal "-Z" axis-bottom)
                    (list 0 0 (- turn)))
                   (t "test"))))
    result))
    
;; takes care of the x-y-z pose when placing the object with the goal of not placing object
;; inside the table
(defun vector-change (place-vector bottom-side object-size)
  (let ((object-depth (first object-size))
        (object-width (second object-size))
        (object-height (third object-size)))
    (print place-vector)
    (print bottom-side)
    (print object-size)
  (cond
    ((equal :right-side bottom-side)
     (vector-offset place-vector
                    (list 0 0 (+ object-width 0.03))))
    ((equal :left-side bottom-side)
     (vector-offset place-vector
                    (list 0 0 (+ object-width 0.03))))
    ((equal :top bottom-side)
     (vector-offset place-vector
                    (list 0 0 (+ object-height 0.03))))
    ((equal :bottom bottom-side)
     (vector-offset place-vector
                    (list 0 0 (+ object-height 0.02))))
    ((equal :front bottom-side)
     (vector-offset place-vector
                    (list 0 0 object-depth)))
    ((equal :back bottom-side)
     (vector-offset place-vector
                    (list 0 0 object-depth)))
    (t (print "not sure how we got here but something is wrong, vector-change")))))


(defun object-desig-shortcut (?object-type)
  (desig:an object (type ?object-type)))


(defun remove-element-from-list (list element)
  (remove nil (mapcar (lambda (x) (if (equal x element)
                          nil
                          x))
                          list)))
  

(defun which-sides (located-sides move non-graspable)
  (let* ((top (opposite-short (first located-sides)))
         (front (third located-sides))
         (right (second located-sides))
         (left (opposite-short right))
         (grasp-list
           (cond
             ((string-equal "right-turn" move)
              (list front top right))
             ((string-equal "left-turn" move)
              (list front top left))
             ((string-equal "front-turn" move)
              (list top left right))
             ((string-equal "back-turn" move)
              (list front left right ))
             ((string-equal "flip" move)
              (list right left front))
             ((string-equal "left-rotation" move)
              (list right front top))
             ((string-equal "right-rotation" move)
              (list left front top))
             (t nil)))
         (non-grasp-removed (remove nil (mapcar (lambda (x)
                                      (if (member x non-graspable)
                                      nil
                                      x))
                                    grasp-list))))
  non-grasp-removed))

(define-condition high-level-place-failure (cpl:simple-plan-failure)
  ((description :initarg :description
                :initform "Failed to place the object and failed to adjust to that fail"
                :reader error-description))
  (:documentation "Failure thrown by high level place function")
  (:report (lambda (condition stream)
             (format stream (error-description condition)))))


(define-condition high-level-grasp-failure (cpl:simple-plan-failure)
  ((description :initarg :description
                :initform "Failed to grasp the object and failed to adjust to that fail"
                :reader error-description))
  (:documentation "Failure thrown by high level grasp function")
  (:report (lambda (condition stream)
             (format stream (error-description condition)))))




