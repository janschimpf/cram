(in-package :cashier)
;;@author Jan Schimpf

(defparameter *place-position-refill*
  (cl-transforms-stamped:make-pose-stamped
   "map"
   0.0
   (cl-transforms:make-3d-vector -1.90 1.4 0.68)
   (cl-transforms:euler->quaternion :ax 0 :ay 0 :az 0)))


(defparameter *scan-area-refill*
  (cl-transforms-stamped:make-pose-stamped
   "map" 0.0
   (cl-transforms:make-3d-vector -1.91 1.4 0.65)
   (cl-transforms:make-quaternion 0 0 0 1)))
   
   
(defparameter *success-area-1-refill*
   (cl-transforms-stamped:make-pose-stamped
   "map" 0.0
   (cl-transforms:make-3d-vector -2 0.8 0.68)
   (cl-transforms:euler->quaternion :ax 0 :ay 0 :az 0)))

(defparameter *success-area-2-refill*
  (cl-transforms-stamped:make-pose-stamped
   "map" 0.0
   (cl-transforms:make-3d-vector -2 0.4 0.68)
   (cl-transforms:euler->quaternion :ax 0 :ay 0 :az (* 1.5 pi))))

(defparameter *unsuccessful-area-1-refill*
  (cl-transforms-stamped:make-pose-stamped
   "map" 0.0
   (cl-transforms:make-3d-vector 0.6 -1.6 0.4)
   (cl-transforms:euler->quaternion :ax 0 :ay 0 :az (* 0.5 pi))))

(defparameter *unsuccessful-area-2-refill*
  (cl-transforms-stamped:make-pose-stamped
   "map" 0.0
   (cl-transforms:make-3d-vector 0.2 -1.6 0.4)
   (cl-transforms:euler->quaternion :ax 0 :ay 0 :az 0)))

(defparameter *spawn-area-1-refill*
  (cl-transforms-stamped:make-pose-stamped
   "map" 0.0
   (cl-transforms:make-3d-vector -2 2.2 0.75)
   (cl-transforms:euler->quaternion :ax 0 :ay 0 :az 0)))

(defparameter *spawn-area-2-refill*
  (cl-transforms-stamped:make-pose-stamped
   "map" 0.0
   (cl-transforms:make-3d-vector -2 2.6 0.75)
   (cl-transforms:euler->quaternion :ax 0 :ay 0 :az (* 0.5 pi))))

   
(defparameter breakfast-cereal-1-refill
  (list 'breakfast-cereal-1 :breakfast-cereal
        '((-2 2 0.75) (0 0 0 1))
        '(0.08 0.03 0.12)
        (list :right-side :left-side) (list nil) :back))

(defparameter small-book-refill
  (list 'small-book-1 :small-book
        '((-2 2 0.75)(0 0 0 1))
        '(0.04 0.07 0.11)
        (list :right :left) (list nil) :top))


(defparameter cup-refill
  (list 'cup-1 :cup
        '((-2 2 0.75)(0 0 0 1))
        '(0.03 0.03 0.10)
        (list nil) (list :front :back :left-side :right-side) nil))

                 

(defun pr2-refill-demo ()
  (setf *scan-area* *scan-area-refill*)
  (let* ((spawn-refill (list breakfast-cereal-1-refill
                             cup-refill
                             small-book-refill)))
  (let* ((spawn-poses (area->pose-stamped-list
                              (list *spawn-area-1-refill*
                                    *spawn-area-2-refill*)
                              0.15)))
  (print spawn-poses)                           
  (loop for object in spawn-refill
        do
           (spawn-object-on-counter-general object
                                            (cram-tf:3d-vector->list
                                             (cl-tf2:origin (car spawn-poses))))
                                                            
           (setf spawn-poses (cdr
                              spawn-poses))
           (setf *goal-list* (append (list (list (first object)
                                                 (car (last object))))
                                     *goal-list*))))
    
  (setf *success-poses-list* (area->pose-stamped-list
                              (list *success-area-1-refill*
                                    *success-area-2-refill*)
                              0.15))
    
  (setf *unsuccessful-poses-list* (area->pose-stamped-list
                                   (list *unsuccessful-area-1-refill*
                                         *unsuccessful-area-2-refill*)
                                                       0.15))
  (btr:simulate btr:*current-bullet-world* 100)
  (urdf-proj:with-simulated-robot
  (loop for object in spawn-refill
        do
  (let* ((?object-name (first object))
        (?object-type (second object))
        (?object-size (fourth object))
         (?goal-side (if (equal :small-book ?object-type)
                         (car (last object))
                         nil))
         (?arms (if (equal :cup ?object-type)
                    (list :right)
                    (list :left)))
        (?non-scanable (sixth object))
        (?non-graspable (fifth object))
        (?search-area (list *spawn-area-1-refill* *spawn-area-2-refill*))
        (?scan-pose *place-position-refill*)
        (?after-poses (list
                        (first *success-poses-list*)
                        (first *unsuccessful-poses-list*)))
        (?object (desig:an object
                           (:type ?object-type)
                           (:size ?object-size)
                           (:non-scanable ?non-scanable)
                           (:non-graspable ?non-graspable)))
         
         (cashier (exe:perform (desig:an action
                           (:type cashier)
                           (:arm ?arms)
                           (:goal-side ?goal-side)
                           (:search-area ?search-area)
                           (:scan-pose ?scan-pose)
                           (:after-poses ?after-poses)
                           (:object ?object)))))
    (if  (desig:desig-prop-value cashier :goal-side)
        (setf *success-poses-list* (cdr *success-poses-list*))
        (setf *unsuccessful-poses-list* (cdr *unsuccessful-poses-list*)))
    cashier)))))


(defun move-setup ()  
  (spawn-refill)
  (urdf-proj:with-simulated-robot

    (let*((?loc *place-position-refill*)
        (?arm (list :left)))
        
    (move (desig:reference (desig:a location (locate ?loc) (arm ?arm)))))))
    
(defun spawn-refill ()
        (btr-utils:spawn-object
         'small-book-1
         :small-book
         :color '(0.2 0.2 0.2)
         :pose (list (list -1.9 1.4 0.75) '(0 0 0 1)))
   (btr:simulate btr:*current-bullet-world* 10))

(defun refill-move (goal-side)
   (urdf-proj:with-simulated-robot

   (let* ((?type :small-book)
          (?loc *place-position-refill*)
          (?size '(0.04 0.07 0.14))
          (?arm '(:right))
          (?perceived-object (perceive-object ?loc ?type))
          (?base-sides (set-sides ?perceived-object ?size)))
     (change-side-retail ?type ?loc ?base-sides goal-side
                            '(:front :back) '(nil) ?arm ?size))))

(defun change-side-retail (?object-type ?scan-pose ?b-sides ?sides-to-check
                              ?non-graspable ?non-scanable ?arm ?object-size)
  
  (let* ((?perceived-object (perceive-object ?scan-pose ?object-type))
         (?sides-located (side-location ?perceived-object ?b-sides))
         (?check-side ?sides-to-check)
         (?goal (list :left-rotation :front-turn)))
    
    (exe:perform
     (desig:an action
               (:type :changing-side)
               (:object ?perceived-object)
               (:object-type ?object-type)
               (:scan-pose ?scan-pose)
               (:arm ?arm)
               (:base-sides ?b-sides)
               (:size ?object-size)
               (:non-scanable ?non-scanable)
               (:non-graspable ?non-graspable)
               (:change-to-side ?check-side)
               (:plan ?goal)))))
    
  
  
(defun spawn-vector-list (3d-pose1 3d-pose2 number-of-places)
  (let* ((2d-vector (list (- (first 3d-pose2)
                             (first 3d-pose1))
                          
                          (- (second 3d-pose2)
                             (second 3d-pose1)))))
    
         (loop for a from 1 to number-of-places
               collect (list (+ (first 3d-pose1)
                                (* (first 2d-vector) (- a 1)))
                             (+ (second 3d-pose1)
                                (* (second 2d-vector) (- a 1)))
                             (third 3d-pose1)
                             ))))
    
(defun vector-addition (v1 v2)
  (cl-tf2:make-3d-vector
   (+ (cl-tf2:x v1) (cl-tf2:x v2))
   (+ (cl-tf2:y v1) (cl-tf2:y v2))
   (+ (cl-tf2:z v1) (cl-tf2:z v2))))
