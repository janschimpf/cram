(in-package :cashier)
;;@author Jan Schimpf

(defparameter *place-position-kitchen*
  (cl-transforms-stamped:make-pose-stamped
   "map"
   0.0
   (cl-transforms:make-3d-vector 1.4 0.5 0.85)
   (cl-transforms:euler->quaternion :ax 0 :ay 0 :az pi)))


(defparameter *scan-area-kitchen*
  (cl-transforms-stamped:make-pose-stamped
   "map" 0.0
   (cl-transforms:make-3d-vector 1.4 0.5 0.85)
   (cl-transforms:euler->quaternion :ax 0 :ay 0 :az pi)))
   
   
(defparameter *success-area-1-kitchen*
   (cl-transforms-stamped:make-pose-stamped
   "map" 0.0
   (cl-transforms:make-3d-vector -2 -1.15 0.7)
   (cl-transforms:euler->quaternion :ax 0 :ay 0 :az (* 0.5 pi))))

(defparameter *success-area-2-kitchen*
  (cl-transforms-stamped:make-pose-stamped
   "map" 0.0
   (cl-transforms:make-3d-vector -2.5 -1.15 0.7)
   (cl-transforms:euler->quaternion :ax 0 :ay 0 :az (* 1.5 pi))))

(defparameter *unsuccessful-area-1-kitchen*
  (cl-transforms-stamped:make-pose-stamped
   "map" 0.0
   (cl-transforms:make-3d-vector -3.2 0 0.8)
   (cl-transforms:euler->quaternion :ax 0 :ay 0 :az 0)))

(defparameter *unsuccessful-area-2-kitchen*
  (cl-transforms-stamped:make-pose-stamped
   "map" 0.0
   (cl-transforms:make-3d-vector -3.2 0.5 0.8)
   (cl-transforms:euler->quaternion :ax 0 :ay 0 :az 0)))

(defparameter *spawn-area-1-kitchen*
  (cl-transforms-stamped:make-pose-stamped
   "map" 0.0
   (cl-transforms:make-3d-vector -1.2 2.2 0.95)
   (cl-transforms:euler->quaternion :ax 0 :ay 0 :az pi)))

(defparameter *spawn-area-2-kitchen*
  (cl-transforms-stamped:make-pose-stamped
   "map" 0.0
   (cl-transforms:make-3d-vector -1.2 2.6 0.95)
   (cl-transforms:euler->quaternion :ax 0 :ay 0 :az (* 0.5 pi))))


(defparameter small-book-kitchen
  (list 'small-book-1 :small-book
        '((-2 2 0.75)(0 0 0 1))
        '(0.04 0.07 0.11)
        (list :front :back) (list nil) nil))

(defparameter bottle-kitchen
  (list 'bottle-1 :bottle
        '((-2 2 0.75)(0 0 0 1))
        '(0.05 0.05 0.10)
        (list nil)
        (list :left :right :front :back :top) :bottom))
                 

(defun pr2-kitchen-demo ()
  (setf *scan-area* *scan-area-kitchen*)
  (let* ((spawn-kitchen (list small-book-kitchen
                              bottle-kitchen)))
    
  (let* ((spawn-poses (area->pose-stamped-list
                              (list *spawn-area-1-kitchen*
                                    *spawn-area-2-kitchen*)
                              0.15)))
  (print spawn-poses)                           
  (loop for object in spawn-kitchen
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
                              (list *success-area-1-kitchen*
                                    *success-area-2-kitchen*)
                              0.15))
    
  (setf *unsuccessful-poses-list* (area->pose-stamped-list
                                   (list *unsuccessful-area-1-kitchen*
                                         *unsuccessful-area-2-kitchen*)
                                                       0.15))
  (btr:simulate btr:*current-bullet-world* 100)
  (urdf-proj:with-simulated-robot
  (loop for object in spawn-kitchen
        do
  (let* ((?object-name (first object))
        (?object-type (second object))
        (?object-size (fourth object))
         (?goal-side nil)
         (?arms (if (equal :small-book ?object-type)
                    (list :right)
                    (list :left)))
        (?non-scanable (fifth object))
        (?non-graspable (sixth object))
        (?search-area (list *spawn-area-1-kitchen* *spawn-area-2-kitchen*))
        (?scan-pose *place-position-kitchen*)
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
    (print cashier)
    ))
    )))

