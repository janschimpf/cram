
(in-package :cashier)

(defparameter *look-nav-pose*
  (cl-transforms-stamped:make-pose-stamped
   "map"
   0.0
   (cl-transforms:make-3d-vector -1.2 2.4 0)
   (cl-transforms:euler->quaternion :ax 0 :ay 0 :az 0)))

(defparameter *second-look-nav-pose*
  (cl-transforms-stamped:make-pose-stamped
   "map"
   0.0
   (cl-transforms:make-3d-vector -1.2 2.6 0)
   (cl-transforms:euler->quaternion :ax 0 :ay 0 :az pi)))

(defparameter *grasp-spawn*
  (cl-transforms-stamped:make-pose-stamped
   "map" 0.0
   (cl-transforms:make-3d-vector -2 2 0.75)
   (cl-transforms:euler->quaternion :ax 0 :ay 0 :az 0)))

(defparameter *spawn-area*
  (cl-transforms-stamped:make-pose-stamped
   "map" 0.0
   (cl-transforms:make-3d-vector -2 2 0.75)
   (cl-transforms:euler->quaternion :ax 0 :ay 0 :az 0)))

(defparameter *second-spawn-area*
  (cl-transforms-stamped:make-pose-stamped
   "map" 0.0
   (cl-transforms:make-3d-vector -2 2.4 0.75)
   (cl-transforms:euler->quaternion :ax 0 :ay 0 :az 0)))


(defparameter *place-nav-pose*
  (cl-transforms-stamped:make-pose-stamped
   "map"
   0.0
   (cl-transforms:make-3d-vector -1.2 1.2 0)
   (cl-transforms:euler->quaternion :ax 0 :ay 0 :az pi)))


(defparameter *place-position*
  (cl-transforms-stamped:make-pose-stamped
   "map"
   0.0
   (cl-transforms:make-3d-vector -2 1.2 0.75)
   (cl-transforms:euler->quaternion :ax 0 :ay 0 :az 0)))

(defparameter *after-scan-nav-pose*
  (cl-transforms-stamped:make-pose-stamped
   "map" 0.0
   (cl-transforms:make-3d-vector -2 0.7 0)
   (cl-transforms:euler->quaternion :ax 0 :ay 0 :az 0)))


(defparameter *after-unsuccessful-scan-nav-pose*
  (cl-transforms-stamped:make-pose-stamped
   "map" 0.0
   (cl-transforms:make-3d-vector -2 0 0)
   (cl-transforms:euler->quaternion :ax 0 :ay 0 :az 0)))

(defparameter *place-pose*
   (list -2 1.2 0.75))

(defparameter spawn-point-1
  (list -2 2 0.85))

(defparameter spawn-point-2
  (list -2 2.15 0.85))

(defparameter success-point-1
  (list -2 0.8 0.75))

(defparameter success-point-2
  (list -2 0.7 0.75))

(defparameter unsuccessful-point-1
  (list -2 0.2 0.75))

(defparameter unsuccesful-point-2
  (list -2 0.1 0.75))

(defparameter *success-poses-list* nil)

(defparameter *unsuccessful-poses-list* nil)


(defun spawn-vector-list (3d-pose1 3d-pose2 number-of-places)
  (let* ((2d-vector (list (- (car 3d-pose2)
                             (car 3d-pose1))
                          
                          (- (cadr 3d-pose2)
                             (cadr 3d-pose1)))))
    
         (loop for a from 1 to number-of-places
               collect (list (+ (car 3d-pose1)
                                (* (car 2d-vector) (- a 1)))
                             (+ (cadr 3d-pose1)
                                (* (cadr 2d-vector) (- a 1)))
                             (caddr 3d-pose1)
                             ))))

(defun navigation-to-goal-for-detect (location)
  (let* ((?location-pose (cl-transforms-stamped:pose->pose-stamped "map" 0 location))
         (?location-designator (desig:a location (pose ?location-pose)))
         (to-see-designator (desig:a location (visible-for pr2)
                                     (location ?location-designator))))
              (desig:reference to-see-designator)))


(defun move (?navigation-goal)
      (exe:perform (desig:an action
                             (type parking-arms)))
      (exe:perform (desig:a motion
                            (type going)
                            (pose ?navigation-goal)))
      (coe:on-event (make-instance 'cpoe:robot-state-changed)))
  
(defun grasp-object-with-handling (?arm ?grasp ?perceived-object)
  (if (equal ?arm '(:left))
      (grasp-object ?arm ?perceived-object :?left-grasp ?grasp)
      (grasp-object ?arm ?perceived-object :?right-grasp ?grasp)
))

(defun grasp-object (?arm
                     ?perceived-object
                     &key
                       ?left-grasp
                       ?right-grasp
                     )
         
  (exe:perform (desig:an action
                         (type picking-up)
                         (arm ?arm)
                         (desig:when ?right-grasp
                           (right-grasp ?right-grasp))
                         (desig:when ?left-grasp
                           (left-grasp ?left-grasp))
                         (object ?perceived-object)))
  
  (exe:perform (desig:an action
                             (type parking-arms))))

(defun place-object-with-handling (?target-pose ?arm ?grasp)
  (if (equal ?arm '(:left))
      (place-object ?target-pose ?arm :?left-grasp ?grasp)
      (place-object ?target-pose ?arm :?right-grasp ?grasp)
))

(defun place-object (?target-pose
                     ?arm
                     &key ?left-grasp ?right-grasp)
  (exe:perform (desig:an action
                             (type parking-arms)))
  
  (exe:perform (desig:an action
                         (type placing)
                         (arm ?arm)
                         (desig:when ?right-grasp
                           (right-grasp ?right-grasp))
                         (desig:when ?left-grasp
                           (left-grasp ?left-grasp))
                         (target (desig:a location
                                          (pose ?target-pose)))))
  
  (exe:perform (desig:an action
                             (type parking-arms))))



(defun pr2-cashier-demo ()
  ;;(table-reenforcement-scan)
  ;;(table-reenforcement-spawn)
  (let* ((spawn-poses (spawn-vector-list spawn-point-1 spawn-point-2
                                         5)))
                            
  (loop for object in spawn-objects-list
        do
           (spawn-object-on-counter-general object (first spawn-poses))
           (setf spawn-poses (cdr spawn-poses)))
    
    (setf *success-poses-list* (spawn-vector-list success-point-1 success-point-2
                                                  5))
    (setf *unsuccessful-poses-list* (spawn-vector-list unsuccessful-point-1 unsuccesful-point-2
                                                       5))
  (btr:simulate btr:*current-bullet-world* 100)
  (urdf-proj:with-simulated-robot
  (loop for object in spawn-objects-list
        do
  (let ((?object-name (first object))
        (?object-type (second object))
        (?object-size (fourth object))
        (?goal-side (car (last object)))
        (?arms (list :right))
        (?non-scanable (fifth object))
        (?non-graspable (sixth object)))

    (exe:perform (desig:an action
                           (type cashier)
                           (object-list object)
                           (arm ?arms)
                           (non-scanable ?non-scanable)
                           (non-graspable ?non-graspable)
                           (object-name ?object-name)
                           (object-type ?object-type)
                           (object-size ?object-size)
                           (goal-side ?goal-side)))))))
  ;;(btr-utils:kill-object ?object-name))))
  (print *sides-log*)
  (setf *sides-log* nil))


                

(defun perceive-object (?pose-to-look-at ?object-type)
  (exe:perform (desig:a motion
                          (type looking)
                          (pose ?pose-to-look-at)))
  
 (exe:perform (desig:an action
                        (type detecting)
                        (object (desig:an object (type ?object-type))))))
  
