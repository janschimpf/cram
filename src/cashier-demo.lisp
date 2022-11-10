(in-package :cashier)

(defparameter *look-nav-pose*
  (cl-transforms-stamped:make-pose-stamped
   "map"
   0.0
   (cl-transforms:make-3d-vector -1.2 2.4 0)
   (cl-transforms:euler->quaternion :ax 0 :ay 0 :az pi)))

(defparameter *grasp-spawn*
  (cl-transforms-stamped:make-pose-stamped
   "map" 0.0
   (cl-transforms:make-3d-vector -2 2 0.75)
   (cl-transforms:euler->quaternion :ax 0 :ay 0 :az pi)))

(defparameter *place-nav-pose*
  (cl-transforms-stamped:make-pose-stamped
   "map"
   0.0
   (cl-transforms:make-3d-vector -1.2 1.4 0)
   (cl-transforms:euler->quaternion :ax 0 :ay 0 :az pi)))

(defparameter *spawn-area*
  (cl-transforms-stamped:make-pose-stamped
   "map" 0.0
   (cl-transforms:make-3d-vector -2 2 0.75)
   (cl-transforms:euler->quaternion :ax 0 :ay 0 :az pi)))

(defparameter *place-pose*
  (cl-transforms-stamped:make-pose-stamped
   "map" 0.0
   (cl-transforms:make-3d-vector -2 1.2 0.75)
   (cl-transforms:make-quaternion 0 0 0 1)))

(defparameter *after-scan-nav-pose*
  (cl-transforms-stamped:make-pose-stamped
   "map" 0.0
   (cl-transforms:make-3d-vector -1.2 0.8 0)
   (cl-transforms:euler->quaternion :ax 0 :ay 0 :az pi)))

(defun place-after-scan-positive (object-name)
   (cl-transforms-stamped:make-pose-stamped
   "map" 0.0
   (cl-transforms:make-3d-vector -2 0.8 0.75)
   (cl-tf2:orientation (btr:object-pose object-name))))

(defun move (?navigation-goal)
      (exe:perform (desig:an action
                             (type parking-arms)))
      (exe:perform (desig:a motion
                            (type going)
                            (pose ?navigation-goal)))
      (coe:on-event (make-instance 'cpoe:robot-state-changed)))
  

(defun grasp-object (?object-type
                     ?arm
                     ?look
                     ?left-grasp)
  
  (exe:perform (desig:a motion
                          (type looking)
                          (pose ?look)))
  
  (let* ((?perceived-object-desig
           (exe:perform (desig:an action
                                  (type detecting)
                                  (object (desig:an object (type ?object-type))))))
         
         (pick-up (desig:an action
                            (type picking-up)
                            (arm ?arm)
                            (left-grasp ?left-grasp)
                            (object ?perceived-object-desig
                                    ))))
    ;;(proj-reasoning:check-picking-up-collisions pick-up)
    (exe:perform pick-up))
  (exe:perform (desig:an action
                             (type parking-arms))))


(defun place-object (?target-pose ?arm
                     &key ?left-grasp ?right-grasp ?object-placed-on)

  (exe:perform (desig:a motion
                        (type looking)
                        (pose ?target-pose)))
  (exe:perform (desig:an action
                         (type placing)
                         (arm ?arm)
                         (desig:when ?right-grasp
                           (right-grasp ?right-grasp))
                         (desig:when ?left-grasp
                           (left-grasp ?left-grasp))
                         (target (desig:a location
                                          (desig:when ?object-placed-on
                                            (on ?object-placed-on))
                                          (pose ?target-pose)))))
  (exe:perform (desig:an action
                             (type parking-arms))))



(defun pr2-cashier-demo ()

  (loop for object in spawn-objects-list
        do
           
  (spawn-object-on-counter-general object)
  (urdf-proj:with-simulated-robot
  (let ((?object-name (first object))
        (?object-type (second object))
        (?object-size (fourth object))
        (?goal-side (car (last object)))
        (?arms (list :left))
        (?non-scanable (list nil))
        (?non-graspable (list :left :right)))

    (exe:perform (desig:an action
                           (type cashier)
                           (object-list object)
                           (arm ?arms)
                           (non-scanable ?non-scanable)
                           (non-graspable ?non-graspable)
                           (object-name ?object-name)
                           (object-type ?object-type)
                           (object-size ?object-size)
                           (goal-side ?goal-side)))
  (btr-utils:kill-object ?object-name))))
  (print *sides-log*)
  (setf *sides-log* nil))


                


