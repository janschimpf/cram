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


(defun test-action-desig (?object-list)
  (let ((?object-type (first ?object-list))
        (?object-name (second ?object-list))
        (?object-size (fourth ?object-list))
        (?goal-side (car (last ?object-list))))
  (desig:an action
            (type :cashier)
            (object-list ?object-list)
            (object-type ?object-name)
            (object-name ?object-type)
            (goal-side ?goal-side)
            (object-sie ?object-size))))

(defun pr2-cashier-demo ()

  (loop for object in spawn-objects-list
        do
           
  (spawn-object-on-counter-general object)
  (urdf-proj:with-simulated-robot
  (let ((?object-name (first object))
        (?object-type (second object))
        (?object-size (fourth object))
        (?goal-side (car (last object)))
        (?arms (list :left)))
    (exe:perform (desig:an action
                           (type cashier)
                           (object-list object)
                           (arm ?arms)
                           (object-name ?object-name)
                           (object-type ?object-type)
                           (object-size ?object-size)
                           (goal-side ?goal-side)))
  (btr-utils:kill-object ?object-name))))
  (print *sides-log*)
  (setf *sides-log* nil))

(defun cashier-object (&key
                         ((:object-type ?object-type))
                         ((:object-name ?object-name))
                         ((:arm ?arm))
                         ((:goal-side ?goal-side))
                         ((:sides-base ?sides-base))
                         ((:sides-transformed ?sides-transformed))
                         ((:object-size ?object-size))
                        &allow-other-keys)
  (declare (type keyword ?object-type ?goal-side)
           (type list ?sides-base ?sides-transformed ?object-size ?arm)
           (type symbol ?object-name))


  (print "sides set")
  (move *look-nav-pose*)
  (print "moved")

  (let ((?grasp (caddr (locate-sides ?sides-transformed (origin->list ?object-name)))))
  (grasp-object ?object-type
                ?arm
                *spawn-area*
                ?grasp)
  (move *place-nav-pose*)
  
  (place-object *place-pose* ?arm :?left-grasp ?grasp))
  
  ;;(if
   (exe:perform (desig:an action
                         (:type :scanning)
                         (:arm ?arm)    
                         (:object-name ?object-name)
                         (:object-type ?object-type)
                         (:object-size ?object-size)
                         (:goal-side ?goal-side)
                         (:sides-base ?sides-base)))
      ;;(sucessful-scan ?object-type ?object-name ?sides-base ?arm)

    ;;  (print "scan failed"))
  )

(defun sucessful-scan (?object-type ?object-name ?sides-base ?arm)
  (print "object was succesfully scanned")

  (let* ((grasp (cdr (locate-sides
           (transforms-map-t-side ?object-name ?sides-base)
           (cram-tf:3d-vector->list
            (cl-tf2:origin (btr:object-pose ?object-name)))))))
    (cpl:with-retry-counters ((grasp-retry 2))
    (cpl:with-failure-handling
        ((common-fail:gripper-closed-completely (e) 
           (roslisp:ros-warn (cashier-demo) "failure happened: ~a~% changing grasp" e)
           (cpl:do-retry grasp-retry
             (setf grasp (cdr grasp))
             (cpl:retry)))
         (desig:designator-error (e)
           (roslisp:ros-warn (cashier-demo) "designator-reference-failure ~a~%" e)
           (cpl:do-retry grasp-retry
           (setf grasp (cdr grasp))
           (cpl:retry))
           (cpl:fail 'common-fail:high-level-failure)))
                             
  (grasp-object ?object-type
                ?arm *place-pose* (first grasp))))

  (move *after-scan-nav-pose*)
    (place-object (place-after-scan-positive ?object-name)
                  ?arm
                  :?left-grasp
                  (first grasp))))


