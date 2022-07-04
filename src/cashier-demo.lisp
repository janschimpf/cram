(in-package :cashier)

(defparameter *look-nav-pose*
  (cl-transforms-stamped:make-pose-stamped
   "map"
   0.0
   (cl-transforms:make-3d-vector -1.1 2 0)
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
   (cl-transforms:make-3d-vector -1.2 1.3 0)
   (cl-transforms:euler->quaternion :ax 0 :ay 0 :az pi)))

(defparameter *spawn-area*
  (cl-transforms-stamped:make-pose-stamped
   "map" 0.0
   (cl-transforms:make-3d-vector -2 2 0.75)
   (cl-transforms:euler->quaternion :ax 0 :ay 0 :az pi)))

(defparameter *place-pose*
  (cl-transforms-stamped:make-pose-stamped
   "map" 0.0
   (cl-transforms:make-3d-vector -2 1.3 0.75)
   (cl-transforms:make-quaternion 0 0 0 1)))

(defparameter *scan-rotation*
  (cl-transforms:make-quaternion 0 0 0 1))

(defparameter *sides* nil)

(defun move (?navigation-goal)
      (exe:perform (desig:an action
                             (type parking-arms)))
      (exe:perform (desig:a motion
                            (type going)
                            (pose ?navigation-goal)))
      (coe:on-event (make-instance 'cpoe:robot-state-changed)))
  

(defun grasp-object (?object-type ?arm ?grasp ?look)
  (print ?look)
  (exe:perform (desig:a motion
                          (type looking)
                          (pose ?look)))  
  (print "looked")
  (let* ((?perceived-object-desig
           (exe:perform (desig:an action
                                  (type detecting)
                                  (object (desig:an object (type ?object-type)))))))
    
      (exe:perform (desig:an action
                             (type picking-up)
                             (arm ?arm)
                             (grasp ?grasp)
                             (object ?perceived-object-desig)))
    ))

(defun place-object (?target-pose ?arm)
    (exe:perform (desig:a motion
                          (type looking)
                          (pose ?target-pose)))
    (exe:perform (desig:an action
                           (type placing)
                           (arm ?arm)
                           (target (desig:a location
                                            (pose ?target-pose))))))


(defun test-action-desig (?object-list)
  (let ((?object-type (first ?object-list))
        (?object-name (second ?object-list))
        (?goal-side (car (last ?object-list))))
  (desig:an action
            (:type :cashier)
            (:object-list ?object-list)
            (:object-type ?object-name)
            (:object-name ?object-type)
            (:goal-side ?goal-side))))

(defun pr2-cashier-demo ()

  (loop for object in spawn-objects-list
        do
           
  (spawn-object-on-counter-general object)
  (init-setup)
  (urdf-proj:with-simulated-robot
  (let ((?object-name (first object))
        (?object-type (second object))
        (?goal-side (car (last object))))
    (exe:perform (desig:an action
                           (:type :cashier)
                           (:object-list object)
                           (:object-type ?object-type)
                           (:object-name ?object-name)
                           (:goal-side ?goal-side)))))))

(defun cashier-object (&key ((:object-type ?object-type))
                             ((:object-name ?name))
                             ((:goal-side ?goal-side))
                             ((:sides-base ?sides-base))
                             ((:sides-transformed ?sides-transformed))
                        &allow-other-keys)
  (declare (type keyword ?object-type ?goal-side)
           (type list ?sides-base ?sides-transformed)
           (type symbol ?name))


  (print "sides set")
  (move *look-nav-pose*)
  (print "moved")

    ;;(let ((test (locate-sides transformed-sides
    ;;                          (origin->list ?name))))
  ;;(print test)
  (grasp-object ?object-type :left
                (caaar (cddr
                        (locate-sides ?sides-transformed
                                      (origin->list ?name)))) *spawn-area*)
  (move *place-nav-pose*)
  (place-object *place-pose* :left)
  
  (exe:perform (desig:an action
                         (:type :scanning)
                         (:object-type ?object-type)
                         (:goal-side ?goal-side)
                         (:object-name ?name)
                         (:sides-base ?sides-base)))
  (print "object was succesfully scanned"))

   ;;   ))
  

(prolog:def-fact-group cashier-plans (desig:action-grounding) 
  (prolog:<- (desig:action-grounding ?action-designator
                                     (cashier-object ?resolved-action-designator))
    (desig-prop ?action-designator (:type :cashier))
    (desig-prop ?action-designator (:object-name ?object-name))
    (desig-prop ?action-designator (:object-type ?object-type))
    (desig-prop ?action-designator (:goal-side ?goal-side))
    (desig-prop ?action-designator (:object-list ?object-list))
    
    (lisp-fun set-sides ?object-name 0.1 0.1 0.1 ?sides-base)
    (lisp-fun transforms ?object-name ?sides-base ?sides-transformed)
    
    (desig:designator :action ((:type :cashier)
                               (:object-type ?object-type)
                               (:object-name ?object-name)
                               (:goal-side ?goal-side)
                               (:sides-base ?sides-base)
                               (:sides-transformed ?sides-transformed))
                      ?resolved-action-designator))
  
  (prolog:<- (desig:action-grounding ?action-designator
                                     (scan-object ?resolved-action-designator))
    (desig-prop ?action-designator (:type :scanning))
    (desig-prop ?action-designator (:sides-base ?sides-base))
    (desig-prop ?action-designator (:goal-side ?goal-side))
    (desig-prop ?action-designator (:object-type ?object-type))
    (desig-prop ?action-designator (:object-name ?name))

    (lisp-fun transforms ?object-name ?sides-base ?sides-transformed)

    (desig:designator :action ((:type :scanning)
                               (:sides-base ?sides-base)
                               (:goal-side ?goal-side)
                               (:object-type ?object-type)
                               (:object-name ?name)
                               (:sides-transformed ?sides-transformed))
                      ?resolved-action-designator))
    )

(defun transforms (object-name side-poses)
  (mapcar (lambda (x)
            (let* ((map-T-object (cl-transforms:pose->transform (btr:object-pose object-name)))
                   (object-T-side (cl-transforms:pose->transform (first (cdr x))))
                   (map-T-side  (cl-transforms:transform* map-T-object object-T-side)))
    (list (car x) (cl-transforms:transform->pose map-T-side))))
          side-poses))
