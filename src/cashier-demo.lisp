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
    (cpl:par
      (exe:perform (desig:an action
                             (type parking-arms)))
      (exe:perform (desig:a motion
                            (type going)
                            (pose ?navigation-goal))))
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
  (cpl:par
    (exe:perform (desig:a motion
                          (type looking)
                          (pose ?target-pose)))
    (exe:perform (desig:an action
                           (type placing)
                           (arm ?arm)
                           (target (desig:a location
                                            (pose ?target-pose)))))))
  
  
(defun pr2-cashier-demo ()
  (urdf-proj:with-simulated-robot
    ;;(pp-plans::park-arms)

    (print "arms")
    (spawn-object-on-counter-general object-list-2)
    (print "spawn")
    (init-setup)
    (print "spawn-finished")
    
    (setf *sides* (change-side-list-to-map (set-sides (first object-list-2) 0.1 0.1 0.1)))

    (print "sides set")
    (move *look-nav-pose*)
    (print "moved")
    
    (grasp-object (second object-list-2) :left (caaar (cddr (locate-sides *sides* (origin->list (first object-list-2)))))
  *spawn-area*)
    
    (move *place-nav-pose*)

    (print "place")
    (print *place-pose*)
    
    (place-object *place-pose* :left)   
    (print "scan")
    (scan (first object-list-2) (last object-list-2) *sides*)

    (scan-all-sides (cdr *sides*) (last object-list-2) (second object-list-2) (first object-list-2))
    ))
  


(defun testing (side-list grasp-pose)
  (let ((grasp-xyz-list (pose-to-vector-list grasp-pose)))
    (shortest-distance-between-all-sides side-list grasp-xyz-list)))
    


(defparameter *tf-broadcaster* nil)
(defun init-tf-broadcaster ()
  (setf *tf-broadcaster* (cram-tf:make-tf-broadcaster "/tf" 0.1)))

(defun get-object-pose (object-name)
  (btr:object-pose object-name))


(defun create-stamped-transform-for-object (object-name)
  (cl-tf:make-transform-stamped "map" object-name 0 
                                (cl-transforms:origin (get-object-pose object-name))
                                (cl-transforms:orientation (get-object-pose object-name))))

(defun update-transform (object-name)
  (cram-tf:add-transform cram-tf:*broadcaster* (create-stamped-transform-for-object object-name))
  (cram-tf::publish-transforms cram-tf:*broadcaster*))

(defun test-transform ()
 (cl-tf2:transform-pose-stamped cram-tf:*transformer*
 :pose (cl-tf2:make-pose-stamped "object" 0
                                 (cl-tf2:make-3d-vector 0.05 0 0)
                                 (cl-transforms:make-quaternion 0 0 0 1))
 :target-frame "map"))

(defun init-setup ()
  (init-tf-broadcaster)
  (update-transform (first object-list-2))
  )
