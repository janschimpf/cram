(in-package :cashier)

(defparameter *look-nav-pose*
  (cl-transforms-stamped:make-pose-stamped
   "map"
   0.0
   (cl-transforms:make-3d-vector -1.2 1.3 0)
   (cl-transforms:euler->quaternion :ax 0 :ay 0 :az pi)))

(defparameter *grasp-spawn*
(cl-transforms-stamped:make-pose-stamped
   "map" 0.0
   (cl-transforms:make-3d-vector -2 1.3 0.75)
   (cl-transforms:euler->quaternion :ax 0 :ay 0 :az pi)))

(defparameter *place-nav-pose*
  (cl-transforms-stamped:make-pose-stamped
   "map"
   0.0
   (cl-transforms:make-3d-vector -1.2 0.3 0)
   (cl-transforms:euler->quaternion :ax 0 :ay 0 :az pi)))

(defparameter *spawn-area*
  (cl-transforms-stamped:make-pose-stamped
   "map" 0.0
   (cl-transforms:make-3d-vector -2 1.3 0.75)
   (cl-transforms:euler->quaternion :ax 0 :ay 0 :az pi)))

(defparameter *place-pose*
  (cl-transforms-stamped:make-pose-stamped
   "map" 0.0
   (cl-transforms:make-3d-vector -2 0.3 0.75)
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
  

(defun grasp-object (?object-type ?arm ?grasp)
  (print ?grasp)
  (let ((?look *spawn-area*))
    (exe:perform (desig:a motion
                          (type looking)
                          (pose ?look)))
    
  (let* ((?perceived-object-desig
           (exe:perform (desig:an action
                                  (type detecting)
                                  (object (desig:an object (type ?object-type)))))))
    
      (exe:perform (desig:an action
                             (type picking-up)
                             (arm ?arm)
                             (grasp ?grasp)
                             (object ?perceived-object-desig)))
    )))

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
    (pp-plans::park-arms)

    (spawn-object-on-counter-general object-list)
    
    (init-setup)
    
    (setf *sides* (change-side-list-to-map (set-sides (first object-list) 0.1 0.1 0.1)))

    (move *look-nav-pose*)
    
    (grasp-object (second object-list) :left (first (first (testing *sides* *grasp-spawn*))))
    
    (move *place-nav-pose*)

    (place-object *place-pose* :left)
    

    (scan (first object-list) (last object-list) *sides*)
    ))
  


(defun testing (side-list grasp-pose)
  (let ((grasp-xyz-list (pose-to-xyz-list grasp-pose)))
  (shortest-distance (distances-for-side-list side-list grasp-xyz-list))))
    

(defun pose-to-xyz-list (pose)
  (cram-tf:3d-vector->list (cl-tf2:origin pose)))


(defparameter *tf-broadcaster* nil)
(defun init-tf-broadcaster ()
  (setf *tf-broadcaster* (cram-tf:make-tf-broadcaster "/tf" 0.1)))

(defun get-object-pose (object-name)
  (btr:object-pose object-name))


(defun create-stamped-transform-for-object (object-name)
  (cl-tf:make-transform-stamped "map" "object" 0 
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
  (spawn-object-on-counter-general object-list)
  (update-transform (first object-list))
  )
