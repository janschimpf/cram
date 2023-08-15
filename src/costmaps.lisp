(in-package :cashier)

(defparameter *robot-arm-offset* 0.12)
(defparameter *robot-pose-offset* 0.7)

(defparameter *test*
  (cl-transforms-stamped:make-pose-stamped
   "map" 0.0
   (cl-transforms:make-3d-vector -2 2 0.70)
   (cl-transforms:euler->quaternion :ax 0 :ay 0 :az pi)))

(def-fact-group costmap-metadata ()
    (<- (costmap-size 12 12))
    (<- (costmap-origin -6 -6))
    (<- (costmap-resolution 0.04))
 
    (<- (costmap-padding 0.3))
    (<- (costmap-manipulation-padding 0.4))
    (<- (costmap-in-reach-distance 0.7))
    (<- (costmap-reach-minimal-distance 0.2))
    (<- (visibility-costmap-size 2))
    (<- (orientation-samples 2))
  (<- (orientation-sample-step 0.1)))


(defmethod location-costmap:costmap-generator-name->score ((name (eql 'locate-cost-function))) 1)

(prolog:def-fact-group cashier-costmap (location-costmap:desig-costmap)
  (prolog:<- (location-costmap:desig-costmap ?designator ?costmap)
    (desig:desig-prop ?designator (:locate ?pose))
    (desig:desig-prop ?designator (:arm ?arm))
    (prolog:lisp-fun cl-transforms:origin ?pose ?pose-origin)
    (prolog:lisp-fun cl-transforms:x ?pose-origin ?ref-x)
    (prolog:lisp-fun cl-transforms:y ?pose-origin ?ref-y)
    (prolog:lisp-fun cl-transforms:orientation ?pose ?orientation)
    (location-costmap:costmap ?costmap)
    (location-costmap:costmap-add-function locate-cost-function
     (make-locate-cost-function ?ref-x ?ref-y ?orientation ?arm) ?costmap)
    (costmap:costmap-add-orientation-generator
     (rotate-robot ?ref-x ?ref-y ?orientation) ?costmap))
)

  ;; (prolog:<- (location-costmap:desig-costmap ?designator ?costmap)
  ;;   (desig:desig-prop ?designator (:spawn ?pose))
  ;;   (prolog:lisp-fun cl-transforms:origin ?pose ?pose-origin)
  ;;   (prolog:lisp-fun cl-transforms:x ?pose-origin ?ref-x)
  ;;   (prolog:lisp-fun cl-transforms:y ?pose-origin ?ref-y)
  ;;   (prolog:lisp-fun cl-transforms:z ?pose-origin ?ref-z)
  ;;   (prolog:lisp-fun cl-transforms:orientation ?pose ?orientation)
  ;;   (location-costmap:costmap ?costmap)
  ;;   (location-costmap:costmap-add-function
  ;;    locate-cost-function
  ;;    (make-spawn-cost-function ?pose) ?costmap)
  ;;   (costmap:costmap-add-orientation-generator
  ;;    (random-orientation)
  ;;    ?costmap)
  ;;   (costmap:costmap-add-cached-height-generator
  ;;    (costmap:make-constant-height-function ?ref-z)
  ;;    ?costmap))
  
  
             

(defun make-locate-cost-function (ref-x ref-y orientation arm)
  (let* ((adjusted-to-arm (if (equal arm :left)
                              (- *robot-arm-offset*) 
                              *robot-arm-offset*))

         (offset-vector (cl-transforms:rotate
                         orientation
                         (cl-transforms:make-3d-vector *robot-pose-offset* adjusted-to-arm 0)))
         
         (added-vectors (cl-transforms:make-3d-vector
                         (+ ref-x (cl-transforms:x offset-vector))
                         (+ ref-y (cl-transforms:y offset-vector))
                         0))
         
         (supp-tf (cl-transforms:make-transform
                   added-vectors
                   orientation))
         
         (world->supp-tf (cl-transforms:transform-inv supp-tf)))
    
    
    (lambda (x y)
      (let* ((point (cl-transforms:transform-point world->supp-tf
                     (cl-transforms:make-3d-vector x y 0))))
        
        (if (and (< (cl-transforms:x point) 0.05)
                 (> (cl-transforms:x point) -0.05)
                 (< (cl-transforms:y point) 0.05)
                 (> (cl-transforms:y point) -0.05))
                     1
                     0)))
    
  ;; use pose to get 2d point and orientation
  ;; use the 2d point as the center for costmap
  ;; use orientation to create a vector to get a small area with possible locations.
  ;; use rotate-robot-to-point to get orientate the pr2 towards the point.
  ;; check if "origin" point is visible for pr2 from that location.
  ;; check for possible collisions at point
    ))


             

(defun make-spawn-cost-function (pose)
  (let* ((origin-pose (cl-transforms:origin pose))
         (orientation (cl-transforms:orientation pose))
         
         (ref-x (cl-transforms:x origin-pose))
         (ref-y (cl-transforms:y origin-pose))

         
         (supp-tf (cl-transforms:make-transform
                   (cl-transforms:make-3d-vector ref-x ref-y 0)
                   (cl-transforms:make-identity-rotation)))
         
         (world->supp-tf (cl-transforms:transform-inv supp-tf)))
    
    
    (lambda (x y)(lambda (x y previous-orientations)
      (let* ((point (cl-transforms:transform-point world->supp-tf
                     (cl-transforms:make-3d-vector x y 0))))
        
        (if (and (< (cl-transforms:x point) 0.05)
                 (> (cl-transforms:x point) -0.05)
                 (< (cl-transforms:y point) 0.05)
                 (> (cl-transforms:y point) -0.05))
                     1
                     0)))
      ))
  )

(defun rotate-robot (ref-x ref-y orientation)
    (lambda (x y previous-orientations)
      (let* ((second-vector (cl-transforms:rotate
                             orientation
                             (cl-transforms:make-3d-vector *robot-pose-offset* 0 0)))
             
             ;; (added-vectors (cl-transforms:make-3d-vector
             ;;                 (+ ref-x (cl-transforms:x second-vector))
             ;;                 (+ ref-y (cl-transforms:y second-vector))
             ;;                 0))
             
             ;; (supp-tf (cl-transforms:make-transform
             ;;           added-vectors
             ;;           orientation))
             ;; (world->supp-tf (cl-transforms:transform-inv supp-tf))
             
             ;; (point (cl-transforms:transform-point world->supp-tf
             ;;         (cl-transforms:make-3d-vector x y 0)))

             ;; (angle (list (2d-vectors-to-angle (list (cl-transforms:x point)
             ;;                                         (cl-transforms:y point))
             ;;                                   (list ref-x ref-y))))
             (pi-list (list pi)))
        
        (mapcar
         (lambda (angle)
           (cl-transforms:q*
            orientation
            (cl-transforms:euler->quaternion
             :ax 0 :ay 0 :az angle)))
         pi-list)
      )
  ))

(defun random-orientation ()
  (lambda (x y previous-orientations)
    (let* ((ax-ran (/ (random 7000) 3500))
           (ay-ran (/ (random 7000) 3500))
           (az-ran (/ (random 7000) 3500)))
                   
      
    (cl-transforms:euler->quaternion
     :ax ax-ran :ay ay-ran :az az-ran))
  ))


(defun align-point-front (pose)
  (let* ((ref-x (cl-transforms:x (cl-tf2:origin pose)))
         (ref-y (cl-transforms:y (cl-tf2:origin pose)))
         (orientation (cl-tf2:orientation pose))
         (offset-vector (cl-transforms:rotate
                         orientation
                         (cl-transforms:make-3d-vector *robot-pose-offset* 0 0)))
         
         (added-vectors (cl-transforms:make-3d-vector
                         (+ ref-x (cl-transforms:x offset-vector))
                         (+ ref-y (cl-transforms:y offset-vector))
                         0))
         
         (supp-tf (cl-transforms:make-transform
                   added-vectors
                   orientation))
         
         (world->supp-tf (cl-transforms:transform-inv supp-tf)))
    (cl-transforms:transform-point world->supp-tf
                                   (cl-transforms:make-3d-vector 0 0 0))))
