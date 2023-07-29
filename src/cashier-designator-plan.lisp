(in-package :cashier)

(defun check-object-size (size)
  (let* ((x (car size))
         (y (cadr size))
         (z (caddr size))
         (non-graspable-list (list nil)))
    (if (and (> x 0.06) (> y 0.06))
        (setf non-graspable-list (append non-graspable-list (list :top :bottom))))
    
    (if (and (> x 0.06) (> z 0.1))
        (setf non-graspable-list (append non-graspable-list (list :right :left))))

    (if (and (> y 0.05) (> z 0.1))
        (setf non-graspable-list (append non-graspable-list (list :front :back))))
    non-graspable-list
    ))


(defun cashier-object (&key
                         ((:object-type ?object-typ))
                         ((:object-name ?object-name))
                         ((:arm ?arm))
                         ((:non-scanable ?non-scanable))
                         ((:non-graspable ?non-graspable))
                         ((:goal-side ?goal-side))
                         ((:sides-base ?sides-base))
                         ((:sides-transformed ?sides-transformed))
                         ((:object-size ?object-size))
                       &allow-other-keys)

  (declare (type keyword
                 ?object-typ
                 ?goal-side)
           
           (type list
                 ?sides-base
                 ?sides-transformed
                 ?object-size
                 ?arm
                 ?non-scanable
                 ?non-graspable)
           
           (type symbol ?object-name))
  
  (let* ((?nav-pose *spawn-area*))
    (move (desig:reference (desig:a location (locate ?nav-pose) (arm (first ?arm))))))
  
  (print "moved to first look nav pose")
  
  (multiple-value-bind (?perceived-object)
      (perceive-object *spawn-area* ?object-typ)

    (print "found the object")
    (print ?perceived-object)
  (if (not (car ?non-graspable))
      (setf ?non-graspable (check-object-size ?object-size)))

  (if (not (car ?non-scanable))
      (setf ?non-scanable (prolog-shape ?object-typ)))
  
  (print "check for non graspable sides")

  (let* ((grasp (which-sides-can-be-grasped
                 (locate-sides ?sides-transformed (origin->list ?object-name))
                 ?non-graspable))
         (orientation (cl-tf2:orientation (btr:object-pose ?object-name))))

    (transport-object *place-position* *spawn-area*
                      grasp ?arm ?object-typ
                      orientation *place-pose*)
  
   (pick-place-alight-object ?object-name ?sides-base ?arm ?object-typ ?non-graspable)))

  
  (if
   (exe:perform (desig:an action
                         (:type :scanning)
                         (:arm ?arm)
                         (:non-scanable ?non-scanable)
                         (:non-graspable ?non-graspable)
                         (:object-name ?object-name)
                         (:object-type ?object-typ)
                         (:object-size ?object-size)
                         (:goal-side ?goal-side)
                         (:sides-base ?sides-base)))
   (sucessful-scan ?object-typ ?object-name ?sides-base ?arm ?non-graspable)
   (unsucessful-scan ?object-typ ?object-name ?sides-base ?arm ?non-graspable)) 
  )



;;=============  placing object after scan ================

(defun sucessful-scan (?object-type ?object-name ?sides-base ?arm ?non-graspable-sides)
  (print "object was succesfully scanned")

  (let* ((grasp (which-sides-can-be-grasped
                 (locate-sides
                  (transforms-map-t-side ?object-name ?sides-base)
                  (cram-tf:3d-vector->list
                   (cl-tf2:origin (btr:object-pose ?object-name))))
                 ?non-graspable-sides))
         
         (orientation (cl-tf2:orientation (btr:object-pose ?object-name))))
    
    (transport-object
     *after-scan-nav-pose*
     *place-position*
     grasp
     ?arm
     ?object-type
     orientation
     (car *success-poses-list*))
  (setf *success-poses-list* (cdr *success-poses-list*))))


(defun unsucessful-scan (?object-type ?object-name ?sides-base ?arm ?non-graspable-sides)
  (print "object could not be scanned")

  (let* ((grasp (which-sides-can-be-grasped
                 (locate-sides
                  (transforms-map-t-side ?object-name ?sides-base)
                  (cram-tf:3d-vector->list
                   (cl-tf2:origin (btr:object-pose ?object-name))))
                 ?non-graspable-sides))
         (orientation (cl-tf2:orientation (btr:object-pose ?object-name))))

    (transport-object
     *after-unsuccessful-scan-nav-pose*
     *place-position*
     grasp
     ?arm
     ?object-type
     orientation
     (car *unsuccessful-poses-list*))
    
    (setf *unsuccessful-poses-list* (cdr *unsuccessful-poses-list*))))


(defun place-after-scan (pose orientation q1 q2)
  (let* ((angle-between-location-quaterions (cl-tf2:angle-between-quaternions q1 q2))
         
         (new-orientation (cl-tf2:q*
                           orientation
                           (cl-tf2:axis-angle->quaternion
                            (cl-tf2:make-3d-vector 0 0 1)
                            angle-between-location-quaterions))))
        
   (cl-transforms-stamped:make-pose-stamped
   "map" 0.0
   (cram-tf:list->3d-vector pose)
   new-orientation)))

(defun transport-object (?move-pose ?look grasp-list
                         ?arm ?object-typ object-orientation
                         place-pose)
  
  (multiple-value-bind (?perceived-object)
      (perceive-object ?look ?object-typ)
    
  (cpl:with-retry-counters ((grasp-retry (length grasp-list)))
    (cpl:with-failure-handling
        ((common-fail:gripper-closed-completely (e) 
           (roslisp:ros-warn (cashier-demo) "failure happened: ~a~% changing grasp" e)
           (cpl:do-retry grasp-retry
             (setf grasp-list (cdr grasp-list))
             (cpl:retry)))
         
         (desig:designator-error (e)
           (roslisp:ros-warn (cashier-demo) "designator-reference-failure ~a~%" e)
           (cpl:do-retry grasp-retry
             (setf grasp-list (cdr grasp-list))
             
           (cpl:retry))
           (cpl:fail 'common-fail:high-level-failure)))
                             
      (grasp-object-with-handling
       ?arm
       (first grasp-list)
       ?perceived-object))))
    

  (let* ((orientation-of-old-pose (cram-tf:orientation (cram-tf:robot-current-pose)))
         (new-position (desig:reference (desig:a location (locate ?move-pose) (arm (first ?arm)))))
         (orientation-of-new-pose (cram-tf:orientation new-position)))
    (move new-position)
  
    (place-object-with-handling
     (place-after-scan place-pose object-orientation orientation-of-old-pose orientation-of-new-pose)
     ?arm
     (first grasp-list))
))

;; ========== algin object rotation with robot ===================
(defun points-to-2dvector (point-1 point-2)
 (list (- (car point-2) (car point-1)) (- (cadr point-2) (cadr point-1))))
       

(defun 2d-vectors-to-angle (vector-1 vector-2)
  (acos (/ (dot-product-2d vector-1 vector-2) (* (2d-vector-magnitude vector-1) (2d-vector-magnitude vector-2))))
)

(defun 2d-vector-magnitude (vector)
  "Returns the magnitude of the vector"
  (sqrt (+ (expt (car vector) 2) (expt (cadr vector) 2))))

(defun dot-product-2d (v-1 v-2)
  "Returns the dot-product of two 2d vectors"
  (+ (* (car v-1) (car v-2))
     (* (cadr v-1) (cadr v-2))))

(defun points-to-angle (p1 p2 p3)
  "creates 2 vectors with p1 as the origin and going towards p2 and p3"
  (2d-vectors-to-angle
   (points-to-2dvector p1 p2)
   (points-to-2dvector p1 p3)))


(defun get-correct-side (transformed-sides-base side)
    (loop for x in transformed-sides-base
          do
             (if (equal (first x) side)
                 (return (second x)))))

(defun angle-direction (robot-base right-side left-side angle)
  (let* ((right-side-mangnitude (2d-vector-magnitude
                                 (points-to-2dvector robot-base right-side)))
         (left-side-magnitude (2d-vector-magnitude
                               (points-to-2dvector robot-base left-side))))
    (if (>= right-side-mangnitude left-side-magnitude)
        angle
        (- 0 angle))
    ))

    
(defun align-object (object-name sides-base)
  (let* ((object-location (cram-tf:3d-vector->list (cl-tf2:origin (btr:object-pose object-name))))
         
         (robot-base-location (cram-tf:3d-vector->list (cl-tf2:origin *place-nav-pose*)))
         
         (located (locate-sides
           (transforms-map-t-side object-name sides-base)
           (cram-tf:3d-vector->list
            (cl-tf2:origin (btr:object-pose object-name)))))
         
         (front-side (car (reverse located)))
         (right-side (cadr located))
         (left-side (opposite-short (cadr located)))
         (front-side-location (cram-tf:3d-vector->list (cl-tf2:origin
                                                        (get-correct-side
                                                         (transforms-map-t-side
                                                          object-name sides-base)
                                                         front-side))))
         
         (right-side-location (cram-tf:3d-vector->list (cl-tf2:origin
                                                        (get-correct-side
                                                         (transforms-map-t-side
                                                          object-name sides-base)
                                                         right-side))))
         
         (left-side-location (cram-tf:3d-vector->list (cl-tf2:origin
                                                        (get-correct-side
                                                         (transforms-map-t-side
                                                          object-name sides-base)
                                                         left-side))))
         
         (angle (points-to-angle object-location robot-base-location front-side-location))

         
         (correct-angle-direction (angle-direction robot-base-location
                                                   right-side-location
                                                   left-side-location
                                                   angle))
         (rotation (list 0 correct-angle-direction 0))
         (axis-found (finding-axis located rotation)))
    (print "after axis")
    (print axis-found)
    (print located)
    (cl-tf2:q*
            (cl-tf2:orientation (btr:object-pose object-name))
            (cl-tf2:euler->quaternion
             :ax (first axis-found)
             :ay (second axis-found)
             :az (third axis-found)))))

(defun pick-place-alight-object (object-name sides-base arm ?object-typ non-graspable)
  
  (multiple-value-bind (?perceived-object)
      (perceive-object *place-position* ?object-typ)
    
  (let* ((transformed (transforms-map-t-side object-name sides-base))
         (located (locate-sides
           transformed
           (cram-tf:3d-vector->list
            (cl-tf2:origin (btr:object-pose object-name)))))
         (grasp (which-sides-can-be-grasped located non-graspable))
         (align (align-object object-name sides-base)))
    
    (cpl:with-retry-counters ((grasp-retry 3))
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
          (cpl:fail 'high-level-grasp-failure)))

        (spawn-side-visualisation transformed "pre-alignment")      
    
        (grasp-object-with-handling
                arm
                (first grasp)
                ?perceived-object)))
  
    (place-object-with-handling
     (cl-transforms-stamped:make-pose-stamped
      "map" 0.0
      (cram-tf:list->3d-vector *place-pose*)
      align)
      arm
      (first grasp))
    (spawn-side-visualisation (transforms-map-t-side object-name sides-base) "after-alignment"))
  ))
;;======================== Check if object can be grasped and which sides can be grasped ========================

;; check if the object can be grasped in the current orientation
(defun can-object-be-grasped (located-sides non-graspable-sides sides-that-can-be-grasped)
  (if (not (car sides-that-can-be-grasped))
      (cpl:fail 'high-level-grasp-failure))
  (if (not (contains-element-negative (car located-sides) non-graspable-sides))
      (cpl:fail 'high-level-grasp-failure))

  )

;;return a list of grasp valid grasp locations give the current orientation and the arm used
(defun which-sides-can-be-grasped (located-sides non-graspable-sides)
  (let* ((sides-list (list
                      (third located-sides) ;; front
                      (opposite-short (first located-sides)) ;; top
                      (second located-sides) ;; right 
                      (opposite-short (second located-sides)) ;; left
                      )) 
         (non-graspable-removed (mass-filter sides-list non-graspable-sides)))

    (print non-graspable-removed)
    non-graspable-removed 
  ))

;;filters list to check if any of the sides are part of the non-graspable list
(defun mass-filter (sides non-graspable)
  (remove nil
          (mapcar
           (lambda (x)
             (contains-element-negative x non-graspable))
             sides)
    ))

;;checks if element is in list if, yes returns nil if no return element
(defun contains-element-negative (side list)
  (if (member side list :test 'equal)
      nil
      side))
