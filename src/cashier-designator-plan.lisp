(in-package :cashier)

(defun check-object-size (size)
  (let* ((x (car size))
         (y (cadr size))
         (z (caddr size))
         (non-graspable-list (list nil)))
    (if (and (> x 0.09) (> y 0.09))
        (setf non-graspable-list (append non-graspable-list (list :top :bottom))))
    (if (and (> y 0.09) (> z 0.09))
        (setf non-graspable-list (append non-graspable-list (list :right :left))))
    (if (and (> x 0.09) (> z 0.09))
        (setf non-graspable-list (append non-graspable-list (list :front :back))))
    non-graspable-list
    ))


(defun cashier-object (&key
                         ((:object-type ?object-type))
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
                 ?object-type
                 ?goal-side)
           
           (type list
                 ?sides-base
                 ?sides-transformed
                 ?object-size
                 ?arm
                 ?non-scanable
                 ?non-graspable)
           
           (type symbol ?object-name))
  (print "sides set")
  (move *look-nav-pose*)
  (print "moved")

  (let* ((?look *spawn-area*))
    (exe:perform (desig:a motion
                          (type looking)
                          (pose ?look)))
    (exe:perform (desig:an action
                           (type detecting)
                           (object (desig:an object (type ?object-type))))))
  (print "looked")
  (if (not (car ?non-graspable))
      (setf ?non-graspable (check-object-size ?object-size)))

  (if (not (car ?non-scanable))
      (setf ?non-scanable (prolog-shape ?object-type)))
  
  (print "check for non graspable sides")

  (let* ((grasp (cdr (locate-sides ?sides-transformed (origin->list ?object-name))))
         (orientation (cl-tf2:orientation (btr:object-pose ?object-name))))
    
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
      
  (grasp-object ?object-type
                ?arm
                *spawn-area*
                (first grasp))))
    
    (move *place-nav-pose*)
   (cpl:with-retry-counters ((place-retry 4))
      (cpl:with-failure-handling
          ((common-fail:manipulation-low-level-failure (e)
           (roslisp:ros-warn (cashier-demo) "retrying failed placement ~a~%" e)
             (cpl:do-retry place-retry
             (setf orientation (align-object ?object-name ?sides-base))
             (print "trying to fix placement")
           (cpl:retry))
           (cpl:fail 'high-level-place-failure)))
      
    (place-object (place-after-scan *place-pose* orientation)
                  ?arm
                  :?left-grasp
                  (first grasp)))))
  (pick-place-alight-object ?object-name ?sides-base ?arm ?object-type)
  
  (if
   (exe:perform (desig:an action
                         (:type :scanning)
                         (:arm ?arm)
                         (:non-scanable ?non-scanable)
                         (:non-graspable ?non-graspable)
                         (:object-name ?object-name)
                         (:object-type ?object-type)
                         (:object-size ?object-size)
                         (:goal-side ?goal-side)
                         (:sides-base ?sides-base)))
   (sucessful-scan ?object-type ?object-name ?sides-base ?arm)
   (unsucessful-scan ?object-type ?object-name ?sides-base ?arm)))


;;=============  placing object after scan ================

(defun sucessful-scan (?object-type ?object-name ?sides-base ?arm)
  (print "object was succesfully scanned")

  (let* ((grasp (cdr (locate-sides
           (transforms-map-t-side ?object-name ?sides-base)
           (cram-tf:3d-vector->list
            (cl-tf2:origin (btr:object-pose ?object-name))))))
         
         (orientation (cl-tf2:orientation (btr:object-pose ?object-name))))
    
    (cpl:with-retry-counters ((grasp-retry (length grasp)))
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
                ?arm *place-position* (first grasp))))

  (move *after-scan-nav-pose*)
    (place-object (place-after-scan (car *success-poses-list*) orientation)
                  ?arm
                  :?left-grasp
                  (first grasp))
    (setf *success-poses-list* (cdr *success-poses-list*))))


(defun unsucessful-scan (?object-type ?object-name ?sides-base ?arm)
  (print "object could not be scanned")

  (let* ((grasp (cdr (locate-sides
           (transforms-map-t-side ?object-name ?sides-base)
           (cram-tf:3d-vector->list
            (cl-tf2:origin (btr:object-pose ?object-name))))))
         (orientation (cl-tf2:orientation (btr:object-pose ?object-name))))
    
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
                ?arm *place-position* (first grasp))))
    
    (move *after-unsuccessful-scan-nav-pose*)
    
    (place-object (place-after-scan (car *unsuccessful-poses-list*) orientation)
                  ?arm
                  :?left-grasp
                  (first grasp))
    
    (setf *success-poses-list* (cdr *success-poses-list*))))


(defun place-after-scan (pose orientation)
   (cl-transforms-stamped:make-pose-stamped
   "map" 0.0
   (cram-tf:list->3d-vector pose)
   orientation))


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
    (if (<= right-side-mangnitude left-side-magnitude)
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
    (cl-tf2:q*
            (cl-tf2:orientation (btr:object-pose object-name))
            (cl-tf2:euler->quaternion
             :ax (first axis-found)
             :ay (second axis-found)
             :az (third axis-found)))))

(defun pick-place-alight-object (object-name sides-base arm object-type)
  (let* ((transformed (transforms-map-t-side object-name sides-base))
         (located (locate-sides
           transformed
           (cram-tf:3d-vector->list
            (cl-tf2:origin (btr:object-pose object-name)))))
         (grasp (cdr located))
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
    
  (grasp-object object-type
                arm
                *place-position*
                (first grasp))))
  
  (place-object (place-after-scan *place-pose* align)
                  arm
                  :?left-grasp
                  (first grasp))
    (spawn-side-visualisation (transforms-map-t-side object-name sides-base) "after-alignment")))
       
