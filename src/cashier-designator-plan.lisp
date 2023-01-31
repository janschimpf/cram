(in-package :cashier)

(defun check-object-size (size)
  (let* ((x (car size))
         (y (cadr size))
         (z (caddr size))
         (test-list (list nil)))
    (if (and (> x 0.09) (> y 0.09))
        (setf test-list (append test-list (list :top :bottom))))
    (if (and (> y 0.09) (> z 0.09))
        (setf test-list (append test-list (list :right :left))))
    (if (and (> x 0.09) (> z 0.09))
        (setf test-list (append test-list (list :front :back))))
    test-list
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


  (let* ((?look *spawn-area*))
    (exe:perform (desig:a motion
                          (type looking)
                          (pose ?look)))
    (exe:perform (desig:an action
                           (type detecting)
                           (object (desig:an object (type ?object-type))))))
  
  (if (not (car ?non-graspable))
      (setf ?non-graspable (check-object-size ?object-size)))

  (if (not (car ?non-scanable))
      (setf ?non-scanable (prolog-shape ?object-type)))
  
  (print "moved")

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
(defun points-to-2dvector (p1 p2)
 (list (- (car p2) (car p1)) (- (cadr p2) (cadr p1))))
       

(defun 2d-vectors-to-angle (v1 v2)
  (acos (/ (dot-product-2d v1 v2) (* (v-norm-2d v1) (v-norm-2d v2))))
)

(defun v-norm-2d (v)
  "Returns the magnitude of the vector"
  (sqrt (dot-product-2d v v)))

(defun dot-product-2d (v-1 v-2)
  "Returns the dot-product of two 2d vectors"
  (+ (* (car v-1) (car v-2))
     (* (cadr v-1) (cadr v-2))))

(defun points-to-angle (p1 p2 p3)
  "creates 2 vectors with p1 as the origin and going towards p2 and p3"
  (print p1)
  (print p2)
  (print p3)
  (2d-vectors-to-angle
   (points-to-2dvector p1 p2)
   (points-to-2dvector p1 p3)))


(defun get-correct-side (transformed-sides-base side)
    (loop for x in transformed-sides-base
          do
             (if (equal (first x) side)
                 (return (second x)))))


    
(defun align-object (object-name sides-base)
  (let* ((point-1 (cram-tf:3d-vector->list (cl-tf2:origin (btr:object-pose object-name))))
         (point-2 (cram-tf:3d-vector->list (cl-tf2:origin *place-nav-pose*)))
         (located (locate-sides
           (transforms-map-t-side object-name sides-base)
           (cram-tf:3d-vector->list
            (cl-tf2:origin (btr:object-pose object-name)))))
         (side (car (reverse located)))
         (point-3 (cram-tf:3d-vector->list (cl-tf2:origin (get-correct-side
                   (transforms-map-t-side object-name sides-base) side))))
         (angle (list 0 0 (points-to-angle point-1 point-3 point-2)))
         (axis-found (finding-axis located angle)))
    (cl-tf2:q*
            (cl-tf2:orientation (btr:object-pose object-name))
            (cl-tf2:euler->quaternion
             :ax (first axis-found)
             :ay (second axis-found)
             :az (third axis-found)))))

(defun pick-place-alight-object (object-name sides-base arm object-type)
  (let* ((located (locate-sides
           (transforms-map-t-side object-name sides-base)
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
    
  (grasp-object object-type
                arm
                *place-position*
                (first grasp))))
  
  (place-object (place-after-scan *place-pose* align)
                  arm
                  :?left-grasp
                  (first grasp))))
       
