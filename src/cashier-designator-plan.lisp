(in-package :cashier)




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
    (cpl:with-retry-counters ((grasp-retry 2))
      (cpl:with-failure-handling
          ((common-fail:gripper-closed-completely (e) 
           (roslisp:ros-warn (cashier-demo) "failure happened: ~a~% changing grasp" e)
           (cpl:do-retry grasp-retry
             (setf grasp (cdr grasp))
             (cpl:retry)))))
          
  (grasp-object ?object-type
                ?arm
                *spawn-area*
                (first grasp)))
    
  (move *place-nav-pose*)

    (place-object (place-after-scan *place-pose* orientation)
                  ?arm
                  :?left-grasp
                  (first grasp)))
  
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
  (move *after-scan-nav-pose*)
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
        
       





