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
  (print "moved")

  (let ((?grasp (caddr (locate-sides ?sides-transformed (origin->list ?object-name)))))
  (grasp-object ?object-type
                ?arm
                *spawn-area*
                ?grasp)
  (move *place-nav-pose*)
  
  (place-object *place-pose* ?arm :?left-grasp ?grasp))
  
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
                ?arm *place-pose* (first grasp))))
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







