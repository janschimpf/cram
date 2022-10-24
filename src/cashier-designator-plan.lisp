(in-package :cashier)

;; =======side-changing ===================
;; returns the bottom side and then a list of
;; the bottom, right and front sides in that order
(defun side-changes (side-list)
  (print "side-list")
  (print side-list)
  (let* ((bottom (first side-list))
         (right (second side-list))
         (front (third side-list)))
    (list (list "right-turn" right (list
                                    right
                                    (opposite-short bottom)
                                    front))
          
          (list "left-turn" (opposite-short right)
                (list (opposite-short right)
                      bottom
                      front))
          
          (list "back-turn" (opposite-short front)
                (list (opposite-short front)
                      right
                      bottom))
          
          (list "front-turn" front
                (list front
                      right
                      (opposite-short bottom)))
          
          (list "flip" (opposite-short bottom)
                (list (opposite-short bottom)
                      right
                      (opposite-short front)))
          
          (list "left-rotation" bottom
                (list bottom
                      (opposite-short front)
                      (opposite-short right)))
          
          (list "right-rotation" bottom
                (list bottom
                      (opposite-short front)
                      right))

          )))


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
  
  ;;(if
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
      ;;(sucessful-scan ?object-type ?object-name ?sides-base ?arm)

      (print "scan failed"))
  

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








