(in-package :cashier)
(defparameter counter 0)
;; iterates over the to scan sides.
;; first gets the current bottom side, front side and right side
;; then the path plan the path, execute the path, update object, scan
(defun scan-plan (&key
                      ((:object-type ?object-type))
                      ((:arm ?arm))
                      ((:non-scanable ?non-scanable))
                      ((:non-graspable ?non-graspable))
                      ((:object-size ?object-size))
                      ((:base-sides ?b-sides))
                      ((:sides-to-check ?sides-to-check))
                      ((:scan-pose ?scan-pose))
                    &allow-other-keys)
  
  (declare (type keyword
                 ?object-type)

           (type cl-tf2:pose-stamped ?scan-pose)
           
           (type list
                 ?b-sides
                 ?object-size
                 ?sides-to-check
                 ?arm
                 ?non-scanable
                 ?non-graspable))
  
  (let* ((?sides-to-be-check ?sides-to-check)) 
  ;; before scan is started
  (cpl:with-retry-counters ((scan-counter-retries (length ?sides-to-be-check)))
    (cpl:with-failure-handling 
    ((common-fail:high-level-failure (e)
       (roslisp:ros-warn (cashier-demo) "Falure happend: ~a~% adjusting side to be scanned" e)
       (cpl:do-retry scan-counter-retries
         (let* ((?perceived-object (perceive-object ?scan-pose ?object-type))
                (?sides-located (side-location ?perceived-object ?b-sides))
                ;; (?object-vector (cram-tf:3d-vector->list
                ;;                  (cl-tf2:origin
                ;;                   (man-int:get-object-pose-in-map ?perceived-object))))
                ;; (?transform (man-int:get-object-transform ?perceived-object))
                
                ;; (?sides-in-map (transform-b-sides-t-x ?b-sides ?transform))
                )
           
           (setf ?sides-to-be-check (remove-element-from-list ?sides-to-be-check
                                                              (first ?sides-located)))

           (let* ((?check-side (next-side-to-check ?sides-located ?sides-to-be-check))
                  (?object (extended-object-desig ?perceived-object
                                                  ?object-size ?non-graspable
                                                  ?non-scanable ?check-side ?b-sides)))
           ;;next side
           (if (not (equal nil ?check-side))
             (exe:perform
              (desig:an action
                        (:type :changing-side)
                        (:object ?object)
                        (:scan-pose ?scan-pose)
                        (:arm ?arm)
                        (:change-to-side ?check-side)
                        (:sides-base ?b-sides)
                        (:object-size ?object-size))))))
           (cpl:retry))
       (cpl:fail 'common-fail:high-level-failure)))
      (let ((?perceived-object (perceive-object ?scan-pose ?object-type)))
        
      ; scanning bottom side
      (if ?sides-to-be-check
          (let* ((scanned (scan ?perceived-object ?b-sides)))
            (print scanned)
            (print "====================================================================================================================================================")
          (if scanned
              (scanned-object-desig ?perceived-object ?b-sides
                                    ?object-size ?non-graspable
                                    ?non-scanable scanned)
              (cpl:fail 'common-fail:high-level-failure)))
            (scanned-object-desig ?perceived-object ?b-sides
                                  ?object-size ?non-graspable
                                  ?non-scanable nil)))))))

(defun next-side-to-check (located-sides sides-to-check)
  (let* ((right (second located-sides))
         (left (opposite-short (second located-sides)))
         (top (opposite-short (first located-sides)))
         (front (third located-sides))
         (back (opposite-short (third located-sides)))
         (side-list (list right left front back top)))
    (print side-list)
    ;; check list if element(s) are part of sides-to-check
    ;; first element that is in the side-to-check list is the one we target next
    (loop for x in side-list
          do
             (when (not (null (member x sides-to-check)))
               (return x)))))

(defun test-next-side-to-check ()
  (let*((side (list :front :right :top))
        (check (list :top :back)))
    (next-side-to-check side check)
    ))

(defun scanned-object-desig (?perceived-object ?b-sides ?size ?n-grasp ?n-scan ?goal)
  (let* ((?type (desig-prop-value ?perceived-object :type))
         (?name (desig-prop-value ?perceived-object :name))
         (?pose (desig-prop-value ?perceived-object :pose)))
    (print ?goal)
    (print "giving over goal")
  (desig:an object
            (:type ?type)
            (:name ?name)
            (:pose ?pose)
            (:base-sides ?b-sides)
            (:size ?size)
            (:non-graspable ?n-grasp)
            (:non-scanable ?n-scan)
            (:goal-side ?goal)
  )))
