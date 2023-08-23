(in-package :cashier)
(defparameter *current-grasp* nil)
(defparameter *base-sides* nil)

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


(defun cashier-plan (&key
                         ((:object-type ?object-type))
                         ((:arm ?arm))
                         ((:non-scanable ?non-scanable))
                         ((:non-graspable ?non-graspable))
                         ((:goal-side ?goal-side))
                         ((:object-size ?object-size))
                         ((:search-poses ?search-poses))
                         ((:scan-pose ?scan-pose))
                         ((:after-poses ?after-poses))
                       &allow-other-keys)

  (declare (type keyword
                 ?object-type)
           
           (type cl-tf2:pose-stamped ?scan-pose)
            
           (type list
                 ?object-size
                 ?arm
                 ?non-scanable
                 ?non-graspable
                 ?search-poses
                 ?after-poses))

  ;;looking for object in search area
  (cpl:with-retry-counters ((percieve-retry (length ?search-poses)))
    (cpl:with-failure-handling
        ((cram-common-failures:perception-object-not-found (e)
           (roslisp:ros-warn (cashier-demo) "perception failue ~a~%" e)
           (cpl:do-retry percieve-retry
             (setf ?search-poses (cdr ?search-poses))    
           (cpl:retry))
           (cpl:fail 'common-fail:high-level-failure)))
      
      (let* ((?nav-pose (first ?search-poses)))
        (move (desig:reference (desig:a location (locate ?nav-pose) (arm (first ?arm)))))
        (perceive-object (first ?search-poses) ?object-type))))

  (let* ((?object-name (desig:desig-prop-value
                        (perceive-object (first ?search-poses) ?object-type) :name))
         (?base-sides (set-sides-helper ?object-name ?object-size)))
  
  (print "bringing object to the scanner")
  (let* ((?perceived-object (perceive-object (first ?search-poses) ?object-type))
         (?object-pose (man-int:get-object-pose-in-map ?perceived-object))
         (?orientation (cl-tf2:orientation ?object-pose)))
    
     (pick-place-object  ?non-graspable ?arm
                         ?perceived-object ?orientation
                         ?scan-pose (first ?search-poses) ?base-sides))

  (print "aligning the object")  
  (let* ((?perceived-object (perceive-object ?scan-pose ?object-type)))

    (pick-place-align-object ?object-name ?base-sides
                              ?arm ?non-graspable
                              ?perceived-object ?scan-pose))


  (let* ((?perceived-object (perceive-object ?scan-pose ?object-type))
         (?object (extended-object-desig ?perceived-object ?object-size
                                         ?non-graspable ?non-scanable
                                         ?goal-side ?base-sides))
         (?scan-state (exe:perform (desig:an action
                             (:type :scanning)
                             (:arm ?arm)
                             (:scan-pose ?scan-pose)
                             (:object ?object)))))
    
    (let* ((?perceived-object-after-scan (perceive-object ?scan-pose ?object-type))
           (?object-pose-after-scan (man-int:get-object-pose-in-map ?perceived-object-after-scan))
           (?orientation (cl-tf2:orientation ?object-pose-after-scan))
           (?scan-status (desig:desig-prop-value ?scan-state :goal-side))
           (?place-pose (if ?scan-status
                            (first ?after-poses)
                            (second ?after-poses))))    
      (pick-place-object
       ?non-graspable ?arm ?perceived-object-after-scan
       ?orientation ?place-pose ?scan-pose ?base-sides )) 
    ?scan-state)))



;;=============  placing object after scan ================

(defun pick-place-object (?non-graspable ?arm ?perceived-object
                          ?object-orientation ?place-pose ?search-pose ?base-sides)
  
  (let* ((?look-pose (cl-tf2:make-pose-stamped
                      "map" 0
                     (cl-tf2:origin (man-int:get-object-pose-in-map ?perceived-object))
                     (cl-tf2:orientation ?search-pose))))
  (move (desig:reference
         (desig:a location (locate ?look-pose) (arm (first ?arm))))))
        

  (let* ((?perceived-object-2
           (perceive-object ?search-pose (desig:desig-prop-value ?perceived-object :type)))

         (?grasp (which-sides-can-be-grasped (side-location ?perceived-object-2 ?base-sides)
                  ?non-graspable))

         (?current-grasp
           (grasp-object-with-handling
            ?arm
            ?grasp
            ?perceived-object-2)))
    

  (let* ((?orientation-of-old-pose (cram-tf:orientation (cram-tf:robot-current-pose)))
         (?new-position (desig:reference
                        (desig:a location (locate ?place-pose) (arm (first ?arm)))))
         (?orientation-of-new-pose (cl-tf2:orientation ?new-position))
         (?place-pose-origin (cl-tf2:origin ?place-pose)))
    
    (move ?new-position)
    
    (cpl:with-retry-counters ((place-retry 3))
    (cpl:with-failure-handling
        ((common-fail:manipulation-low-level-failure (e)
           (roslisp:ros-warn (cashier-demo) "failure happened: ~a~% changing grasp" e)
           (cpl:do-retry place-retry
             (move (desig:reference
                        (desig:a location (locate ?place-pose) (arm (first ?arm)))))
           (cpl:retry))
           (cpl:fail 'high-level-grasp-failure)))
      
    (place-object-with-handling
     (place-after-scan ?place-pose-origin
                       ?object-orientation
                       ?orientation-of-old-pose
                       ?orientation-of-new-pose)
     ?arm  ?current-grasp)))
    )))

(defun place-after-scan (pose orientation q1 q2)
  (let* ((angle-between-location-quaterions (cl-tf2:angle-between-quaternions q1 q2))
         
         (new-orientation (cl-tf2:q*
                           orientation
                           (cl-tf2:axis-angle->quaternion
                            (cl-tf2:make-3d-vector 0 0 1)
                            angle-between-location-quaterions))))
        
   (cl-transforms-stamped:make-pose-stamped
   "map" 0.0
   pose
   new-orientation)))

;; ========== algin object rotation with robot ===================
(defun points-to-3d-vector (point-1 point-2)
  (cl-tf2:make-3d-vector
   (- (first point-2) (first point-1))
   (- (second point-2) (second point-1))
   (- (third point-2) (third point-1))))
       

(defun 3d-vectors-to-angle (vector-1 vector-2)
  (print (cl-tf2:dot-product vector-1 vector-2))
  (acos (/ (cl-tf2:dot-product vector-1 vector-2)
           (* (3d-vector-magnitude vector-1)
              (3d-vector-magnitude vector-2))))
)

(defun 3d-vector-magnitude (vector)
  "Returns the magnitude of the vector"
  (sqrt (+ (expt (cl-tf2:x vector) 2)
           (expt (cl-tf2:y vector) 2)
           (expt (cl-tf2:z vector) 2))))

(defun points-to-angle (p1 p2 p3)
  "creates 2 vectors with p1 as the origin and going towards p2 and p3
   then calculates the angle betwen the two vectors"
  (let*((point-1 (list (first p1) (second p1) 0))
        (point-2 (list (first p2) (second p2) 0))
        (point-3 (list (first p3) (second p3) 0)))
  (3d-vectors-to-angle
   (points-to-3d-vector point-1 point-2)
   (points-to-3d-vector point-1 point-3))))


(defun get-correct-side (transformed-sides-base side)
    (loop for x in transformed-sides-base
          do
             (if (equal (first x) side)
                 (return (second x)))))

(defun angle-direction (robot-base right-side left-side angle)
  (print "angle-direction")
  
  (let* ((right-side-mangnitude (3d-vector-magnitude
                                 (points-to-3d-vector robot-base right-side)))
         (left-side-magnitude (3d-vector-magnitude
                               (points-to-3d-vector robot-base left-side))))
    (if (<= right-side-mangnitude left-side-magnitude)
        (- angle)
         angle)
    )
  )

    
(defun align-object (object-name base-sides object-location ?perceived-object)
  (let* ((align-point (cram-tf:3d-vector->list
                       (cl-tf2:origin (cram-tf:robot-current-pose))))          
         
         (located (side-location ?perceived-object base-sides))
         
         (front-side (third located))
         (right-side (second located))
         (left-side (opposite-short right-side))
         (transformed-map-t-side (transforms-map-t-side
                                  object-name base-sides))
          
         (front-side-location (cram-tf:3d-vector->list (cl-tf2:origin
                                                        (get-correct-side
                                                         transformed-map-t-side
                                                         front-side))))
         
         (right-side-location (cram-tf:3d-vector->list (cl-tf2:origin
                                                        (get-correct-side
                                                         transformed-map-t-side
                                                         right-side))))
         
         (left-side-location (cram-tf:3d-vector->list (cl-tf2:origin
                                                        (get-correct-side
                                                         transformed-map-t-side
                                                         left-side))))
         
         (angle (points-to-angle object-location align-point front-side-location))
         
         (correct-angle-direction (angle-direction align-point
                                                   right-side-location
                                                   left-side-location
                                                   angle))
         
         (rotation (list 0 correct-angle-direction 0)))
    
    (cl-tf2:q*
            (cl-tf2:orientation (btr:object-pose object-name))
            (cl-tf2:euler->quaternion
             :ax 0
             :ay 0
             :az correct-angle-direction))))

(defun pick-place-align-object (object-name ?base-sides
                                 arm non-graspable
                                 ?perceived-object ?place-pose)
      
  (let* ((object-pose-in-map (man-int:get-object-pose-in-map ?perceived-object))
         (object-vector-list (cram-tf:3d-vector->list
                              (cl-tf2:origin object-pose-in-map)))
         (transformed (transforms-map-t-side object-name ?base-sides))
         (grasp (which-sides-can-be-grasped (side-location ?perceived-object ?base-sides)
                                            non-graspable))
         (align (align-object object-name ?base-sides object-vector-list ?perceived-object)))

        (spawn-side-visualisation transformed "pre-alignment")      
    
    (let* ((?current-grasp
           (grasp-object-with-handling
             arm
             grasp
             ?perceived-object)))
  
    (place-object-with-handling
     (cl-transforms-stamped:make-pose-stamped
      "map" 0.0
      (cl-tf2:origin ?place-pose)
      align)
      arm
      ?current-grasp))
      
    (spawn-side-visualisation (transforms-map-t-side object-name ?base-sides) "after-alignment"))
  )
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
    (print "sides to be grasped ==============")
    (print sides-list)

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

