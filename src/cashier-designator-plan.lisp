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


(defun cashier-object (&key
                         ((:object-type ?object-type))
                         ((:arm ?arm))
                         ((:non-scanable ?non-scanable))
                         ((:non-graspable ?non-graspable))
                         ((:goal-side ?goal-side))
                         ((:sides-base ?sides-base))
                         ((:object-size ?object-size))
                         ((:search-poses ?search-poses))
                         ((:scan-pose ?scan-pose))
                         ((:success-poses ?success-pose))
                         ((:failed-poses ?failed-pose))
                       &allow-other-keys)

  (declare (type keyword
                 ?object-type)
           
           (type list
                 ?object-size
                 ?arm
                 ?non-scanable
                 ?non-graspable
                 ?search-poses
                 ?scan-pose
                 ?success-pose
                 ?failed-pose))
  
  
  (let* ((?nav-pose (first ?search-poses)))
    (move (desig:reference (desig:a location (locate ?nav-pose) (arm (first ?arm))))))
  (print "moved to first look nav pose")
  
  (multiple-value-bind (?perceived-object)
      (perceive-object (first ?search-poses) ?object-type)
    
  (let* ((?object-name (desig:desig-prop-value ?perceived-object :name))
         (?base-sides (set-sides-helper ?object-name ?object-size))
         (?sides-transformed (transforms-map-t-side ?object-name ?base-sides)))
    (setf *base-sides* ?base-sides)

  ;; (if (not (car ?non-graspable))
  ;;     (setf ?non-graspable (check-object-size ?object-size)))

  ;; (if (not (car ?non-scanable))
  ;;     (setf ?non-scanable (prolog-shape ?object-type)))
  
  (print "check for non graspable sides")

    (let* ((object-pose (man-int:get-object-pose-in-map ?perceived-object))
           (grasp (which-sides-can-be-grasped
                   (locate-sides ?sides-transformed
                                 (cram-tf:3d-vector->list (cl-tf2:origin object-pose)))
                 ?non-graspable))
           (orientation (cl-tf2:orientation object-pose)))
    
      (pick-place-object (car ?scan-pose) (first ?search-poses)
                       grasp ?arm ?object-type
                       orientation *place-pose*))))
  
  (multiple-value-bind (?perceived-object)
      (perceive-object (first ?scan-pose) ?object-type)
    (let* ((?object-name (desig:desig-prop-value ?perceived-object :name)))
      
      (pick-place-alight-object ?object-name *base-sides* ?arm
                              ?object-type ?non-graspable ?perceived-object)

  
  (if (exe:perform (desig:an action
                             (:type :scanning)
                             (:arm ?arm)
                             (:non-scanable ?non-scanable)
                             (:non-graspable ?non-graspable)
                             (:object-name ?object-name)
                             (:object-type ?object-type)
                             (:object-size ?object-size)
                             (:goal-side ?goal-side)
                             (:sides-base *base-sides*)))
   
   (sucessful-scan ?object-type ?object-name *base-sides*
                   ?arm ?non-graspable (first ?scan-pose))
   
   (unsucessful-scan ?object-type ?object-name *base-sides*
                     ?arm ?non-graspable (first ?scan-pose)))
  

      )))



;;=============  placing object after scan ================

(defun sucessful-scan (?object-typ ?object-name ?sides-base ?arm
                       ?non-graspable-sides ?look)
  (print "object was succesfully scanned")

  (multiple-value-bind (?perceived-object)
      (perceive-object *place-position* ?object-typ)
    
    (let* ((object-pose-in-map (man-int:get-object-pose-in-map ?perceived-object))

           (grasp (which-sides-can-be-grasped
                   (locate-sides
                    (transforms-map-t-side ?object-name ?sides-base)
                    (cram-tf:3d-vector->list
                     (cl-tf2:origin object-pose-in-map)))
                   ?non-graspable-sides))
         
         (orientation (cl-tf2:orientation (man-int:get-object-pose-in-map ?perceived-object))))
    
    (pick-place-object
     *after-scan-nav-pose*
     ?look
     grasp
     ?arm
     ?object-typ
     orientation
     (car *success-poses-list*))
  (setf *success-poses-list* (cdr *success-poses-list*)))))


(defun unsucessful-scan (?object-typ ?object-name ?sides-base ?arm ?non-graspable-sides ?look)
  (print "object could not be scanned")
  (multiple-value-bind (?perceived-object)
      (perceive-object *place-position* ?object-typ)

  (let* ((object-pose-in-map (man-int:get-object-pose-in-map ?perceived-object))
         (grasp (which-sides-can-be-grasped
                 (locate-sides
                  (transforms-map-t-side ?object-name ?sides-base)
                  (cram-tf:3d-vector->list
                   (cl-tf2:origin object-pose-in-map)))
                 ?non-graspable-sides))
         (orientation (cl-tf2:orientation object-pose-in-map)))

    (pick-place-object
     *after-unsuccessful-scan-nav-pose*
     ?look
     grasp
     ?arm
     ?object-typ
     orientation
     (car *unsuccessful-poses-list*))
    
    (setf *unsuccessful-poses-list* (cdr *unsuccessful-poses-list*)))))


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

(defun pick-place-object (?move-pose ?look grasp-list
                         ?arm ?object-typ object-orientation
                         place-pose)
  
  (multiple-value-bind (?perceived-object)
      (perceive-object ?look ?object-typ)
    
      (grasp-object-with-handling
       ?arm
       grasp-list
       ?perceived-object)
    

  (let* ((orientation-of-old-pose (cram-tf:orientation (cram-tf:robot-current-pose)))
         (new-position (desig:reference (desig:a location (locate ?move-pose) (arm (first ?arm)))))
         (orientation-of-new-pose (cram-tf:orientation new-position)))
    
    (move new-position)
  
    (place-object-with-handling
     (place-after-scan place-pose
                       object-orientation
                       orientation-of-old-pose
                       orientation-of-new-pose)
     ?arm
     *current-grasp*)
)))

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
    (if (>= right-side-mangnitude left-side-magnitude)
        angle
        (- 0 angle))
    ))

    
(defun align-object (object-name sides-base object-location)
  (let* ((align-point (cram-tf:3d-vector->list
                       (cl-tf2:origin (cram-tf:robot-current-pose))))
                       
         
         (located (locate-sides
           (transforms-map-t-side object-name sides-base)
           object-location))
         
         (front-side (third located))
         (right-side (second located))
         (left-side (opposite-short right-side))
         (transformed-map-t-side (transforms-map-t-side
                                  object-name sides-base))
          
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
         
         (rotation (list 0 correct-angle-direction 0))
         
         (axis-found (finding-axis located rotation)))
    
    (cl-tf2:q*
            (cl-tf2:orientation (btr:object-pose object-name))
            (cl-tf2:euler->quaternion
             :ax (first axis-found)
             :ay (second axis-found)
             :az (third axis-found)))))

(defun pick-place-alight-object (object-name sides-base arm ?object-typ non-graspable
                                 ?perceived-object)
      
  (let* ((object-pose-in-map (man-int:get-object-pose-in-map ?perceived-object))
         (object-vector-list (cram-tf:3d-vector->list
                              (cl-tf2:origin object-pose-in-map)))
         (transformed (transforms-map-t-side object-name sides-base))
         (located (locate-sides transformed object-vector-list))
         (grasp (which-sides-can-be-grasped located non-graspable))
         (align (align-object object-name sides-base object-vector-list)))

        (spawn-side-visualisation transformed "pre-alignment")      
    
        (grasp-object-with-handling
                arm
                grasp
                ?perceived-object)
  
    (place-object-with-handling
     (cl-transforms-stamped:make-pose-stamped
      "map" 0.0
      (cl-tf2:origin *place-position*)
      align)
      arm
      *current-grasp*)
      
    (spawn-side-visualisation (transforms-map-t-side object-name sides-base) "after-alignment"))
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

(defun defaul-distance ()
  0.2)
