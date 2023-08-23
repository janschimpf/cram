(in-package :cashier)

;;=====================setting the sides =============================================
;;Sets the sides that are needed for scanning and object handling
(defun set-sides (object-name object-x-size object-y-size object-z-size)
  (let ((object-rotation (cl-tf2:orientation
                          (cram-tf::pose-stamped->pose (btr:object-pose object-name))))
       (side-list (list
                   (list :top (cl-transforms:make-3d-vector 0 0 object-z-size))
                   (list :bottom (cl-transforms:make-3d-vector 0 0 (- object-z-size)))
                   (list :left (cl-transforms:make-3d-vector 0 object-y-size 0))
                   (list :right (cl-transforms:make-3d-vector 0 (- 0 object-y-size ) 0))
                   (list :front (cl-transforms:make-3d-vector object-x-size 0 0 ))
                   (list :back (cl-transforms:make-3d-vector (- 0 object-x-size ) 0 0)))))
    (let ((side (mapcar (lambda (x) (list (first x) (cl-tf:make-pose
                         (second x)
                         object-rotation)))
                        side-list)))
      side)))

(defun set-sides-helper (object-name object-size-list)
  (set-sides object-name
             (first object-size-list)
             (second object-size-list)
             (third object-size-list)))

;; ================locating the current-side================

;;returns relative bottom, right and front sides in form of a list.
(defun locate-sides (side-list object-vector)
  (let* ((right-list (vector-offset object-vector (list +0.1 0 0)))
         (front-list (vector-offset object-vector (list 0 +0.1 0)))
         (bottom-list  (vector-offset object-vector (list 0 0 -0.1)))
         
         (right (caar (shortest-distance-between-all-sides side-list right-list)))
         (updated-side-list-1 (filter-sides (filter-sides side-list right) (opposite-short right)))
         
         (front (caar (shortest-distance-between-all-sides updated-side-list-1 front-list)))
         (updated-side-list-2 (filter-sides (filter-sides updated-side-list-1 front) (opposite-short front)))
         
         (bottom (caar (shortest-distance-between-all-sides updated-side-list-2 bottom-list))))
    (list bottom right front)
    ))

(defun vector-offset (vector offset-list)
  (let ((x (first offset-list))
        (y (second offset-list))
        (z (third offset-list)))
  (list (+ (first vector) x)
        (+ (second vector) y)
        (+ (third vector) z))))


(defun filter-sides (side-list already-located-side)
  (remove nil
          (mapcar (lambda (x)
                    (if (equal (car x) already-located-side)
                        nil
                        x))
                  side-list) 
  ))

;; ==================== Transforms ===================
(defun transforms-map-T-side (object-name side-poses)
  (mapcar (lambda (x)
            (let* ((map-T-object (cl-transforms:pose->transform (btr:object-pose object-name)))
                   (object-T-side (cl-transforms:pose->transform (first (cdr x))))
                   (map-T-side  (cl-transforms:transform* map-T-object object-T-side)))
    (list (car x) (cl-transforms:transform->pose map-T-side))))
          side-poses))

(defun transform-b-sides-T-X (b-sides transform)
  (mapcar (lambda (x)
            (let* ((map-T-object  transform)
                   (object-T-side (cl-transforms:pose->transform (first (cdr x))))
                   (map-T-side  (cl-transforms:transform* map-T-object object-T-side)))
    (list (car x) (cl-transforms:transform->pose map-T-side))))
          b-sides))


;;==================Test-setup ======================
(defun Test-setup ()
  
  (spawn-breakfast)
  (urdf-proj:with-simulated-robot

    (let*((?loc *place-position-other-side*)
        (?arm (list :left)))
        
    (move (desig:reference (desig:a location (locate ?loc) (arm ?arm))))

  
    )))

(defun Test-call ()
  (urdf-proj:with-simulated-robot

  (let* ((?type :bottle)
         (?loc *place-position*)
         (?size (list 0.08 0.04 0.15))
         (?perceived-object (perceive-object ?loc ?type))
         (?object-name (desig:desig-prop-value ?perceived-object :name))
         (?base-sides (set-sides-helper ?object-name ?size))
         (?object-trans (man-int:get-object-transform ?perceived-object))
         (transformed (transforms-map-t-side ?object-name ?base-sides))
         (transformed-x (transform-b-sides-t-x ?base-sides ?object-trans))
         (?object-vector (cram-tf:3d-vector->list (cl-tf2:origin
                                                   (man-int:get-object-pose-in-map
                                                    ?perceived-object))))
         (?o-vector (cram-tf:3d-vector->list (cl-tf2:origin
                                              (man-int:get-object-pose ?perceived-object))))
         (located (locate-sides transformed ?object-vector))
         (located-x (side-location ?perceived-object ?base-sides)))
    (spawn-side-visualisation transformed "test")
    
    (print "==========located-sides-transform")
    (print located)
    (print "==========located-towards-robot-base")
    (print located-x)
    (print (man-int:calculate-object-faces ?object-trans))
    )))

(defun test-change ()
  (urdf-proj:with-simulated-robot
    (btr-utils:move-object 'breakfast-cereal-1
                           (cl-tf2:make-pose-stamped "map" 0
                                                     (cl-tf2:make-3d-vector -2 1.3 0.8)
                                                     (cl-tf2:euler->quaternion :ax pi :ay 0 :az 0)))
    (btr:simulate btr:*current-bullet-world* 100))
)

(defun helper-test-change-side ()
    (urdf-proj:with-simulated-robot
   (let* ((?type :breakfast-cereal)
          (?loc *place-position-other-side*)
          (?size '(0.10 0.05 0.15))
          (?arm '(:left))
          (?perceived-object (perceive-object ?loc ?type))
         (?object-name (desig:desig-prop-value ?perceived-object :name))
         (?base-sides (set-sides-helper ?object-name ?size)))
     (change-side-test-call ?type ?loc ?base-sides (list :front :back :bottom :top)
                            '(:right :left) '(nil) ?arm ?size))))

(defun change-side-test-call (?object-type ?scan-pose ?b-sides ?sides-to-check
                              ?non-graspable ?non-scanable ?arm ?object-size)
  (let* ((?perceived-object (perceive-object ?scan-pose ?object-type))
         (?sides-located (side-location ?perceived-object ?b-sides))
         (?object-vector (cram-tf:3d-vector->list
                          (cl-tf2:origin
                           (man-int:get-object-pose-in-map ?perceived-object))))
         (?transform (man-int:get-object-transform ?perceived-object))
         (?sides-in-map (transform-b-sides-t-x ?b-sides ?transform))
         ;(?located-sides (locate-sides ?sides-in-map ?object-vector))
         )

    (let* ((?check-side (next-side-to-check ?sides-located ?sides-to-check))
           (?object (extended-object-desig ?perceived-object
                                           ?object-size ?non-graspable
                                           ?non-scanable ?check-side ?b-sides)))
             (print "next side")

           (if (not (equal nil ?check-side))
             (exe:perform
              (desig:an action
                        (:type :changing-side)
                        (:object ?object)
                        (:scan-pose ?scan-pose)
                        (:arm ?arm)
                        (:change-to-side ?check-side)
                        (:sides-base ?b-sides)
                        (:sides-transformed ?sides-in-map)
                        (:object-size ?object-size)
                        (:object-vector ?object-vector)))))))

(defun test-run ()
  (roslisp-utilities:startup-ros)
  (test-setup)
  (test-call))

(defun side-location (?perceived-object ?b-sides)
  (let* ((object-vector (cram-tf:3d-vector->list (cl-tf2:origin
                                                   (man-int:get-object-pose
                                                    ?perceived-object))))
         
         (transformed-sides (transform-b-sides-t-x ?b-sides
                                                   (man-int:get-object-transform
                                                    ?perceived-object)))
         
         (right-list (cram-tf:list->3d-vector  (list 0 -0.5 (third object-vector))))
         
         (front-list (cl-tf2:make-3d-vector 0 0 (third object-vector)))
         
         (bottom-list (cram-tf:list->3d-vector
                       (vector-offset object-vector (list 0 0 -0.1))))

         (bottom (caar (shortest-distance
                        (side-distance-to-point transformed-sides bottom-list))))
         
         (updated-sides (mass-filter-sides
                         transformed-sides
                         (list bottom (opposite-short bottom))))

         (front (caar (shortest-distance
                       (side-distance-to-point updated-sides front-list))))

         (second-update (mass-filter-sides
                         updated-sides
                         (list front (opposite-short front))))
         
         (right (caar (shortest-distance
                       (side-distance-to-point second-update right-list)))))
    (list bottom right front )))

(defun side-distance-to-point (transformed-sides point)
  (mapcar (lambda (x) (list (first x) (cl-tf2:v-dist
                       (cl-tf2:origin (second x))
                       point)))
          transformed-sides))

(defun mass-filter-sides (sides sides-to-be-removed)
  (remove nil
          (mapcar
           (lambda (x)
             (if (contains-element-negative (car x) sides-to-be-removed)
                 x
                 nil))
             sides)
    ))
