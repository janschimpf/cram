(in-package :cashier)
;;@author Jan Schimpf

;;=====================setting the sides =============================================
;;Sets the sides that are needed for scanning and object handling
(defun set-sides (?perceived-object object-size-list)
  (let* ((object-x-size (first object-size-list))
         (object-y-size (second object-size-list))
         (object-z-size (third object-size-list))
         (object-rotation (cl-tf2:orientation (man-int:get-object-pose-in-map ?perceived-object)))
       (side-list (list
                   (list :top (cl-transforms:make-3d-vector 0 0 object-z-size))
                   (list :bottom (cl-transforms:make-3d-vector 0 0 (- object-z-size)))
                   (list :left-side (cl-transforms:make-3d-vector 0 object-y-size 0))
                   (list :right-side (cl-transforms:make-3d-vector 0 (- object-y-size ) 0))
                   (list :front (cl-transforms:make-3d-vector object-x-size 0 0 ))
                   (list :back (cl-transforms:make-3d-vector (- object-x-size ) 0 0))))

         (side (mapcar (lambda (x) (list (first x) (cl-tf:make-pose
                         (second x)
                         object-rotation)))
                        side-list)))
      side))

;; ================locating the current-side================

;;returns relative bottom, right and front sides in form of a list.

(defun side-location (?perceived-object ?b-sides)
  (let* ((object-vector (cram-tf:3d-vector->list (cl-tf2:origin
                                                   (man-int:get-object-pose
                                                    ?perceived-object))))
         (transformed-sides (transform-b-sides-t-x ?b-sides
                                                   (man-int:get-object-transform
                                                    ?perceived-object)))
         (right-list (cram-tf:list->3d-vector
                      (vector-offset object-vector
                                     (list 0 -0.5 0))))
         (front-list (cl-tf2:make-3d-vector 0 0 (third object-vector)))
         (bottom-list (cram-tf:list->3d-vector
                       (vector-offset object-vector
                                      (list 0 0 -0.3))))
         (bottom (caar (shortest-distance
                        (side-distance-to-point transformed-sides bottom-list))))
         (updated-sides (mass-filter-sides
                         transformed-sides
                         (list bottom (opposite-short bottom))))
         (right (caar (shortest-distance
                       (side-distance-to-point updated-sides right-list))))
         (second-update (mass-filter-sides
                         updated-sides
                         (list right (opposite-short right))))
         (front (caar (shortest-distance
                       (side-distance-to-point second-update front-list)))))
    (list bottom right front )))

(defun side-distance-to-point (transformed-sides point)
  (mapcar (lambda (x) (list (first x) (cl-tf2:v-dist
                       (cl-tf2:origin (second x))
                       point)))
          transformed-sides))

(defun mass-filter-sides (sides sides-to-be-removed)
  (remove nil (mapcar (lambda (x)
                        (if (contains-element-negative (car x) sides-to-be-removed)
                            x
                            nil))
                      sides)))

(defun vector-offset (vector offset-list)
  (let ((x (first offset-list))
        (y (second offset-list))
        (z (third offset-list)))
  (list (+ (first vector) x)
        (+ (second vector) y)
        (+ (third vector) z))))


(defun filter-sides (side-list already-located-side)
  (remove nil (mapcar (lambda (x)
                        (if (equal (car x) already-located-side)
                            nil
                            x))
                      side-list)))

;; ==================== Transforms ===================
(defun transforms-map-T-side (object-pose side-poses)
  (mapcar (lambda (x)
            (let* ((map-T-object (cl-transforms:pose->transform object-pose))
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

    (let*((?loc *place-position*)
        (?arm (list :left)))
        
    (move (desig:reference (desig:a location (locate ?loc) (arm ?arm))))
    )))

(defun Test-call (test)
  (urdf-proj:with-simulated-robot

  (let* ((?type :small-book)
         (?loc *place-position*)
         (?size (list 0.04 0.06 0.12))
         (?perceived-object (perceive-object ?loc ?type))         
         (?base-sides (set-sides ?perceived-object ?size))
         (transformed (transforms-map-t-side (man-int:get-object-pose-in-map ?perceived-object)
                                             ?base-sides))
         (located-x (side-location ?perceived-object ?base-sides))
         (changes (side-changes located-x (list :front :top) (list nil)))
         (path (path-plan-next-side changes (list :front :top) (list nil) test)))
    ;;(spawn-side-visualisation transformed "test")
    ;; (print located-x)
    ;; (print ?base-sides)
    (print changes)
    (print path)
    )))

(defun test-input-change-side (test)
  (urdf-proj:with-simulated-robot
    (helper-test-change-side test))
)
  

(defun helper-test-change-side (goal-side)
   (let* ((?type :small-book)
          (?loc *place-position*)
          (?size '(0.04 0.06 0.12))
          (?arm '(:right))
          (?perceived-object (perceive-object ?loc ?type))
          (?base-sides (set-sides ?perceived-object ?size)))
     (change-side-test-call ?type ?loc ?base-sides goal-side
                            '(:front :top :left-side :back) '(nil) ?arm ?size)))

(defun change-side-test-call (?object-type ?scan-pose ?b-sides ?sides-to-check
                              ?non-graspable ?non-scanable ?arm ?object-size)
  
  (let* ((?perceived-object (perceive-object ?scan-pose ?object-type))
         (?sides-located (side-location ?perceived-object ?b-sides))
         (?check-side (next-side-to-check ?sides-located ?sides-to-check)))
           (if (not (equal nil ?check-side))
             (exe:perform
              (desig:an action
                        (:type :changing-side)
                        (:object ?perceived-object)
                        (:object-type ?object-type)
                        (:scan-pose ?scan-pose)
                        (:arm ?arm)
                        (:base-sides ?b-sides)
                        (:object-size ?object-size)
                        (:non-scanable ?non-scanable)
                        (:non-graspable ?non-graspable)
                        (:change-to-side ?check-side))))))


