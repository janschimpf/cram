(in-package :cashier)


;; ================locating the current-side================

;;returns relative bottom, right and front sides in form of a list.
(defun locate-sides (side-list object-vector)
  (let* ((right-list (vector-offset object-vector (list 0 -0.1 0)))
         (front-list (vector-offset object-vector (list +0.1 0 0)))
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

;; ============= executing the path plan ====================



(defun origin->list (object-name)
  (cram-tf:3d-vector->list (cl-tf2:origin (btr:object-pose object-name))))

;; executes the path plan 
;; iterates over the to scan sides.
;; first gets the current bottom side, front side and right side
;; then the path plan the path, execute the path, update object, scan
(defun scan-plan (&key
                      ((:object-type ?object-type))
                      ((:object-name ?object-name))
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
                 ?non-graspable)
           
           (type symbol
                 ?object-name))
  
  (print "before scan start")
  (cpl:with-retry-counters ((scan-counter-retries (length ?sides-to-check)))
    (cpl:with-failure-handling 
    ((common-fail:high-level-failure (e)
       (roslisp:ros-warn (cashier-demo) "Falure happend: ~a~% adjusting side to be scanned" e)
       (cpl:do-retry scan-counter-retries
         (let* ((?perceived-object (perceive-object ?scan-pose ?object-type))
                (?object-vector (cram-tf:3d-vector->list
                                 (cl-tf2:origin
                                  (man-int:get-object-pose-in-map ?perceived-object))))

                (?sides-in-map (transforms-map-t-side ?object-name ?b-sides))
                (?located-sides (locate-sides ?sides-in-map ?object-vector)))


           (setf ?sides-to-check (remove-element-from-list ?sides-to-check
                                                           (first ?located-sides)))
           

           (let* ((?check-side (next-side-to-check ?located-sides ?sides-to-check))
                  (?object (extended-object-desig ?perceived-object
                                                  ?object-size ?non-graspable
                                                  ?non-scanable ?check-side ?b-sides)))

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
                        (:object-vector ?object-vector))))
             

             ))
           (cpl:retry))
       (cpl:fail 'common-fail:high-level-failure)))
      (let ((?perceived-object (perceive-object ?scan-pose ?object-type)))
        (print "scanning")
        (print ?sides-to-check)
      (if (equal nil ?sides-to-check)
          nil
          (if (not (scan ?perceived-object ?b-sides))
              (cpl:fail 'common-fail:high-level-failure)
              T))))))

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
