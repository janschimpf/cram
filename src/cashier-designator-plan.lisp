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

(defun opposite-short (side)
  (cdaar (prolog:prolog `(or (opposite ,side ?x)
                             (opposite ?x ,side)))))
(defun axis-short (side)
  (cdaar (prolog:prolog `(axis ,side ?x))))

(def-fact-group sides-predicates (opposite)
  (<- (opposite :top :bottom))
  (<- (opposite :left :right))
  (<- (opposite :front :back))

  (<- (axis :front y))
  (<- (axis :back -y))
  (<- (axis :right x))
  (<- (axis :left -x))
  (<- (axis :top -z))
  (<- (axis :bottom z))
  )



;; ================locating the current-side================

;;returns relative bottom, right and front sides in form of a list.
(defun locate-sides (side-list object-vector)
  (let* ((right-list (vector-offset object-vector (list 0 -0.05 0)))
         (front-list (vector-offset object-vector (list +0.05 0 0)))
         (scan-list  (vector-offset object-vector (list 0 0 -0.05))))
    (let* ((right (caar (shortest-distance-between-all-sides side-list right-list)))
           (front (caar (shortest-distance-between-all-sides side-list front-list)))
           (bottom (caar (shortest-distance-between-all-sides side-list scan-list))))
    (list bottom right front)
    )))

(defun vector-offset (vector offset-list)
  (let ((x (first offset-list))
        (y (second offset-list))
        (z (third offset-list)))
  (list (+ (first vector) x)
        (+ (second vector) y)
        (+ (third vector) z))))


;; ============= executing the path plan ====================

;; executes the path plan 

(defun origin->list (object-name)
    (cram-tf:3d-vector->list (cl-tf2:origin (btr:object-pose object-name))))

;; iterates over the to scan sides.
;; first gets the current bottom side, front side and right side
;; then the path plan the path, execute the path, update object, scan
(defun scan-object (&key
                      ((:object-type ?object-type))
                      ((:object-name ?object-name))
                      ((:arm ?arm))
                      ((:non-scanable ?non-scanable))
                      ((:non-graspable ?non-graspable))
                      ((:object-size ?object-size))
                      ((:goal-side ?goal-side))
                      ((:sides-base ?sides-base))
                      ((:sides-transformed ?sides-transformed))
                      ((:sides-to-check ?sides-to-check))
                    &allow-other-keys)
  
  (declare (type keyword
                 ?object-type
                 ?goal-side)
           
           (type list
                 ?sides-base
                 ?sides-transformed
                 ?object-size
                 ?sides-to-check
                 ?arm
                 ?non-scanable
                 ?non-graspable)
           
           (type symbol
                 ?object-name))
  (setf ?sides-transformed (transforms-map-t-side ?object-name ?sides-base))

  (cpl:with-retry-counters ((scan-counter-retries (length ?sides-to-check)))
    (cpl:with-failure-handling 
    ((common-fail:high-level-failure (e)
       (roslisp:ros-warn (cashier-demo) "Falure happend: ~a~% adjusting place postion" e)
       (cpl:do-retry scan-counter-retries
         (let* ((?object-vector (cram-tf:3d-vector->list
                                (cl-tf2:origin (btr:object-pose ?object-name))))
                (?located-sides (locate-sides ?sides-transformed ?object-vector)))

           (setf ?sides-to-check (remove-element-from-list ?sides-to-check
                                                           (first ?located-sides)))

           (let* ((?check-side (next-side-to-check ?located-sides ?sides-to-check)))

           (if (equal nil ?check-side)
                 (cpl:fail 'common-fail:high-level-failure))

             (exe:perform
              (desig:an action
                      (:type :changing-side)
                      (:object-name ?object-name)
                      (:object-type ?object-type)
                      (:arm ?arm)
                      (:non-scanable ?non-scanable)
                      (:non-graspable ?non-graspable)
                      (:change-to-side ?check-side)
                      (:sides-base ?sides-base)
                      (:sides-transformed ?sides-transformed)
                      (:object-size ?object-size)
                      (:object-vector ?object-vector)))
           (setf ?sides-transformed (transforms-map-t-side ?object-name ?sides-base))))
           (cpl:retry))
       (cpl:fail 'common-fail:high-level-failure)))
      (if (not (scan ?object-name ?goal-side ?sides-transformed))
          (cpl:fail 'common-fail:high-level-failure))
      T)))

(defun next-side-to-check (located-sides sides-to-check)
  (let* ((right (second located-sides))
         (left (opposite-short (second located-sides)))
         (top (opposite-short (first located-sides)))
         (front (third located-sides))
         (back (opposite-short (third located-sides)))
         (side-list (list right left front back top)))
    ;; check list if element(s) are part of sides-to-check
    ;; first element that is in the side-to-check list is the one we target next
    (loop for x in side-list
          do
             (when (not (null (member x sides-to-check)))
               (return x)))))

  

;;; ===== pose changes for placing the object with a different orientation / side ======

(defun place-pose-stability-adjustment
    (orientation origin-list
     object-type object-name
     offset-start)
  (let ((offset offset-start))
    (cpl:with-retry-counters ((pose-adjustment-retries 15))
    (cpl:with-failure-handling 
    ((common-fail:high-level-failure (e)
       (roslisp:ros-warn (cashier-demo) "Falure happend: ~a~% adjusting place postion" e)
       (cpl:do-retry pose-adjustment-retries
         (setf offset (- offset 0.001))
         (cpl:retry))
       (cpl:fail 'common-fail:high-level-failure)))
      

       (let* ((target-pose
           (cl-tf2:make-pose-stamped "map" 0 
                                     (cram-tf:list->3d-vector
                                      (vector-offset origin-list (list 0 0 offset)))
                                     orientation )))
         (let ((?target-pose-test (btr:ensure-pose target-pose)))
           (print "test return before target-pose")
           (shortcut-pose-stability (object-desig-shortcut object-type)
                                    (a location (pose ?target-pose-test)))
           ?target-pose-test
           ))))))


;;takes care of the the needed orientation change to turn the object
(defun orientation-change (object-name move located-sides)
  (let* ((turn
           (cond
             ((string-equal "right-turn" move)
              (list (/ pi 2) 0  0))
             ((string-equal "left-turn" move)
              (list (-(/ pi 2)) 0 0))
             ((string-equal "back-turn" move)
              (list 0 (- (/ pi 2)) 0))
             ((string-equal "front-turn" move)
              (list 0 (/ pi 2) 0))
             ((string-equal "flip" move)
              (list pi 0 0))
             ((string-equal "left-roation" move)
              (list 0 0 (- (/ pi 2))))
             ((string-equal "back-turn" move)
              (list 0 0 (/ pi 2)))
             (t (print move))))
         (axis-bottom (axis-short (first located-sides)))
         (axis-right (axis-short (third located-sides)))
         (axis-front (axis-short (second located-sides)))
         (axis-list (list axis-right axis-front axis-bottom))
         (result (mapcar (lambda (x) (matching-axis x turn))
                         axis-list)))
    (cl-tf2:q*
            (cl-tf2:orientation (btr:object-pose object-name))
            (cl-tf2:euler->quaternion
             :ax (first result)
             :ay (third result)
             :az (second result)))))

(defun matching-axis (axis list-match)
  (cond
    ((string-equal "X" axis)
     (first list-match))
    ((string-equal "-X" axis)
     (- (first list-match)))
    ((string-equal "Y" axis)
     (second list-match))
    ((string-equal "-Y" axis)
     (- (second list-match)))
    ((string-equal "Z" axis)
     (third list-match))
    ((string-equal "-Z" axis)
     (- (third list-match)))))
    
    
    


(defun orientation-change-new (object-name move bottom-side)
  (let* ((turn
           (cond
             ((string-equal "right-turn" move)
              (list (/ pi 2) 0  0))
             ((string-equal "left-turn" move)
              (list (-(/ pi 2)) 0 0))
             ((string-equal "back-turn" move)
              (list 0 (- (/ pi 2)) 0))
             ((string-equal "front-turn" move)
              (list 0 (/ pi 2) 0))
             ((string-equal "flip" move)
              (list pi 0 0))
             ((string-equal "left-roation" move)
              (list 0 0 (- (/ pi 2))))
             ((string-equal "back-turn" move)
              (list 0 0 (/ pi 2)))
             (t (print move))))) ;; change into throwing error
    (cond
      ((string-equal "bottom" bottom-side)
       (cl-tf2:q*
            (cl-tf2:orientation (btr:object-pose object-name))
            (cl-tf2:euler->quaternion
             :ax (first turn)
             :ay (second turn)
             :az (third turn))))
      ((string-equal "top" bottom-side)
       (cl-tf2:q*
            (cl-tf2:orientation (btr:object-pose object-name))
            (cl-tf2:euler->quaternion
             :ax (first turn)
             :ay (second turn)
             :az (third turn))))
      ((or (string-equal "left" bottom-side)
           (string-equal "right" bottom-side))
       (cl-tf2:q*
            (cl-tf2:orientation (btr:object-pose object-name))
            (cl-tf2:euler->quaternion
             :ax (first turn)
             :ay (third turn)
             :az (second turn))))
      
      ((or (string-equal "front" bottom-side)
           (string-equal "back" bottom-side))
       (cl-tf2:q*
            (cl-tf2:orientation (btr:object-pose object-name))
            (cl-tf2:euler->quaternion
             :ax (first turn)
             :ay (third turn)
             :az (second turn))))
      (t (print turn)) ;; change into throwing error
          
  )))

;; takes care of the x-y-z pose when placing the object with the goal of not placing object
;; inside the table
(defun vector-change (place-vector bottom-side object-size)
  (let ((object-depth (first object-size))
        (object-width (second object-size))
        (object-height (third object-size)))
  (cond
    ((equal :right bottom-side)
     (vector-offset place-vector
                    (list 0 object-depth 0.04)))
    ((equal :left bottom-side)
     (vector-offset place-vector
                    (list 0 object-depth 0.04)))
    ((equal :top bottom-side)
     (vector-offset place-vector
                    (list 0 0 0.04)))
    ((equal :bottom bottom-side)
     (vector-offset place-vector
                    (list 0 0 0.04)))
    ((equal :front bottom-side)
     (vector-offset place-vector
                    (list -0.05 0 0.04)))
    ((equal :back bottom-side)
     (vector-offset place-vector
                    (list 0 (/  object-width 4) 0.04)))
    (t (print "not sure how we got here but something is wrong, vector-change")))))

;;3-sides
;;list of sides of the 



(defun object-desig-shortcut (?object-type)
  (desig:an object (type ?object-type)))

(defun shortcut-pose-stability (object-desig placing-location)
  (proj-reasoning:check-placing-pose-stability
   object-desig
   placing-location))


(defun change-side (&key
                      ((:plan ?plan))
                      ((:object-type ?object-type))
                      ((:object-name ?object-name))
                      ((:object-size ?object-size))
                      ((:arm ?arm))
                      ((:side-goal ?side-goal))
                      ((:sides-base ?sides-base))
                    &allow-other-keys)
  (declare (type list ?plan ?object-size ?sides-base ?arm)
           (type keyword ?object-type ?side-goal)
           (type symbol ?object-name)) 
  (loop for move in (remove nil ?plan)
        do
           (let* ((object-vector (cram-tf:3d-vector->list
                                (cl-tf2:origin (btr:object-pose ?object-name))))
                  (3d-list (vector-change
                           (cram-tf:3d-vector->list (cl-tf2:origin *place-pose*))
                             ?side-goal
                             ?object-size))
                  (located-sides (locate-sides
                                  (transforms-map-t-side ?object-name ?sides-base)
                                 object-vector))
                  (orientation (orientation-change ?object-name move located-sides))
                  (?grasp (which-sides located-sides move)))
             (setf *sides-log* (append (list ?grasp (list move) located-sides) *sides-log*))
             (let ((target
                     (place-pose-stability-adjustment
                      orientation
                      3d-list
                      ?object-type
                      ?object-name
                      0)))
               (execute-change-side ?object-type ?arm ?grasp target)
               ))))

(defun execute-change-side (?object-type arm grasp target-pose)
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
                arm *place-pose* (first grasp))))
  (move *place-nav-pose*)

  (place-object target-pose arm :?left-grasp (first grasp)))




(defun remove-element-from-list (list element)
  (remove nil (mapcar (lambda (x) (if (equal x element)
                          nil
                          x))
                          list)))

(defun which-sides (located-sides move)
  (let* ((top (opposite-short (first located-sides)))
         (front (third located-sides))
         (right (second located-sides))
         (left (opposite-short right)))
  (cond
    ((string-equal "right-turn" move)
     (list front left top))
    ((string-equal "left-turn" move)
     (list front right top))
    ((string-equal "front-turn" move)
     (list right left top))
    ((string-equal "back-turn" move)
     (list right left front))
    ((string-equal "flip" move)
     (list right left front))
    ((string-equal "left-rotation" move)
     (list left front top))
    ((string-equal "right-rotation" move)
     (list right front top))
    (t (print "test")) ;;throw error
  )))
