(in-package :cashier)

;; =======side-changing ===================
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
          
          (list "front-turn" (opposite-short front)
                (list (opposite-short front)
                      front
                      bottom))
          (list "back-turn" front
                (list front
                      bottom
                      (opposite-short right))))))

(defun opposite-short (side)
  (cdaar (prolog:prolog `(or (opposite ,side ?x)
                             (opposite ?x ,side)))))


(def-fact-group sides-predicates (opposite)
  (<- (connection front right))
  (<- (connection front left))
  (<- (connection front top))
  (<- (connection front bottom))
  (<- (connection back right))
  (<- (connection back left))
  (<- (connection back top))
  (<- (connection back bottom))
  (<- (connection top left))
  (<- (connection top right))
  (<- (connection bottom left))
  (<- (connection bottom right))
  
  (<- (opposite :top :bottom))
  (<- (opposite :left :right))
  (<- (opposite :front :back)))



;; ================locating the current-side================

;;returns relative bottom, right and front sides in form of a list.
(defun locate-sides (side-list object-vector)
  (let* ((right-list (vector-offset object-vector (list 0 -0.05 0)))
         (front-list (vector-offset object-vector (list -0.05 0 0)))
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
                      ((:object-size ?object-size))
                      ((:goal-side ?goal-side))
                      ((:sides-base ?sides-base))
                      ((:sides-transformed ?sides-transformed))
                      ((:sides-to-check ?sides-to-check))
                    &allow-other-keys)
  
  (declare (type keyword ?object-type ?goal-side)
           (type list ?sides-base ?sides-transformed ?object-size ?sides-to-check)
           (type symbol ?object-name))
  (setf ?sides-transformed (transforms-map-t-side ?object-name ?sides-base))
  (setf *sides-log* (append ?sides-to-check *sides-log*))

  (cpl:with-retry-counters ((scan-counter-retries (length ?sides-to-check)))
    (cpl:with-failure-handling 
    ((common-fail:high-level-failure (e)
       (roslisp:ros-warn (cashier-demo) "Falure happend: ~a~% adjusting place postion" e)
       (cpl:do-retry scan-counter-retries
         (setf ?sides-to-check (cdr ?sides-to-check))
         (let* ((?object-vector (cram-tf:3d-vector->list
                                (cl-tf2:origin (btr:object-pose ?object-name))))
                (?arm :left)
                (?check-side (first ?sides-to-check)))
           (let* ((?grasp (reverse (locate-sides ?sides-transformed ?object-vector))))
             (if (equal nil ?check-side)
                 (cpl:fail 'common-fail:high-level-failure))

             (exe:perform
              (desig:an action
                      (:type :changing-side)
                      (:object-name ?object-name)
                      (:object-type ?object-type)
                      (:arm ?arm)
                      (:grasp ?grasp)
                      (:change-to-side ?check-side)
                      (:sides-transformed ?sides-transformed)
                      (:object-size ?object-size)
                      (:object-vector ?object-vector)))
           (setf ?sides-transformed (transforms-map-t-side ?object-name ?sides-base))))
           (cpl:retry))
       (cpl:fail 'common-fail:high-level-failure)))
      (if (not (scan ?object-name ?goal-side ?sides-transformed))
          (cpl:fail 'common-fail:high-level-failure))
      T)))


;;; ===== pose changes for placing the object with a different orientation / side ======

(defun grasping-direction (located-side-list move)
  (cond
    ((string-equal "right-turn" move)
    (print "test right-turn"))
    ((string-equal "left-turn" move)
     (print "test left-turn"))
    ((string-equal "front-turn" move)
     (print "test front-turn"))
    ((string-equal "back-turn" move)
     (print "test back-turn"))
    (t (print "something went wrong")))
  )

(defun place-pose-stability-adjustment (orientation origin-list object-type object-name offset-start)
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
(defun orientation-change (object-name move)
  (cond
    ((string-equal "right-turn" move)
     (cl-tf2:q*
            (cl-tf2:orientation (btr:object-pose object-name))
            (cl-tf2:euler->quaternion :ax (/ pi 2) :ay 0 :az 0)))
    ((string-equal  "left-turn" move)
     (cl-tf2:q*
            (cl-tf2:orientation (btr:object-pose object-name))
            (cl-tf2:euler->quaternion :ax (-(/ pi 2)) :ay 0 :az 0)))
    ((string-equal "front-turn" move)
     (cl-tf2:q*
            (cl-tf2:orientation (btr:object-pose object-name))
            (cl-tf2:euler->quaternion :ax 0 :ay (/ pi 2) :az 0)))
    ((string-equal "back-turn" move)
     (cl-tf2:q*
            (cl-tf2:orientation (btr:object-pose object-name))
            (cl-tf2:euler->quaternion :ax 0 :ay (- (/ pi 2)) :az 0)))
   (t (print move))))


;; takes care of the x-y-z pose when placing the object with the goal of not placing object
;; inside the table
(defun vector-change (place-vector bottom-side object-size)
  (let ((object-depth (first object-size))
        (object-width (second object-size))
        (object-height (third object-size)))
  (cond
    ((equal :right bottom-side)
     (vector-offset place-vector
                    (list 0 (- (/ object-depth 2)) 0)))
    ((equal :left bottom-side)
     (vector-offset place-vector
                    (list 0 (/  object-depth 2) 0)))
    ((equal :top bottom-side)
     (vector-offset place-vector
                    (list 0 0 (- object-height))))
    ((equal :bottom bottom-side)
     (vector-offset place-vector
                    (list 0 0 0)))
    ((equal :front bottom-side)
     (vector-offset place-vector
                    (list 0 (- (/  object-width 2)) 0)))
    ((equal :back bottom-side)
     (vector-offset place-vector
                    (list 0 (/  object-width 2) 0)))
    (t (print "not sure how we got here but something is wrong, vector-change")))))

;;3-sides
;;list of sides of the 

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
                arm (first grasp) *place-pose*)))
  (place-object target-pose arm))

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
                      ((:grasp ?grasp))
                      ((:bottom-side ?bottom))
                    &allow-other-keys)
  (declare (type list ?plan ?object-size ?grasp)
           (type keyword ?arm ?object-type ?bottom)
           (type symbol ?object-name))
  (print ?bottom)
 
  (loop for move in (remove nil ?plan)
        do
           (let ((3d-list (vector-change
                           (cram-tf:3d-vector->list (cl-tf2:origin *place-pose*))
                             ?bottom
                             ?object-size))
                 (orientation (orientation-change ?object-name move)))
             (let ((target
                     (place-pose-stability-adjustment
                      orientation
                      3d-list
                      ?object-type
                      ?object-name
                      0)))
               (execute-change-side ?object-type ?arm ?grasp target)
               ))))

