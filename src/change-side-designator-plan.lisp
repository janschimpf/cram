(in-package :cashier)

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
  (let* ((90-turn (/ pi 2))
         (turn
           (cond
             ((string-equal "back-turn" move)
              (list (- (/ pi 2)) 0 0))
             ((string-equal "front-turn" move)
              (list (/ pi 2) 0 0))
             ((string-equal "right-turn" move)
              (list 0 0 (/ pi 2) ))
             ((string-equal "left-turn" move)
              (list 0 0 (-(/ pi 2))))
             ((string-equal "flip" move)
              (list 0 pi 0))
             ((string-equal "left-roation" move)
              (list 0 (- (/ pi 2)) 0))
             ((string-equal "right-rotation" move)
              (list 0 (/ pi 2)) 0)
             (t (print move))))
         (result (finding-axis located-sides turn)))
    (cl-tf2:q*
            (cl-tf2:orientation (btr:object-pose object-name))
            (cl-tf2:euler->quaternion
             :ax (first result)
             :ay (third result)
             :az (second result)))))

(defun finding-axis (located-sides turn)
  (let* ((axis-bottom (axis-short (first located-sides)))
         (axis-front (axis-short (second located-sides)))
         (axis-right (axis-short (third located-sides)))
         (axis-list (list axis-right axis-bottom axis-front)))
    (mapcar (lambda (x) (matching-axis x turn))
                         axis-list)))


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
     (- (third list-match)))
    ))
    
;; takes care of the x-y-z pose when placing the object with the goal of not placing object
;; inside the table
(defun vector-change (place-vector bottom-side object-size)
  (let ((object-depth (first object-size))
        (object-width (second object-size))
        (object-height (third object-size)))
  (cond
    ((equal :right bottom-side)
     (vector-offset place-vector
                    (list 0 object-depth -0.05)))
    ((equal :left bottom-side)
     (vector-offset place-vector
                    (list 0 object-depth 0)))
    ((equal :top bottom-side)
     (vector-offset place-vector
                    (list 0 0 0.07)))
    ((equal :bottom bottom-side)
     (vector-offset place-vector
                    (list 0 0 0)))
    ((equal :front bottom-side)
     (vector-offset place-vector
                    (list -0.05 0 0.04)))
    ((equal :back bottom-side)
     (vector-offset place-vector
                    (list 0 0 0)))
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
                            (cram-tf:3d-vector->list (cl-tf2:origin *place-position*))
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
  
  (let* ((?look *place-position*)
         (?start-pose nil))
    
    ;;(exe:perform (desig:a motion
    ;;                      (type looking)
    ;;                      (pose ?look)))
    ;;
    ;;(setf ?start-pose (exe:perform (desig:an action
    ;;                       (type detecting)
    ;;                       (object (desig:an object (type ?object-type))))))

  (cpl:with-retry-counters ((grasp-retry 3))
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
           (cpl:fail 'high-level-grasp-failure)))
                             
      (grasp-object ?object-type
                    arm
                    *place-position*
                    (first grasp))))
  
    (move *place-nav-pose*)

    (cpl:with-retry-counters ((place-retry 3))
    (cpl:with-failure-handling
        ((common-fail:manipulation-low-level-failure (e)
           (roslisp:ros-warn (cashier-demo) "failure happened: ~a~% changing grasp" e)
           (cpl:do-retry place-retry
           (cpl:retry))
           (cpl:fail 'high-level-grasp-failure)))
    (place-object target-pose arm :?left-grasp (first grasp))))))




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


(defun opposite-short (side)
  (cdaar (prolog:prolog `(or (opposite ,side ?x)
                             (opposite ?x ,side)))))
(defun axis-short (side)
  (cdaar (prolog:prolog `(axis ,side ?x))))

(defun prolog-shape (object-type)
  (cdaar (prolog:prolog `(btr-spatial-cm::%item-type-shape ,object-type ?x))))

(defun prolog-disabled-side (shape)
  (mapcar (lambda (x) (cdar x))
          (prolog::force-ll  (prolog:prolog `(shape-disabled-sides ,shape ?x)))))

(defun prolog-scan-gtin (object-type)
   (cdaar (prolog:prolog `(productype-to-gtin ,object-type ?x))))


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

  (<- (productype-to-gtin :snackbar "8718951045118"))
  (<- (productype-to-gtin :small-cube "4062300020719"))
  (<- (productype-to-gtin :cup "4062300025318"))
  (<- (productype-to-gtin :bottle "4062300265998"))
  (<- (productype-to-gtin :fruit-juice "4015000010320"))
  (<- (productype-to-gtin :small-book "4004980506206"))
  (<- (productype-to-gtin :breakfast-cereal "4015000010511"))
  (<- (productype-to-gtin :pringle "4013162004027"))


  (<- (shape-disabled-sides :circle :left))
  (<- (shape-disabled-sides :circle :right))
  (<- (shape-disabled-sides :circle :front))
  (<- (shape-disabled-sides :circle :back)))

(define-condition high-level-place-failure (cpl:simple-plan-failure)
  ((description :initarg :description
                :initform "Failed to place the object"
                :reader error-description))
  (:documentation "Failure thrown by high level place function")
  (:report (lambda (condition stream)
             (format stream (error-description condition)))))


(define-condition high-level-grasp-failure (cpl:simple-plan-failure)
  ((description :initarg :description
                :initform "Failed to grasp the object"
                :reader error-description))
  (:documentation "Failure thrown by high level grasp function")
  (:report (lambda (condition stream)
             (format stream (error-description condition)))))
