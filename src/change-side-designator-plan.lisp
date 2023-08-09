(in-package :cashier)

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
              (list 0 (- (/ pi 2)) 0))
             ((string-equal "front-turn" move)
              (list 0 (/ pi 2) 0))
             ((string-equal "right-turn" move)
              (list (/ pi 2) 0 0 ))
             ((string-equal "left-turn" move)
              (list (-(/ pi 2)) 0 0))
             ((string-equal "flip" move)
              (list pi 0 0))
             ((string-equal "left-roation" move)
              (list 0 0 (- (/ pi 2))))
             ((string-equal "right-rotation" move)
              (list 0 0 (/ pi 2)))
             (t (print move))))
         (result (finding-axis located-sides turn)))
    (cl-tf2:q*
            (cl-tf2:orientation (btr:object-pose object-name))
            (cl-tf2:euler->quaternion
             :ax (first turn)
             :ay (second turn)
             :az (third turn)))))

(defun finding-axis (located-sides turn)
  (let* ((axis-bottom (axis-short (first located-sides)))
         (axis-right (axis-short (second located-sides)))
         (axis-front (axis-short (third located-sides)))

         (axis-list (list axis-right axis-front axis-bottom)))
    (print (mapcar (lambda (x) (matching-axis x turn))
                   axis-list))
    (print "after match")
    (mapcar (lambda (x) (matching-axis x turn))
                   axis-list)))

;; located list is bottom right front
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


(defun get-axis (located-sides)
  (let* ((bottom (nth 1 located-sides))
         (right (nth 2 located-sides))
         (front (nth 3 located-sides)))
  ))


;; <<<---->>> Testing
(defun test-finding-axis ()
  (let* ((located (list :front :right :top))
         (turn (list 0 0 (/ pi 2))))
  (finding-axis located turn)
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
                    (list 0 0 0)))
    ((equal :left bottom-side)
     (vector-offset place-vector
                    (list 0 0 0)))
    ((equal :top bottom-side)
     (vector-offset place-vector
                    (list 0 0 0)))
    ((equal :bottom bottom-side)
     (vector-offset place-vector
                    (list 0 0 0)))
    ((equal :front bottom-side)
     (vector-offset place-vector
                    (list 0 0 0)))
    ((equal :back bottom-side)
     (vector-offset place-vector
                    (list 0 0 0)))
    (t (print "not sure how we got here but something is wrong, vector-change")))))


(defun object-desig-shortcut (?object-type)
  (desig:an object (type ?object-type)))

(defun shortcut-pose-stability (object-desig placing-location)
  (proj-reasoning:check-placing-pose-stability
   object-desig
   placing-location))




(defun execute-change-side (?object-typ arm grasp target-pose)
  
  (multiple-value-bind (?perceived-object)
      (perceive-object *place-position* ?object-typ)

                             
      (grasp-object-with-handling
       arm
       grasp
       ?perceived-object)
  
    (move *place-nav-pose*)

    (cpl:with-retry-counters ((place-retry 3))
    (cpl:with-failure-handling
        ((common-fail:manipulation-low-level-failure (e)
           (roslisp:ros-warn (cashier-demo) "failure happened: ~a~% changing grasp" e)
           (cpl:do-retry place-retry
           (cpl:retry))
           (cpl:fail 'high-level-grasp-failure)))
      
      (place-object-with-handling
       target-pose
       arm
       *current-grasp*
       ))))
)



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

  (<- (axis :front z))
  (<- (axis :back -z))
  (<- (axis :right x))
  (<- (axis :left -x))
  (<- (axis :top -y))
  (<- (axis :bottom y))

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
