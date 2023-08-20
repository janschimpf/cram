(in-package :cashier)

(defun change-side-plan (&key
                           ((:plan ?plan))
                           ((:object-type ?object-type))
                           ((:object-name ?object-name))
                           ((:object-size ?object-size))
                           ((:arm ?arm))
                           ((:side-goal ?side-goal))
                           ((:base-sides ?b-sides))
                           ((:scan-pose ?scan-pose))
                    &allow-other-keys)
  (declare (type list ?plan ?object-size ?b-sides ?arm)
           (type keyword ?object-type)
           (type symbol ?object-name))
  
  (loop for move in (remove nil ?plan)
        do
           (print move)
           (let* ((object-vector (cram-tf:3d-vector->list
                                  (cl-tf2:origin (btr:object-pose ?object-name))))
                  
                  (3d-list (vector-change
                            (cram-tf:3d-vector->list (cl-tf2:origin ?scan-pose))
                             ?side-goal
                             ?object-size))
                  (perceived-object (perceive-object ?scan-pose ?object-type))
                  (transformed (transform-b-sides-t-x ?b-sides 
                                                                (man-int:get-object-transform
                                                                perceived-object)))
                  
                  (located-sides (side-location perceived-object ?b-sides))
                  
                  (orientation-change (orientation-change-with-move move located-sides))
                  (new-orientation (helper-function
                                    (man-int:get-object-pose perceived-object)
                                    orientation-change
                                    (cram-tf:robot-current-transform)))
                  (?grasp (which-sides located-sides move)))

             (print "============================================")
             (print "SHOW ME WHICH SIDES YOU WANT TO GRASP DAMNED")
             (print "============================================")
             (print ?grasp)
             (print orientation-change)
             (print new-orientation)
             (setf *sides-log* (append (list ?grasp (list move) located-sides) *sides-log*))
             (let ((target
                     (place-pose-stability-adjustment
                      3d-list
                      new-orientation
                      ?object-type
                      ?object-name
                      0
                      )))
               
               (print target)
               (execute-change-side ?object-type ?arm ?grasp target ?scan-pose)              
               )
             )))

(defun helper-function (pose orientation transform)
  (let* ((origin (cl-tf2:origin pose))
         (pose-orientation (cl-tf2:orientation pose))
         (new-orientation (cl-tf2:q*
                           pose-orientation
                           orientation))
         (new-pose (cl-transforms-stamped:transform transform
                                                    (cl-tf2:make-pose origin new-orientation))))
    ;; (print "pose and new orientation")
    ;; (print orientation)
    ;; (print pose-orientation)
    ;; (print new-orientation)
    (cl-tf2:orientation new-pose)
      ))

;;; ===== pose changes for placing the object with a different orientation / side ======

(defun place-pose-stability-adjustment
    (origin-list
     orientation
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
(defun orientation-change (move located-sides)
  (print move)
  (let* ((90-turn (/ pi 2))
         (turn
           (cond
             ((string-equal "back-turn" move)
              (list 0 (- (/ pi 2)) 0))
             ((string-equal "front-turn" move)
              (list 0 (/ pi 2) 0))
             ((string-equal "right-turn" move)
              (list (/ pi 2) 0 0))
             ((string-equal "left-turn" move)
              (list (- (/ pi 2)) 0 0))
             ((string-equal "flip" move)
              (list pi 0 0))
             ((string-equal "left-roation" move)
              (list 0 0 (- (/ pi 2))))
             ((string-equal "right-rotation" move)
              (list 0 0 (/ pi 2)))
             (t (print move))))
         (result (finding-axis located-sides turn move)))
    (cl-tf:euler->quaternion
     :ax (first result)
     :ay (second result)
     :az (third result))))

(defun finding-axis (located-sides turn move)
  (let* ((axis-bottom (axis-short (first located-sides)))
         (axis-right (axis-short (second located-sides)))
         (axis-front (axis-short (third located-sides)))
         (axis-list (if (or (equal move "front-turn")
                            (equal move "back-turn"))
                        (list axis-front axis-right axis-bottom)
                        (list axis-front axis-bottom axis-right))))
    (print "====================== axis-list")
    (print axis-list)
    (print located-sides)

    (mapcar (lambda (x) (matching-axis x turn))
                    axis-list))
  )

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

(defun axis-correction (axis turn)
  (cond
    ((string-equal "Y" axis)
     (- turn))
    ((string-equal "-Y" axis)
     turn)))


(defun orientation-change-with-move (move located-sides)
  (print move)
  (let* ((90-turn (/ pi 2))
         (turn
           (cond
             ((or (string-equal "back-turn" move)
                  (string-equal "left-turn" move)
                  (string-equal "left-roation" move))
              (- 90-turn))
             ((or (string-equal "front-turn" move)
                  (string-equal "right-turn" move)
                  (string-equal "right-rotation" move))
              90-turn)
             ((string-equal "flip" move)
              pi)
             (t (print move))))
         (result (if (or (string-equal "front-turn" move)
                         (string-equal "back-turn" move))
                     (finding-axis-front/back located-sides turn move)
                     (finding-axis-rest located-sides turn move))))
    (cl-tf:euler->quaternion
     :ax (first result)
     :ay (second result)
     :az (third result))))

(defun finding-axis-front/back (located-sides turn move)
  (let* ((axis-bottom (axis-short (first located-sides)))
         (axis-right (axis-short (second located-sides)))
         (axis-front (axis-short (third located-sides)))
         (axis-list (list axis-front axis-right axis-bottom)))
    
    (print "====================== axis-list-with turn")
    (print axis-list)
    (print located-sides)

    (mapcar (lambda (x) (matching-axis-with-turn x move turn))
            axis-list)))


(defun finding-axis-rest (located-sides turn move)
  (let* ((axis-bottom (axis-short (first located-sides)))
         (axis-right (axis-short (second located-sides)))
         (axis-front (axis-short (third located-sides)))
         (turn-direction (car (remove nil (list (axis-correction axis-bottom turn)
                                                (axis-correction axis-right turn)
                                                (axis-correction axis-front turn)))))
         (result (cond ((or (string-equal "Y" axis-bottom)
                            (string-equal "-Y" axis-bottom))
                        (list turn-direction 0 0))
                       ((or (string-equal "Y" axis-right)
                            (string-equal "-Y" axis-right))
                        (list turn-direction 0 0))
                       ((or (string-equal "Y" axis-front)
                           (string-equal "-Y" axis-front))
                           (list 0 turn-direction 0))))

         (axis-list (list axis-bottom axis-right axis-front)))
    
    (print "====================== axis-list-with turn")
    (print axis-list)
    (print located-sides)
    (print result)
    result))

;; located list is bottom right front
(defun matching-axis-with-turn (axis move turn)
  (cond
    ((and (string-equal "X" axis)
          (or (string-equal "left-turn" move)
              (string-equal "right-turn" move)))
     turn)
    ((and (string-equal "-X" axis)
          (or (string-equal "left-turn" move)
              (string-equal "right-turn" move)))
     (- turn))
    
    ((and (string-equal "Y" axis)
          (or (string-equal "front-turn" move)
              (string-equal "back-trun" move)))
     turn)
    ((and (string-equal "-Y" axis)
          (or (string-equal "front-turn" move)
              (string-equal "back-trun" move)))
     (- turn))
    
    ((and (string-equal "Z" axis)
          (or (string-equal "left-rotation" move)
              (string-equal "right-roation" move)))
     turn)
    ((and (string-equal "-Z" axis)
          (or (string-equal "left-rotation" move)
              (string-equal "right-roation" move)))
     (- turn))
    (t 0)
    ))



(defun get-axis (located-sides)
  (let* ((bottom (nth 1 located-sides))
         (right (nth 2 located-sides))
         (front (nth 3 located-sides)))
  ))


;; <<<---->>> Testing
;; (defun test-finding-axis ()
;;   (let* ((located (list :front :right :top))
;;          (turn (list 0 0 (/ pi 2))))
;;   (finding-axis located turn)
;;     ))


    
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
                    (list 0 0 (/ object-height 2))))
    ((equal :bottom bottom-side)
     (vector-offset place-vector
                    (list 0 0 (/ object-height 2))))
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




(defun execute-change-side (?object-typ ?arm ?grasp target-pose ?scan-pose)
  
  (multiple-value-bind (?perceived-object)
      (perceive-object ?scan-pose ?object-typ)
    (print ?arm)
    (print ?grasp)
    (let* ((?current-grasp
           (grasp-object-with-handling
             ?arm
             ?grasp
             ?perceived-object)))
  

      (move (desig:reference (desig:a location (locate ?scan-pose) (arm (first ?arm)))))
    (cpl:with-retry-counters ((place-retry 3))
    (cpl:with-failure-handling
        ((common-fail:manipulation-low-level-failure (e)
           (roslisp:ros-warn (cashier-demo) "failure happened: ~a~% changing grasp" e)
           (cpl:do-retry place-retry
           (cpl:retry))
           (cpl:fail 'high-level-grasp-failure)))
      
      (place-object-with-handling
       target-pose
       ?arm
       ?current-grasp
       )))
      )
))



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

  (<- (axis :front x))
  (<- (axis :back -x))
  (<- (axis :right -y))
  (<- (axis :left y))
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
                :initform "Failed to place the object and failed to adjust to that fail"
                :reader error-description))
  (:documentation "Failure thrown by high level place function")
  (:report (lambda (condition stream)
             (format stream (error-description condition)))))


(define-condition high-level-grasp-failure (cpl:simple-plan-failure)
  ((description :initarg :description
                :initform "Failed to grasp the object and failed to adjust to that fail"
                :reader error-description))
  (:documentation "Failure thrown by high level grasp function")
  (:report (lambda (condition stream)
             (format stream (error-description condition)))))




