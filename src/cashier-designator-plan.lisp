(in-package :cashier)

(defun side-changes (side-list)
  (let* ((bottom (caar (first side-list)))
         (right (caar (second side-list)))
         (front (caar (third side-list))))
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
                      (opposite-short right))))
    
    ))

(defun opposite-short (side)
  (cdaar (prolog:prolog `(or (opposite ,side ?x)
                              (opposite ?x ,side)))))


;;returns relative bottom, right and front sides in form of a list.
(defun locate-sides (side-list object-vector)
  (let* ((right-list (vector-offset object-vector 0 -0.05 0))
         (front-list (vector-offset object-vector -0.05 0 0))
         (scan-list  (vector-offset object-vector 0 0 -0.05)))
    (let* ((right (shortest-distance-between-all-sides side-list right-list))
           (front (shortest-distance-between-all-sides side-list front-list))
           (bottom (shortest-distance-between-all-sides side-list scan-list)))
    (list bottom right front)
    )))

(defun vector-offset (vector x-off y-off z-off)
  (list (+ (first vector) x-off)
        (+ (second vector) y-off)
        (+ (third vector) z-off)
  ))

(defun shortest-distance-between-all-sides (side-list xyz-list)
  (shortest-distance (distances-for-side-list side-list xyz-list)))


;; plans the path between the current bottom side and the side that should be scanned next
;; then returns said path (list were the movement are the elements)
(defun path-plan-next-side (side-list goal) ;;(current-side next-side side-list object-vector)
  (let ((sorted-sides-list (check-sides-moves side-list goal)))
    (print sorted-sides-list)
    (if (null sorted-sides-list)
        (path-second-step side-list goal)
     (list (first (car sorted-sides-list))
    ))))

(defun check-sides-moves (sides-list goal)
  (remove nil (mapcar (lambda (x)
                        (if (equal goal (second x)) x))
                      sides-list)))

(defun path-second-step (move-list goal)
  (let ((new-sides (car (reverse (car move-list)))))
    (let ((second-move (car (check-sides-moves (side-changes new-sides) goal)))) 
      (list (first (car move-list)) second-move))))



;; executes the path plan 
(defun change-side (&key
                      ((:plan ?plan))
                      ((:object-type ?object-type))
                      ((:object-name ?object-name))
                      ((:arm ?arm))
                      ((:grasp ?grasp))
                    &allow-other-keys)
  (declare (type list ?plan)
           (type keyword ?arm ?grasp ?object-type)
           (type symbol ?object-name))
  (loop for move in ?plan
        do (cond
             ((string-equal "right-turn" move) (right-turn ?object-type ?arm ?grasp ?object-name))
             ((string-equal  "left-turn" move) (left-turn ?object-type ?arm ?grasp ?object-name))
             ((string-equal "front-turn" move) (front-turn ?object-type ?arm ?grasp ?object-name))
             ((string-equal "back-turn" move) (back-turn ?object-type ?arm ?grasp ?object-name))
             (t (print move)))))
)  
(defun right-turn (object-type arm grasp object-name)
  (let* ((orientation (cl-tf2:q*
            (cl-tf2:orientation (btr:object-pose object-name))
            (cl-tf2:euler->quaternion :ax (/ pi 2) :ay 0 :az 0))))
    (print "right")
    (execute-change-side object-type arm grasp
                         (place-pose-stability-adjustment orientation object-type object-name 0))))

(defun left-turn (object-type arm grasp object-name)
  (let* ((orientation (cl-tf2:q*
            (cl-tf2:orientation (btr:object-pose object-name))
            (cl-tf2:euler->quaternion :ax (- (/ pi 2)) :ay 0 :az 0))))
    
    (print "left")

    (execute-change-side object-type arm grasp
                         (place-pose-stability-adjustment orientation object-type object-name 0))))

(defun front-turn (object-type arm grasp object-name)
  (let* ((orientation
           (cl-tf2:q*
            (cl-tf2:orientation (btr:object-pose object-name))
            (cl-tf2:euler->quaternion :ax 0 :ay (/ pi 2) :az 0))))
    (print "front")
    (execute-change-side object-type arm grasp
                         (place-pose-stability-adjustment orientation object-type object-name 0))))

(defun back-turn (object-type arm grasp object-name)
  (let* ((orientation
           (cl-tf2:q*
            (cl-tf2:orientation (btr:object-pose object-name))
            (cl-tf2:euler->quaternion :ax 0 :ay (-(/ pi 2)) :az 0))))
    (print "left-adjusted")
    (execute-change-side object-type arm grasp
                         (place-pose-stability-adjustment orientation object-type object-name 0)))
)

(defun origin->list (object-name)
    (cram-tf:3d-vector->list (cl-tf2:origin (btr:object-pose object-name))))
  

(defun adjust-height (bottom-side object-size)
  (cond
    ((or (string-equal "right" bottom-side) (string-equal "left" bottom-side)) +0.05)
    ((or (string-equal "front" bottom-side) (string-equal "back" bottom-side)) +0.05)
    ((or (string-equal "top" bottom-side) (string-equal "top" bottom-side)) 0)
    (t (print "something went wrong when adjusting the height")  
  )))

;; iterates over the to scan sides.
;; first gets the current bottom side, front side and right side
;; then the path plan the path, execute the path, update object, scan
(defun scan-object (&key
                      ((:object-type ?object-type))
                      ((:object-name ?object-name))
                      ((:goal-side ?goal-side))
                      ((:sides-base ?sides-base))
                      ((:sides-transformed ?sides-transformed))
                    &allow-other-keys)
  
  (declare (type keyword ?object-type ?goal-side)
           (type list ?sides-base ?sides-transformed)
           (type symbol ?object-name))
  (print ?sides-transformed)
  (setf ?sides-transformed (transforms-map-t-side ?object-name ?sides-base))
  (cpl:with-retry-counters ((scan-counter-retries 6))
    (cpl:with-failure-handling 
    ((common-fail:high-level-failure (e)
       (roslisp:ros-warn (cashier-demo) "Falure happend: ~a~% adjusting place postion" e)
       (cpl:do-retry scan-counter-retries
         (let* ((?object-vector (cram-tf:3d-vector->list
                                (cl-tf2:origin (btr:object-pose ?object-name))))
                (?grasp :front)
                (?arm :left))           
           (let* ((plan (path-plan-next-side (side-changes
                                              (locate-sides ?sides-transformed ?object-vector))
                                             ?goal-side)))
             (exe:perform
              (desig:an action
                      (:type :changing-side)
                      (:object-name ?object-name)
                      (:object-type ?object-type)
                      (:arm ?arm)
                      (:grasp ?grasp)
                      (:change-to-side ?goal-side)
                      (:sides-transformed ?sides-transformed)
                      (:object-vector ?object-vector)))
             (setf ?sides-transformed (transforms-map-t-side ?object-name ?sides-base))
             (print plan)))
           (cpl:retry))
       (cpl:fail 'common-fail:high-level-failure)))
      (if (not (scan ?object-name ?goal-side ?sides-transformed))
        (cpl:fail 'common-fail:high-level-failure)
      ))))

;;3-sides
;;list of sides of the 

(defun execute-change-side (object-type arm grasp target-pose)

  (grasp-object ?object-type
                arm grasp *place-pose*)
  (place-object target-pose arm))

(defun object-desig-shortcut (?object-type)
  (desig:an object (type ?object-type)))

(defun shortcut-pose-stability (object-desig placing-location)
  (proj-reasoning:check-placing-pose-stability
   object-desig
   placing-location))

(defun place-pose-stability-adjustment (orientation object-type object-name offset-start)
  (let ((offset offset-start))
    (cpl:with-retry-counters ((pose-adjustment-retries 100))
    (cpl:with-failure-handling 
    ((common-fail:high-level-failure (e)
       (roslisp:ros-warn (cashier-demo) "Falure happend: ~a~% adjusting place postion" e)
       (cpl:do-retry pose-adjustment-retries
         (setf offset (- offset 0.001))
         (cpl:retry))
       (cpl:fail 'common-fail:high-level-failure)))
      

       (let* ((target-pose
           (cl-tf2:make-pose-stamped "map" 0 
           (cram-tf:list->3d-vector (vector-offset (origin->list object-name) 0 0 offset))
           orientation )))
         (let ((?target-pose-test (btr:ensure-pose target-pose)))
           (shortcut-pose-stability (object-desig-shortcut object-type)
                                    (a location (pose ?target-pose-test)))
           ?target-pose-test
           ))))))


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
