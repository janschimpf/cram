(in-package :cashier)


(defun get-scan-area ()
  (cl-transforms-stamped:make-pose-stamped
   "map" 0.0
   (cl-transforms:make-3d-vector -2 1.3 0.70)
   (cl-transforms:make-quaternion 0 0 0 1)))

;;for changing the relation of the side-pose from the object to the map
(defun set-sides (object-name object-x object-y object-z)
  (let ((object-vector (cram-tf:3d-vector->list
                        (cl-tf2:origin (btr:object-pose object-name))))
        
        (object-rotation (cl-tf2:orientation
                          (cram-tf::pose-stamped->pose (btr:object-pose object-name))))
       (side-list (list
                   (list :top (cl-transforms:make-3d-vector 0 0 0.05))
                   (list :bottom (cl-transforms:make-3d-vector 0 0 -0.05))
                   
                   (list :left (cl-transforms:make-3d-vector 0 0.05 0))
                   (list :right (cl-transforms:make-3d-vector 0 -0.05 0))
                   
                   (list :back (cl-transforms:make-3d-vector 0.05 0 0))
                   (list :front (cl-transforms:make-3d-vector -0.05 0 0)))))
    (let ((side (mapcar (lambda (x) (list (first x) (cl-tf:make-pose
                         (second x)
                         object-rotation)))
                        side-list)))
      side)))


(defun change-side-list-to-map (side-list)
  (mapcar (lambda (x) (list (first x) (second x))) side-list))

(defun scan (object-name side side-list)
  (let ((scan-area-vector (first (cram-tf:pose->list
                                  (cram-tf::pose-stamped->pose (get-scan-area)))))
        
        (object-vector (cram-tf:3d-vector->list
                        (cl-tf2:origin (btr:object-pose object-name))))
        
        (object-rotation (cl-transforms:orientation (btr:object-pose object-name))))
    
    (if (x-y-z-pose-check scan-area-vector object-vector)
        (roslisp:ros-info (scan-object) "object is in the scan area")
        (roslisp:ros-info (scan-object) "object is not inside the scan area"))
    
    (if (side-check side object-vector side-list)
        (roslisp:ros-info (scan-object) "Object has the correct rotation and so the code was scaned")
        (roslisp:ros-info (scan-object) "Object has the wrong rotation"))        
    (if (and (side-check side object-vector side-list) (x-y-z-pose-check scan-area-vector object-vector))
        t
        nil
        )
  ))


(defun x-y-z-pose-check (scan-area-vector object-vector)
  (let*  ((scan-x (first scan-area-vector))
         (scan-y (second scan-area-vector))
         (scan-z (third scan-area-vector))
         (object-x (first object-vector))
         (object-y (second object-vector))
          (object-z (third object-vector)))
    (print scan-area-vector)
    (print object-vector)
    
    (if (and (< (- scan-x 0.05) object-x)
             (< object-x  (+ scan-x 0.05))
             (< (- scan-y 0.05) object-y)
             (< object-y  (+ scan-y 0.05))
             (< (- scan-z 0.05) object-z)
             (< object-z   (+ scan-z 0.15)))
        t
        nil
        )) 
  )


(defun side-check (side-to-be object-vector side-list)
  (let ((side-as-is (caaar (locate-sides side-list object-vector))))
    (print side-as-is)
    (print side-to-be)
    (print (locate-sides side-list object-vector))    
        (if (equal side-to-be side-as-is)
            t
            nil
  )))


(defun distances-for-side-list (side-list scan-vector)
  (let ((map-side-list (change-side-list-to-map side-list)))
    
   (mapcar (lambda (x) (list (first x)
                             (distance-between-vectors scan-vector
                                                       (pose-to-vector-list (second x)))))
           map-side-list)))

;;gets two poses that are in relation to the map and then returns the
;;distance between these two poses
(defun distance-between-vectors (scan-vector object-vector)
  (let ((3d-vector-1 scan-vector)
        (3d-vector-2 object-vector))
    (sqrt (+ (expt (- (first 3d-vector-2) (first 3d-vector-1)) 2)
             (expt (- (second 3d-vector-2) (second 3d-vector-1)) 2)
             (expt (- (third 3d-vector-2) (third 3d-vector-1)) 2)))))
  
(defun shortest-distance (side-list)
  (sort side-list #'< :key 'second))


(defun pose-to-vector-list (pose)
   (cram-tf:3d-vector->list (cl-tf2:origin pose)))


;;returns touples of the side that can be reached via movement
;;side-list bottom right front
;;right-turn changes the current right side to be at the bottom
;;left-turn changes the current left side to be at the bottom
;;front-turn changes the current back side to be at the bottom
;;left-adjusted -y change followed by a left turn
(defun side-changes (side-list)
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
(defun path-plan-next-side (testinput goal) ;;(current-side next-side side-list object-vector)
  ;;(let ((sides-after-move (side-changes (locate-sides object-vector side-list))))
  (let ((get-side-lists testinput))
    (let ((sorted-sides-list (check-sides-moves get-side-lists goal)))
    (if (null sorted-sides-list)
        (path-second-step get-side-lists goal)
      (list (car sorted-sides-list))
    ))))

(defun check-sides-moves (sides-list goal)
  (first (remove nil (remove-duplicates
                      (mapcar (lambda (x)
                                (if (equal goal (second x)) x))
                               sides-list)))))

(defun path-second-step (move-list goal)
  (let ((new-sides (car (reverse (car move-list)))))
    (let ((second-move (car (check-sides-moves (side-changes new-sides) goal)))) 
      (list (car (car move-list)) second-move))))



;; executes the path plan 
(defun execute-side-path-plan (plan object-type arm grasp object-name)
  (loop for move in plan
        do (cond
             ((string-equal "right-turn" move) (right-turn object-type arm grasp object-name))
             ((string-equal  "left-turn" move) (left-turn object-type arm grasp object-name))
             ((string-equal "front-turn" move) (front-turn object-type arm grasp object-name))
             ((string-equal "back-turn" move) (back-turn object-type arm grasp object-name))
             (t (print move)))))

(defun right-turn (object-type arm grasp object-name)
  (let* ((orientation (cl-tf2:q*
            (cl-tf2:orientation (btr:object-pose object-name))
            (cl-tf2:euler->quaternion :ax (-(/ pi 2)) :ay 0 :az 0))))
    (print "right")
    (execute-change-side object-type arm grasp
                         (place-pose-stability-adjustment orientation object-type object-name 0))))

(defun left-turn (object-type arm grasp object-name)
  (let* ((orientation (cl-tf2:q*
            (cl-tf2:orientation (btr:object-pose object-name))
            (cl-tf2:euler->quaternion :ax (/ pi 2) :ay 0 :az 0))))
    
    (print "left")

    (execute-change-side object-type arm grasp
                         (place-pose-stability-adjustment orientation object-type object-name 0)))
  )

(defun front-turn (object-type arm grasp object-name)
  (let* ((orientation
           (cl-tf2:q*
            (cl-tf2:orientation (btr:object-pose object-name))
            (cl-tf2:euler->quaternion :ax 0 :ay (- (/ pi 2)) :az 0))))
    (print "front")
    (execute-change-side object-type arm grasp
                         (place-pose-stability-adjustment orientation object-type object-name 0))))

(defun back-turn (object-type arm grasp object-name)
  (let* ((orientation
           (cl-tf2:q*
            (cl-tf2:orientation (btr:object-pose object-name))
            (cl-tf2:euler->quaternion :ax 0 :ay (/ pi 2) :az 0))))
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

  (cpl:with-retry-counters ((scan-counter-retries 6))
    (cpl:with-failure-handling 
    ((common-fail:high-level-failure (e)
       (roslisp:ros-warn (cashier-demo) "Falure happend: ~a~% adjusting place postion" e)
       (cpl:do-retry scan-counter-retries
         (let* ((object-vector (cram-tf:3d-vector->list
                                (cl-tf2:origin (btr:object-pose ?object-name))))
                (?grasp :front)
                (?arm :left))
           (let* ((plan (path-plan-next-side (side-changes
                                              (locate-sides ?sides-transformed object-vector))
                                             (car ?sides-base))))
             (execute-side-path-plan plan ?object-type ?arm ?grasp ?object-name))
           (setf ?sides-transformed (transforms ?object-name ?sides-base))
           (cpl:retry)))
       (cpl:fail 'common-fail:high-level-failure)))
      (if (not (scan ?object-name ?goal-side ?sides-transformed))
        (print "scan was unsuccesful ! insert throw erros! to keep looping")
      ))))

;;3-sides
;;list of sides of the 

(defun execute-change-side (object-type arm grasp target-pose)
  (print target-pose)
  (grasp-object object-type arm grasp *place-pose*)
  (place-object target-pose arm)
  )

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
  
  (<- (opposite "top" "bottom"))
  (<- (opposite "left" "right"))
  (<- (opposite "front" "back")))
