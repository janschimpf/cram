(in-package :cashier)

;; =============== Transform + Logic for which sides to check =====


(defun transforms-map-T-side (object-name side-poses)
  (mapcar (lambda (x)
            (let* ((map-T-object (cl-transforms:pose->transform (btr:object-pose object-name)))
                   (object-T-side (cl-transforms:pose->transform (first (cdr x))))
                   (map-T-side  (cl-transforms:transform* map-T-object object-T-side)))
    (list (car x) (cl-transforms:transform->pose map-T-side))))
          side-poses))

(defun sides-to-check (sides-base non-scanable non-graspable)
  (let* ((combined-list (append non-scanable non-graspable))
         (sides (mapcar (lambda (x) (car x)) sides-base))
         (updated-sides (remove-if (lambda (x)
                                     (member x combined-list
                                             :test #'equal))
                                   sides)))
    (print updated-sides)
    (setf *sides-log* (append combined-list *sides-log*))
    updated-sides))

(defun test-sides-to-check()
  (let* ((sides (list (list :front) (list :back) (list :left) (list :right) (list :top)  (list :bottom)))
         (non-grasp (list :left :right))
         (non-scan (list nil)))
    (sides-to-check sides non-grasp non-scan)
    
  ))

;; =======side-changing ===================
;; returns the bottom side and then a list of
;; the bottom, right and front sides in that order
(defun side-changes (side-list non-graspable)
  (let* ((bottom (first side-list))
         (right (second side-list))
         (front (third side-list))
         (move-list (list (list "right-turn" right
                                (list right
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
                                      right)))))
         ;;(checked-for-grasp (remove nil (viable-grasp side-list move-list non-graspable))))
    ;;checked-for-grasp
    (print (list right (list right (opposite-short bottom) front)))
    (print bottom)
    (print right)
    (print front)
         (print move-list)))

(defun viable-grasp (side-list move-list non-graspable)
  (mapcar (lambda (x) (if (null (remove nil
                                        (check-grasp
                                         (which-sides side-list (first x))
                                         non-graspable)))
                                nil
                                x))
          move-list))

(defun check-grasp (grasp-side non-graspable)
  (mapcar (lambda (x) (if (member x non-graspable)
                          nil
                          x))
            grasp-side))

;; ============ planning for which moves to do to change a side ==============


;; plans the path between the current bottom side and the side that should be scanned next
;; then returns said path (list were the movement are the elements)
(defun path-plan-next-side (side-list non-scanable non-graspable goal)
  ;;(current-side next-side side-list object-vector)
  
  (let* ((combined-list (append non-scanable non-graspable))
         
         (sorted-sides-list (remove-if (lambda (x)
                                     (member (second x) combined-list
                                             :test #'equal))
                                       (check-sides-moves side-list goal))))
    (print sorted-sides-list)
    (if (null sorted-sides-list)
        (path-second-step side-list goal non-graspable)
      (list (first (car sorted-sides-list))))))

(defun check-sides-moves (sides-list goal)
  (remove nil (mapcar (lambda (x)
                        (if (equal goal (second x))
                            x))
                      sides-list)))

(defun path-second-step (move-list goal non-graspable)
  (let ((new-sides (second  move-list)))
    (let ((second-move (remove nil (caar
                                    (check-sides-moves
                                     (side-changes new-sides non-graspable) goal)))))
      (list (first (car move-list)) second-move))))


(defun test-side-change()
  
  (let* ((test-located-list (list :bottom :right :front)))
    (side-changes test-located-list (list nil))))


(defun test-path-plan-bottom-right-front (goal-side)
  (let* ((test-located-list (list :bottom :right :front))
         (test-side-changes (side-changes test-located-list (list nil))))
    (path-plan-next-side test-side-changes (list nil) (list nil) goal-side)))


(defun test-path-plan-front-right-top (goal-side)
  (let* ((test-located-list (list :front :right :top))
         (test-side-changes (side-changes test-located-list (list nil))))
    (path-plan-next-side test-side-changes (list nil) (list nil) goal-side)))

(defun create-poses (?area)
  ?area)

;; ================ Designator def =================================
(prolog:def-fact-group cashier-plans (desig:action-grounding) 
  (prolog:<- (desig:action-grounding ?action-designator
                                     (cashier-object ?resolved-action-designator))

    (desig-prop ?action-designator (:type :cashier))

    (desig-prop ?action-designator (:object ?object-designator))
    (desig:current-designator ?object-designator ?current-desig)
    (desig-prop ?current-desig (:type ?type))
    
    (and (desig-prop ?action-designator (:arm ?arm))
         (not (equal ?arm (nil))))

    
    (or (and (desig-prop ?current-desig (:size ?size))
             (not (equal ?size (nil))))
        (lisp-fun add-size-for-unkown ?size))
             
    (or (and (desig-prop ?current-desig (:non-scanable ?n-scan))
             (not (equal ?n-scan (nil))))
        (lisp-fun prolog-shape ?type ?n-scan))

    (or (and (desig-prop ?current-desig (:non-graspable ?n-grasp))
             (not (equal ?n-scan (nil))))
        (lisp-fun check-object-size ?size ?n-grasp))
    
    (or (desig-prop ?action-designator (:distance-between-spots ?dis))
        (lisp-fun defaul-distance ?dis))

    
    (and (desig-prop ?action-designator (:search-area ?search-area))
         (not (equal ?search-area (nil)))
         (or (and (lisp-fun single-pose-or-area ?search-area ?T)
                  (lisp-fun area->pose-stamped-list
                            ?search-area
                            ?dis
                            ?search-poses))
             (lisp-fun single-pose-search ?search-area ?search-poses)))
    
    (or (and (desig-prop ?action-designator (:goal-side ?goal-side))
             (not (equal nil)))
        (lisp-fun unknown-goal ?goal-side))
             
    (and (desig-prop ?action-designator (:scan-pose ?scan-pose))
         (not (equal ?scan-pose nil)))
    
    (and (desig-prop ?action-designator (:success-pose ?success-pose))
         (not (equal ?success-pose nil)))

    (and (desig-prop ?action-designator (:failed-pose ?failed-pose))
         (not (equal ?failed-pose nil)))
    
    (desig:designator :action ((:type :cashier)
                               (:object-type ?type)
                               (:arm ?arm)
                               (:non-scanable ?n-scan)
                               (:non-graspable ?n-grasp)
                               (:goal-side ?goal-side)
                               (:object-size ?size)
                               (:search-poses ?search-poses)
                               (:scan-pose ?scan-pose)
                               (:success-pose ?success-pose) 
                               (:failed-pose ?failed-pose)
                               )
                      ?resolved-action-designator))
  
  (prolog:<- (desig:action-grounding ?action-designator
                                     (scan-object ?resolved-action-designator))
    
    (desig-prop ?action-designator (:type :scanning))
    (desig-prop ?action-designator (:object-name ?name))
    (desig-prop ?action-designator (:object-type ?object-type))
    (desig-prop ?action-designator (:arm ?arm))
    (desig-prop ?action-designator (:non-scanable ?non-scanable))
    (desig-prop ?action-designator (:non-graspable ?non-graspable))
    (desig-prop ?action-designator (:object-size ?object-size))
    (desig-prop ?action-designator (:sides-base ?sides-base))
    (desig-prop ?action-designator (:goal-side ?goal-side))


    (lisp-fun transforms-map-T-side ?object-name ?sides-base ?sides-transformed)
    (lisp-fun sides-to-check ?sides-transformed ?non-scanable ?non-graspable ?sides-to-check)

    (desig:designator :action ((:type :scanning)
                               (:object-name ?name)
                               (:object-type ?object-type)
                               (:object-size ?object-size)
                               (:arm ?arm)
                               (:non-scanable ?non-scanable)
                               (:non-graspable ?non-graspable)
                               (:sides-base ?sides-base)
                               (:goal-side ?goal-side)
                               (:sides-transformed ?sides-transformed)
                               (:sides-to-check ?sides-to-check))
                      ?resolved-action-designator))

  (prolog:<- (desig:action-grounding ?action-designator
                                     (change-side ?resolved-action-designator))
    (desig-prop ?action-designator (:type :changing-side))
    (desig-prop ?action-designator (:object-type ?object-type))
    (desig-prop ?action-designator (:object-name ?object-name))
    (desig-prop ?action-designator (:object-size ?object-size))
    (desig-prop ?action-designator (:arm ?arm))
    (desig-prop ?action-designator (:non-scanable ?non-scanable))
    (desig-prop ?action-designator (:non-graspable ?non-graspable))
    (desig-prop ?action-designator (:sides-base ?sides-base))
    (desig-prop ?action-designator (:change-to-side ?side-goal))
    (desig-prop ?action-designator (:object-vector ?object-vector))
    (desig-prop ?action-designator (:sides-transformed ?sides-transformed))


    (lisp-fun locate-sides ?sides-transformed ?object-vector ?located-sides)
    (lisp-fun side-changes ?located-sides ?non-graspable ?side-changes)
    (lisp-fun path-plan-next-side ?side-changes ?non-scanable ?non-graspable ?side-goal ?plan)

    (desig:designator :action ((:type :changing-side)
                               (:object-name ?object-name)
                               (:object-type ?object-type)
                               (:object-size ?object-size)
                               (:arm ?arm)
                               (:sides-base ?sides-base)
                               (:plan ?plan)
                               (:side-goal ?side-goal))
                      
                      ?resolved-action-designator))
  
   (prolog:<- (desig:action-grounding ?action-designator
                                      (align-object ?resolved-action-designator))
     
     (desig-prop ?action-designator (:type :align-side))
     (desig-prop ?action-designator (:object ?object-designator))
     (spec:property ?action-designator (:object ?object-designator))
     (desig:current-designator ?object-designator ?current-desig)
     (spec:property ?current-desig (:type ?object-type))
     (spec:property ?current-desig (:name ?object-name))
     
     (desig-prop ?action-designator (:arm ?arm))
     (desig-prop ?action-designator (:non-scanable ?non-scanable))     
     (desig-prop ?action-designator (:non-graspable ?non-graspable))
     (desig-prop ?action-designator (:sides-base ?sides-base))
     (desig-prop ?action-designator (:center-point ?center))
     (desig-prop ?action-designator (:align-point ?align))
     (desig-prop ?action-designator (:front-point ?front))
                 

    (desig:designator :action ((:type :align-side)
                               (:object-name ?object-name)
                               (:object-type ?object-type)
                               (:object-size ?object-size)
                               (:arm ?arm)
                               (:sides-base ?sides-base)
                               (:plan ?plan)
                               (:side-goal ?side-goal))
                      
                      ?resolved-action-designator)))


(defun area->pose-stamped-list (?search-area distance-between-spots)
  ;;first pose is the starting place and gives the orientation of all other sports
  ;;second pose gives the overall distance and how the offset vectors should be orientated.
  
  (let* ((pose-1 (first ?search-area))
         (pose-2 (second ?search-area))
         (origin-vector-1 (cl-tf2:origin pose-1))
         (orientation-pose-1 (cl-tf2:orientation pose-1))
         (origin-vector-2 (cl-tf2:origin pose-2))
         (orientation-pose-2 (cl-tf2:orientation pose-2))
         
         (length (cl-tf2:v-dist origin-vector-1 origin-vector-2))
         (times (/ length distance-between-spots))
         (rounded (if (>= times 1)
                      (round times)
                      0)))
    (loop for num from 0 to rounded
          collect (cl-transforms-stamped:make-pose-stamped
                   "map" 0
                   (vector-addition
                    origin-vector-1
                    (cl-transforms:rotate
                     orientation-pose-2
                     (cl-tf2:make-3d-vector (* distance-between-spots num) 0 0)))
                   orientation-pose-1)
          )))

(defun add-size-for-unkown ()
  (list 0.05 0.05 0.05))

(defun single-pose-or-area (list)
  (if (> (length list) 1)
      t
      nil))

(defun single-pose-search (pose)
  pose)

(defun unknown-goal ()
  nil)

(defun prolog-helper (type)
  (prolog-shape type))
