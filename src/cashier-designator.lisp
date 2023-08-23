(in-package :cashier)

;; =============== Logic for which sides to check =====

(defun sides-to-check (sides-base non-scanable non-graspable ?potential-goal)

  (let* ((combined-list (append non-scanable non-graspable))
         (sides (mapcar (lambda (x) (car x)) sides-base))
         (updated-sides (remove-if (lambda (x)
                                         (member x combined-list
                                                 :test #'equal))
                                   sides))
         
         (goal-side (if (member ?potential-goal updated-sides :test #'equal)
                        (list ?potential-goal)
                        (list nil))))

    (setf *sides-log* (append (list (list "goal" goal-side)) *sides-log*))
    
    (if (not (equal goal-side '(nil)))
        goal-side
        updated-sides)))

(defun test-sides-to-check()
  (let* ((sides (list (list :front) (list :back) (list :left)
                      (list :right) (list :top)  (list :bottom)))
         (non-grasp (list :left :right))
         (non-scan (list nil)))
    (sides-to-check sides non-grasp non-scan :top)
    
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
    (print "side-list")
    (print side-list)
    (print "sorted-sides-list")
    (print sorted-sides-list)
    (print (list (first (car sorted-sides-list))))
    (print (null sorted-sides-list))
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
  (prolog:<- (desig:action-grounding
              ?action-designator
              (cashier-plan ?resolved-action-designator))

    (desig-prop ?action-designator (:type :cashier))

    (desig-prop ?action-designator (:object ?object-designator))
    (desig:current-designator ?object-designator ?current-desig)
    (desig-prop ?current-desig (:type ?type))
    
    (or (and (desig-prop ?current-desig (:size ?size))
             (not (equal ?size (nil))))
        (lisp-fun add-size-for-unkown ?size))     

    (or (and (desig-prop ?current-desig (:non-scanable ?n-scan))
             (not (equal ?n-scan nil)))
        (and (lisp-fun prolog-shape ?type ?shape)
             (lisp-fun prolog-disabled-side ?shape ?n-scan)))

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

    (and (desig-prop ?action-designator (:arm ?arm))
         (not (equal ?arm (nil))))
             
    (and (desig-prop ?action-designator (:scan-pose ?scan-pose))
         (not (equal ?scan-pose nil)))
    
    (and (desig-prop ?action-designator (:after-poses ?after-poses))
         (not (equal ?after-poses (nil))))
      
    (or (and (desig-prop ?action-designator (:goal-side ?goal-side))
             (not (equal nil)))
        (lisp-fun unknown-goal ?goal-side))
    
    (desig:designator :action ((:type :cashier)
                               (:object-type ?type)
                               (:arm ?arm)
                               (:non-scanable ?n-scan)
                               (:non-graspable ?n-grasp)
                               (:goal-side ?goal-side)
                               (:object-size ?size)
                               (:search-poses ?search-poses)
                               (:scan-pose ?scan-pose)
                               (:after-poses ?after-poses))
                      ?resolved-action-designator))
  
  (prolog:<- (desig:action-grounding ?action-designator
                                     (scan-plan ?resolved-action-designator))
    
    (desig-prop ?action-designator (:type :scanning))
    (desig-prop ?action-designator (:arm ?arm))

    (desig-prop ?action-designator (:object ?object-designator))
    (desig:current-designator ?object-designator ?current-desig)
    (desig-prop ?current-desig (:type ?type))
    (desig-prop ?current-desig (:pose ?pose))
    
    (or (and (desig-prop ?current-desig (:size ?size))
             (not (equal ?size (nil))))
        (lisp-fun add-size-for-unkown ?size))

    (or (and (desig-prop ?current-desig (:base-sides ?b-sides))
             (not (equal ?b-sides (nil))))
        (lisp-fun set-sides-helper ?name ?size ?b-sides))

    (or (and (desig-prop ?current-desig (:non-scanable ?n-scan))
             (not (equal ?n-scan nil)))
        (and (lisp-fun prolog-shape ?type ?shape)
             (lisp-fun prolog-disabled-side ?shape ?n-scan)))

    (or (and (desig-prop ?current-desig (:non-graspable ?n-grasp))
             (not (equal ?n-scan (nil))))
        (lisp-fun check-object-size ?size ?n-grasp))

    (desig-prop ?action-designator (:scan-pose ?scan-pose))

    (or (and (desig-prop ?current-desig (:goal-side ?goal))
             (not (equal nil)))
        (lisp-fun unknown-goal ?goal-side))
    
    (lisp-fun man-int:get-object-transform ?object-designator ?transform)
    (lisp-fun transform-b-sides-t-x ?b-sides ?transform ?sides-transformed)
    (lisp-fun sides-to-check ?sides-transformed ?n-scan ?n-grasp ?goal ?sides-to-check)

    (desig:designator :action ((:type :scanning)
                               (:object-type ?type)
                               (:object-size ?size)
                               (:arm ?arm)
                               (:non-scanable ?n-scan)
                               (:non-graspable ?n-grasp)
                               (:base-sides ?b-sides)
                               (:sides-to-check ?sides-to-check)
                               (:scan-pose ?scan-pose))
                      ?resolved-action-designator))

  (prolog:<- (desig:action-grounding ?action-designator
                                     (change-side-plan ?resolved-action-designator))
    
    (desig-prop ?action-designator (:type :changing-side))
    (desig-prop ?action-designator (:object ?object-designator))
    (desig-prop ?action-designator (:scan-pose ?scan-pose))

    (desig-prop ?action-designator (:object ?object-designator))
    (desig:current-designator ?object-designator ?current-desig)
    (desig-prop ?current-desig (:type ?type))
    (desig-prop ?current-desig (:name ?name))
    (desig-prop ?current-desig (:pose ?pose))
    
    (or (and (desig-prop ?current-desig (:size ?size))
             (not (equal ?size (nil))))
        (lisp-fun add-size-for-unkown ?size))

    (or (and (desig-prop ?current-desig (:base-sides ?b-sides))
             (not (equal ?b-sides (nil))))
        (lisp-fun set-sides-helper ?name ?size ?b-sides))

    (or (and (desig-prop ?current-desig (:non-scanable ?n-scan))
             (not (equal ?n-scan nil)))
        (and (lisp-fun prolog-shape ?type ?shape)
             (lisp-fun prolog-disabled-side ?shape ?n-scan)))

    (or (and (desig-prop ?current-desig (:non-graspable ?n-grasp))
             (not (equal ?n-scan (nil))))
        (lisp-fun check-object-size ?size ?n-grasp))

    (or (and (desig-prop ?current-desig (:goal-side ?goal))
             (not (equal nil)))
        (lisp-fun unknown-goal ?goal-side))

    (desig-prop ?action-designator (:arm ?arm))
    (desig-prop ?action-designator (:change-to-side ?side-goal))
    
    (lisp-fun side-location ?object-designator ?b-sides ?located-sides)
    (lisp-fun side-changes ?located-sides ?n-grasp ?side-changes)
    (lisp-fun path-plan-next-side ?side-changes ?n-scan ?n-grasp ?goal ?plan)

    (desig:designator :action ((:type :changing-side)
                               (:object-name ?name)
                               (:object-type ?type)
                               (:object-size ?size)
                               (:arm ?arm)
                               (:base-sides ?b-sides)
                               (:plan ?plan)
                               (:side-goal ?goal)
                               (:scan-pose ?scan-pose))
                      
                      ?resolved-action-designator)))

;;;;; <---------- Area-poses-gen ------------------>


(defun area->pose-stamped-list (?search-area distance-between-spots)
  ;;first pose is the starting place and gives the orientation of all other spots
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

(defun defaul-distance ()
  0.2)

(defun extended-object-desig (?percieved-object ?size ?n-grasp ?n-scan ?goal ?b-sides)
  (let* ((?type (desig-prop-value ?percieved-object :type))
         (?name (desig-prop-value ?percieved-object :name))
         (?pose (desig-prop-value ?percieved-object :pose)))
  (desig:an object
            (:type ?type)
            (:name ?name)
            (:pose ?pose)
            (:base-sides ?b-sides)
            (:size ?size)
            (:non-graspable ?n-grasp)
            (:non-scanable ?n-scan)
            (:goal-side ?goal)
  )))
