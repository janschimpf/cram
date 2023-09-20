(in-package :cashier)
;;@author Jan Schimpf
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

;; (defun test-sides-to-check()
;;   (let* ((sides (list (list :front) (list :back) (list :left)
;;                       (list :right) (list :top)  (list :bottom)))
;;          (non-grasp (list :left :right))
;;          (non-scan (list nil)))
;;     (sides-to-check sides non-grasp non-scan :top)
;;   ))

;; =======side-changing ===================
;; returns the bottom side and then a list of
;; the bottom, right and front sides in that order
(defun side-changes (side-list non-graspable non-scanable)
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
                                      (opposite-short right)
                                      front))
                                      
                          (list "left-rotation" bottom
                                (list bottom
                                      (opposite-short front)
                                      right))
                                      
                          (list "right-rotation" bottom
                                (list bottom
                                      front
                                      (opposite-short right)))))
         (checked-for-grasp (viable-move move-list non-graspable non-scanable)))  
    checked-for-grasp))

;; as name says checks if the grasp for that move is viable or if they are all blocked
(defun viable-grasp (side-list move-list non-graspable non-scanable)
  (let* ((combined-list (append non-scanable non-graspable)))
  (mapcar (lambda (x) (if (null (remove nil
                                        (check-grasp
                                         (which-sides side-list (first x) non-graspable)
                                         combined-list)))
                                nil
                                x))
          move-list)))

;; checks if the move is viable or if the side that it would rotated to is one that the object can't be placed on
(defun viable-move (move-list non-graspable non-scanable)
  (let* ((combined-list (append non-scanable non-graspable)))
    (remove nil (mapcar (lambda (x)
                          (if (contains-element-negative (second x) combined-list)
                            x
                            nil))
                      move-list))))


(defun check-grasp (grasp-side non-graspable)
 (remove nil (mapcar (lambda (x) (if (member x non-graspable)
                          nil
                          x))
            grasp-side)))


;; ============ planning for which moves to do to change a side ==============


;; plans the path between the current bottom side and the side that should be scanned next
;; then returns said path (list were the movement are the elements)
(defun path-plan-next-side (move-list non-graspable non-scanable goal)
  (let* ((combined-list (append non-scanable non-graspable))
         (path-checked (path-check-for-goal move-list combined-list goal))
         (sides (which-sides (second (cdar path-checked))
                            (caar path-checked)
                            non-graspable))
         (grasp-checked (check-grasp sides non-graspable)))
    
    (if (contains-element-negative goal combined-list)
        (if (car grasp-checked)
            (list (caar path-checked))
            (path-second-step move-list goal non-graspable non-scanable))
        nil)))

;; returns the path  
(defun path-check-for-goal (move-list comb-list goal)
  (let ((path (remove nil (mapcar (lambda (x)
                                     (if (contains-element-negative (second x) (list goal))
                                         nil
                                         x))
                                  move-list))))
    
    (if (contains-element-negative (second (car path)) comb-list)
                           path
                           nil)))

;; checks if the move can be executed or if non of the sides needed can be grasped
(defun grasp-check (sides n-grasp)
  (remove nil (mapcar (lambda (x)
                        (if (member x n-grasp :test 'equal)
                            x
                            nil))
                      sides)))

;; tries to find two move plan that get the object side to the bottom
(defun path-second-step (move-list goal non-graspable non-scanable)
  (let* ((full-move-list (mapcar (lambda (x)
                                     (list (first x) (side-changes (car (last x))
                                                         non-graspable
                                                         non-scanable)))
                                 (reverse move-list)))
         (second-path-plan (mapcar (lambda (x)
                                            (list (first x) (car (look-for-goal (second x) goal non-graspable))))
                                   full-move-list)))
  second-path-plan))

(defun look-for-goal (list goal non-graspable)
  (remove nil (mapcar (lambda (x)
                        (if (member (second x) (list goal)) 
                x
                nil))
                      list)))


;; ================ Designator def =================================
(prolog:def-fact-group cashier-plans (desig:action-grounding) 
  (prolog:<- (desig:action-grounding
              ?action-designator
              (cashier-plan ?resolved-action-designator))

    (desig-prop ?action-designator (:type :cashier))

    (desig-prop ?action-designator (:object ?object-designator))
    (desig:current-designator ?object-designator ?current-desig)
    (desig-prop ?current-desig (:type ?type))
    
    (or (desig-prop ?current-desig (:size ?size))
        (lisp-fun add-size-for-unkown ?size))     

    (or (desig-prop ?current-desig (:non-scanable ?n-scan))
        (and (lisp-fun prolog-shape ?type ?shape)
             (lisp-fun prolog-disabled-side ?shape ?n-scan)))

    (or (desig-prop ?current-desig (:non-graspable ?n-grasp))
        (lisp-fun check-object-size ?size ?n-grasp))    
    
    (or (desig-prop ?action-designator (:distance-between-spots ?dis))
        (lisp-fun defaul-distance ?dis))
    
    (and (desig-prop ?action-designator (:search-area ?search-area))
         (or (and (lisp-fun single-pose-or-area ?search-area ?T)
                  (lisp-fun area->pose-stamped-list
                            ?search-area
                            ?dis
                            ?search-poses))
             (lisp-fun single-pose-search ?search-area ?search-poses)))

    (desig-prop ?action-designator (:arm ?arm))
             
    (desig-prop ?action-designator (:scan-pose ?scan-pose))
    
    (desig-prop ?action-designator (:after-poses ?after-poses))
      
    (or (desig-prop ?action-designator (:goal-side ?goal-side))
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
    
    (or (desig-prop ?current-desig (:size ?size))
        (lisp-fun add-size-for-unkown ?size))

    (or (desig-prop ?current-desig (:base-sides ?b-sides))
        (lisp-fun set-sides-helper ?name ?size ?b-sides))

    (or (desig-prop ?current-desig (:non-scanable ?n-scan))
        (and (lisp-fun prolog-shape ?type ?shape)
             (lisp-fun prolog-disabled-side ?shape ?n-scan)))

    (or (desig-prop ?current-desig (:non-graspable ?n-grasp))
        (lisp-fun check-object-size ?size ?n-grasp))

    (desig-prop ?action-designator (:scan-pose ?scan-pose))

    (or (desig-prop ?current-desig (:goal-side ?goal))
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
    (desig-prop ?action-designator (:scan-pose ?scan-pose))
    (desig-prop ?action-designator (:object-type ?type))

    (desig-prop ?action-designator (:object ?object))
    
    (desig-prop ?action-designator (:scan-pose ?scan-pose))
    (desig-prop ?action-designator (:arm ?arm))
    
    (or (desig-prop ?action-designator (:size ?size))
        (lisp-fun add-size-for-unkown ?size))

    (or (desig-prop ?action-designator (:base-sides ?b-sides))
        (lisp-fun set-sides ?object ?size ?b-sides))

    (or (desig-prop ?action-designator (:non-scanable ?n-scan))
        (and (lisp-fun prolog-shape ?type ?shape)
             (lisp-fun prolog-disabled-side ?shape ?n-scan)))

    (or (desig-prop ?action-designator (:non-graspable ?n-grasp))
        (lisp-fun check-object-size ?size ?n-grasp))
    
    (desig-prop ?action-designator (:change-to-side ?side-goal))

    (or (desig-prop ?action-designator (:plan ?plan))
        (and (lisp-fun side-location ?object ?b-sides ?located-sides)
             (lisp-fun side-changes ?located-sides ?n-grasp ?n-scan ?side-changes)
             (lisp-fun path-plan-next-side ?side-changes ?n-grasp ?n-scan ?side-goal ?plan)))

    (desig:designator :action ((:type :changing-side)
                               (:object-type ?type)
                               (:object-size ?size)
                               (:arm ?arm)
                               (:base-sides ?b-sides)
                               (:scan-pose ?scan-pose)
                               (:non-graspable ?n-grasp)
                               (:side-goal ?side-goal)
                               (:plan ?plan))                     
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
  0.25)

;; compress the object desig for scan a bit more
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
            (:goal-side ?goal))))
