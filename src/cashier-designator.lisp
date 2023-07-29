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


;; ================ Designator def =================================
(prolog:def-fact-group cashier-plans (desig:action-grounding) 
  (prolog:<- (desig:action-grounding ?action-designator
                                     (cashier-object ?resolved-action-designator))
    (desig-prop ?action-designator (:type :cashier))
    (desig-prop ?action-designator (:object-name ?object-name))
    (desig-prop ?action-designator (:object-type ?object-type))
    (desig-prop ?action-designator (:arm ?arm))
    (desig-prop ?action-designator (:non-scanable ?non-scanable))
    (desig-prop ?action-designator (:non-graspable ?non-graspable))
    (desig-prop ?action-designator (:goal-side ?goal-side))
    (desig-prop ?action-designator (:object-list ?object-list))
    (desig-prop ?action-designator (:object-size ?object-size))
    
    (lisp-fun set-sides-helper ?object-name ?object-size  ?sides-base)
    (lisp-fun transforms-map-t-side ?object-name ?sides-base ?sides-transformed)
    
    (desig:designator :action ((:type :cashier)
                               (:object-type ?object-type)
                               (:object-name ?object-name)
                               (:arm ?arm)
                               (:non-scanable ?non-scanable)
                               (:non-graspable ?non-graspable)
                               (:goal-side ?goal-side)
                               (:sides-base ?sides-base)
                               (:object-size ?object-size)
                               (:sides-transformed ?sides-transformed))
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
                      
                      ?resolved-action-designator)))



