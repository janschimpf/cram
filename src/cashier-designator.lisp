(in-package :cashier)

;; =============== Transform + Logic for which sides to check =====


(defun transforms-map-T-side (object-name side-poses)
  (mapcar (lambda (x)
            (let* ((map-T-object (cl-transforms:pose->transform (btr:object-pose object-name)))
                   (object-T-side (cl-transforms:pose->transform (first (cdr x))))
                   (map-T-side  (cl-transforms:transform* map-T-object object-T-side)))
    (list (car x) (cl-transforms:transform->pose map-T-side))))
          side-poses))

(defun sides-to-check (sides-base object-size non-scanable non-graspable)
  (let* ((combined-list (append non-scanable non-graspable))
         (sides (mapcar (lambda (x) (car x)) sides-base))
         (updated-sides (remove-if (lambda (x)
                                     (member x combined-list
                                             :test #'equal))
                                   sides)))
         (setf *sides-log* (append combined-list *sides-log*))
  updated-sides))

;; ============ planning for which moves to do to change a side ==============


;; plans the path between the current bottom side and the side that should be scanned next
;; then returns said path (list were the movement are the elements)
(defun path-plan-next-side (side-list non-scanable non-graspable goal) ;;(current-side next-side side-list object-vector)
  (let* ((combined-list (append non-scanable non-graspable))
         (sorted-sides-list (remove-if (lambda (x)
                                     (member (second x) combined-list
                                             :test #'equal))
                                   (check-sides-moves side-list goal))))
    (if (null sorted-sides-list)
        (path-second-step side-list goal)
     (remove nil (list (first (car sorted-sides-list)))))))

(defun check-sides-moves (sides-list goal)
  (remove nil (mapcar (lambda (x)
                        (if (equal goal (second x)) x))
                      sides-list)))

(defun path-second-step (move-list goal)
  (let ((new-sides (car (reverse (car move-list)))))
    (let ((second-move (remove nil (caar (check-sides-moves (side-changes new-sides) goal)))))
      (list (first (car move-list)) second-move))))

(defun resolve-plan (plan)
  (let ((new-plan nil))
    (mapcar (lambda (x) (setf new-plan (append new-plan x)))
            (mapcar (lambda (x)
                      (cond
                        ((string-equal "back-turn" x)
                         (list "left-turn" "back-turn"))
                        ((string-equal "front-turn" x)
                         (list "right-turn" "front-turn"))
                        (t (list x)))) plan))
    new-plan))
        
          



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
    
    (lisp-fun set-sides ?object-name 0.1 0.1 0.1 ?sides-base)
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
    (lisp-fun sides-to-check ?sides-transformed ?object-size ?non-scanable ?non-graspable ?sides-to-check)

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
    (lisp-fun side-changes ?located-sides ?side-changes)
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

