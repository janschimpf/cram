(in-package :cashier)

;; =============== Transform + Logic for which sides to check =====


(defun transforms-map-T-side (object-name side-poses)
  (mapcar (lambda (x)
            (let* ((map-T-object (cl-transforms:pose->transform (btr:object-pose object-name)))
                   (object-T-side (cl-transforms:pose->transform (first (cdr x))))
                   (map-T-side  (cl-transforms:transform* map-T-object object-T-side)))
    (list (car x) (cl-transforms:transform->pose map-T-side))))
          side-poses))

(defun sides-to-check (sides-base)
  (mapcar (lambda (x) (car x)) sides-base))


;; ================ Path planning for changing side ================


;; ================ Designator def =================================
(prolog:def-fact-group cashier-plans (desig:action-grounding) 
  (prolog:<- (desig:action-grounding ?action-designator
                                     (cashier-object ?resolved-action-designator))
    (desig-prop ?action-designator (:type :cashier))
    (desig-prop ?action-designator (:object-name ?object-name))
    (desig-prop ?action-designator (:object-type ?object-type))
    (desig-prop ?action-designator (:goal-side ?goal-side))
    (desig-prop ?action-designator (:object-list ?object-list))
    
    (lisp-fun set-sides ?object-name 0.1 0.1 0.1 ?sides-base)
    (lisp-fun transforms-map-t-side ?object-name ?sides-base ?sides-transformed)
    
    (desig:designator :action ((:type :cashier)
                               (:object-type ?object-type)
                               (:object-name ?object-name)
                               (:goal-side ?goal-side)
                               (:sides-base ?sides-base)
                               (:sides-transformed ?sides-transformed))
                      ?resolved-action-designator))
  
  (prolog:<- (desig:action-grounding ?action-designator
                                     (scan-object ?resolved-action-designator))
    (desig-prop ?action-designator (:type :scanning))
    (desig-prop ?action-designator (:sides-base ?sides-base))
    (desig-prop ?action-designator (:goal-side ?goal-side))
    (desig-prop ?action-designator (:object-type ?object-type))
    (desig-prop ?action-designator (:object-name ?name))

    (lisp-fun transforms-map-T-side ?object-name ?sides-base ?sides-transformed)

    (desig:designator :action ((:type :scanning)
                               (:sides-base ?sides-base)
                               (:goal-side ?goal-side)
                               (:object-type ?object-type)
                               (:object-name ?name)
                               (:sides-transformed ?sides-transformed))
                      ?resolved-action-designator))

  (prolog:<- (desig:action-grounding ?action-designator
                                     (change-side ?resolved-action-designator))
    (desig-prop ?action-designator (:type :changing-side))
    (desig-prop ?action-designator (:object-type ?object-type))
    (desig-prop ?action-designator (:object-name ?object-name))
    (desig-prop ?action-designator (:sides-transformed ?sides-transformed))
    (desig-prop ?action-designator (:arm ?arm))
    (desig-prop ?action-designator (:grasp ?grasp))
    (desig-prop ?action-designator (:change-to-side ?side-goal))
    (desig-prop ?action-designator (:object-vector ?object-vector))

    (lisp-fun locate-sides ?sides-transformed ?object-vector ?located-sides)
    (lisp-fun side-changes ?located-sides ?side-changes)
    (lisp-fun path-plan-next-side ?side-changes ?side-goal ?plan)

    (desig:designator :action ((:type :changing-side)
                               (:object-type ?object-type)
                               (:object-name ?object-name)
                               (:arm ?arm)
                               (:grasp ?grasp)
                               (:plan ?plan))
                      
                      ?resolved-action-designator))                   
  )

