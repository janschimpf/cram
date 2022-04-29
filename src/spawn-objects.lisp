(in-package :cashier)

(defparameter object-list
  (list 'bottle-1 :bottle (list (list -2 1.3 0.8) (list 0 0 0 1)) :bottom ))

(defun spawn-object-on-counter-general (list)
  (let ((name (first list))
        (type (second list))
        (pose (third list)))
  (btr-utils:spawn-object name type :color '(1 0 0) :pose pose)
  (btr:simulate btr:*current-bullet-world* 10)))

(defun spawn-cylinder (list)
  "Spawn primitive cylinder as :pringles item and try to pick up from table."
  (let ((name (first list))
        (type (second list))
        (pose (third list)))
    (btr:add-object btr:*current-bullet-world* :cylinder-item name
                    pose
                    :mass 0.2
                    :size (cl-transforms:make-3d-vector 0.03 0.03 0.08)
                    :item-type type)))

(defun spawn-bottle ()
  (btr-utils:spawn-object 'bottle-1 :bottle :color '(1 0 0) :pose '((-2 1.3 0.8) (0 0 0 1)))
  (btr:simulate btr:*current-bullet-world* 10))

(defun spawn-pickup-cylinder-air ()
  "Spawn primitive cylinder as :pringles item and try to pick up."
    (btr:add-object btr:*current-bullet-world* :cylinder-item 'cylinder-1
                    '((-2 1.3 0.8) (0 0 0 1))
                    :mass 0.2 :size (cl-transforms:make-3d-vector 0.03 0.03 0.08)
                    :item-type :pringles))
