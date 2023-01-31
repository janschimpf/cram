(in-package :cashier)

(defparameter object-list-bottle
  (list 'bottle-1 :bottle
        '((-2 2 0.75)(0 0 0 1)) '(0.05 0.05 0.15) (list :left :right :front :back :top) (list nil) :top))


(defparameter object-list-pringles
  (list 'pringle-1 :pringle
        '((-2 2 0.75)(0 0 0 1)) '(0.05 0.05 0.15) (list nil) (list :left :right) :top))


(defparameter object-list-breakfast-cereal
  (list 'breakfast-cereal-1 :breakfast-cereal
        '((-2 2 0.75) (0 0 0 1)) '(0.05 0.15 0.2) (list nil) (list nil) :front))

(defparameter object-list-cup
  (list 'cup-1 :cup
        '((-2 2 0.75)(0 0 0 1)) '(0.05 0.05 0.15) (list :front :back :left :right) (list nil) :top))

(defparameter object-list-small-cube
  (list 'small-cube-1 :small-cube
        '((-2 2 0.75)(0 0 0 1)) '(0.05 0.05 0.15) (list :front :back :left :right) (list nil) :top))

(defparameter object-list-snackbar
  (list 'snackbar-1 :snackbar
        '((-2 2 0.75)(0 0 0 1)) '(0.05 0.05 0.15) (list :front :back :left :right) (list nil) :top))


(defparameter object-list-fruit-juice
  (list 'fruit-juice-1 :fruit-juice
        '((-2 2 0.75)(0 0 0 1)) '(0.05 0.05 0.15) (list :top) (list nil) :right))

(defparameter object-list-small-book
  (list 'small-book-1 :small-book
        '((-2 2 0.75)(0 0 0 1)) '(0.05 0.05 0.15) (list :front :back :left :right) (list nil) :bottom))



(defparameter spawn-objects-list (list
                                       object-list-bottle
                                       object-list-cup
                                       ;;object-list-fruit-juice
                                       object-list-breakfast-cereal
                                       object-list-small-book
                                       object-list-snackbar
                                       object-list-small-cube
                                       ))

(defun spawn-object-on-counter-general (list spawn-pose)
  (let ((name (first list))
        (type (second list))
        (pose (list spawn-pose (second (third list)))))
  (btr-utils:spawn-object name type :color '(0 1 0) :pose pose :mass 0.2)
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


(defparameter object-list-small-book-non-mesh
  (list 'small-book-1 :small-book
        '((-2 2 0.75)(0 0 0 1)) '(0.05 0.05 0.1) (list nil) (list nil) :bottom))

(defun spawn-highlight-box (pose size)
  (btr:add-object btr:*current-bullet-world* :box 'box-1
                  pose 
                  :color '(1.0 0.0 0.0 1.0)
                  :mass 1
                  :size size
                  ))
