(in-package :cashier)
;;@author Jan Schimpf

(defparameter object-list-bottle
  (list 'bottle-1 :bottle
        '((-2 2 0.75)(0 0 0 1)) ;; '(0.05 0.05 0.10)
        '(nil)
        (list :left :right :front :top) (list :back) :top))


(defparameter object-list-pringles
  (list 'pringle-1 :pringle
        '((-2 2 0.75)(0 0 0 1)) '(0.05 0.05 0.15) (list nil) (list :left :right) :top))


(defparameter object-list-breakfast-cereal
  (list 'breakfast-cereal-1 :breakfast-cereal
        '((-2 2 0.75) (0 0 0 1))
        '(0.08 0.03 0.12)
        (list :right :left) (list nil) :front))


(defparameter object-list-milk
  (list 'milk-1 :milk
        '((-2 2 0.75) (0 0 0 1))
        '(0.04 0.04 0.11)
        (list nil) (list nil) :bottom))

(defparameter object-list-cup
  (list 'cup-1 :cup
        '((-2 2 0.75)(0 0 0 1))
        '(0.03 0.03 0.10)
        (list :front :back :left :right) (list nil) :top))

(defparameter object-list-snackbar
  (list 'snackbar-1 :snackbar
        '((-2 2 0.75)(0 0 0 1)) '(0.05 0.05 0.15) (list :front :back :left :right) (list nil) :top))


(defparameter object-list-fruit-juice
  (list 'fruit-juice-1 :fruit-juice
        '((-2 2 0.75)(0 0 0 1))
        '(0.04 0.04 0.1)
        (list :top)
        (list nil)
        :left))

(defparameter object-list-small-book
  (list 'small-book-1 :small-book
        '((-2 2 0.75)(0 0 0 1))
        '(0.04 0.04 0.10)
        (list :front :back :left :right) (list nil) :bottom))



(defparameter spawn-objects-list (list
                                       object-list-bottle
                                       ;;object-list-cup
                                       object-list-breakfast-cereal
                                       ;;object-list-milk
                                       ;object-list-fruit-juice
                                       object-list-small-book
                                       ;;object-list-snackbar
                                       ;;object-list-small-cube
                                       ))

(defun spawn-object-on-counter-general (list spawn-pose)
  (let ((name (first list))
        (type (second list))
        (pose (list spawn-pose (second (third list)))))
  (btr-utils:spawn-object name type :color '(0.2 0.2 0.2) :pose pose :mass 0.2)
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
  (btr-utils:spawn-object 'bottle-1 :bottle :color '(0 1 0) :pose '((-2 1.3 0.8) (0 0 0 1)))
  (btr:simulate btr:*current-bullet-world* 10))

(defun spawn-breakfast ()
  (let* ((orientation
           (cl-tf2:euler->quaternion :ax (- (/ pi 2)) :ay 0 :az 0))
         (front '(-0.7044579189291776d0 0.013910529484711321d0 -0.7096014702720645d0 -0.0033601518999026667d0))
         (right '(0 0 0.7071067811865475d0 0.7071067811865476d0))
         (left '(-0.7071067811865475d0 0.0d0 0.0d0 0.7071067811865476d0))
         (test '(7.096579573215435d-9 -0.9999999828857289d0 -8.896134450347508d-10 5.5511151122311594d-17)))
        (btr-utils:spawn-object
         'small-book-1
         :small-book
         :color '(0 1 0)
         :pose (list (list -2 1.3 0.75) '(0 0 0 1)))
   (btr:simulate btr:*current-bullet-world* 10)
    ))

(defun spawn-pickup-cylinder-air ()
  "Spawn primitive cylinder as :pringles item and try to pick up."
    (btr:add-object btr:*current-bullet-world* :cylinder-item 'cylinder-1
                    '((-2 1.3 0.8) (0 0 0 1))
                    :mass 0.2 :size (cl-transforms:make-3d-vector 0.03 0.03 0.08)
                    :item-type :pringles))


(defun spawn-highlight-box (pose size color)
  (btr:add-object btr:*current-bullet-world* :visualization-box 'box-1
                   pose 
                  :color color
                  :mass 1
                  :size size
                  ))

(defun table-reenforcement-scan ()
  (btr:add-object btr:*current-bullet-world* :box 'table-scan
                  '((-2.2 1.3 0.68) (0 0 0 1))
                  :color '(0 1 0.8)
                  :mass 1
                  :size '(0.4 0.8 0.02)
                  ))

(defun table-reenforcement-spawn ()
  (btr:add-object btr:*current-bullet-world* :box 'table-spawn
                  '((-2 2.3 0.68) (0 0 0 1))
                  :color '(0 1 1)
                  :mass 1
                  :size '(0.4 0.8 0.02)
                  ))
             


(defun spawn-highlight-box-2 (pose size color name)
  (btr:add-object btr:*current-bullet-world*
                  :visualization-box
                  name
                  pose 
                  :color color
                  :mass 1
                  :size size
                  ))

(defun spawn-side-visualisation (transformed-side-list purpose)
  (let* ((color-list (list  '(1 1 0) ;; yellow
                            '(1 1 1) ;; grey
                            '(1 0 1) ;; Purple
                            '(0 1 1) ;; Bright blue
                            '(1 0 0) ;; Red
                            '(0 0 1)) ;;dark blue
                           )
         (path-name (concatenate 'string "/tmp/"
                                 purpose
                                 ".png")))
    (loop for side in transformed-side-list
          for color in color-list
        do
           (spawn-highlight-box-2 (cadr side) (list 0.01 0.01 0.01) color (car side))
           (print color)
           (print (car side))
          )
    (btr::png-from-camera-view :png-path path-name)
    (loop for side in transformed-side-list
          do
             (btr-utils:kill-object (car side)))
    ))

