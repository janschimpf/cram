
(in-package :location-costmap)

(defun make-padding-mask (size)
  (let ((mask (make-array `(,(* size 2) ,(* size 2)) :element-type 'fixnum))
        (dist^2 (expt size 2)))
    (declare (type (simple-array fixnum *) mask))
    (dotimes (row (* size 2))
      (dotimes (col (* size 2))
        (if (<= (+ (expt (- col size) 2) (expt (- row size) 2))
                dist^2)
            (setf (aref mask row col) 1)
            (setf (aref mask row col) 0))))
    mask))

(declaim (ftype (function ((simple-array fixnum 2) fixnum fixnum (simple-array fixnum 2))
                          boolean) point-in-range-mask-p))
(defun point-in-padding-mask-p (map x y range-mask)
  (declare (type (simple-array fixnum 2) map range-mask)
           (type fixnum x y))
  (let ((array-height (array-dimension map 0))
        (array-width (array-dimension map 1))
        (mask-height (array-dimension range-mask 0))
        (mask-width (array-dimension range-mask 1)))
    (let* ((mask-width/2 (round (/ mask-width 2)))
           (mask-height/2 (round (/ mask-height 2)))
           (start-x (if (< (- x mask-width/2) 0)
                        x mask-width/2))
           (end-x (if (> (+ x mask-width/2) array-width)
                      (- array-width x)
                      mask-width/2))
           (start-y (if (< (- y mask-height/2) 0)
                        y mask-height/2))
           (end-y (if (> (+ y mask-height/2) array-height)
                      (- array-height y)
                      mask-height/2)))
      (do ((y-i (- y start-y) (+ y-i 1))
           (mask-y (- mask-height/2 start-y) (+ mask-y 1)))
          ((>= y-i (+ y end-y)))
        (do ((x-i (- x start-x) (+ x-i 1))
             (mask-x (- mask-width/2 start-x) (+ mask-x 1)))
            ((>= x-i (+ x end-x)))
          (let ((mask-value (aref range-mask mask-y mask-x))
                (map-value (aref map y-i x-i)))
            (when (and (> map-value 0) (> mask-value 0))
              (return-from point-in-padding-mask-p t)))))))
  nil)

(defun occupancy-grid-put-mask (x y grid mask &key (coords-raw-p nil))
  "Puts the mask into grid, at position x and y. Please note that the
  mask must completely fit, i.e. x/resolution must be >= 0.5
  mask-size-x. `coords-raw-p indicates if x and y are direct indices
  in the grid array or floating point coordinates in the reference
  coordinate system."
  (let ((x (if coords-raw-p x (round (/ (- x (origin-x grid)) (resolution grid)))))
        (y (if coords-raw-p y (round (/ (- y (origin-y grid)) (resolution grid)))))
        (grid-arr (grid grid))
        (mask-size-x (array-dimension mask 1))
        (mask-size-y (array-dimension mask 0)))
    (declare (type fixnum x y)
             (type (simple-array fixnum *) grid-arr mask))
    (do ((grid-row (- y (truncate (/ (array-dimension mask 0) 2)))
           (+ grid-row 1))
         (mask-row 0 (+ mask-row 1)))
        ((>= mask-row mask-size-y))
      (do ((grid-col (- x (truncate (/ (array-dimension mask 1) 2)))
             (+ grid-col 1))
           (mask-col 0 (+ mask-col 1)))
          ((>= mask-col mask-size-x))
        (when (eql (aref mask mask-row mask-col) 1)
          (setf (aref grid-arr grid-row grid-col) 1))))))

