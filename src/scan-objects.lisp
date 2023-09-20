(in-package :cashier)
;;@author Jan Schimpf


(defparameter *goal-list* '(nil))
(defparameter *sides-log* '(nil))
(defparameter *knowledge-node* nil)
(defparameter *scan-area*
  (cl-transforms-stamped:make-pose-stamped
   "map" 0.0
   (cl-transforms:make-3d-vector -2 1.4 0.7)
   (cl-transforms:make-quaternion 0 0 0 1)))

(defun scan (?perceived-object b-sides)
  (let* ((scan-area-origin (cl-tf2:origin *scan-area*))
         (pose-in-map (man-int:get-object-pose-in-map ?perceived-object))
         (object-type (desig-prop-value ?perceived-object :type))
         (object-vector (cram-tf:3d-vector->list
                         (cl-tf2:origin pose-in-map)))
         (sides-in-map (transforms-map-t-side pose-in-map b-sides))
         (transformed-map-t-side (transforms-map-t-side
                                  (man-int:get-object-pose-in-map ?perceived-object)
                                   b-sides))
         (bottom-side (car (side-location ?perceived-object b-sides)))
         (bottom-side-location (cram-tf:3d-vector->list (cl-tf2:origin
                                                        (get-correct-side
                                                         transformed-map-t-side
                                                         bottom-side))))
         (scanned nil))
    
    
    (if (and (side-check ?perceived-object b-sides)
             (x-y-z-pose-check scan-area-origin bottom-side-location))
        (setf scanned bottom-side))
    
    (if (and scanned *knowledge-node*)
        (setf scanned (append (product-type-and-dan-from-gtin (prolog-scan-gtin object-type)) scanned)))

    (spawn-highlight-box *scan-area*
                         (list 0.05 0.05 0.05)
                         (if scanned
                             '(0 1 0)
                             '(1 0 0)))
    
    (spawn-side-visualisation sides-in-map (concatenate 'string (format nil "~a" (list object-type bottom-side))))
    (btr-utils:kill-object 'box-1)
    scanned))

(defun x-y-z-pose-check (scan-area-vector object-vector)
  (let*  ((scan-x (cl-tf2:x scan-area-vector))
         (scan-y (cl-tf2:y scan-area-vector))
         (scan-z (cl-tf2:z scan-area-vector))
         (object-x (first object-vector))
         (object-y (second object-vector))
          (object-z (third object-vector)))

    (if (and (< (- scan-x 0.06) object-x)
             (< object-x  (+ scan-x 0.06))
             (< (- scan-y 0.06) object-y)
             (< object-y  (+ scan-y 0.06))
             (< (- scan-z 0.05) object-z)
             (< object-z (+ scan-z 0.05)))
        t
        nil)))


(defun side-check (perceived-object b-sides)
  (let* ((bottom-side (car (side-location perceived-object b-sides)))
         (object-name (desig-prop-value perceived-object :name))
         (goal-tuple (list object-name bottom-side)))
    (setf *sides-log* (append (list goal-tuple *goal-list*) *sides-log*))
    (if (member goal-tuple *goal-list* :test #'equal)
            t
            nil)))


(defun distances-for-side-list (side-list scan-vector)
  (let ((map-side-list (change-side-list-to-map side-list)))   
   (mapcar (lambda (x) (list (first x)
                             (distance-between-poses scan-vector
                                                     (cram-tf:3d-vector->list (cl-tf2:origin (second x))))))
           map-side-list)))

(defun change-side-list-to-map (side-list)
  (mapcar (lambda (x) (list (first x) (second x))) side-list))

;;gets two poses that are in relation to the map and then returns the
;;distance between these two poses
(defun distance-between-poses (pose-1 pose-2)
  (sqrt (+ (expt (- (first pose-2) (first pose-1)) 2)
           (expt (- (second pose-2) (second pose-1)) 2)
           (expt (- (third pose-2) (third pose-1)) 2))))
  
(defun shortest-distance (side-list)
  (sort side-list #'< :key 'second))


(defun pose-to-vector-list (pose)
   (cram-tf:3d-vector->list (cl-tf2:origin pose)))


(defun shortest-distance-between-all-sides (side-list xyz-list)
  (shortest-distance (distances-for-side-list side-list xyz-list)))
