(in-package :cashier)

(defparameter *sides-log* '(nil))
(defparameter *scan-area*
  (cl-transforms-stamped:make-pose-stamped
   "map" 0.0
   (cl-transforms:make-3d-vector -2 1.2 0.7)
   (cl-transforms:make-quaternion 0 0 0 1)))

;;Sets the sides that are needed for scanning and object handling
(defun set-sides (object-name object-x-size object-y-size object-z-size)
  (let ((object-rotation (cl-tf2:orientation
                          (cram-tf::pose-stamped->pose (btr:object-pose object-name))))
       (side-list (list
                   (list :top (cl-transforms:make-3d-vector 0 0 object-z-size))
                   (list :bottom (cl-transforms:make-3d-vector 0 0 0))
                   (list :left (cl-transforms:make-3d-vector 0 object-y-size 0))
                   (list :right (cl-transforms:make-3d-vector 0 (- 0 object-y-size ) 0))
                   (list :front (cl-transforms:make-3d-vector object-x-size 0 0 ))
                   (list :back (cl-transforms:make-3d-vector (- 0 object-x-size ) 0 0)))))
    (let ((side (mapcar (lambda (x) (list (first x) (cl-tf:make-pose
                         (second x)
                         object-rotation)))
                        side-list)))
      side)))

(defun set-sides-helper (object-name object-size-list)
  (set-sides object-name (first object-size-list) (second object-size-list) (third object-size-list)))

(defun scan (object-name type side side-list)
  (let* ((scan-area-vector (first (cram-tf:pose->list
                                   (cram-tf::pose-stamped->pose *scan-area*))))
        
        (object-vector (cram-tf:3d-vector->list
                        (cl-tf2:origin (btr:object-pose object-name))))
         (scanned nil))
    
    (spawn-highlight-box *scan-area* (list 0.03 0.03 0.03))
    
    (if (and (side-check side object-vector side-list object-name)
             (x-y-z-pose-check scan-area-vector object-vector))
        (setf scanned t))
    (if scanned
        (publish-rviz-marker (product-type-and-dan-from-gtin (prolog-scan-gtin type))))
    
    (btr-utils:kill-object 'box-1)
    scanned
    ))


(defun x-y-z-pose-check (scan-area-vector object-vector)
  (print scan-area-vector)
  (print object-vector)
  (let*  ((scan-x (first scan-area-vector))
         (scan-y (second scan-area-vector))
         (scan-z (third scan-area-vector))
         (object-x (first object-vector))
         (object-y (second object-vector))
          (object-z (third object-vector)))
    (print "x-y-z-pose check")
    (if (and (< (- scan-x 0.1) object-x)
             (< object-x  (+ scan-x 0.10))
             (< (- scan-y 0.1) object-y)
             (< object-y  (+ scan-y 0.1))
             (< (- scan-z 0.1) object-z)
             (< object-z   (+ scan-z 0.15)))
        t
        nil
        )) 
  )


(defun side-check (side-to-be object-vector side-list object-name)
  (let* ((side-as-is (car (locate-sides side-list object-vector)))
         (path-name (concatenate 'string "/tmp/"
                                 (format nil "~a" object-name)
                                 "-"
                                 (format nil"~a" side-as-is)
                                 ".png")))
    (setf *sides-log* (append (list side-as-is) *sides-log*))
    (btr::png-from-camera-view 
     :png-path path-name)
    
    (if (equal side-to-be side-as-is)
            t
            nil
  )))


(defun distances-for-side-list (side-list scan-vector)
  (let ((map-side-list (change-side-list-to-map side-list)))   
   (mapcar (lambda (x) (list (first x)
                             (distance-between-poses scan-vector
                                                       (pose-to-vector-list (second x)))))
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