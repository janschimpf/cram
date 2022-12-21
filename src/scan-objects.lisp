(in-package :cashier)

(defparameter *sides-log* '(nil))

(defun get-scan-area ()
  (cl-transforms-stamped:make-pose-stamped
   "map" 0.0
   (cl-transforms:make-3d-vector -2 1.2 0.70)
   (cl-transforms:make-quaternion 0 0 0 1)))

;;for changing the relation of the side-pose from the object to the map
(defun set-sides (object-name object-x object-y object-z)
  (let ((object-rotation (cl-tf2:orientation
                          (cram-tf::pose-stamped->pose (btr:object-pose object-name))))
       (side-list (list
                   (list :top (cl-transforms:make-3d-vector 0 0 0.05))
                   (list :bottom (cl-transforms:make-3d-vector 0 0 -0.05))
                   
                   (list :left (cl-transforms:make-3d-vector 0 0.05 0))
                   (list :right (cl-transforms:make-3d-vector 0 -0.05 0))
                   
                   (list :front (cl-transforms:make-3d-vector
                                 0.05 0 0 ))
                   (list :back (cl-transforms:make-3d-vector
                                -0.05 0 0)))))
    (let ((side (mapcar (lambda (x) (list (first x) (cl-tf:make-pose
                         (second x)
                         object-rotation)))
                        side-list)))
      side)))

(defun scan (object-name side side-list)
  (let* ((scan-area-vector (first (cram-tf:pose->list
                                  (cram-tf::pose-stamped->pose (get-scan-area)))))
        
        (object-vector (cram-tf:3d-vector->list
                        (cl-tf2:origin (btr:object-pose object-name))))
        (scan nil))
    (spawn-highlight-box (get-scan-area) (list 0.1 0.1 0.05))
    (if (and (side-check side object-vector side-list object-name)
             (x-y-z-pose-check scan-area-vector object-vector))
        (setf scan t))
    (btr-utils:kill-object 'box-1)
    scan
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
    (print "side-check")
    (print side-as-is)
    (print side-to-be)
    (if (equal side-to-be side-as-is)
            t
            nil
  )))


(defun distances-for-side-list (side-list scan-vector)
  (let ((map-side-list (change-side-list-to-map side-list)))   
   (mapcar (lambda (x) (list (first x)
                             (distance-between-vectors scan-vector
                                                       (pose-to-vector-list (second x)))))
           map-side-list)))

(defun change-side-list-to-map (side-list)
  (mapcar (lambda (x) (list (first x) (second x))) side-list))

;;gets two poses that are in relation to the map and then returns the
;;distance between these two poses
(defun distance-between-vectors (scan-vector object-vector)
  (let ((3d-vector-1 scan-vector)
        (3d-vector-2 object-vector))
    (sqrt (+ (expt (- (first 3d-vector-2) (first 3d-vector-1)) 2)
             (expt (- (second 3d-vector-2) (second 3d-vector-1)) 2)
             (expt (- (third 3d-vector-2) (third 3d-vector-1)) 2)))))
  
(defun shortest-distance (side-list)
  (sort side-list #'< :key 'second))


(defun pose-to-vector-list (pose)
   (cram-tf:3d-vector->list (cl-tf2:origin pose)))


(defun shortest-distance-between-all-sides (side-list xyz-list)
  (shortest-distance (distances-for-side-list side-list xyz-list)))
