(in-package :cashier)

(defun scan (object-name side)
  (let* ((3d-vector (first (cram-tf:pose->list (cram-tf::pose-stamped->pose (get-scan-area)))))
         (object-vector (cram-tf:3d-vector->list (cl-tf2:origin (btr:object-pose object-name)))
         (object-rotation (cl-transforms:orientation (btr:object-pose object-name))))
    
    (if (x-y-z-pose-check 3d-vector object-vector)
        (roslisp:ros-info (scan-object) "object is in the scan area")
        (roslisp:ros-info (scan-object) "object is no inside the scan area"))
    
    (if (side-check side 3d-vector object-vector)
        (roslisp:ros-info (scan-object) "Object has the correct rotation and so the code was scaned")
        (roslisp:ros-info (scan-object) "Object has the wrong rotation"))        
        
  ))


(defun x-y-z-pose-check (3d-vector object-vector)
  (let*  ((scan-x (first 3d-vector))
         (scan-y (second 3d-vector))
         (scan-z (third 3d-vector))
         (object-x (first object-vector))
         (object-y (second object-vector))
         (object-z (third object-vector)))
    
    (if (and (< (- scan-x 0.05) object-x)
             (< object-x  (+ scan-x 0.05))
             (< (- scan-y 0.05) object-y)
             (< object-y  (+ scan-y 0.05))
             (< (- scan-z 0.05) object-z)
             (< object-z   (+ scan-z 0.05)))
        t
        nil
        )) 
  )


(defun side-check (side-to-be 3d-vector object-vector)
  (let ((side-as-is (shortest-distance (distance-between-two-poses 3d-vector object-vector))))
        
        (if (equal side-to-be side-as-is)
            t
            nil
  )))


;;for changing the relation of the side-pose from the object to the map
(defun set-sides (object-name object-x object-y object-z)
  (let ((object-pose (cram-tf:3d-vector->list (cl-tf2:origin (cram-tf::pose-stamped->pose
                                               (btr:object-pose object-name)))))
        
        (object-rotation (cl-tf2:orientation (cram-tf::pose-stamped->pose
                                                                  (btr:object-pose object-name))))
       (side-list (list
                   (list :top (cl-transforms:make-3d-vector 0 0 (/ object-z 2)))
                   (list :bottom (cl-transforms:make-3d-vector 0 0 (- (/ object-z 2))))
                   (list :left (cl-transforms:make-3d-vector 0 (/ object-y 2) 0))
                   (list :right (cl-transforms:make-3d-vector 0 ( -(/ object-y 2)) 0))
                   (list :front (cl-transforms:make-3d-vector (/ object-x 2) 0 0))
                   (list :back (cl-transforms:make-3d-vector (- (/ object-x 2)) 0 0)))))
    (let ((side (mapcar (lambda (x) (list (first x) (cl-tf:make-pose-stamped
                         "object" 0
                         (second x)
                         object-rotation))) side-list)))
      side)))


;;gets two poses that are in relation to the map and then returns the distance between these two poses
(defun distance-between-two-poses (pose-1 pose-2)
  (let ((3d-vector-1 (first (cram-tf:pose->list (cram-tf::pose-stamped->pose pose-1))))
        (3d-vector-2 (first (cram-tf:pose->list (cram-tf::pose-stamped->pose pose-2)))))
    (sqrt (+ (exp (- (first 3d-vector-1) (first 3d-vector-2)))
             (exp (- (second 3d-vector-1) (second 3d-vector-2)))
             (exp (- (third 3d-vector-1) (third 3d-vector-2)))))))
  
(defun shortest-distance (list)
    list)
