(in-package :cashier)

(defvar *marker* nil "text marker publisher")

(defmacro with-safe-prolog (&body body)
  `(handler-case
      ,@body
     (simple-error ()
       (roslisp:ros-error (json-prolog-client)
                            "Json prolog client error. Check your query again."))
     (SB-KERNEL:CASE-FAILURE ()
       (roslisp:ros-error (json-prolog-client) "Startup your rosnode first"))
     (ROSLISP::ROS-RPC-ERROR ()
       (roslisp:ros-error (json-prolog-client)
                            "Is the json_prolog server running?"))))
                            
(defun testing ()
  (roslisp:ros-info (json-prolog-client) "Testing connection.")
  (with-safe-prolog
                         (json-prolog:prolog-simple "subclass_of(A, dul:'PhysicalObject')."
                                                    :package :cashier)))

                            
(defun testing-2 ()
  (roslisp:ros-info (json-prolog-client) "Testing query get_product_type.")
  (cdaar (with-safe-prolog
    (json-prolog:prolog-simple "get_product_type(8718951045118, ProductType)."
                               :package :cashier))))
(defun testing-3 ()
  (roslisp:ros-info (json-prolog-client) "Testing query get_product_type.")
  (cdaar (with-safe-prolog
    (json-prolog:prolog-simple "(number(9120063680993) -> atom_number(Gtin_atom, 9120063680993); Gtin_atom is 9120063680993)."
                               :package :cashier))))
(defun testing-4 ()
  (roslisp:ros-info (json-prolog-client) "Testing query get_product_gtin.")
  (cut:force-ll (with-safe-prolog (json-prolog:prolog-simple      "get_product_gtin(Productype, Gtin)."
                               :package :cashier))))

(defun testing-5 ()
  (roslisp:ros-info (json-prolog-client) "Testing query get_product_gtin.")
  (with-safe-prolog
    (json-prolog:prolog-simple "get_product_gtin(ProductWithAN472572, gtin)."
                               :package :cashier)))


(defun get-product-dan-from-type (product-type)
  (roslisp:ros-info (json-prolog-client) "Testing query get_product_dan.")
  (let* ((raw-response
  (cut:force-ll (with-safe-prolog
    (json-prolog:prolog-simple (concatenate 'string "get_product_dan('"product-type"', Dan).")
                               :package :cashier)))))
    (if (eq raw-response 1)
          (roslisp:ros-warn (knowrob-client)
                            "Query get_product_type didn't reach any solution.")
           (string-trim "'" (cdar (cut:lazy-car raw-response))))))

(defun get-product-from-productgtin (productnum)
  (roslisp:ros-info (json-prolog-client) "query for product-info")
  (let* ((raw-response
           (with-safe-prolog
             (json-prolog:prolog-simple (concatenate 'string "get_product_type("productnum",ProductType).")
                                        :package :cashier))))
    (if (eq raw-response 1)
          (roslisp:ros-warn (knowrob-client)
                            "Query get_product_type didn't reach any solution.")
            (string-trim "'" (cdar (cut:lazy-car raw-response))))))
  
(defun product-type-and-dan-from-gtin (gtin)
  (let* ((product-type (get-product-from-productgtin gtin))
         (dan (get-product-dan-from-type product-type)))
    (concatenate 'string "gtin:"gtin ", product-type:" product-type ", dan:" dan)
    ))

(defun init-marker ()
  (setf *marker* (roslisp:advertise (format nil "/visualization_marker") "visualization_msgs/Marker")))

(defun publish-rviz-marker (text)
  (let* ((position (roslisp:make-msg "geometry_msgs/Point" (x) 1 (y) 1 (z) 1))
         (orientation (roslisp:make-msg "geometry_msgs/Quaternion"
                                        (x) 0 (y) 1 (z) 0 (w) 1))
         (pose (roslisp:make-msg "geometry_msgs/Pose" (position) position
                               (orientation) orientation))
         (header (roslisp:make-msg "std_msgs/Header" (frame_id) "map"))
         (scale (roslisp:make-msg "geometry_msgs/vector3" (x) 1 (y) 0.1 (z) 0.1))
         (message (roslisp:make-message "visualization_msgs/Marker"
                                         (header) header
                                         (ns) "pr2_cashier"
                                         (id) 9
                                         (type) 9
                                         (action) 0
;;                                         (pose) (roslisp:make-msg "geometry_msgs/PoseStamped"
;;                                                                  (header) header
;;                                                                  (pose) pose)
                                         (color) (roslisp:make-msg "std_msgs/ColorRGBA"(r) 1 (g) 0 (b) 0 (a) 1)
                                         (scale) scale
                                         (text) text
                                         )))
    (roslisp:publish (roslisp:advertise (format nil "/visualization_marker") "visualization_msgs/Marker") message)))
