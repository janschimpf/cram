(in-package :cashier)
;;@author Jan Schimpf

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
    (concatenate 'string "gtin:"gtin ", product-type:" product-type ", dan:" dan)))



(defun opposite-short (side)
  (cdaar (prolog:prolog `(or (opposite ,side ?x)
                             (opposite ?x ,side)))))
(defun axis-short (side)
  (cdaar (prolog:prolog `(axis ,side ?x))))

(defun prolog-shape (object-type)
  (cdaar (prolog:prolog `(btr-spatial-cm::%item-type-shape ,object-type ?x))))

(defun prolog-disabled-side (shape)
  (mapcar (lambda (x) (cdar x))
          (prolog::force-ll  (prolog:prolog `(shape-disabled-sides ,shape ?x)))))

(defun prolog-scan-gtin (object-type)
  (cdaar (prolog:prolog `(productype-to-gtin ,object-type ?x))))

    
(def-fact-group sides-predicates (opposite)
  (<- (opposite :top :bottom))
  (<- (opposite :left-side :right-side))
  (<- (opposite :front :back))

  (<- (axis :front x))
  (<- (axis :back -x))
  (<- (axis :left-side -y))
  (<- (axis :right-side y))
  (<- (axis :top -z))
  (<- (axis :bottom z))


  (<- (productype-to-gtin :snackbar "8718951045118"))
  (<- (productype-to-gtin :small-cube "4062300020719"))
  (<- (productype-to-gtin :cup "4062300025318"))
  (<- (productype-to-gtin :bottle "4062300265998"))
  (<- (productype-to-gtin :fruit-juice "4015000010320"))
  (<- (productype-to-gtin :small-book "4004980506206"))
  (<- (productype-to-gtin :breakfast-cereal "4015000010511"))
  (<- (productype-to-gtin :pringle "4013162004027"))


  (<- (shape-disabled-sides :circle :left))
  (<- (shape-disabled-sides :circle :right))
  (<- (shape-disabled-sides :circle :front))
  (<- (shape-disabled-sides :circle :back)))
