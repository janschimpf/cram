(in-package :cashier)


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
  (cut:force-ll (with-safe-prolog (json-prolog:prolog-simple 
     "get_product_gtin(Productype, Gtin)."
                               :package :cashier))))

(defun testing-5 ()
  (roslisp:ros-info (json-prolog-client) "Testing query get_product_gtin.")
  (with-safe-prolog
    (json-prolog:prolog-simple "get_product_gtin(ProductWithAN472572, gtin)."
                               :package :cashier)))

(defun testing-6 ()
  (roslisp:ros-info (json-prolog-client) "Testing query get_product_dan.")
  (cut:force-ll (with-safe-prolog
    (json-prolog:prolog-simple "get_product_dan(Product, Dan)."
                               :package :cashier))))

(defun testing-7 ()
  (roslisp:ros-info (json-prolog-client) "Testing query get_product_location.")
  (cut:force-ll (with-safe-prolog
    (json-prolog:prolog-simple "get_product_location(ProductType, Item, Shelf, ShelfLayer, Facing)."
                               :package :cashier))))





(defun get-product-from-productgtin (productnum)
  (roslisp:ros-info (json-prolog-client) "query for product-info")
  (let* ((raw-response
           (with-safe-prolog
             (json-prolog:prolog-simple (concatenate 'string "get_product_type("productnum",ProductType).")
                                        :package :cashier))))
    (if (eq raw-response 1)
          (roslisp:ros-warn (knowrob-client)
                            "Query get_product_type didn't reach any solution.")
            (cut:lazy-car raw-response))))
  
