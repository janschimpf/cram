(defsystem pr2-cashier-test
  :depends-on (pr2-cashier
               lisp-unit)
  :components ((:module "tests"
                :components
                ((:file "package")
                 (:file "projection-demo-tests" :depends-on ("package"))))))
