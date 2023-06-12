(defsystem pr2-cashier
  :author "Jan Schimpf"
  :license "BSD"

  :depends-on (roslisp
  		roslisp-utilities 

               cl-transforms
               cl-transforms-stamped
               cl-tf
               cl-tf2
               cram-tf

               cram-language
               cram-executive
               cram-designators
               cram-prolog
               cram-json-prolog
               cram-projection
               cram-occasions-events
               cram-utilities 

               cram-common-failures
               cram-mobile-pick-place-plans
               cram-object-knowledge

               cram-cloud-logger

               cram-physics-utils     
               cl-bullet 
               cram-bullet-reasoning
               cram-bullet-reasoning-belief-state
               cram-bullet-reasoning-utilities

               cram-location-costmap
               cram-btr-visibility-costmap
               cram-btr-spatial-relations-costmap
               cram-robot-pose-gaussian-costmap
               cram-occupancy-grid-costmap

               cram-urdf-projection     
               cram-urdf-projection-reasoning 
               cram-fetch-deliver-plans
               cram-urdf-environment-manipulation

               cram-pr2-description
               cram-boxy-description
               cram-donbot-description
               cram-hsrb-description
               
               visualization_msgs-msg
               )

  :components
  ((:module "src"
    :components
    ((:file "package")
     (:file "setup" :depends-on ("package"))
     (:file "object-knowledge" :depends-on ("package" ))
     (:file "knowrob-client" :depends-on ("package" ))
     (:file "spawn-objects" :depends-on ("package"))
     (:file "scan-objects" :depends-on ("package"))
     (:file "cashier-designator" :depends-on ("package" "scan-objects"))
     (:file "change-side-designator-plan" :depends-on ("package" "scan-objects" "cashier-designator"))
     (:file "scan-designator-plan" :depends-on ("package" "scan-objects" "cashier-designator" "change-side-designator-plan"))
     (:file "cashier-designator-plan" :depends-on ("package" "scan-objects" "cashier-designator" "scan-designator-plan"))
     (:file "cashier-demo" :depends-on ("package" "object-knowledge" "spawn-objects" "scan-objects" "cashier-designator" "cashier-designator-plan"))
     ))))
