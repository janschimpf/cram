;;;
;;; Copyright (c) 2017, Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
;;; All rights reserved.
;;;
;;; Redistribution and use in source and binary forms, with or without
;;; modification, are permitted provided that the following conditions are met:
;;;
;;;     * Redistributions of source code must retain the above copyright
;;;       notice, this list of conditions and the following disclaimer.
;;;     * Redistributions in binary form must reproduce the above copyright
;;;       notice, this list of conditions and the following disclaimer in the
;;;       documentation and/or other materials provided with the distribution.
;;;     * Neither the name of the Intelligent Autonomous Systems Group/
;;;       Technische Universitaet Muenchen nor the names of its contributors 
;;;       may be used to endorse or promote products derived from this software 
;;;       without specific prior written permission.
;;;
;;; THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
;;; AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
;;; IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
;;; ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
;;; LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
;;; CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
;;; SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
;;; INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
;;; CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
;;; ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
;;; POSSIBILITY OF SUCH DAMAGE.

(defsystem pr2-cashier
  :author "Jan Schimpf"
  :license "BSD"

  :depends-on (roslisp-utilities 

               cl-transforms
               cl-transforms-stamped
               cl-tf
               cl-tf2
               cram-tf

               cram-language
               cram-executive
               cram-designators
               cram-prolog
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
               cram-hsrb-description)

  :components
  ((:module "src"
    :components
    ((:file "package")
     (:file "setup" :depends-on ("package"))
     (:file "object-knowledge" :depends-on ("package" ))
     (:file "spawn-objects" :depends-on ("package"))
     (:file "scan-objects" :depends-on ("package"))
     (:file "cashier-designator" :depends-on ("package" "scan-objects"))
     (:file "cashier-designator-plan" :depends-on ("package" "scan-objects" "cashier-designator"))
     (:file "cashier-demo" :depends-on ("package" "object-knowledge" "spawn-objects" "scan-objects" "cashier-designator" "cashier-designator-plan"))
     ))))
