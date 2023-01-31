(in-package :cashier)


(defun init-projection ()
  (coe:clear-belief)
  (btr-belief:setup-world-database)

  (setf cram-tf:*tf-default-timeout* 2.0)
  (setf cram-tf:*tf-broadcasting-enabled* t)

  (btr:clear-costmap-vis-object)



  (setf proj-reasoning::*projection-reasoning-enabled* nil)


 (btr:add-objects-to-mesh-list "pr2_cashier"))

(roslisp-utilities:register-ros-init-function init-projection)
