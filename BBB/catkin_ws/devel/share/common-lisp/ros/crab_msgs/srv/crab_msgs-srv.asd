
(cl:in-package :asdf)

(defsystem "crab_msgs-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :crab_msgs-msg
)
  :components ((:file "_package")
    (:file "GetLegIKSolver" :depends-on ("_package_GetLegIKSolver"))
    (:file "_package_GetLegIKSolver" :depends-on ("_package"))
  ))