
(in-package :asdf)

(defsystem "coax_client-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :roslib-msg
)
  :components ((:file "_package")
    (:file "CoaxStateFiltered" :depends-on ("_package"))
    (:file "_package_CoaxStateFiltered" :depends-on ("_package"))
    ))
