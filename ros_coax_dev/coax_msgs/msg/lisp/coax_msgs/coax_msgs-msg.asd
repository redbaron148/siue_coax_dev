
(in-package :asdf)

(defsystem "coax_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :roslib-msg
)
  :components ((:file "_package")
    (:file "CoaxRawControl" :depends-on ("_package"))
    (:file "_package_CoaxRawControl" :depends-on ("_package"))
    (:file "CoaxConstants" :depends-on ("_package"))
    (:file "_package_CoaxConstants" :depends-on ("_package"))
    (:file "CoaxVersion" :depends-on ("_package"))
    (:file "_package_CoaxVersion" :depends-on ("_package"))
    (:file "CoaxModes" :depends-on ("_package"))
    (:file "_package_CoaxModes" :depends-on ("_package"))
    (:file "CoaxControlParameters" :depends-on ("_package"))
    (:file "_package_CoaxControlParameters" :depends-on ("_package"))
    (:file "Coax3DPose" :depends-on ("_package"))
    (:file "_package_Coax3DPose" :depends-on ("_package"))
    (:file "CoaxTrimMode" :depends-on ("_package"))
    (:file "_package_CoaxTrimMode" :depends-on ("_package"))
    (:file "CoaxKeepAlive" :depends-on ("_package"))
    (:file "_package_CoaxKeepAlive" :depends-on ("_package"))
    (:file "CoaxControl" :depends-on ("_package"))
    (:file "_package_CoaxControl" :depends-on ("_package"))
    (:file "CoaxContent" :depends-on ("_package"))
    (:file "_package_CoaxContent" :depends-on ("_package"))
    (:file "CoaxState" :depends-on ("_package"))
    (:file "_package_CoaxState" :depends-on ("_package"))
    ))