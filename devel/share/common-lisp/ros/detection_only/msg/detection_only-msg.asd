
(cl:in-package :asdf)

(defsystem "detection_only-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Bbox6Array" :depends-on ("_package_Bbox6Array"))
    (:file "_package_Bbox6Array" :depends-on ("_package"))
    (:file "Bbox_6" :depends-on ("_package_Bbox_6"))
    (:file "_package_Bbox_6" :depends-on ("_package"))
  ))