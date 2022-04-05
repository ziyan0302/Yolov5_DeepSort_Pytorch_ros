
(cl:in-package :asdf)

(defsystem "detection_only-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "Bbox6Array" :depends-on ("_package_Bbox6Array"))
    (:file "_package_Bbox6Array" :depends-on ("_package"))
    (:file "Bbox_6" :depends-on ("_package_Bbox_6"))
    (:file "_package_Bbox_6" :depends-on ("_package"))
    (:file "Image" :depends-on ("_package_Image"))
    (:file "_package_Image" :depends-on ("_package"))
  ))