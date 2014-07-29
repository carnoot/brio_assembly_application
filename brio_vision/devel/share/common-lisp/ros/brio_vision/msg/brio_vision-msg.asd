
(cl:in-package :asdf)

(defsystem "brio_vision-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Data_Type" :depends-on ("_package_Data_Type"))
    (:file "_package_Data_Type" :depends-on ("_package"))
    (:file "Container" :depends-on ("_package_Container"))
    (:file "_package_Container" :depends-on ("_package"))
  ))