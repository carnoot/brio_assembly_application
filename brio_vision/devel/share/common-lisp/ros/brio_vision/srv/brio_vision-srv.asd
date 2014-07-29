
(cl:in-package :asdf)

(defsystem "brio_vision-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :brio_vision-msg
)
  :components ((:file "_package")
    (:file "Is_This_Easier" :depends-on ("_package_Is_This_Easier"))
    (:file "_package_Is_This_Easier" :depends-on ("_package"))
  ))