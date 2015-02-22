; -*- Mode: Lisp; Syntax: ANSI-Common-Lisp; Base: 10 -*-

(asdf:defsystem nmmi-executive
  :name "nmmi-executive"
  :author "Georg Bartels <georg.bartels@cs.uni-bremen.de>"
  :maintainer "Georg Bartels <georg.bartels@cs.uni-bremen.de>"
  :licence "BSD"
  :description "CRAM executive for the NMMI challenge."
  :depends-on (:roslisp
               :cl-transforms
               :cl-tf2
               :cram-language
               :geometry_msgs-msg
               :iai_qb_cube_msgs-msg)
  :components
  ((:module "src"
            :components
            ((:file "package")
             (:file "arm-control" :depends-on ("package"))
             (:file "main" :depends-on ("package" "arm-control"))))))
