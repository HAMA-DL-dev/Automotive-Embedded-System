
(cl:in-package :asdf)

(defsystem "custom_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "waypoints" :depends-on ("_package_waypoints"))
    (:file "_package_waypoints" :depends-on ("_package"))
  ))