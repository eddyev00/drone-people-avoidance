
(cl:in-package :asdf)

(defsystem "yolov8_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "ConfidenceScore" :depends-on ("_package_ConfidenceScore"))
    (:file "_package_ConfidenceScore" :depends-on ("_package"))
    (:file "CustomMsg" :depends-on ("_package_CustomMsg"))
    (:file "_package_CustomMsg" :depends-on ("_package"))
    (:file "DepthResult" :depends-on ("_package_DepthResult"))
    (:file "_package_DepthResult" :depends-on ("_package"))
    (:file "InferenceResult" :depends-on ("_package_InferenceResult"))
    (:file "_package_InferenceResult" :depends-on ("_package"))
    (:file "PlotCoordinates" :depends-on ("_package_PlotCoordinates"))
    (:file "_package_PlotCoordinates" :depends-on ("_package"))
    (:file "Yolov8Inference" :depends-on ("_package_Yolov8Inference"))
    (:file "_package_Yolov8Inference" :depends-on ("_package"))
  ))