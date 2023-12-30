;; Auto-generated. Do not edit!


(when (boundp 'yolov8_msgs::DepthResult)
  (if (not (find-package "YOLOV8_MSGS"))
    (make-package "YOLOV8_MSGS"))
  (shadow 'DepthResult (find-package "YOLOV8_MSGS")))
(unless (find-package "YOLOV8_MSGS::DEPTHRESULT")
  (make-package "YOLOV8_MSGS::DEPTHRESULT"))

(in-package "ROS")
;;//! \htmlinclude DepthResult.msg.html


(defclass yolov8_msgs::DepthResult
  :super ros::object
  :slots (_depth_result ))

(defmethod yolov8_msgs::DepthResult
  (:init
   (&key
    ((:depth_result __depth_result) "")
    )
   (send-super :init)
   (setq _depth_result (string __depth_result))
   self)
  (:depth_result
   (&optional __depth_result)
   (if __depth_result (setq _depth_result __depth_result)) _depth_result)
  (:serialization-length
   ()
   (+
    ;; string _depth_result
    4 (length _depth_result)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _depth_result
       (write-long (length _depth_result) s) (princ _depth_result s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _depth_result
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _depth_result (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get yolov8_msgs::DepthResult :md5sum-) "68d8f88c522a11ab48e8159691a2eee1")
(setf (get yolov8_msgs::DepthResult :datatype-) "yolov8_msgs/DepthResult")
(setf (get yolov8_msgs::DepthResult :definition-)
      "string depth_result
")



(provide :yolov8_msgs/DepthResult "68d8f88c522a11ab48e8159691a2eee1")


