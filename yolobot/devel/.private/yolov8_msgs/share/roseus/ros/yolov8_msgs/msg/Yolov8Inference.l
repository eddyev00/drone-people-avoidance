;; Auto-generated. Do not edit!


(when (boundp 'yolov8_msgs::Yolov8Inference)
  (if (not (find-package "YOLOV8_MSGS"))
    (make-package "YOLOV8_MSGS"))
  (shadow 'Yolov8Inference (find-package "YOLOV8_MSGS")))
(unless (find-package "YOLOV8_MSGS::YOLOV8INFERENCE")
  (make-package "YOLOV8_MSGS::YOLOV8INFERENCE"))

(in-package "ROS")
;;//! \htmlinclude Yolov8Inference.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass yolov8_msgs::Yolov8Inference
  :super ros::object
  :slots (_header _yolov8_inference ))

(defmethod yolov8_msgs::Yolov8Inference
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:yolov8_inference __yolov8_inference) ())
    )
   (send-super :init)
   (setq _header __header)
   (setq _yolov8_inference __yolov8_inference)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:yolov8_inference
   (&rest __yolov8_inference)
   (if (keywordp (car __yolov8_inference))
       (send* _yolov8_inference __yolov8_inference)
     (progn
       (if __yolov8_inference (setq _yolov8_inference (car __yolov8_inference)))
       _yolov8_inference)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; yolov8_msgs/InferenceResult[] _yolov8_inference
    (apply #'+ (send-all _yolov8_inference :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; yolov8_msgs/InferenceResult[] _yolov8_inference
     (write-long (length _yolov8_inference) s)
     (dolist (elem _yolov8_inference)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; yolov8_msgs/InferenceResult[] _yolov8_inference
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _yolov8_inference (let (r) (dotimes (i n) (push (instance yolov8_msgs::InferenceResult :init) r)) r))
     (dolist (elem- _yolov8_inference)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get yolov8_msgs::Yolov8Inference :md5sum-) "081ce2824cbae866b6721f38bc29b475")
(setf (get yolov8_msgs::Yolov8Inference :datatype-) "yolov8_msgs/Yolov8Inference")
(setf (get yolov8_msgs::Yolov8Inference :definition-)
      "std_msgs/Header header #adds timestamp 
InferenceResult[] yolov8_inference #each detecte object will be appended as an inference result message.

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: yolov8_msgs/InferenceResult
string class_name
int64 top
int64 left
int64 bottom
int64 right

")



(provide :yolov8_msgs/Yolov8Inference "081ce2824cbae866b6721f38bc29b475")


