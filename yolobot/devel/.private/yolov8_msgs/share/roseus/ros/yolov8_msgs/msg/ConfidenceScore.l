;; Auto-generated. Do not edit!


(when (boundp 'yolov8_msgs::ConfidenceScore)
  (if (not (find-package "YOLOV8_MSGS"))
    (make-package "YOLOV8_MSGS"))
  (shadow 'ConfidenceScore (find-package "YOLOV8_MSGS")))
(unless (find-package "YOLOV8_MSGS::CONFIDENCESCORE")
  (make-package "YOLOV8_MSGS::CONFIDENCESCORE"))

(in-package "ROS")
;;//! \htmlinclude ConfidenceScore.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass yolov8_msgs::ConfidenceScore
  :super ros::object
  :slots (_header _conf ))

(defmethod yolov8_msgs::ConfidenceScore
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:conf __conf) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _conf (float __conf))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:conf
   (&optional __conf)
   (if __conf (setq _conf __conf)) _conf)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float64 _conf
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float64 _conf
       (sys::poke _conf (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float64 _conf
     (setq _conf (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get yolov8_msgs::ConfidenceScore :md5sum-) "86ad5b8fcf7de5c78bec638d1ee58116")
(setf (get yolov8_msgs::ConfidenceScore :datatype-) "yolov8_msgs/ConfidenceScore")
(setf (get yolov8_msgs::ConfidenceScore :definition-)
      "std_msgs/Header header #adds timestamp 
float64 conf
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

")



(provide :yolov8_msgs/ConfidenceScore "86ad5b8fcf7de5c78bec638d1ee58116")


