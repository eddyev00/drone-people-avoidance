;; Auto-generated. Do not edit!


(when (boundp 'yolov8_msgs::CustomMsg)
  (if (not (find-package "YOLOV8_MSGS"))
    (make-package "YOLOV8_MSGS"))
  (shadow 'CustomMsg (find-package "YOLOV8_MSGS")))
(unless (find-package "YOLOV8_MSGS::CUSTOMMSG")
  (make-package "YOLOV8_MSGS::CUSTOMMSG"))

(in-package "ROS")
;;//! \htmlinclude CustomMsg.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass yolov8_msgs::CustomMsg
  :super ros::object
  :slots (_header _yolo_output _optitrack_output ))

(defmethod yolov8_msgs::CustomMsg
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:yolo_output __yolo_output) 0.0)
    ((:optitrack_output __optitrack_output) (instance geometry_msgs::PoseStamped :init))
    )
   (send-super :init)
   (setq _header __header)
   (setq _yolo_output (float __yolo_output))
   (setq _optitrack_output __optitrack_output)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:yolo_output
   (&optional __yolo_output)
   (if __yolo_output (setq _yolo_output __yolo_output)) _yolo_output)
  (:optitrack_output
   (&rest __optitrack_output)
   (if (keywordp (car __optitrack_output))
       (send* _optitrack_output __optitrack_output)
     (progn
       (if __optitrack_output (setq _optitrack_output (car __optitrack_output)))
       _optitrack_output)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float64 _yolo_output
    8
    ;; geometry_msgs/PoseStamped _optitrack_output
    (send _optitrack_output :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float64 _yolo_output
       (sys::poke _yolo_output (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; geometry_msgs/PoseStamped _optitrack_output
       (send _optitrack_output :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float64 _yolo_output
     (setq _yolo_output (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; geometry_msgs/PoseStamped _optitrack_output
     (send _optitrack_output :deserialize buf ptr-) (incf ptr- (send _optitrack_output :serialization-length))
   ;;
   self)
  )

(setf (get yolov8_msgs::CustomMsg :md5sum-) "88d09e9aa5098510534f16d66c34f02b")
(setf (get yolov8_msgs::CustomMsg :datatype-) "yolov8_msgs/CustomMsg")
(setf (get yolov8_msgs::CustomMsg :definition-)
      "# CustomMsg.msg
std_msgs/Header header
float64 yolo_output
geometry_msgs/PoseStamped optitrack_output

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
MSG: geometry_msgs/PoseStamped
# A Pose with reference coordinate frame and timestamp
Header header
Pose pose

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

")



(provide :yolov8_msgs/CustomMsg "88d09e9aa5098510534f16d66c34f02b")


