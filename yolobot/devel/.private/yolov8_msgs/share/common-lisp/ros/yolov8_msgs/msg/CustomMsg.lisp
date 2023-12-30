; Auto-generated. Do not edit!


(cl:in-package yolov8_msgs-msg)


;//! \htmlinclude CustomMsg.msg.html

(cl:defclass <CustomMsg> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (yolo_output
    :reader yolo_output
    :initarg :yolo_output
    :type cl:float
    :initform 0.0)
   (optitrack_output
    :reader optitrack_output
    :initarg :optitrack_output
    :type geometry_msgs-msg:PoseStamped
    :initform (cl:make-instance 'geometry_msgs-msg:PoseStamped)))
)

(cl:defclass CustomMsg (<CustomMsg>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CustomMsg>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CustomMsg)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name yolov8_msgs-msg:<CustomMsg> is deprecated: use yolov8_msgs-msg:CustomMsg instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <CustomMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader yolov8_msgs-msg:header-val is deprecated.  Use yolov8_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'yolo_output-val :lambda-list '(m))
(cl:defmethod yolo_output-val ((m <CustomMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader yolov8_msgs-msg:yolo_output-val is deprecated.  Use yolov8_msgs-msg:yolo_output instead.")
  (yolo_output m))

(cl:ensure-generic-function 'optitrack_output-val :lambda-list '(m))
(cl:defmethod optitrack_output-val ((m <CustomMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader yolov8_msgs-msg:optitrack_output-val is deprecated.  Use yolov8_msgs-msg:optitrack_output instead.")
  (optitrack_output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CustomMsg>) ostream)
  "Serializes a message object of type '<CustomMsg>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'yolo_output))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'optitrack_output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CustomMsg>) istream)
  "Deserializes a message object of type '<CustomMsg>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'yolo_output) (roslisp-utils:decode-double-float-bits bits)))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'optitrack_output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CustomMsg>)))
  "Returns string type for a message object of type '<CustomMsg>"
  "yolov8_msgs/CustomMsg")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CustomMsg)))
  "Returns string type for a message object of type 'CustomMsg"
  "yolov8_msgs/CustomMsg")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CustomMsg>)))
  "Returns md5sum for a message object of type '<CustomMsg>"
  "88d09e9aa5098510534f16d66c34f02b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CustomMsg)))
  "Returns md5sum for a message object of type 'CustomMsg"
  "88d09e9aa5098510534f16d66c34f02b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CustomMsg>)))
  "Returns full string definition for message of type '<CustomMsg>"
  (cl:format cl:nil "# CustomMsg.msg~%std_msgs/Header header~%float64 yolo_output~%geometry_msgs/PoseStamped optitrack_output~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CustomMsg)))
  "Returns full string definition for message of type 'CustomMsg"
  (cl:format cl:nil "# CustomMsg.msg~%std_msgs/Header header~%float64 yolo_output~%geometry_msgs/PoseStamped optitrack_output~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CustomMsg>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     8
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'optitrack_output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CustomMsg>))
  "Converts a ROS message object to a list"
  (cl:list 'CustomMsg
    (cl:cons ':header (header msg))
    (cl:cons ':yolo_output (yolo_output msg))
    (cl:cons ':optitrack_output (optitrack_output msg))
))
