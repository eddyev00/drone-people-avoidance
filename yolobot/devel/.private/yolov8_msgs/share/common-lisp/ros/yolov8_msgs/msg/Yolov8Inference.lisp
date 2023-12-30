; Auto-generated. Do not edit!


(cl:in-package yolov8_msgs-msg)


;//! \htmlinclude Yolov8Inference.msg.html

(cl:defclass <Yolov8Inference> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (yolov8_inference
    :reader yolov8_inference
    :initarg :yolov8_inference
    :type (cl:vector yolov8_msgs-msg:InferenceResult)
   :initform (cl:make-array 0 :element-type 'yolov8_msgs-msg:InferenceResult :initial-element (cl:make-instance 'yolov8_msgs-msg:InferenceResult))))
)

(cl:defclass Yolov8Inference (<Yolov8Inference>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Yolov8Inference>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Yolov8Inference)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name yolov8_msgs-msg:<Yolov8Inference> is deprecated: use yolov8_msgs-msg:Yolov8Inference instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <Yolov8Inference>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader yolov8_msgs-msg:header-val is deprecated.  Use yolov8_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'yolov8_inference-val :lambda-list '(m))
(cl:defmethod yolov8_inference-val ((m <Yolov8Inference>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader yolov8_msgs-msg:yolov8_inference-val is deprecated.  Use yolov8_msgs-msg:yolov8_inference instead.")
  (yolov8_inference m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Yolov8Inference>) ostream)
  "Serializes a message object of type '<Yolov8Inference>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'yolov8_inference))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'yolov8_inference))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Yolov8Inference>) istream)
  "Deserializes a message object of type '<Yolov8Inference>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'yolov8_inference) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'yolov8_inference)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'yolov8_msgs-msg:InferenceResult))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Yolov8Inference>)))
  "Returns string type for a message object of type '<Yolov8Inference>"
  "yolov8_msgs/Yolov8Inference")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Yolov8Inference)))
  "Returns string type for a message object of type 'Yolov8Inference"
  "yolov8_msgs/Yolov8Inference")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Yolov8Inference>)))
  "Returns md5sum for a message object of type '<Yolov8Inference>"
  "081ce2824cbae866b6721f38bc29b475")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Yolov8Inference)))
  "Returns md5sum for a message object of type 'Yolov8Inference"
  "081ce2824cbae866b6721f38bc29b475")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Yolov8Inference>)))
  "Returns full string definition for message of type '<Yolov8Inference>"
  (cl:format cl:nil "std_msgs/Header header #adds timestamp ~%InferenceResult[] yolov8_inference #each detecte object will be appended as an inference result message.~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: yolov8_msgs/InferenceResult~%string class_name~%int64 top~%int64 left~%int64 bottom~%int64 right~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Yolov8Inference)))
  "Returns full string definition for message of type 'Yolov8Inference"
  (cl:format cl:nil "std_msgs/Header header #adds timestamp ~%InferenceResult[] yolov8_inference #each detecte object will be appended as an inference result message.~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: yolov8_msgs/InferenceResult~%string class_name~%int64 top~%int64 left~%int64 bottom~%int64 right~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Yolov8Inference>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'yolov8_inference) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Yolov8Inference>))
  "Converts a ROS message object to a list"
  (cl:list 'Yolov8Inference
    (cl:cons ':header (header msg))
    (cl:cons ':yolov8_inference (yolov8_inference msg))
))
