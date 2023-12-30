; Auto-generated. Do not edit!


(cl:in-package yolov8_msgs-msg)


;//! \htmlinclude DepthResult.msg.html

(cl:defclass <DepthResult> (roslisp-msg-protocol:ros-message)
  ((depth_result
    :reader depth_result
    :initarg :depth_result
    :type cl:string
    :initform ""))
)

(cl:defclass DepthResult (<DepthResult>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DepthResult>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DepthResult)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name yolov8_msgs-msg:<DepthResult> is deprecated: use yolov8_msgs-msg:DepthResult instead.")))

(cl:ensure-generic-function 'depth_result-val :lambda-list '(m))
(cl:defmethod depth_result-val ((m <DepthResult>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader yolov8_msgs-msg:depth_result-val is deprecated.  Use yolov8_msgs-msg:depth_result instead.")
  (depth_result m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DepthResult>) ostream)
  "Serializes a message object of type '<DepthResult>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'depth_result))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'depth_result))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DepthResult>) istream)
  "Deserializes a message object of type '<DepthResult>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'depth_result) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'depth_result) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DepthResult>)))
  "Returns string type for a message object of type '<DepthResult>"
  "yolov8_msgs/DepthResult")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DepthResult)))
  "Returns string type for a message object of type 'DepthResult"
  "yolov8_msgs/DepthResult")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DepthResult>)))
  "Returns md5sum for a message object of type '<DepthResult>"
  "68d8f88c522a11ab48e8159691a2eee1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DepthResult)))
  "Returns md5sum for a message object of type 'DepthResult"
  "68d8f88c522a11ab48e8159691a2eee1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DepthResult>)))
  "Returns full string definition for message of type '<DepthResult>"
  (cl:format cl:nil "string depth_result~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DepthResult)))
  "Returns full string definition for message of type 'DepthResult"
  (cl:format cl:nil "string depth_result~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DepthResult>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'depth_result))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DepthResult>))
  "Converts a ROS message object to a list"
  (cl:list 'DepthResult
    (cl:cons ':depth_result (depth_result msg))
))
