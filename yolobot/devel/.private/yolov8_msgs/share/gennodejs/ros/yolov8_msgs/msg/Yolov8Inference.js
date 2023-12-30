// Auto-generated. Do not edit!

// (in-package yolov8_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let InferenceResult = require('./InferenceResult.js');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class Yolov8Inference {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.yolov8_inference = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('yolov8_inference')) {
        this.yolov8_inference = initObj.yolov8_inference
      }
      else {
        this.yolov8_inference = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Yolov8Inference
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [yolov8_inference]
    // Serialize the length for message field [yolov8_inference]
    bufferOffset = _serializer.uint32(obj.yolov8_inference.length, buffer, bufferOffset);
    obj.yolov8_inference.forEach((val) => {
      bufferOffset = InferenceResult.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Yolov8Inference
    let len;
    let data = new Yolov8Inference(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [yolov8_inference]
    // Deserialize array length for message field [yolov8_inference]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.yolov8_inference = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.yolov8_inference[i] = InferenceResult.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    object.yolov8_inference.forEach((val) => {
      length += InferenceResult.getMessageSize(val);
    });
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'yolov8_msgs/Yolov8Inference';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '081ce2824cbae866b6721f38bc29b475';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header #adds timestamp 
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
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Yolov8Inference(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.yolov8_inference !== undefined) {
      resolved.yolov8_inference = new Array(msg.yolov8_inference.length);
      for (let i = 0; i < resolved.yolov8_inference.length; ++i) {
        resolved.yolov8_inference[i] = InferenceResult.Resolve(msg.yolov8_inference[i]);
      }
    }
    else {
      resolved.yolov8_inference = []
    }

    return resolved;
    }
};

module.exports = Yolov8Inference;
