// Auto-generated. Do not edit!

// (in-package yolov8_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class InferenceResult {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.class_name = null;
      this.top = null;
      this.left = null;
      this.bottom = null;
      this.right = null;
    }
    else {
      if (initObj.hasOwnProperty('class_name')) {
        this.class_name = initObj.class_name
      }
      else {
        this.class_name = '';
      }
      if (initObj.hasOwnProperty('top')) {
        this.top = initObj.top
      }
      else {
        this.top = 0;
      }
      if (initObj.hasOwnProperty('left')) {
        this.left = initObj.left
      }
      else {
        this.left = 0;
      }
      if (initObj.hasOwnProperty('bottom')) {
        this.bottom = initObj.bottom
      }
      else {
        this.bottom = 0;
      }
      if (initObj.hasOwnProperty('right')) {
        this.right = initObj.right
      }
      else {
        this.right = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type InferenceResult
    // Serialize message field [class_name]
    bufferOffset = _serializer.string(obj.class_name, buffer, bufferOffset);
    // Serialize message field [top]
    bufferOffset = _serializer.int64(obj.top, buffer, bufferOffset);
    // Serialize message field [left]
    bufferOffset = _serializer.int64(obj.left, buffer, bufferOffset);
    // Serialize message field [bottom]
    bufferOffset = _serializer.int64(obj.bottom, buffer, bufferOffset);
    // Serialize message field [right]
    bufferOffset = _serializer.int64(obj.right, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type InferenceResult
    let len;
    let data = new InferenceResult(null);
    // Deserialize message field [class_name]
    data.class_name = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [top]
    data.top = _deserializer.int64(buffer, bufferOffset);
    // Deserialize message field [left]
    data.left = _deserializer.int64(buffer, bufferOffset);
    // Deserialize message field [bottom]
    data.bottom = _deserializer.int64(buffer, bufferOffset);
    // Deserialize message field [right]
    data.right = _deserializer.int64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.class_name);
    return length + 36;
  }

  static datatype() {
    // Returns string type for a message object
    return 'yolov8_msgs/InferenceResult';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '953d486c7f880d6374bef7ecbdfb1210';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
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
    const resolved = new InferenceResult(null);
    if (msg.class_name !== undefined) {
      resolved.class_name = msg.class_name;
    }
    else {
      resolved.class_name = ''
    }

    if (msg.top !== undefined) {
      resolved.top = msg.top;
    }
    else {
      resolved.top = 0
    }

    if (msg.left !== undefined) {
      resolved.left = msg.left;
    }
    else {
      resolved.left = 0
    }

    if (msg.bottom !== undefined) {
      resolved.bottom = msg.bottom;
    }
    else {
      resolved.bottom = 0
    }

    if (msg.right !== undefined) {
      resolved.right = msg.right;
    }
    else {
      resolved.right = 0
    }

    return resolved;
    }
};

module.exports = InferenceResult;
