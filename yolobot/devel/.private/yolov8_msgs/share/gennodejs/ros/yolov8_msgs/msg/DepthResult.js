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

class DepthResult {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.depth_result = null;
    }
    else {
      if (initObj.hasOwnProperty('depth_result')) {
        this.depth_result = initObj.depth_result
      }
      else {
        this.depth_result = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type DepthResult
    // Serialize message field [depth_result]
    bufferOffset = _serializer.string(obj.depth_result, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type DepthResult
    let len;
    let data = new DepthResult(null);
    // Deserialize message field [depth_result]
    data.depth_result = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.depth_result);
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'yolov8_msgs/DepthResult';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '68d8f88c522a11ab48e8159691a2eee1';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string depth_result
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new DepthResult(null);
    if (msg.depth_result !== undefined) {
      resolved.depth_result = msg.depth_result;
    }
    else {
      resolved.depth_result = ''
    }

    return resolved;
    }
};

module.exports = DepthResult;
