// Auto-generated. Do not edit!

// (in-package detection_only.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class bbox {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.bbox_info = null;
    }
    else {
      if (initObj.hasOwnProperty('bbox_info')) {
        this.bbox_info = initObj.bbox_info
      }
      else {
        this.bbox_info = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type bbox
    // Serialize message field [bbox_info]
    bufferOffset = _arraySerializer.float32(obj.bbox_info, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type bbox
    let len;
    let data = new bbox(null);
    // Deserialize message field [bbox_info]
    data.bbox_info = _arrayDeserializer.float32(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 4 * object.bbox_info.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'detection_only/bbox';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '913b440d53de141b21b184494ddea913';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # 1 bbox 
    # [x1,y1,x2,y2,conf,class]
    float32[] bbox_info 
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new bbox(null);
    if (msg.bbox_info !== undefined) {
      resolved.bbox_info = msg.bbox_info;
    }
    else {
      resolved.bbox_info = []
    }

    return resolved;
    }
};

module.exports = bbox;
