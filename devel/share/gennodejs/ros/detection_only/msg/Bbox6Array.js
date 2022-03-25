// Auto-generated. Do not edit!

// (in-package detection_only.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let Bbox_6 = require('./Bbox_6.js');

//-----------------------------------------------------------

class Bbox6Array {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.bboxes = null;
    }
    else {
      if (initObj.hasOwnProperty('bboxes')) {
        this.bboxes = initObj.bboxes
      }
      else {
        this.bboxes = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Bbox6Array
    // Serialize message field [bboxes]
    // Serialize the length for message field [bboxes]
    bufferOffset = _serializer.uint32(obj.bboxes.length, buffer, bufferOffset);
    obj.bboxes.forEach((val) => {
      bufferOffset = Bbox_6.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Bbox6Array
    let len;
    let data = new Bbox6Array(null);
    // Deserialize message field [bboxes]
    // Deserialize array length for message field [bboxes]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.bboxes = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.bboxes[i] = Bbox_6.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 24 * object.bboxes.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'detection_only/Bbox6Array';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '61b8f2a52aff12e25f2957c1314c78d3';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # bboxes 
    # [x1,y1,x2,y2,conf,class]
    Bbox_6[] bboxes 
    ================================================================================
    MSG: detection_only/Bbox_6
    # 1 bbox 
    # [x1,y1,x2,y2,conf,class]
    # float32[] bbox_info
    float32 x1
    float32 y1
    float32 x2
    float32 y2
    float32 conf
    float32 cls
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Bbox6Array(null);
    if (msg.bboxes !== undefined) {
      resolved.bboxes = new Array(msg.bboxes.length);
      for (let i = 0; i < resolved.bboxes.length; ++i) {
        resolved.bboxes[i] = Bbox_6.Resolve(msg.bboxes[i]);
      }
    }
    else {
      resolved.bboxes = []
    }

    return resolved;
    }
};

module.exports = Bbox6Array;
