// Auto-generated. Do not edit!

// (in-package detection_only.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let Track_6 = require('./Track_6.js');

//-----------------------------------------------------------

class Track6Array {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.tracks = null;
    }
    else {
      if (initObj.hasOwnProperty('tracks')) {
        this.tracks = initObj.tracks
      }
      else {
        this.tracks = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Track6Array
    // Serialize message field [tracks]
    // Serialize the length for message field [tracks]
    bufferOffset = _serializer.uint32(obj.tracks.length, buffer, bufferOffset);
    obj.tracks.forEach((val) => {
      bufferOffset = Track_6.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Track6Array
    let len;
    let data = new Track6Array(null);
    // Deserialize message field [tracks]
    // Deserialize array length for message field [tracks]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.tracks = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.tracks[i] = Track_6.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 24 * object.tracks.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'detection_only/Track6Array';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'bcae8edf8c2ae4b9b6e115eb41e70bec';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # bboxes 
    # [x1,y1,x2,y2,idx,class]
    Track_6[] tracks 
    ================================================================================
    MSG: detection_only/Track_6
    # 1 bbox 
    # [x1,y1,x2,y2,conf,class]
    # float32[] bbox_info
    float32 x1
    float32 y1
    float32 x2
    float32 y2
    float32 id
    float32 cls
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Track6Array(null);
    if (msg.tracks !== undefined) {
      resolved.tracks = new Array(msg.tracks.length);
      for (let i = 0; i < resolved.tracks.length; ++i) {
        resolved.tracks[i] = Track_6.Resolve(msg.tracks[i]);
      }
    }
    else {
      resolved.tracks = []
    }

    return resolved;
    }
};

module.exports = Track6Array;
