// Auto-generated. Do not edit!

// (in-package detection_only.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let Image = require('./Image.js');
let Bbox_6 = require('./Bbox_6.js');

//-----------------------------------------------------------

class Bbox6Array {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.image = null;
      this.bboxes = null;
    }
    else {
      if (initObj.hasOwnProperty('image')) {
        this.image = initObj.image
      }
      else {
        this.image = new Image();
      }
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
    // Serialize message field [image]
    bufferOffset = Image.serialize(obj.image, buffer, bufferOffset);
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
    // Deserialize message field [image]
    data.image = Image.deserialize(buffer, bufferOffset);
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
    length += Image.getMessageSize(object.image);
    length += 24 * object.bboxes.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'detection_only/Bbox6Array';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd023923a68c57000f2065e0f664c2a6d';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # bboxes 
    # [x1,y1,x2,y2,conf,class]
    Image image
    Bbox_6[] bboxes 
    ================================================================================
    MSG: detection_only/Image
    # This message contains an uncompressed image
    # (0, 0) is at top-left corner of image
    #
    
    Header header        # Header timestamp should be acquisition time of image
                         # Header frame_id should be optical frame of camera
                         # origin of frame should be optical center of camera
                         # +x should point to the right in the image
                         # +y should point down in the image
                         # +z should point into to plane of the image
                         # If the frame_id here and the frame_id of the CameraInfo
                         # message associated with the image conflict
                         # the behavior is undefined
    
    uint32 height         # image height, that is, number of rows
    uint32 width          # image width, that is, number of columns
    
    # The legal values for encoding are in file src/image_encodings.cpp
    # If you want to standardize a new string format, join
    # ros-users@lists.sourceforge.net and send an email proposing a new encoding.
    
    string encoding       # Encoding of pixels -- channel meaning, ordering, size
                          # taken from the list of strings in include/sensor_msgs/image_encodings.h
    
    uint8 is_bigendian    # is this data bigendian?
    uint32 step           # Full row length in bytes
    uint8[] data          # actual matrix data, size is (step * rows)
    
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
    if (msg.image !== undefined) {
      resolved.image = Image.Resolve(msg.image)
    }
    else {
      resolved.image = new Image()
    }

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
