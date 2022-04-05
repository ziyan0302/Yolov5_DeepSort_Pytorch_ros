; Auto-generated. Do not edit!


(cl:in-package detection_only-msg)


;//! \htmlinclude Bbox6Array.msg.html

(cl:defclass <Bbox6Array> (roslisp-msg-protocol:ros-message)
  ((image
    :reader image
    :initarg :image
    :type detection_only-msg:Image
    :initform (cl:make-instance 'detection_only-msg:Image))
   (bboxes
    :reader bboxes
    :initarg :bboxes
    :type (cl:vector detection_only-msg:Bbox_6)
   :initform (cl:make-array 0 :element-type 'detection_only-msg:Bbox_6 :initial-element (cl:make-instance 'detection_only-msg:Bbox_6))))
)

(cl:defclass Bbox6Array (<Bbox6Array>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Bbox6Array>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Bbox6Array)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name detection_only-msg:<Bbox6Array> is deprecated: use detection_only-msg:Bbox6Array instead.")))

(cl:ensure-generic-function 'image-val :lambda-list '(m))
(cl:defmethod image-val ((m <Bbox6Array>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader detection_only-msg:image-val is deprecated.  Use detection_only-msg:image instead.")
  (image m))

(cl:ensure-generic-function 'bboxes-val :lambda-list '(m))
(cl:defmethod bboxes-val ((m <Bbox6Array>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader detection_only-msg:bboxes-val is deprecated.  Use detection_only-msg:bboxes instead.")
  (bboxes m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Bbox6Array>) ostream)
  "Serializes a message object of type '<Bbox6Array>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'image) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'bboxes))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'bboxes))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Bbox6Array>) istream)
  "Deserializes a message object of type '<Bbox6Array>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'image) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'bboxes) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'bboxes)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'detection_only-msg:Bbox_6))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Bbox6Array>)))
  "Returns string type for a message object of type '<Bbox6Array>"
  "detection_only/Bbox6Array")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Bbox6Array)))
  "Returns string type for a message object of type 'Bbox6Array"
  "detection_only/Bbox6Array")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Bbox6Array>)))
  "Returns md5sum for a message object of type '<Bbox6Array>"
  "d023923a68c57000f2065e0f664c2a6d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Bbox6Array)))
  "Returns md5sum for a message object of type 'Bbox6Array"
  "d023923a68c57000f2065e0f664c2a6d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Bbox6Array>)))
  "Returns full string definition for message of type '<Bbox6Array>"
  (cl:format cl:nil "# bboxes ~%# [x1,y1,x2,y2,conf,class]~%Image image~%Bbox_6[] bboxes ~%================================================================================~%MSG: detection_only/Image~%# This message contains an uncompressed image~%# (0, 0) is at top-left corner of image~%#~%~%Header header        # Header timestamp should be acquisition time of image~%                     # Header frame_id should be optical frame of camera~%                     # origin of frame should be optical center of camera~%                     # +x should point to the right in the image~%                     # +y should point down in the image~%                     # +z should point into to plane of the image~%                     # If the frame_id here and the frame_id of the CameraInfo~%                     # message associated with the image conflict~%                     # the behavior is undefined~%~%uint32 height         # image height, that is, number of rows~%uint32 width          # image width, that is, number of columns~%~%# The legal values for encoding are in file src/image_encodings.cpp~%# If you want to standardize a new string format, join~%# ros-users@lists.sourceforge.net and send an email proposing a new encoding.~%~%string encoding       # Encoding of pixels -- channel meaning, ordering, size~%                      # taken from the list of strings in include/sensor_msgs/image_encodings.h~%~%uint8 is_bigendian    # is this data bigendian?~%uint32 step           # Full row length in bytes~%uint8[] data          # actual matrix data, size is (step * rows)~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: detection_only/Bbox_6~%# 1 bbox ~%# [x1,y1,x2,y2,conf,class]~%# float32[] bbox_info~%float32 x1~%float32 y1~%float32 x2~%float32 y2~%float32 conf~%float32 cls~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Bbox6Array)))
  "Returns full string definition for message of type 'Bbox6Array"
  (cl:format cl:nil "# bboxes ~%# [x1,y1,x2,y2,conf,class]~%Image image~%Bbox_6[] bboxes ~%================================================================================~%MSG: detection_only/Image~%# This message contains an uncompressed image~%# (0, 0) is at top-left corner of image~%#~%~%Header header        # Header timestamp should be acquisition time of image~%                     # Header frame_id should be optical frame of camera~%                     # origin of frame should be optical center of camera~%                     # +x should point to the right in the image~%                     # +y should point down in the image~%                     # +z should point into to plane of the image~%                     # If the frame_id here and the frame_id of the CameraInfo~%                     # message associated with the image conflict~%                     # the behavior is undefined~%~%uint32 height         # image height, that is, number of rows~%uint32 width          # image width, that is, number of columns~%~%# The legal values for encoding are in file src/image_encodings.cpp~%# If you want to standardize a new string format, join~%# ros-users@lists.sourceforge.net and send an email proposing a new encoding.~%~%string encoding       # Encoding of pixels -- channel meaning, ordering, size~%                      # taken from the list of strings in include/sensor_msgs/image_encodings.h~%~%uint8 is_bigendian    # is this data bigendian?~%uint32 step           # Full row length in bytes~%uint8[] data          # actual matrix data, size is (step * rows)~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: detection_only/Bbox_6~%# 1 bbox ~%# [x1,y1,x2,y2,conf,class]~%# float32[] bbox_info~%float32 x1~%float32 y1~%float32 x2~%float32 y2~%float32 conf~%float32 cls~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Bbox6Array>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'image))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'bboxes) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Bbox6Array>))
  "Converts a ROS message object to a list"
  (cl:list 'Bbox6Array
    (cl:cons ':image (image msg))
    (cl:cons ':bboxes (bboxes msg))
))
