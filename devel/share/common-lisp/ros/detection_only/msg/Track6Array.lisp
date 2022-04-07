; Auto-generated. Do not edit!


(cl:in-package detection_only-msg)


;//! \htmlinclude Track6Array.msg.html

(cl:defclass <Track6Array> (roslisp-msg-protocol:ros-message)
  ((tracks
    :reader tracks
    :initarg :tracks
    :type (cl:vector detection_only-msg:Track_6)
   :initform (cl:make-array 0 :element-type 'detection_only-msg:Track_6 :initial-element (cl:make-instance 'detection_only-msg:Track_6))))
)

(cl:defclass Track6Array (<Track6Array>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Track6Array>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Track6Array)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name detection_only-msg:<Track6Array> is deprecated: use detection_only-msg:Track6Array instead.")))

(cl:ensure-generic-function 'tracks-val :lambda-list '(m))
(cl:defmethod tracks-val ((m <Track6Array>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader detection_only-msg:tracks-val is deprecated.  Use detection_only-msg:tracks instead.")
  (tracks m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Track6Array>) ostream)
  "Serializes a message object of type '<Track6Array>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'tracks))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'tracks))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Track6Array>) istream)
  "Deserializes a message object of type '<Track6Array>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'tracks) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'tracks)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'detection_only-msg:Track_6))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Track6Array>)))
  "Returns string type for a message object of type '<Track6Array>"
  "detection_only/Track6Array")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Track6Array)))
  "Returns string type for a message object of type 'Track6Array"
  "detection_only/Track6Array")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Track6Array>)))
  "Returns md5sum for a message object of type '<Track6Array>"
  "bcae8edf8c2ae4b9b6e115eb41e70bec")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Track6Array)))
  "Returns md5sum for a message object of type 'Track6Array"
  "bcae8edf8c2ae4b9b6e115eb41e70bec")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Track6Array>)))
  "Returns full string definition for message of type '<Track6Array>"
  (cl:format cl:nil "# bboxes ~%# [x1,y1,x2,y2,idx,class]~%Track_6[] tracks ~%================================================================================~%MSG: detection_only/Track_6~%# 1 bbox ~%# [x1,y1,x2,y2,conf,class]~%# float32[] bbox_info~%float32 x1~%float32 y1~%float32 x2~%float32 y2~%float32 id~%float32 cls~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Track6Array)))
  "Returns full string definition for message of type 'Track6Array"
  (cl:format cl:nil "# bboxes ~%# [x1,y1,x2,y2,idx,class]~%Track_6[] tracks ~%================================================================================~%MSG: detection_only/Track_6~%# 1 bbox ~%# [x1,y1,x2,y2,conf,class]~%# float32[] bbox_info~%float32 x1~%float32 y1~%float32 x2~%float32 y2~%float32 id~%float32 cls~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Track6Array>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'tracks) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Track6Array>))
  "Converts a ROS message object to a list"
  (cl:list 'Track6Array
    (cl:cons ':tracks (tracks msg))
))
