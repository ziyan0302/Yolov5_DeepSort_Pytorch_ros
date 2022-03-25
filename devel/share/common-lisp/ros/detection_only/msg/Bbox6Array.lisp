; Auto-generated. Do not edit!


(cl:in-package detection_only-msg)


;//! \htmlinclude Bbox6Array.msg.html

(cl:defclass <Bbox6Array> (roslisp-msg-protocol:ros-message)
  ((bboxes
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

(cl:ensure-generic-function 'bboxes-val :lambda-list '(m))
(cl:defmethod bboxes-val ((m <Bbox6Array>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader detection_only-msg:bboxes-val is deprecated.  Use detection_only-msg:bboxes instead.")
  (bboxes m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Bbox6Array>) ostream)
  "Serializes a message object of type '<Bbox6Array>"
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
  "61b8f2a52aff12e25f2957c1314c78d3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Bbox6Array)))
  "Returns md5sum for a message object of type 'Bbox6Array"
  "61b8f2a52aff12e25f2957c1314c78d3")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Bbox6Array>)))
  "Returns full string definition for message of type '<Bbox6Array>"
  (cl:format cl:nil "# bboxes ~%# [x1,y1,x2,y2,conf,class]~%Bbox_6[] bboxes ~%================================================================================~%MSG: detection_only/Bbox_6~%# 1 bbox ~%# [x1,y1,x2,y2,conf,class]~%# float32[] bbox_info~%float32 x1~%float32 y1~%float32 x2~%float32 y2~%float32 conf~%float32 cls~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Bbox6Array)))
  "Returns full string definition for message of type 'Bbox6Array"
  (cl:format cl:nil "# bboxes ~%# [x1,y1,x2,y2,conf,class]~%Bbox_6[] bboxes ~%================================================================================~%MSG: detection_only/Bbox_6~%# 1 bbox ~%# [x1,y1,x2,y2,conf,class]~%# float32[] bbox_info~%float32 x1~%float32 y1~%float32 x2~%float32 y2~%float32 conf~%float32 cls~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Bbox6Array>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'bboxes) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Bbox6Array>))
  "Converts a ROS message object to a list"
  (cl:list 'Bbox6Array
    (cl:cons ':bboxes (bboxes msg))
))
