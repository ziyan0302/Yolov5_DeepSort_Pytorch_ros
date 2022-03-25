; Auto-generated. Do not edit!


(cl:in-package detection_only-msg)


;//! \htmlinclude bbox.msg.html

(cl:defclass <bbox> (roslisp-msg-protocol:ros-message)
  ((bbox_info
    :reader bbox_info
    :initarg :bbox_info
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass bbox (<bbox>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <bbox>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'bbox)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name detection_only-msg:<bbox> is deprecated: use detection_only-msg:bbox instead.")))

(cl:ensure-generic-function 'bbox_info-val :lambda-list '(m))
(cl:defmethod bbox_info-val ((m <bbox>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader detection_only-msg:bbox_info-val is deprecated.  Use detection_only-msg:bbox_info instead.")
  (bbox_info m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <bbox>) ostream)
  "Serializes a message object of type '<bbox>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'bbox_info))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'bbox_info))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <bbox>) istream)
  "Deserializes a message object of type '<bbox>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'bbox_info) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'bbox_info)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<bbox>)))
  "Returns string type for a message object of type '<bbox>"
  "detection_only/bbox")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'bbox)))
  "Returns string type for a message object of type 'bbox"
  "detection_only/bbox")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<bbox>)))
  "Returns md5sum for a message object of type '<bbox>"
  "913b440d53de141b21b184494ddea913")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'bbox)))
  "Returns md5sum for a message object of type 'bbox"
  "913b440d53de141b21b184494ddea913")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<bbox>)))
  "Returns full string definition for message of type '<bbox>"
  (cl:format cl:nil "# 1 bbox ~%# [x1,y1,x2,y2,conf,class]~%float32[] bbox_info ~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'bbox)))
  "Returns full string definition for message of type 'bbox"
  (cl:format cl:nil "# 1 bbox ~%# [x1,y1,x2,y2,conf,class]~%float32[] bbox_info ~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <bbox>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'bbox_info) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <bbox>))
  "Converts a ROS message object to a list"
  (cl:list 'bbox
    (cl:cons ':bbox_info (bbox_info msg))
))
