; Auto-generated. Do not edit!


(cl:in-package detection_only-msg)


;//! \htmlinclude Bbox_6.msg.html

(cl:defclass <Bbox_6> (roslisp-msg-protocol:ros-message)
  ((x1
    :reader x1
    :initarg :x1
    :type cl:float
    :initform 0.0)
   (y1
    :reader y1
    :initarg :y1
    :type cl:float
    :initform 0.0)
   (x2
    :reader x2
    :initarg :x2
    :type cl:float
    :initform 0.0)
   (y2
    :reader y2
    :initarg :y2
    :type cl:float
    :initform 0.0)
   (conf
    :reader conf
    :initarg :conf
    :type cl:float
    :initform 0.0)
   (cls
    :reader cls
    :initarg :cls
    :type cl:float
    :initform 0.0))
)

(cl:defclass Bbox_6 (<Bbox_6>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Bbox_6>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Bbox_6)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name detection_only-msg:<Bbox_6> is deprecated: use detection_only-msg:Bbox_6 instead.")))

(cl:ensure-generic-function 'x1-val :lambda-list '(m))
(cl:defmethod x1-val ((m <Bbox_6>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader detection_only-msg:x1-val is deprecated.  Use detection_only-msg:x1 instead.")
  (x1 m))

(cl:ensure-generic-function 'y1-val :lambda-list '(m))
(cl:defmethod y1-val ((m <Bbox_6>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader detection_only-msg:y1-val is deprecated.  Use detection_only-msg:y1 instead.")
  (y1 m))

(cl:ensure-generic-function 'x2-val :lambda-list '(m))
(cl:defmethod x2-val ((m <Bbox_6>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader detection_only-msg:x2-val is deprecated.  Use detection_only-msg:x2 instead.")
  (x2 m))

(cl:ensure-generic-function 'y2-val :lambda-list '(m))
(cl:defmethod y2-val ((m <Bbox_6>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader detection_only-msg:y2-val is deprecated.  Use detection_only-msg:y2 instead.")
  (y2 m))

(cl:ensure-generic-function 'conf-val :lambda-list '(m))
(cl:defmethod conf-val ((m <Bbox_6>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader detection_only-msg:conf-val is deprecated.  Use detection_only-msg:conf instead.")
  (conf m))

(cl:ensure-generic-function 'cls-val :lambda-list '(m))
(cl:defmethod cls-val ((m <Bbox_6>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader detection_only-msg:cls-val is deprecated.  Use detection_only-msg:cls instead.")
  (cls m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Bbox_6>) ostream)
  "Serializes a message object of type '<Bbox_6>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'x1))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'y1))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'x2))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'y2))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'conf))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'cls))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Bbox_6>) istream)
  "Deserializes a message object of type '<Bbox_6>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'x1) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'y1) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'x2) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'y2) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'conf) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'cls) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Bbox_6>)))
  "Returns string type for a message object of type '<Bbox_6>"
  "detection_only/Bbox_6")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Bbox_6)))
  "Returns string type for a message object of type 'Bbox_6"
  "detection_only/Bbox_6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Bbox_6>)))
  "Returns md5sum for a message object of type '<Bbox_6>"
  "5a76d49beb9ad80ed19e0ba292e46abc")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Bbox_6)))
  "Returns md5sum for a message object of type 'Bbox_6"
  "5a76d49beb9ad80ed19e0ba292e46abc")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Bbox_6>)))
  "Returns full string definition for message of type '<Bbox_6>"
  (cl:format cl:nil "# 1 bbox ~%# [x1,y1,x2,y2,conf,class]~%# float32[] bbox_info~%float32 x1~%float32 y1~%float32 x2~%float32 y2~%float32 conf~%float32 cls~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Bbox_6)))
  "Returns full string definition for message of type 'Bbox_6"
  (cl:format cl:nil "# 1 bbox ~%# [x1,y1,x2,y2,conf,class]~%# float32[] bbox_info~%float32 x1~%float32 y1~%float32 x2~%float32 y2~%float32 conf~%float32 cls~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Bbox_6>))
  (cl:+ 0
     4
     4
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Bbox_6>))
  "Converts a ROS message object to a list"
  (cl:list 'Bbox_6
    (cl:cons ':x1 (x1 msg))
    (cl:cons ':y1 (y1 msg))
    (cl:cons ':x2 (x2 msg))
    (cl:cons ':y2 (y2 msg))
    (cl:cons ':conf (conf msg))
    (cl:cons ':cls (cls msg))
))
