; Auto-generated. Do not edit!


(cl:in-package sb_msgs-msg)


;//! \htmlinclude VisionNav.msg.html

(cl:defclass <VisionNav> (roslisp-msg-protocol:ros-message)
  ((confidence
    :reader confidence
    :initarg :confidence
    :type cl:integer
    :initform 0)
   (direction
    :reader direction
    :initarg :direction
    :type cl:integer
    :initform 0)
   (distance
    :reader distance
    :initarg :distance
    :type cl:integer
    :initform 0))
)

(cl:defclass VisionNav (<VisionNav>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <VisionNav>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'VisionNav)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name sb_msgs-msg:<VisionNav> is deprecated: use sb_msgs-msg:VisionNav instead.")))

(cl:ensure-generic-function 'confidence-val :lambda-list '(m))
(cl:defmethod confidence-val ((m <VisionNav>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sb_msgs-msg:confidence-val is deprecated.  Use sb_msgs-msg:confidence instead.")
  (confidence m))

(cl:ensure-generic-function 'direction-val :lambda-list '(m))
(cl:defmethod direction-val ((m <VisionNav>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sb_msgs-msg:direction-val is deprecated.  Use sb_msgs-msg:direction instead.")
  (direction m))

(cl:ensure-generic-function 'distance-val :lambda-list '(m))
(cl:defmethod distance-val ((m <VisionNav>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sb_msgs-msg:distance-val is deprecated.  Use sb_msgs-msg:distance instead.")
  (distance m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <VisionNav>) ostream)
  "Serializes a message object of type '<VisionNav>"
  (cl:let* ((signed (cl:slot-value msg 'confidence)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'direction)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'distance)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <VisionNav>) istream)
  "Deserializes a message object of type '<VisionNav>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'confidence) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'direction) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'distance) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<VisionNav>)))
  "Returns string type for a message object of type '<VisionNav>"
  "sb_msgs/VisionNav")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'VisionNav)))
  "Returns string type for a message object of type 'VisionNav"
  "sb_msgs/VisionNav")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<VisionNav>)))
  "Returns md5sum for a message object of type '<VisionNav>"
  "3551098c9d8ba0ab66ec155e8ee6b271")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'VisionNav)))
  "Returns md5sum for a message object of type 'VisionNav"
  "3551098c9d8ba0ab66ec155e8ee6b271")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<VisionNav>)))
  "Returns full string definition for message of type '<VisionNav>"
  (cl:format cl:nil "int32 confidence #a percentage~%int32 direction  ~%int32 distance ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'VisionNav)))
  "Returns full string definition for message of type 'VisionNav"
  (cl:format cl:nil "int32 confidence #a percentage~%int32 direction  ~%int32 distance ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <VisionNav>))
  (cl:+ 0
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <VisionNav>))
  "Converts a ROS message object to a list"
  (cl:list 'VisionNav
    (cl:cons ':confidence (confidence msg))
    (cl:cons ':direction (direction msg))
    (cl:cons ':distance (distance msg))
))
