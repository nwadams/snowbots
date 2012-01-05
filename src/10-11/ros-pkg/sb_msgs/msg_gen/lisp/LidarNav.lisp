; Auto-generated. Do not edit!


(cl:in-package sb_msgs-msg)


;//! \htmlinclude LidarNav.msg.html

(cl:defclass <LidarNav> (roslisp-msg-protocol:ros-message)
  ((direction
    :reader direction
    :initarg :direction
    :type cl:float
    :initform 0.0)
   (distance
    :reader distance
    :initarg :distance
    :type cl:float
    :initform 0.0)
   (confidence
    :reader confidence
    :initarg :confidence
    :type cl:integer
    :initform 0))
)

(cl:defclass LidarNav (<LidarNav>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <LidarNav>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'LidarNav)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name sb_msgs-msg:<LidarNav> is deprecated: use sb_msgs-msg:LidarNav instead.")))

(cl:ensure-generic-function 'direction-val :lambda-list '(m))
(cl:defmethod direction-val ((m <LidarNav>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sb_msgs-msg:direction-val is deprecated.  Use sb_msgs-msg:direction instead.")
  (direction m))

(cl:ensure-generic-function 'distance-val :lambda-list '(m))
(cl:defmethod distance-val ((m <LidarNav>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sb_msgs-msg:distance-val is deprecated.  Use sb_msgs-msg:distance instead.")
  (distance m))

(cl:ensure-generic-function 'confidence-val :lambda-list '(m))
(cl:defmethod confidence-val ((m <LidarNav>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sb_msgs-msg:confidence-val is deprecated.  Use sb_msgs-msg:confidence instead.")
  (confidence m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <LidarNav>) ostream)
  "Serializes a message object of type '<LidarNav>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'direction))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'distance))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let* ((signed (cl:slot-value msg 'confidence)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <LidarNav>) istream)
  "Deserializes a message object of type '<LidarNav>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'direction) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'distance) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'confidence) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<LidarNav>)))
  "Returns string type for a message object of type '<LidarNav>"
  "sb_msgs/LidarNav")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'LidarNav)))
  "Returns string type for a message object of type 'LidarNav"
  "sb_msgs/LidarNav")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<LidarNav>)))
  "Returns md5sum for a message object of type '<LidarNav>"
  "f176eab4d8b8cab198d7795b194bf566")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'LidarNav)))
  "Returns md5sum for a message object of type 'LidarNav"
  "f176eab4d8b8cab198d7795b194bf566")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<LidarNav>)))
  "Returns full string definition for message of type '<LidarNav>"
  (cl:format cl:nil "#Message from LidarNav node to commander~%~%float32 direction~%float32 distance~%int32 confidence #a percentage~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'LidarNav)))
  "Returns full string definition for message of type 'LidarNav"
  (cl:format cl:nil "#Message from LidarNav node to commander~%~%float32 direction~%float32 distance~%int32 confidence #a percentage~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <LidarNav>))
  (cl:+ 0
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <LidarNav>))
  "Converts a ROS message object to a list"
  (cl:list 'LidarNav
    (cl:cons ':direction (direction msg))
    (cl:cons ':distance (distance msg))
    (cl:cons ':confidence (confidence msg))
))
