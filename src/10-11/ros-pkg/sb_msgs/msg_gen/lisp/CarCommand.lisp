; Auto-generated. Do not edit!


(cl:in-package sb_msgs-msg)


;//! \htmlinclude CarCommand.msg.html

(cl:defclass <CarCommand> (roslisp-msg-protocol:ros-message)
  ((throttle
    :reader throttle
    :initarg :throttle
    :type cl:float
    :initform 0.0)
   (steering
    :reader steering
    :initarg :steering
    :type cl:float
    :initform 0.0))
)

(cl:defclass CarCommand (<CarCommand>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CarCommand>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CarCommand)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name sb_msgs-msg:<CarCommand> is deprecated: use sb_msgs-msg:CarCommand instead.")))

(cl:ensure-generic-function 'throttle-val :lambda-list '(m))
(cl:defmethod throttle-val ((m <CarCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sb_msgs-msg:throttle-val is deprecated.  Use sb_msgs-msg:throttle instead.")
  (throttle m))

(cl:ensure-generic-function 'steering-val :lambda-list '(m))
(cl:defmethod steering-val ((m <CarCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sb_msgs-msg:steering-val is deprecated.  Use sb_msgs-msg:steering instead.")
  (steering m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CarCommand>) ostream)
  "Serializes a message object of type '<CarCommand>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'throttle))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'steering))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CarCommand>) istream)
  "Deserializes a message object of type '<CarCommand>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'throttle) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'steering) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CarCommand>)))
  "Returns string type for a message object of type '<CarCommand>"
  "sb_msgs/CarCommand")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CarCommand)))
  "Returns string type for a message object of type 'CarCommand"
  "sb_msgs/CarCommand")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CarCommand>)))
  "Returns md5sum for a message object of type '<CarCommand>"
  "39f463d271c2ca10c14182802c72c029")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CarCommand)))
  "Returns md5sum for a message object of type 'CarCommand"
  "39f463d271c2ca10c14182802c72c029")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CarCommand>)))
  "Returns full string definition for message of type '<CarCommand>"
  (cl:format cl:nil "#Car command that arduinoDriver or furiousDriver can understand~%~%float64 throttle # throttle value -1 < x < 1~%float64 steering # steer value -1 < x < 1~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CarCommand)))
  "Returns full string definition for message of type 'CarCommand"
  (cl:format cl:nil "#Car command that arduinoDriver or furiousDriver can understand~%~%float64 throttle # throttle value -1 < x < 1~%float64 steering # steer value -1 < x < 1~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CarCommand>))
  (cl:+ 0
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CarCommand>))
  "Converts a ROS message object to a list"
  (cl:list 'CarCommand
    (cl:cons ':throttle (throttle msg))
    (cl:cons ':steering (steering msg))
))
