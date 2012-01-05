; Auto-generated. Do not edit!


(cl:in-package sb_msgs-msg)


;//! \htmlinclude TurretCommand.msg.html

(cl:defclass <TurretCommand> (roslisp-msg-protocol:ros-message)
  ((pan
    :reader pan
    :initarg :pan
    :type cl:float
    :initform 0.0)
   (tilt
    :reader tilt
    :initarg :tilt
    :type cl:float
    :initform 0.0))
)

(cl:defclass TurretCommand (<TurretCommand>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TurretCommand>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TurretCommand)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name sb_msgs-msg:<TurretCommand> is deprecated: use sb_msgs-msg:TurretCommand instead.")))

(cl:ensure-generic-function 'pan-val :lambda-list '(m))
(cl:defmethod pan-val ((m <TurretCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sb_msgs-msg:pan-val is deprecated.  Use sb_msgs-msg:pan instead.")
  (pan m))

(cl:ensure-generic-function 'tilt-val :lambda-list '(m))
(cl:defmethod tilt-val ((m <TurretCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sb_msgs-msg:tilt-val is deprecated.  Use sb_msgs-msg:tilt instead.")
  (tilt m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TurretCommand>) ostream)
  "Serializes a message object of type '<TurretCommand>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'pan))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'tilt))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TurretCommand>) istream)
  "Deserializes a message object of type '<TurretCommand>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'pan) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'tilt) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TurretCommand>)))
  "Returns string type for a message object of type '<TurretCommand>"
  "sb_msgs/TurretCommand")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TurretCommand)))
  "Returns string type for a message object of type 'TurretCommand"
  "sb_msgs/TurretCommand")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TurretCommand>)))
  "Returns md5sum for a message object of type '<TurretCommand>"
  "da61f3d6b381bd4af7a066f22fdfa441")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TurretCommand)))
  "Returns md5sum for a message object of type 'TurretCommand"
  "da61f3d6b381bd4af7a066f22fdfa441")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TurretCommand>)))
  "Returns full string definition for message of type '<TurretCommand>"
  (cl:format cl:nil "# A command for setting the position of a two-servo turret~%float64 pan  # between -1.0 and 1.0~%float64 tilt # between -1.0 and 1.0~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TurretCommand)))
  "Returns full string definition for message of type 'TurretCommand"
  (cl:format cl:nil "# A command for setting the position of a two-servo turret~%float64 pan  # between -1.0 and 1.0~%float64 tilt # between -1.0 and 1.0~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TurretCommand>))
  (cl:+ 0
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TurretCommand>))
  "Converts a ROS message object to a list"
  (cl:list 'TurretCommand
    (cl:cons ':pan (pan msg))
    (cl:cons ':tilt (tilt msg))
))
