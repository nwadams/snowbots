; Auto-generated. Do not edit!


(cl:in-package sb_msgs-msg)


;//! \htmlinclude ServoCommand.msg.html

(cl:defclass <ServoCommand> (roslisp-msg-protocol:ros-message)
  ((id
    :reader id
    :initarg :id
    :type cl:integer
    :initform 0)
   (pwm
    :reader pwm
    :initarg :pwm
    :type cl:integer
    :initform 0)
   (throttle
    :reader throttle
    :initarg :throttle
    :type cl:integer
    :initform 0)
   (steering
    :reader steering
    :initarg :steering
    :type cl:integer
    :initform 0)
   (pan
    :reader pan
    :initarg :pan
    :type cl:integer
    :initform 0)
   (tilt
    :reader tilt
    :initarg :tilt
    :type cl:integer
    :initform 0)
   (usingServo
    :reader usingServo
    :initarg :usingServo
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass ServoCommand (<ServoCommand>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ServoCommand>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ServoCommand)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name sb_msgs-msg:<ServoCommand> is deprecated: use sb_msgs-msg:ServoCommand instead.")))

(cl:ensure-generic-function 'id-val :lambda-list '(m))
(cl:defmethod id-val ((m <ServoCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sb_msgs-msg:id-val is deprecated.  Use sb_msgs-msg:id instead.")
  (id m))

(cl:ensure-generic-function 'pwm-val :lambda-list '(m))
(cl:defmethod pwm-val ((m <ServoCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sb_msgs-msg:pwm-val is deprecated.  Use sb_msgs-msg:pwm instead.")
  (pwm m))

(cl:ensure-generic-function 'throttle-val :lambda-list '(m))
(cl:defmethod throttle-val ((m <ServoCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sb_msgs-msg:throttle-val is deprecated.  Use sb_msgs-msg:throttle instead.")
  (throttle m))

(cl:ensure-generic-function 'steering-val :lambda-list '(m))
(cl:defmethod steering-val ((m <ServoCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sb_msgs-msg:steering-val is deprecated.  Use sb_msgs-msg:steering instead.")
  (steering m))

(cl:ensure-generic-function 'pan-val :lambda-list '(m))
(cl:defmethod pan-val ((m <ServoCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sb_msgs-msg:pan-val is deprecated.  Use sb_msgs-msg:pan instead.")
  (pan m))

(cl:ensure-generic-function 'tilt-val :lambda-list '(m))
(cl:defmethod tilt-val ((m <ServoCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sb_msgs-msg:tilt-val is deprecated.  Use sb_msgs-msg:tilt instead.")
  (tilt m))

(cl:ensure-generic-function 'usingServo-val :lambda-list '(m))
(cl:defmethod usingServo-val ((m <ServoCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sb_msgs-msg:usingServo-val is deprecated.  Use sb_msgs-msg:usingServo instead.")
  (usingServo m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ServoCommand>) ostream)
  "Serializes a message object of type '<ServoCommand>"
  (cl:let* ((signed (cl:slot-value msg 'id)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'pwm)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'throttle)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'steering)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'pan)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'tilt)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'usingServo) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ServoCommand>) istream)
  "Deserializes a message object of type '<ServoCommand>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'id) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'pwm) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'throttle) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'steering) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'pan) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'tilt) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:setf (cl:slot-value msg 'usingServo) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ServoCommand>)))
  "Returns string type for a message object of type '<ServoCommand>"
  "sb_msgs/ServoCommand")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ServoCommand)))
  "Returns string type for a message object of type 'ServoCommand"
  "sb_msgs/ServoCommand")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ServoCommand>)))
  "Returns md5sum for a message object of type '<ServoCommand>"
  "0638299d4e64a023ad0af9c6a2fd2ef2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ServoCommand)))
  "Returns md5sum for a message object of type 'ServoCommand"
  "0638299d4e64a023ad0af9c6a2fd2ef2")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ServoCommand>)))
  "Returns full string definition for message of type '<ServoCommand>"
  (cl:format cl:nil "# A servo command that the furiousDriver.py node can understand.~%~%int32 id  # The integer id of the servo~%int32 pwm # The raw servo value~%~%int32 throttle~%int32 steering~%int32 pan~%int32 tilt~%~%bool usingServo~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ServoCommand)))
  "Returns full string definition for message of type 'ServoCommand"
  (cl:format cl:nil "# A servo command that the furiousDriver.py node can understand.~%~%int32 id  # The integer id of the servo~%int32 pwm # The raw servo value~%~%int32 throttle~%int32 steering~%int32 pan~%int32 tilt~%~%bool usingServo~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ServoCommand>))
  (cl:+ 0
     4
     4
     4
     4
     4
     4
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ServoCommand>))
  "Converts a ROS message object to a list"
  (cl:list 'ServoCommand
    (cl:cons ':id (id msg))
    (cl:cons ':pwm (pwm msg))
    (cl:cons ':throttle (throttle msg))
    (cl:cons ':steering (steering msg))
    (cl:cons ':pan (pan msg))
    (cl:cons ':tilt (tilt msg))
    (cl:cons ':usingServo (usingServo msg))
))
