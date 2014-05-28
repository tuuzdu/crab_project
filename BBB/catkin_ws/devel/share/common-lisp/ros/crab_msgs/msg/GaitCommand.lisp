; Auto-generated. Do not edit!


(cl:in-package crab_msgs-msg)


;//! \htmlinclude GaitCommand.msg.html

(cl:defclass <GaitCommand> (roslisp-msg-protocol:ros-message)
  ((cmd
    :reader cmd
    :initarg :cmd
    :type cl:integer
    :initform 0)
   (fi
    :reader fi
    :initarg :fi
    :type cl:float
    :initform 0.0)
   (velocity
    :reader velocity
    :initarg :velocity
    :type cl:float
    :initform 0.0)
   (alpha
    :reader alpha
    :initarg :alpha
    :type cl:float
    :initform 0.0)
   (scale
    :reader scale
    :initarg :scale
    :type cl:float
    :initform 0.0))
)

(cl:defclass GaitCommand (<GaitCommand>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GaitCommand>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GaitCommand)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name crab_msgs-msg:<GaitCommand> is deprecated: use crab_msgs-msg:GaitCommand instead.")))

(cl:ensure-generic-function 'cmd-val :lambda-list '(m))
(cl:defmethod cmd-val ((m <GaitCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader crab_msgs-msg:cmd-val is deprecated.  Use crab_msgs-msg:cmd instead.")
  (cmd m))

(cl:ensure-generic-function 'fi-val :lambda-list '(m))
(cl:defmethod fi-val ((m <GaitCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader crab_msgs-msg:fi-val is deprecated.  Use crab_msgs-msg:fi instead.")
  (fi m))

(cl:ensure-generic-function 'velocity-val :lambda-list '(m))
(cl:defmethod velocity-val ((m <GaitCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader crab_msgs-msg:velocity-val is deprecated.  Use crab_msgs-msg:velocity instead.")
  (velocity m))

(cl:ensure-generic-function 'alpha-val :lambda-list '(m))
(cl:defmethod alpha-val ((m <GaitCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader crab_msgs-msg:alpha-val is deprecated.  Use crab_msgs-msg:alpha instead.")
  (alpha m))

(cl:ensure-generic-function 'scale-val :lambda-list '(m))
(cl:defmethod scale-val ((m <GaitCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader crab_msgs-msg:scale-val is deprecated.  Use crab_msgs-msg:scale instead.")
  (scale m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<GaitCommand>)))
    "Constants for message type '<GaitCommand>"
  '((:RUNRIPPLE . 1)
    (:RUNTRIPOD . 2)
    (:STOP . 3)
    (:PAUSE . 4))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'GaitCommand)))
    "Constants for message type 'GaitCommand"
  '((:RUNRIPPLE . 1)
    (:RUNTRIPOD . 2)
    (:STOP . 3)
    (:PAUSE . 4))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GaitCommand>) ostream)
  "Serializes a message object of type '<GaitCommand>"
  (cl:let* ((signed (cl:slot-value msg 'cmd)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'fi))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'velocity))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'alpha))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'scale))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GaitCommand>) istream)
  "Deserializes a message object of type '<GaitCommand>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'cmd) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'fi) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'velocity) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'alpha) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'scale) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GaitCommand>)))
  "Returns string type for a message object of type '<GaitCommand>"
  "crab_msgs/GaitCommand")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GaitCommand)))
  "Returns string type for a message object of type 'GaitCommand"
  "crab_msgs/GaitCommand")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GaitCommand>)))
  "Returns md5sum for a message object of type '<GaitCommand>"
  "47aecd62b438a8407fd0203311383fc4")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GaitCommand)))
  "Returns md5sum for a message object of type 'GaitCommand"
  "47aecd62b438a8407fd0203311383fc4")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GaitCommand>)))
  "Returns full string definition for message of type '<GaitCommand>"
  (cl:format cl:nil "int32 cmd~%float64 fi~%float64 velocity~%float64 alpha~%float64 scale~%~%int32 RUNRIPPLE=1~%int32 RUNTRIPOD=2~%int32 STOP=3~%int32 PAUSE=4~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GaitCommand)))
  "Returns full string definition for message of type 'GaitCommand"
  (cl:format cl:nil "int32 cmd~%float64 fi~%float64 velocity~%float64 alpha~%float64 scale~%~%int32 RUNRIPPLE=1~%int32 RUNTRIPOD=2~%int32 STOP=3~%int32 PAUSE=4~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GaitCommand>))
  (cl:+ 0
     4
     8
     8
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GaitCommand>))
  "Converts a ROS message object to a list"
  (cl:list 'GaitCommand
    (cl:cons ':cmd (cmd msg))
    (cl:cons ':fi (fi msg))
    (cl:cons ':velocity (velocity msg))
    (cl:cons ':alpha (alpha msg))
    (cl:cons ':scale (scale msg))
))
