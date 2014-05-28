; Auto-generated. Do not edit!


(cl:in-package crab_msgs-msg)


;//! \htmlinclude BodyCommand.msg.html

(cl:defclass <BodyCommand> (roslisp-msg-protocol:ros-message)
  ((cmd
    :reader cmd
    :initarg :cmd
    :type cl:integer
    :initform 0))
)

(cl:defclass BodyCommand (<BodyCommand>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <BodyCommand>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'BodyCommand)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name crab_msgs-msg:<BodyCommand> is deprecated: use crab_msgs-msg:BodyCommand instead.")))

(cl:ensure-generic-function 'cmd-val :lambda-list '(m))
(cl:defmethod cmd-val ((m <BodyCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader crab_msgs-msg:cmd-val is deprecated.  Use crab_msgs-msg:cmd instead.")
  (cmd m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<BodyCommand>)))
    "Constants for message type '<BodyCommand>"
  '((:STAND_UP_CMD . 1)
    (:SEAT_DOWN_CMD . 2)
    (:IMU_START_CMD . 3)
    (:IMU_STOP_CMD . 4))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'BodyCommand)))
    "Constants for message type 'BodyCommand"
  '((:STAND_UP_CMD . 1)
    (:SEAT_DOWN_CMD . 2)
    (:IMU_START_CMD . 3)
    (:IMU_STOP_CMD . 4))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <BodyCommand>) ostream)
  "Serializes a message object of type '<BodyCommand>"
  (cl:let* ((signed (cl:slot-value msg 'cmd)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <BodyCommand>) istream)
  "Deserializes a message object of type '<BodyCommand>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'cmd) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<BodyCommand>)))
  "Returns string type for a message object of type '<BodyCommand>"
  "crab_msgs/BodyCommand")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'BodyCommand)))
  "Returns string type for a message object of type 'BodyCommand"
  "crab_msgs/BodyCommand")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<BodyCommand>)))
  "Returns md5sum for a message object of type '<BodyCommand>"
  "1abfa573ddc640038a45f99b26d93a23")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'BodyCommand)))
  "Returns md5sum for a message object of type 'BodyCommand"
  "1abfa573ddc640038a45f99b26d93a23")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<BodyCommand>)))
  "Returns full string definition for message of type '<BodyCommand>"
  (cl:format cl:nil "int32 cmd~%~%int32 STAND_UP_CMD=1~%int32 SEAT_DOWN_CMD=2~%int32 IMU_START_CMD=3~%int32 IMU_STOP_CMD=4~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'BodyCommand)))
  "Returns full string definition for message of type 'BodyCommand"
  (cl:format cl:nil "int32 cmd~%~%int32 STAND_UP_CMD=1~%int32 SEAT_DOWN_CMD=2~%int32 IMU_START_CMD=3~%int32 IMU_STOP_CMD=4~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <BodyCommand>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <BodyCommand>))
  "Converts a ROS message object to a list"
  (cl:list 'BodyCommand
    (cl:cons ':cmd (cmd msg))
))
