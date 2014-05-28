; Auto-generated. Do not edit!


(cl:in-package crab_msgs-msg)


;//! \htmlinclude LegIKRequest.msg.html

(cl:defclass <LegIKRequest> (roslisp-msg-protocol:ros-message)
  ((leg_number
    :reader leg_number
    :initarg :leg_number
    :type cl:integer
    :initform 0)
   (current_joints
    :reader current_joints
    :initarg :current_joints
    :type crab_msgs-msg:LegJointsState
    :initform (cl:make-instance 'crab_msgs-msg:LegJointsState))
   (target_position
    :reader target_position
    :initarg :target_position
    :type crab_msgs-msg:LegPositionState
    :initform (cl:make-instance 'crab_msgs-msg:LegPositionState)))
)

(cl:defclass LegIKRequest (<LegIKRequest>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <LegIKRequest>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'LegIKRequest)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name crab_msgs-msg:<LegIKRequest> is deprecated: use crab_msgs-msg:LegIKRequest instead.")))

(cl:ensure-generic-function 'leg_number-val :lambda-list '(m))
(cl:defmethod leg_number-val ((m <LegIKRequest>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader crab_msgs-msg:leg_number-val is deprecated.  Use crab_msgs-msg:leg_number instead.")
  (leg_number m))

(cl:ensure-generic-function 'current_joints-val :lambda-list '(m))
(cl:defmethod current_joints-val ((m <LegIKRequest>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader crab_msgs-msg:current_joints-val is deprecated.  Use crab_msgs-msg:current_joints instead.")
  (current_joints m))

(cl:ensure-generic-function 'target_position-val :lambda-list '(m))
(cl:defmethod target_position-val ((m <LegIKRequest>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader crab_msgs-msg:target_position-val is deprecated.  Use crab_msgs-msg:target_position instead.")
  (target_position m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <LegIKRequest>) ostream)
  "Serializes a message object of type '<LegIKRequest>"
  (cl:let* ((signed (cl:slot-value msg 'leg_number)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'current_joints) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'target_position) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <LegIKRequest>) istream)
  "Deserializes a message object of type '<LegIKRequest>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'leg_number) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'current_joints) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'target_position) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<LegIKRequest>)))
  "Returns string type for a message object of type '<LegIKRequest>"
  "crab_msgs/LegIKRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'LegIKRequest)))
  "Returns string type for a message object of type 'LegIKRequest"
  "crab_msgs/LegIKRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<LegIKRequest>)))
  "Returns md5sum for a message object of type '<LegIKRequest>"
  "bcb4917a6750a0f98ecbb5e98212cbec")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'LegIKRequest)))
  "Returns md5sum for a message object of type 'LegIKRequest"
  "bcb4917a6750a0f98ecbb5e98212cbec")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<LegIKRequest>)))
  "Returns full string definition for message of type '<LegIKRequest>"
  (cl:format cl:nil "int32 leg_number~%crab_msgs/LegJointsState current_joints~%crab_msgs/LegPositionState target_position~%~%================================================================================~%MSG: crab_msgs/LegJointsState~%float64[3] joint~%~%================================================================================~%MSG: crab_msgs/LegPositionState~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'LegIKRequest)))
  "Returns full string definition for message of type 'LegIKRequest"
  (cl:format cl:nil "int32 leg_number~%crab_msgs/LegJointsState current_joints~%crab_msgs/LegPositionState target_position~%~%================================================================================~%MSG: crab_msgs/LegJointsState~%float64[3] joint~%~%================================================================================~%MSG: crab_msgs/LegPositionState~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <LegIKRequest>))
  (cl:+ 0
     4
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'current_joints))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'target_position))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <LegIKRequest>))
  "Converts a ROS message object to a list"
  (cl:list 'LegIKRequest
    (cl:cons ':leg_number (leg_number msg))
    (cl:cons ':current_joints (current_joints msg))
    (cl:cons ':target_position (target_position msg))
))
