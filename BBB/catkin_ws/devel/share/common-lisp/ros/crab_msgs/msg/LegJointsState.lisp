; Auto-generated. Do not edit!


(cl:in-package crab_msgs-msg)


;//! \htmlinclude LegJointsState.msg.html

(cl:defclass <LegJointsState> (roslisp-msg-protocol:ros-message)
  ((joint
    :reader joint
    :initarg :joint
    :type (cl:vector cl:float)
   :initform (cl:make-array 3 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass LegJointsState (<LegJointsState>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <LegJointsState>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'LegJointsState)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name crab_msgs-msg:<LegJointsState> is deprecated: use crab_msgs-msg:LegJointsState instead.")))

(cl:ensure-generic-function 'joint-val :lambda-list '(m))
(cl:defmethod joint-val ((m <LegJointsState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader crab_msgs-msg:joint-val is deprecated.  Use crab_msgs-msg:joint instead.")
  (joint m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <LegJointsState>) ostream)
  "Serializes a message object of type '<LegJointsState>"
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'joint))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <LegJointsState>) istream)
  "Deserializes a message object of type '<LegJointsState>"
  (cl:setf (cl:slot-value msg 'joint) (cl:make-array 3))
  (cl:let ((vals (cl:slot-value msg 'joint)))
    (cl:dotimes (i 3)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<LegJointsState>)))
  "Returns string type for a message object of type '<LegJointsState>"
  "crab_msgs/LegJointsState")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'LegJointsState)))
  "Returns string type for a message object of type 'LegJointsState"
  "crab_msgs/LegJointsState")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<LegJointsState>)))
  "Returns md5sum for a message object of type '<LegJointsState>"
  "8c9183fb9f551c0c8a1fa0044edf51fc")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'LegJointsState)))
  "Returns md5sum for a message object of type 'LegJointsState"
  "8c9183fb9f551c0c8a1fa0044edf51fc")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<LegJointsState>)))
  "Returns full string definition for message of type '<LegJointsState>"
  (cl:format cl:nil "float64[3] joint~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'LegJointsState)))
  "Returns full string definition for message of type 'LegJointsState"
  (cl:format cl:nil "float64[3] joint~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <LegJointsState>))
  (cl:+ 0
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'joint) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <LegJointsState>))
  "Converts a ROS message object to a list"
  (cl:list 'LegJointsState
    (cl:cons ':joint (joint msg))
))
