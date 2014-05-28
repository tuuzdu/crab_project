; Auto-generated. Do not edit!


(cl:in-package crab_msgs-msg)


;//! \htmlinclude LegsJointsState.msg.html

(cl:defclass <LegsJointsState> (roslisp-msg-protocol:ros-message)
  ((joints_state
    :reader joints_state
    :initarg :joints_state
    :type (cl:vector crab_msgs-msg:LegJointsState)
   :initform (cl:make-array 6 :element-type 'crab_msgs-msg:LegJointsState :initial-element (cl:make-instance 'crab_msgs-msg:LegJointsState))))
)

(cl:defclass LegsJointsState (<LegsJointsState>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <LegsJointsState>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'LegsJointsState)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name crab_msgs-msg:<LegsJointsState> is deprecated: use crab_msgs-msg:LegsJointsState instead.")))

(cl:ensure-generic-function 'joints_state-val :lambda-list '(m))
(cl:defmethod joints_state-val ((m <LegsJointsState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader crab_msgs-msg:joints_state-val is deprecated.  Use crab_msgs-msg:joints_state instead.")
  (joints_state m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <LegsJointsState>) ostream)
  "Serializes a message object of type '<LegsJointsState>"
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'joints_state))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <LegsJointsState>) istream)
  "Deserializes a message object of type '<LegsJointsState>"
  (cl:setf (cl:slot-value msg 'joints_state) (cl:make-array 6))
  (cl:let ((vals (cl:slot-value msg 'joints_state)))
    (cl:dotimes (i 6)
    (cl:setf (cl:aref vals i) (cl:make-instance 'crab_msgs-msg:LegJointsState))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<LegsJointsState>)))
  "Returns string type for a message object of type '<LegsJointsState>"
  "crab_msgs/LegsJointsState")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'LegsJointsState)))
  "Returns string type for a message object of type 'LegsJointsState"
  "crab_msgs/LegsJointsState")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<LegsJointsState>)))
  "Returns md5sum for a message object of type '<LegsJointsState>"
  "6fe1b620769e6858203cf2a048e48ef3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'LegsJointsState)))
  "Returns md5sum for a message object of type 'LegsJointsState"
  "6fe1b620769e6858203cf2a048e48ef3")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<LegsJointsState>)))
  "Returns full string definition for message of type '<LegsJointsState>"
  (cl:format cl:nil "crab_msgs/LegJointsState[6] joints_state~%~%================================================================================~%MSG: crab_msgs/LegJointsState~%float64[3] joint~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'LegsJointsState)))
  "Returns full string definition for message of type 'LegsJointsState"
  (cl:format cl:nil "crab_msgs/LegJointsState[6] joints_state~%~%================================================================================~%MSG: crab_msgs/LegJointsState~%float64[3] joint~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <LegsJointsState>))
  (cl:+ 0
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'joints_state) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <LegsJointsState>))
  "Converts a ROS message object to a list"
  (cl:list 'LegsJointsState
    (cl:cons ':joints_state (joints_state msg))
))
