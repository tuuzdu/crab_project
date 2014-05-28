; Auto-generated. Do not edit!


(cl:in-package crab_msgs-srv)


;//! \htmlinclude GetLegIKSolver-request.msg.html

(cl:defclass <GetLegIKSolver-request> (roslisp-msg-protocol:ros-message)
  ((leg_number
    :reader leg_number
    :initarg :leg_number
    :type (cl:vector cl:integer)
   :initform (cl:make-array 0 :element-type 'cl:integer :initial-element 0))
   (current_joints
    :reader current_joints
    :initarg :current_joints
    :type (cl:vector crab_msgs-msg:LegJointsState)
   :initform (cl:make-array 0 :element-type 'crab_msgs-msg:LegJointsState :initial-element (cl:make-instance 'crab_msgs-msg:LegJointsState)))
   (target_position
    :reader target_position
    :initarg :target_position
    :type (cl:vector crab_msgs-msg:LegPositionState)
   :initform (cl:make-array 0 :element-type 'crab_msgs-msg:LegPositionState :initial-element (cl:make-instance 'crab_msgs-msg:LegPositionState))))
)

(cl:defclass GetLegIKSolver-request (<GetLegIKSolver-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetLegIKSolver-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetLegIKSolver-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name crab_msgs-srv:<GetLegIKSolver-request> is deprecated: use crab_msgs-srv:GetLegIKSolver-request instead.")))

(cl:ensure-generic-function 'leg_number-val :lambda-list '(m))
(cl:defmethod leg_number-val ((m <GetLegIKSolver-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader crab_msgs-srv:leg_number-val is deprecated.  Use crab_msgs-srv:leg_number instead.")
  (leg_number m))

(cl:ensure-generic-function 'current_joints-val :lambda-list '(m))
(cl:defmethod current_joints-val ((m <GetLegIKSolver-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader crab_msgs-srv:current_joints-val is deprecated.  Use crab_msgs-srv:current_joints instead.")
  (current_joints m))

(cl:ensure-generic-function 'target_position-val :lambda-list '(m))
(cl:defmethod target_position-val ((m <GetLegIKSolver-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader crab_msgs-srv:target_position-val is deprecated.  Use crab_msgs-srv:target_position instead.")
  (target_position m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetLegIKSolver-request>) ostream)
  "Serializes a message object of type '<GetLegIKSolver-request>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'leg_number))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    ))
   (cl:slot-value msg 'leg_number))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'current_joints))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'current_joints))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'target_position))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'target_position))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetLegIKSolver-request>) istream)
  "Deserializes a message object of type '<GetLegIKSolver-request>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'leg_number) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'leg_number)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296)))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'current_joints) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'current_joints)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'crab_msgs-msg:LegJointsState))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'target_position) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'target_position)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'crab_msgs-msg:LegPositionState))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetLegIKSolver-request>)))
  "Returns string type for a service object of type '<GetLegIKSolver-request>"
  "crab_msgs/GetLegIKSolverRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetLegIKSolver-request)))
  "Returns string type for a service object of type 'GetLegIKSolver-request"
  "crab_msgs/GetLegIKSolverRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetLegIKSolver-request>)))
  "Returns md5sum for a message object of type '<GetLegIKSolver-request>"
  "aab6d4778f2d7a09001d31408c116ed3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetLegIKSolver-request)))
  "Returns md5sum for a message object of type 'GetLegIKSolver-request"
  "aab6d4778f2d7a09001d31408c116ed3")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetLegIKSolver-request>)))
  "Returns full string definition for message of type '<GetLegIKSolver-request>"
  (cl:format cl:nil "int32[] leg_number~%crab_msgs/LegJointsState[] current_joints~%crab_msgs/LegPositionState[] target_position~%~%~%================================================================================~%MSG: crab_msgs/LegJointsState~%float64[3] joint~%~%================================================================================~%MSG: crab_msgs/LegPositionState~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetLegIKSolver-request)))
  "Returns full string definition for message of type 'GetLegIKSolver-request"
  (cl:format cl:nil "int32[] leg_number~%crab_msgs/LegJointsState[] current_joints~%crab_msgs/LegPositionState[] target_position~%~%~%================================================================================~%MSG: crab_msgs/LegJointsState~%float64[3] joint~%~%================================================================================~%MSG: crab_msgs/LegPositionState~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetLegIKSolver-request>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'leg_number) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'current_joints) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'target_position) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetLegIKSolver-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetLegIKSolver-request
    (cl:cons ':leg_number (leg_number msg))
    (cl:cons ':current_joints (current_joints msg))
    (cl:cons ':target_position (target_position msg))
))
;//! \htmlinclude GetLegIKSolver-response.msg.html

(cl:defclass <GetLegIKSolver-response> (roslisp-msg-protocol:ros-message)
  ((target_joints
    :reader target_joints
    :initarg :target_joints
    :type (cl:vector crab_msgs-msg:LegJointsState)
   :initform (cl:make-array 0 :element-type 'crab_msgs-msg:LegJointsState :initial-element (cl:make-instance 'crab_msgs-msg:LegJointsState)))
   (error_codes
    :reader error_codes
    :initarg :error_codes
    :type cl:integer
    :initform 0))
)

(cl:defclass GetLegIKSolver-response (<GetLegIKSolver-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetLegIKSolver-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetLegIKSolver-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name crab_msgs-srv:<GetLegIKSolver-response> is deprecated: use crab_msgs-srv:GetLegIKSolver-response instead.")))

(cl:ensure-generic-function 'target_joints-val :lambda-list '(m))
(cl:defmethod target_joints-val ((m <GetLegIKSolver-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader crab_msgs-srv:target_joints-val is deprecated.  Use crab_msgs-srv:target_joints instead.")
  (target_joints m))

(cl:ensure-generic-function 'error_codes-val :lambda-list '(m))
(cl:defmethod error_codes-val ((m <GetLegIKSolver-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader crab_msgs-srv:error_codes-val is deprecated.  Use crab_msgs-srv:error_codes instead.")
  (error_codes m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<GetLegIKSolver-response>)))
    "Constants for message type '<GetLegIKSolver-response>"
  '((:IK_FOUND . 1)
    (:IK_NOT_FOUND . -1)
    (:TIMED_OUT . -2))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'GetLegIKSolver-response)))
    "Constants for message type 'GetLegIKSolver-response"
  '((:IK_FOUND . 1)
    (:IK_NOT_FOUND . -1)
    (:TIMED_OUT . -2))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetLegIKSolver-response>) ostream)
  "Serializes a message object of type '<GetLegIKSolver-response>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'target_joints))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'target_joints))
  (cl:let* ((signed (cl:slot-value msg 'error_codes)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetLegIKSolver-response>) istream)
  "Deserializes a message object of type '<GetLegIKSolver-response>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'target_joints) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'target_joints)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'crab_msgs-msg:LegJointsState))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'error_codes) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetLegIKSolver-response>)))
  "Returns string type for a service object of type '<GetLegIKSolver-response>"
  "crab_msgs/GetLegIKSolverResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetLegIKSolver-response)))
  "Returns string type for a service object of type 'GetLegIKSolver-response"
  "crab_msgs/GetLegIKSolverResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetLegIKSolver-response>)))
  "Returns md5sum for a message object of type '<GetLegIKSolver-response>"
  "aab6d4778f2d7a09001d31408c116ed3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetLegIKSolver-response)))
  "Returns md5sum for a message object of type 'GetLegIKSolver-response"
  "aab6d4778f2d7a09001d31408c116ed3")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetLegIKSolver-response>)))
  "Returns full string definition for message of type '<GetLegIKSolver-response>"
  (cl:format cl:nil "crab_msgs/LegJointsState[] target_joints~%int32 error_codes~%~%int32 IK_FOUND=1~%int32 IK_NOT_FOUND=-1~%int32 TIMED_OUT=-2~%~%~%================================================================================~%MSG: crab_msgs/LegJointsState~%float64[3] joint~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetLegIKSolver-response)))
  "Returns full string definition for message of type 'GetLegIKSolver-response"
  (cl:format cl:nil "crab_msgs/LegJointsState[] target_joints~%int32 error_codes~%~%int32 IK_FOUND=1~%int32 IK_NOT_FOUND=-1~%int32 TIMED_OUT=-2~%~%~%================================================================================~%MSG: crab_msgs/LegJointsState~%float64[3] joint~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetLegIKSolver-response>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'target_joints) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetLegIKSolver-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetLegIKSolver-response
    (cl:cons ':target_joints (target_joints msg))
    (cl:cons ':error_codes (error_codes msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetLegIKSolver)))
  'GetLegIKSolver-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetLegIKSolver)))
  'GetLegIKSolver-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetLegIKSolver)))
  "Returns string type for a service object of type '<GetLegIKSolver>"
  "crab_msgs/GetLegIKSolver")