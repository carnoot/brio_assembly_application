; Auto-generated. Do not edit!


(cl:in-package brio_vision-srv)


;//! \htmlinclude Is_This_Easier-request.msg.html

(cl:defclass <Is_This_Easier-request> (roslisp-msg-protocol:ros-message)
  ((response_status
    :reader response_status
    :initarg :response_status
    :type cl:string
    :initform ""))
)

(cl:defclass Is_This_Easier-request (<Is_This_Easier-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Is_This_Easier-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Is_This_Easier-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name brio_vision-srv:<Is_This_Easier-request> is deprecated: use brio_vision-srv:Is_This_Easier-request instead.")))

(cl:ensure-generic-function 'response_status-val :lambda-list '(m))
(cl:defmethod response_status-val ((m <Is_This_Easier-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader brio_vision-srv:response_status-val is deprecated.  Use brio_vision-srv:response_status instead.")
  (response_status m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Is_This_Easier-request>) ostream)
  "Serializes a message object of type '<Is_This_Easier-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'response_status))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'response_status))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Is_This_Easier-request>) istream)
  "Deserializes a message object of type '<Is_This_Easier-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'response_status) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'response_status) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Is_This_Easier-request>)))
  "Returns string type for a service object of type '<Is_This_Easier-request>"
  "brio_vision/Is_This_EasierRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Is_This_Easier-request)))
  "Returns string type for a service object of type 'Is_This_Easier-request"
  "brio_vision/Is_This_EasierRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Is_This_Easier-request>)))
  "Returns md5sum for a message object of type '<Is_This_Easier-request>"
  "b793477c3837845ccb4fa4623a02615d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Is_This_Easier-request)))
  "Returns md5sum for a message object of type 'Is_This_Easier-request"
  "b793477c3837845ccb4fa4623a02615d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Is_This_Easier-request>)))
  "Returns full string definition for message of type '<Is_This_Easier-request>"
  (cl:format cl:nil "string response_status~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Is_This_Easier-request)))
  "Returns full string definition for message of type 'Is_This_Easier-request"
  (cl:format cl:nil "string response_status~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Is_This_Easier-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'response_status))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Is_This_Easier-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Is_This_Easier-request
    (cl:cons ':response_status (response_status msg))
))
;//! \htmlinclude Is_This_Easier-response.msg.html

(cl:defclass <Is_This_Easier-response> (roslisp-msg-protocol:ros-message)
  ((get_message
    :reader get_message
    :initarg :get_message
    :type brio_vision-msg:Container
    :initform (cl:make-instance 'brio_vision-msg:Container)))
)

(cl:defclass Is_This_Easier-response (<Is_This_Easier-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Is_This_Easier-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Is_This_Easier-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name brio_vision-srv:<Is_This_Easier-response> is deprecated: use brio_vision-srv:Is_This_Easier-response instead.")))

(cl:ensure-generic-function 'get_message-val :lambda-list '(m))
(cl:defmethod get_message-val ((m <Is_This_Easier-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader brio_vision-srv:get_message-val is deprecated.  Use brio_vision-srv:get_message instead.")
  (get_message m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Is_This_Easier-response>) ostream)
  "Serializes a message object of type '<Is_This_Easier-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'get_message) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Is_This_Easier-response>) istream)
  "Deserializes a message object of type '<Is_This_Easier-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'get_message) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Is_This_Easier-response>)))
  "Returns string type for a service object of type '<Is_This_Easier-response>"
  "brio_vision/Is_This_EasierResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Is_This_Easier-response)))
  "Returns string type for a service object of type 'Is_This_Easier-response"
  "brio_vision/Is_This_EasierResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Is_This_Easier-response>)))
  "Returns md5sum for a message object of type '<Is_This_Easier-response>"
  "b793477c3837845ccb4fa4623a02615d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Is_This_Easier-response)))
  "Returns md5sum for a message object of type 'Is_This_Easier-response"
  "b793477c3837845ccb4fa4623a02615d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Is_This_Easier-response>)))
  "Returns full string definition for message of type '<Is_This_Easier-response>"
  (cl:format cl:nil "brio_vision/Container get_message~%~%~%================================================================================~%MSG: brio_vision/Container~%Data_Type[] date_container~%~%================================================================================~%MSG: brio_vision/Data_Type~%int32 center_index~%int32 head_conn_index~%int32 back_conn_index~%string piece_type~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Is_This_Easier-response)))
  "Returns full string definition for message of type 'Is_This_Easier-response"
  (cl:format cl:nil "brio_vision/Container get_message~%~%~%================================================================================~%MSG: brio_vision/Container~%Data_Type[] date_container~%~%================================================================================~%MSG: brio_vision/Data_Type~%int32 center_index~%int32 head_conn_index~%int32 back_conn_index~%string piece_type~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Is_This_Easier-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'get_message))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Is_This_Easier-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Is_This_Easier-response
    (cl:cons ':get_message (get_message msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Is_This_Easier)))
  'Is_This_Easier-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Is_This_Easier)))
  'Is_This_Easier-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Is_This_Easier)))
  "Returns string type for a service object of type '<Is_This_Easier>"
  "brio_vision/Is_This_Easier")