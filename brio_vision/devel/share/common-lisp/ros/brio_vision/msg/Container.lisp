; Auto-generated. Do not edit!


(cl:in-package brio_vision-msg)


;//! \htmlinclude Container.msg.html

(cl:defclass <Container> (roslisp-msg-protocol:ros-message)
  ((date_container
    :reader date_container
    :initarg :date_container
    :type (cl:vector brio_vision-msg:Data_Type)
   :initform (cl:make-array 0 :element-type 'brio_vision-msg:Data_Type :initial-element (cl:make-instance 'brio_vision-msg:Data_Type))))
)

(cl:defclass Container (<Container>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Container>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Container)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name brio_vision-msg:<Container> is deprecated: use brio_vision-msg:Container instead.")))

(cl:ensure-generic-function 'date_container-val :lambda-list '(m))
(cl:defmethod date_container-val ((m <Container>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader brio_vision-msg:date_container-val is deprecated.  Use brio_vision-msg:date_container instead.")
  (date_container m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Container>) ostream)
  "Serializes a message object of type '<Container>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'date_container))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'date_container))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Container>) istream)
  "Deserializes a message object of type '<Container>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'date_container) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'date_container)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'brio_vision-msg:Data_Type))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Container>)))
  "Returns string type for a message object of type '<Container>"
  "brio_vision/Container")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Container)))
  "Returns string type for a message object of type 'Container"
  "brio_vision/Container")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Container>)))
  "Returns md5sum for a message object of type '<Container>"
  "e6647dd9c70ed31c9b91d70f7c90288f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Container)))
  "Returns md5sum for a message object of type 'Container"
  "e6647dd9c70ed31c9b91d70f7c90288f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Container>)))
  "Returns full string definition for message of type '<Container>"
  (cl:format cl:nil "Data_Type[] date_container~%~%================================================================================~%MSG: brio_vision/Data_Type~%int32 center_index~%int32 head_conn_index~%int32 back_conn_index~%string piece_type~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Container)))
  "Returns full string definition for message of type 'Container"
  (cl:format cl:nil "Data_Type[] date_container~%~%================================================================================~%MSG: brio_vision/Data_Type~%int32 center_index~%int32 head_conn_index~%int32 back_conn_index~%string piece_type~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Container>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'date_container) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Container>))
  "Converts a ROS message object to a list"
  (cl:list 'Container
    (cl:cons ':date_container (date_container msg))
))
