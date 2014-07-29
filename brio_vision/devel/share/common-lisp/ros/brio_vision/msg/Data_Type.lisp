; Auto-generated. Do not edit!


(cl:in-package brio_vision-msg)


;//! \htmlinclude Data_Type.msg.html

(cl:defclass <Data_Type> (roslisp-msg-protocol:ros-message)
  ((center_index
    :reader center_index
    :initarg :center_index
    :type cl:integer
    :initform 0)
   (head_conn_index
    :reader head_conn_index
    :initarg :head_conn_index
    :type cl:integer
    :initform 0)
   (back_conn_index
    :reader back_conn_index
    :initarg :back_conn_index
    :type cl:integer
    :initform 0)
   (piece_type
    :reader piece_type
    :initarg :piece_type
    :type cl:string
    :initform ""))
)

(cl:defclass Data_Type (<Data_Type>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Data_Type>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Data_Type)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name brio_vision-msg:<Data_Type> is deprecated: use brio_vision-msg:Data_Type instead.")))

(cl:ensure-generic-function 'center_index-val :lambda-list '(m))
(cl:defmethod center_index-val ((m <Data_Type>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader brio_vision-msg:center_index-val is deprecated.  Use brio_vision-msg:center_index instead.")
  (center_index m))

(cl:ensure-generic-function 'head_conn_index-val :lambda-list '(m))
(cl:defmethod head_conn_index-val ((m <Data_Type>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader brio_vision-msg:head_conn_index-val is deprecated.  Use brio_vision-msg:head_conn_index instead.")
  (head_conn_index m))

(cl:ensure-generic-function 'back_conn_index-val :lambda-list '(m))
(cl:defmethod back_conn_index-val ((m <Data_Type>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader brio_vision-msg:back_conn_index-val is deprecated.  Use brio_vision-msg:back_conn_index instead.")
  (back_conn_index m))

(cl:ensure-generic-function 'piece_type-val :lambda-list '(m))
(cl:defmethod piece_type-val ((m <Data_Type>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader brio_vision-msg:piece_type-val is deprecated.  Use brio_vision-msg:piece_type instead.")
  (piece_type m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Data_Type>) ostream)
  "Serializes a message object of type '<Data_Type>"
  (cl:let* ((signed (cl:slot-value msg 'center_index)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'head_conn_index)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'back_conn_index)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'piece_type))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'piece_type))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Data_Type>) istream)
  "Deserializes a message object of type '<Data_Type>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'center_index) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'head_conn_index) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'back_conn_index) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'piece_type) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'piece_type) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Data_Type>)))
  "Returns string type for a message object of type '<Data_Type>"
  "brio_vision/Data_Type")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Data_Type)))
  "Returns string type for a message object of type 'Data_Type"
  "brio_vision/Data_Type")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Data_Type>)))
  "Returns md5sum for a message object of type '<Data_Type>"
  "56f44d48ac24562d5f582b0744aa0e09")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Data_Type)))
  "Returns md5sum for a message object of type 'Data_Type"
  "56f44d48ac24562d5f582b0744aa0e09")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Data_Type>)))
  "Returns full string definition for message of type '<Data_Type>"
  (cl:format cl:nil "int32 center_index~%int32 head_conn_index~%int32 back_conn_index~%string piece_type~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Data_Type)))
  "Returns full string definition for message of type 'Data_Type"
  (cl:format cl:nil "int32 center_index~%int32 head_conn_index~%int32 back_conn_index~%string piece_type~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Data_Type>))
  (cl:+ 0
     4
     4
     4
     4 (cl:length (cl:slot-value msg 'piece_type))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Data_Type>))
  "Converts a ROS message object to a list"
  (cl:list 'Data_Type
    (cl:cons ':center_index (center_index msg))
    (cl:cons ':head_conn_index (head_conn_index msg))
    (cl:cons ':back_conn_index (back_conn_index msg))
    (cl:cons ':piece_type (piece_type msg))
))
