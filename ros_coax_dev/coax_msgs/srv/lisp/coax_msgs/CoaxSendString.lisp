; Auto-generated. Do not edit!


(in-package coax_msgs-srv)


;//! \htmlinclude CoaxSendString-request.msg.html

(defclass <CoaxSendString-request> (ros-message)
  ((text
    :reader text-val
    :initarg :text
    :type string
    :initform ""))
)
(defmethod serialize ((msg <CoaxSendString-request>) ostream)
  "Serializes a message object of type '<CoaxSendString-request>"
  (let ((__ros_str_len (length (slot-value msg 'text))))
    (write-byte (ldb (byte 8 0) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 8) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 16) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 24) __ros_str_len) ostream))
  (map nil #'(lambda (c) (write-byte (char-code c) ostream)) (slot-value msg 'text))
)
(defmethod deserialize ((msg <CoaxSendString-request>) istream)
  "Deserializes a message object of type '<CoaxSendString-request>"
  (let ((__ros_str_len 0))
    (setf (ldb (byte 8 0) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 8) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 16) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 24) __ros_str_len) (read-byte istream))
    (setf (slot-value msg 'text) (make-string __ros_str_len))
    (dotimes (__ros_str_idx __ros_str_len msg)
      (setf (char (slot-value msg 'text) __ros_str_idx) (code-char (read-byte istream)))))
  msg
)
(defmethod ros-datatype ((msg (eql '<CoaxSendString-request>)))
  "Returns string type for a service object of type '<CoaxSendString-request>"
  "coax_msgs/CoaxSendStringRequest")
(defmethod md5sum ((type (eql '<CoaxSendString-request>)))
  "Returns md5sum for a message object of type '<CoaxSendString-request>"
  "05354734935e371f83dc4d09f1c13d77")
(defmethod message-definition ((type (eql '<CoaxSendString-request>)))
  "Returns full string definition for message of type '<CoaxSendString-request>"
  (format nil "string text~%~%"))
(defmethod serialization-length ((msg <CoaxSendString-request>))
  (+ 0
     4 (length (slot-value msg 'text))
))
(defmethod ros-message-to-list ((msg <CoaxSendString-request>))
  "Converts a ROS message object to a list"
  (list '<CoaxSendString-request>
    (cons ':text (text-val msg))
))
;//! \htmlinclude CoaxSendString-response.msg.html

(defclass <CoaxSendString-response> (ros-message)
  ((result
    :reader result-val
    :initarg :result
    :type fixnum
    :initform 0))
)
(defmethod serialize ((msg <CoaxSendString-response>) ostream)
  "Serializes a message object of type '<CoaxSendString-response>"
    (write-byte (ldb (byte 8 0) (slot-value msg 'result)) ostream)
)
(defmethod deserialize ((msg <CoaxSendString-response>) istream)
  "Deserializes a message object of type '<CoaxSendString-response>"
  (setf (ldb (byte 8 0) (slot-value msg 'result)) (read-byte istream))
  msg
)
(defmethod ros-datatype ((msg (eql '<CoaxSendString-response>)))
  "Returns string type for a service object of type '<CoaxSendString-response>"
  "coax_msgs/CoaxSendStringResponse")
(defmethod md5sum ((type (eql '<CoaxSendString-response>)))
  "Returns md5sum for a message object of type '<CoaxSendString-response>"
  "05354734935e371f83dc4d09f1c13d77")
(defmethod message-definition ((type (eql '<CoaxSendString-response>)))
  "Returns full string definition for message of type '<CoaxSendString-response>"
  (format nil "int8 result~%~%~%~%"))
(defmethod serialization-length ((msg <CoaxSendString-response>))
  (+ 0
     1
))
(defmethod ros-message-to-list ((msg <CoaxSendString-response>))
  "Converts a ROS message object to a list"
  (list '<CoaxSendString-response>
    (cons ':result (result-val msg))
))
(defmethod service-request-type ((msg (eql 'CoaxSendString)))
  '<CoaxSendString-request>)
(defmethod service-response-type ((msg (eql 'CoaxSendString)))
  '<CoaxSendString-response>)
(defmethod ros-datatype ((msg (eql 'CoaxSendString)))
  "Returns string type for a service object of type '<CoaxSendString>"
  "coax_msgs/CoaxSendString")
