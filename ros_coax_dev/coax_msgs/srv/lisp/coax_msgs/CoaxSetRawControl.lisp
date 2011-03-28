; Auto-generated. Do not edit!


(in-package coax_msgs-srv)


;//! \htmlinclude CoaxSetRawControl-request.msg.html

(defclass <CoaxSetRawControl-request> (ros-message)
  ((motor1
    :reader motor1-val
    :initarg :motor1
    :type float
    :initform 0.0)
   (motor2
    :reader motor2-val
    :initarg :motor2
    :type float
    :initform 0.0)
   (servo1
    :reader servo1-val
    :initarg :servo1
    :type float
    :initform 0.0)
   (servo2
    :reader servo2-val
    :initarg :servo2
    :type float
    :initform 0.0))
)
(defmethod serialize ((msg <CoaxSetRawControl-request>) ostream)
  "Serializes a message object of type '<CoaxSetRawControl-request>"
  (let ((bits (roslisp-utils:encode-single-float-bits (slot-value msg 'motor1))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream))
  (let ((bits (roslisp-utils:encode-single-float-bits (slot-value msg 'motor2))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream))
  (let ((bits (roslisp-utils:encode-single-float-bits (slot-value msg 'servo1))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream))
  (let ((bits (roslisp-utils:encode-single-float-bits (slot-value msg 'servo2))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream))
)
(defmethod deserialize ((msg <CoaxSetRawControl-request>) istream)
  "Deserializes a message object of type '<CoaxSetRawControl-request>"
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (slot-value msg 'motor1) (roslisp-utils:decode-single-float-bits bits)))
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (slot-value msg 'motor2) (roslisp-utils:decode-single-float-bits bits)))
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (slot-value msg 'servo1) (roslisp-utils:decode-single-float-bits bits)))
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (slot-value msg 'servo2) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(defmethod ros-datatype ((msg (eql '<CoaxSetRawControl-request>)))
  "Returns string type for a service object of type '<CoaxSetRawControl-request>"
  "coax_msgs/CoaxSetRawControlRequest")
(defmethod md5sum ((type (eql '<CoaxSetRawControl-request>)))
  "Returns md5sum for a message object of type '<CoaxSetRawControl-request>"
  "f5de27a093ebcc425d781d1a75a2f9d3")
(defmethod message-definition ((type (eql '<CoaxSetRawControl-request>)))
  "Returns full string definition for message of type '<CoaxSetRawControl-request>"
  (format nil "float32 motor1~%float32 motor2~%float32 servo1~%float32 servo2~%~%"))
(defmethod serialization-length ((msg <CoaxSetRawControl-request>))
  (+ 0
     4
     4
     4
     4
))
(defmethod ros-message-to-list ((msg <CoaxSetRawControl-request>))
  "Converts a ROS message object to a list"
  (list '<CoaxSetRawControl-request>
    (cons ':motor1 (motor1-val msg))
    (cons ':motor2 (motor2-val msg))
    (cons ':servo1 (servo1-val msg))
    (cons ':servo2 (servo2-val msg))
))
;//! \htmlinclude CoaxSetRawControl-response.msg.html

(defclass <CoaxSetRawControl-response> (ros-message)
  ((result
    :reader result-val
    :initarg :result
    :type fixnum
    :initform 0))
)
(defmethod serialize ((msg <CoaxSetRawControl-response>) ostream)
  "Serializes a message object of type '<CoaxSetRawControl-response>"
    (write-byte (ldb (byte 8 0) (slot-value msg 'result)) ostream)
)
(defmethod deserialize ((msg <CoaxSetRawControl-response>) istream)
  "Deserializes a message object of type '<CoaxSetRawControl-response>"
  (setf (ldb (byte 8 0) (slot-value msg 'result)) (read-byte istream))
  msg
)
(defmethod ros-datatype ((msg (eql '<CoaxSetRawControl-response>)))
  "Returns string type for a service object of type '<CoaxSetRawControl-response>"
  "coax_msgs/CoaxSetRawControlResponse")
(defmethod md5sum ((type (eql '<CoaxSetRawControl-response>)))
  "Returns md5sum for a message object of type '<CoaxSetRawControl-response>"
  "f5de27a093ebcc425d781d1a75a2f9d3")
(defmethod message-definition ((type (eql '<CoaxSetRawControl-response>)))
  "Returns full string definition for message of type '<CoaxSetRawControl-response>"
  (format nil "int8 result~%~%~%~%"))
(defmethod serialization-length ((msg <CoaxSetRawControl-response>))
  (+ 0
     1
))
(defmethod ros-message-to-list ((msg <CoaxSetRawControl-response>))
  "Converts a ROS message object to a list"
  (list '<CoaxSetRawControl-response>
    (cons ':result (result-val msg))
))
(defmethod service-request-type ((msg (eql 'CoaxSetRawControl)))
  '<CoaxSetRawControl-request>)
(defmethod service-response-type ((msg (eql 'CoaxSetRawControl)))
  '<CoaxSetRawControl-response>)
(defmethod ros-datatype ((msg (eql 'CoaxSetRawControl)))
  "Returns string type for a service object of type '<CoaxSetRawControl>"
  "coax_msgs/CoaxSetRawControl")
