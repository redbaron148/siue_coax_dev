; Auto-generated. Do not edit!


(in-package coax_msgs-srv)


;//! \htmlinclude CoaxSetControl-request.msg.html

(defclass <CoaxSetControl-request> (ros-message)
  ((roll
    :reader roll-val
    :initarg :roll
    :type float
    :initform 0.0)
   (pitch
    :reader pitch-val
    :initarg :pitch
    :type float
    :initform 0.0)
   (yaw
    :reader yaw-val
    :initarg :yaw
    :type float
    :initform 0.0)
   (altitude
    :reader altitude-val
    :initarg :altitude
    :type float
    :initform 0.0))
)
(defmethod serialize ((msg <CoaxSetControl-request>) ostream)
  "Serializes a message object of type '<CoaxSetControl-request>"
  (let ((bits (roslisp-utils:encode-single-float-bits (slot-value msg 'roll))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream))
  (let ((bits (roslisp-utils:encode-single-float-bits (slot-value msg 'pitch))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream))
  (let ((bits (roslisp-utils:encode-single-float-bits (slot-value msg 'yaw))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream))
  (let ((bits (roslisp-utils:encode-single-float-bits (slot-value msg 'altitude))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream))
)
(defmethod deserialize ((msg <CoaxSetControl-request>) istream)
  "Deserializes a message object of type '<CoaxSetControl-request>"
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (slot-value msg 'roll) (roslisp-utils:decode-single-float-bits bits)))
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (slot-value msg 'pitch) (roslisp-utils:decode-single-float-bits bits)))
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (slot-value msg 'yaw) (roslisp-utils:decode-single-float-bits bits)))
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (slot-value msg 'altitude) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(defmethod ros-datatype ((msg (eql '<CoaxSetControl-request>)))
  "Returns string type for a service object of type '<CoaxSetControl-request>"
  "coax_msgs/CoaxSetControlRequest")
(defmethod md5sum ((type (eql '<CoaxSetControl-request>)))
  "Returns md5sum for a message object of type '<CoaxSetControl-request>"
  "3462c8168747cd838501461513fced34")
(defmethod message-definition ((type (eql '<CoaxSetControl-request>)))
  "Returns full string definition for message of type '<CoaxSetControl-request>"
  (format nil "float32 roll~%float32 pitch~%float32 yaw~%float32 altitude~%~%"))
(defmethod serialization-length ((msg <CoaxSetControl-request>))
  (+ 0
     4
     4
     4
     4
))
(defmethod ros-message-to-list ((msg <CoaxSetControl-request>))
  "Converts a ROS message object to a list"
  (list '<CoaxSetControl-request>
    (cons ':roll (roll-val msg))
    (cons ':pitch (pitch-val msg))
    (cons ':yaw (yaw-val msg))
    (cons ':altitude (altitude-val msg))
))
;//! \htmlinclude CoaxSetControl-response.msg.html

(defclass <CoaxSetControl-response> (ros-message)
  ((result
    :reader result-val
    :initarg :result
    :type fixnum
    :initform 0))
)
(defmethod serialize ((msg <CoaxSetControl-response>) ostream)
  "Serializes a message object of type '<CoaxSetControl-response>"
    (write-byte (ldb (byte 8 0) (slot-value msg 'result)) ostream)
)
(defmethod deserialize ((msg <CoaxSetControl-response>) istream)
  "Deserializes a message object of type '<CoaxSetControl-response>"
  (setf (ldb (byte 8 0) (slot-value msg 'result)) (read-byte istream))
  msg
)
(defmethod ros-datatype ((msg (eql '<CoaxSetControl-response>)))
  "Returns string type for a service object of type '<CoaxSetControl-response>"
  "coax_msgs/CoaxSetControlResponse")
(defmethod md5sum ((type (eql '<CoaxSetControl-response>)))
  "Returns md5sum for a message object of type '<CoaxSetControl-response>"
  "3462c8168747cd838501461513fced34")
(defmethod message-definition ((type (eql '<CoaxSetControl-response>)))
  "Returns full string definition for message of type '<CoaxSetControl-response>"
  (format nil "int8 result~%~%~%~%"))
(defmethod serialization-length ((msg <CoaxSetControl-response>))
  (+ 0
     1
))
(defmethod ros-message-to-list ((msg <CoaxSetControl-response>))
  "Converts a ROS message object to a list"
  (list '<CoaxSetControl-response>
    (cons ':result (result-val msg))
))
(defmethod service-request-type ((msg (eql 'CoaxSetControl)))
  '<CoaxSetControl-request>)
(defmethod service-response-type ((msg (eql 'CoaxSetControl)))
  '<CoaxSetControl-response>)
(defmethod ros-datatype ((msg (eql 'CoaxSetControl)))
  "Returns string type for a service object of type '<CoaxSetControl>"
  "coax_msgs/CoaxSetControl")
