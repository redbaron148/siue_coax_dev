; Auto-generated. Do not edit!


(in-package coax_msgs-msg)


;//! \htmlinclude CoaxRawControl.msg.html

(defclass <CoaxRawControl> (ros-message)
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
(defmethod serialize ((msg <CoaxRawControl>) ostream)
  "Serializes a message object of type '<CoaxRawControl>"
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
(defmethod deserialize ((msg <CoaxRawControl>) istream)
  "Deserializes a message object of type '<CoaxRawControl>"
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
(defmethod ros-datatype ((msg (eql '<CoaxRawControl>)))
  "Returns string type for a message object of type '<CoaxRawControl>"
  "coax_msgs/CoaxRawControl")
(defmethod md5sum ((type (eql '<CoaxRawControl>)))
  "Returns md5sum for a message object of type '<CoaxRawControl>"
  "b66c77c051c7057221fbc455368c06bc")
(defmethod message-definition ((type (eql '<CoaxRawControl>)))
  "Returns full string definition for message of type '<CoaxRawControl>"
  (format nil "float32 motor1~%float32 motor2~%float32 servo1~%float32 servo2~%~%~%~%"))
(defmethod serialization-length ((msg <CoaxRawControl>))
  (+ 0
     4
     4
     4
     4
))
(defmethod ros-message-to-list ((msg <CoaxRawControl>))
  "Converts a ROS message object to a list"
  (list '<CoaxRawControl>
    (cons ':motor1 (motor1-val msg))
    (cons ':motor2 (motor2-val msg))
    (cons ':servo1 (servo1-val msg))
    (cons ':servo2 (servo2-val msg))
))
