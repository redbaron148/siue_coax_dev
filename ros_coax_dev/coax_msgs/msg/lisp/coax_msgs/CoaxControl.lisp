; Auto-generated. Do not edit!


(in-package coax_msgs-msg)


;//! \htmlinclude CoaxControl.msg.html

(defclass <CoaxControl> (ros-message)
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
(defmethod serialize ((msg <CoaxControl>) ostream)
  "Serializes a message object of type '<CoaxControl>"
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
(defmethod deserialize ((msg <CoaxControl>) istream)
  "Deserializes a message object of type '<CoaxControl>"
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
(defmethod ros-datatype ((msg (eql '<CoaxControl>)))
  "Returns string type for a message object of type '<CoaxControl>"
  "coax_msgs/CoaxControl")
(defmethod md5sum ((type (eql '<CoaxControl>)))
  "Returns md5sum for a message object of type '<CoaxControl>"
  "fb03125013e0c3e7f30559de7a629a22")
(defmethod message-definition ((type (eql '<CoaxControl>)))
  "Returns full string definition for message of type '<CoaxControl>"
  (format nil "float32 roll~%float32 pitch~%float32 yaw~%float32 altitude~%~%~%"))
(defmethod serialization-length ((msg <CoaxControl>))
  (+ 0
     4
     4
     4
     4
))
(defmethod ros-message-to-list ((msg <CoaxControl>))
  "Converts a ROS message object to a list"
  (list '<CoaxControl>
    (cons ':roll (roll-val msg))
    (cons ':pitch (pitch-val msg))
    (cons ':yaw (yaw-val msg))
    (cons ':altitude (altitude-val msg))
))
