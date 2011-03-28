; Auto-generated. Do not edit!


(in-package coax_msgs-msg)


;//! \htmlinclude CoaxSpeed.msg.html

(defclass <CoaxSpeed> (ros-message)
  ((state
    :reader state-val
    :initarg :state
    :type fixnum
    :initform 0)
   (light
    :reader light-val
    :initarg :light
    :type fixnum
    :initform 0)
   (vel_x
    :reader vel_x-val
    :initarg :vel_x
    :type float
    :initform 0.0)
   (vel_y
    :reader vel_y-val
    :initarg :vel_y
    :type float
    :initform 0.0))
)
(defmethod serialize ((msg <CoaxSpeed>) ostream)
  "Serializes a message object of type '<CoaxSpeed>"
    (write-byte (ldb (byte 8 0) (slot-value msg 'state)) ostream)
    (write-byte (ldb (byte 8 0) (slot-value msg 'light)) ostream)
  (let ((bits (roslisp-utils:encode-single-float-bits (slot-value msg 'vel_x))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream))
  (let ((bits (roslisp-utils:encode-single-float-bits (slot-value msg 'vel_y))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream))
)
(defmethod deserialize ((msg <CoaxSpeed>) istream)
  "Deserializes a message object of type '<CoaxSpeed>"
  (setf (ldb (byte 8 0) (slot-value msg 'state)) (read-byte istream))
  (setf (ldb (byte 8 0) (slot-value msg 'light)) (read-byte istream))
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (slot-value msg 'vel_x) (roslisp-utils:decode-single-float-bits bits)))
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (slot-value msg 'vel_y) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(defmethod ros-datatype ((msg (eql '<CoaxSpeed>)))
  "Returns string type for a message object of type '<CoaxSpeed>"
  "coax_msgs/CoaxSpeed")
(defmethod md5sum ((type (eql '<CoaxSpeed>)))
  "Returns md5sum for a message object of type '<CoaxSpeed>"
  "c66c2b729ae81addf32019af30f8271e")
(defmethod message-definition ((type (eql '<CoaxSpeed>)))
  "Returns full string definition for message of type '<CoaxSpeed>"
  (format nil "~%uint8 state~%uint8 light~%float32 vel_x~%float32 vel_y~%~%~%~%"))
(defmethod serialization-length ((msg <CoaxSpeed>))
  (+ 0
     1
     1
     4
     4
))
(defmethod ros-message-to-list ((msg <CoaxSpeed>))
  "Converts a ROS message object to a list"
  (list '<CoaxSpeed>
    (cons ':state (state-val msg))
    (cons ':light (light-val msg))
    (cons ':vel_x (vel_x-val msg))
    (cons ':vel_y (vel_y-val msg))
))
