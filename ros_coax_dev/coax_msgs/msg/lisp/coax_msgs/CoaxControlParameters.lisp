; Auto-generated. Do not edit!


(in-package coax_msgs-msg)


;//! \htmlinclude CoaxControlParameters.msg.html

(defclass <CoaxControlParameters> (ros-message)
  ((baseThrust
    :reader baseThrust-val
    :initarg :baseThrust
    :type float
    :initform 0.0)
   (yawOffset
    :reader yawOffset-val
    :initarg :yawOffset
    :type float
    :initform 0.0)
   (altitudeKp
    :reader altitudeKp-val
    :initarg :altitudeKp
    :type float
    :initform 0.0)
   (altitudeKi
    :reader altitudeKi-val
    :initarg :altitudeKi
    :type float
    :initform 0.0)
   (altitudeKd
    :reader altitudeKd-val
    :initarg :altitudeKd
    :type float
    :initform 0.0)
   (yawKp
    :reader yawKp-val
    :initarg :yawKp
    :type float
    :initform 0.0)
   (yawKi
    :reader yawKi-val
    :initarg :yawKi
    :type float
    :initform 0.0)
   (yawKd
    :reader yawKd-val
    :initarg :yawKd
    :type float
    :initform 0.0))
)
(defmethod serialize ((msg <CoaxControlParameters>) ostream)
  "Serializes a message object of type '<CoaxControlParameters>"
  (let ((bits (roslisp-utils:encode-single-float-bits (slot-value msg 'baseThrust))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream))
  (let ((bits (roslisp-utils:encode-single-float-bits (slot-value msg 'yawOffset))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream))
  (let ((bits (roslisp-utils:encode-single-float-bits (slot-value msg 'altitudeKp))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream))
  (let ((bits (roslisp-utils:encode-single-float-bits (slot-value msg 'altitudeKi))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream))
  (let ((bits (roslisp-utils:encode-single-float-bits (slot-value msg 'altitudeKd))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream))
  (let ((bits (roslisp-utils:encode-single-float-bits (slot-value msg 'yawKp))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream))
  (let ((bits (roslisp-utils:encode-single-float-bits (slot-value msg 'yawKi))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream))
  (let ((bits (roslisp-utils:encode-single-float-bits (slot-value msg 'yawKd))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream))
)
(defmethod deserialize ((msg <CoaxControlParameters>) istream)
  "Deserializes a message object of type '<CoaxControlParameters>"
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (slot-value msg 'baseThrust) (roslisp-utils:decode-single-float-bits bits)))
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (slot-value msg 'yawOffset) (roslisp-utils:decode-single-float-bits bits)))
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (slot-value msg 'altitudeKp) (roslisp-utils:decode-single-float-bits bits)))
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (slot-value msg 'altitudeKi) (roslisp-utils:decode-single-float-bits bits)))
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (slot-value msg 'altitudeKd) (roslisp-utils:decode-single-float-bits bits)))
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (slot-value msg 'yawKp) (roslisp-utils:decode-single-float-bits bits)))
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (slot-value msg 'yawKi) (roslisp-utils:decode-single-float-bits bits)))
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (slot-value msg 'yawKd) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(defmethod ros-datatype ((msg (eql '<CoaxControlParameters>)))
  "Returns string type for a message object of type '<CoaxControlParameters>"
  "coax_msgs/CoaxControlParameters")
(defmethod md5sum ((type (eql '<CoaxControlParameters>)))
  "Returns md5sum for a message object of type '<CoaxControlParameters>"
  "c63db7fc4cf7a25c749b80856e6f5d4d")
(defmethod message-definition ((type (eql '<CoaxControlParameters>)))
  "Returns full string definition for message of type '<CoaxControlParameters>"
  (format nil "# Base thrust around which the control operates */~%float32 baseThrust~%# Difference of operating point between up and down motors */~%float32 yawOffset~%# Altitude gain: Kp*/~%float32 altitudeKp~%# Altitude gain: Ki*/~%float32 altitudeKi~%# Altitude gain: Kd*/~%float32 altitudeKd~%~%# Yaw gain: Kp*/~%float32 yawKp~%# Yaw gain: Ki*/~%float32 yawKi~%# Yaw gain: Kd*/~%float32 yawKd~%~%~%~%"))
(defmethod serialization-length ((msg <CoaxControlParameters>))
  (+ 0
     4
     4
     4
     4
     4
     4
     4
     4
))
(defmethod ros-message-to-list ((msg <CoaxControlParameters>))
  "Converts a ROS message object to a list"
  (list '<CoaxControlParameters>
    (cons ':baseThrust (baseThrust-val msg))
    (cons ':yawOffset (yawOffset-val msg))
    (cons ':altitudeKp (altitudeKp-val msg))
    (cons ':altitudeKi (altitudeKi-val msg))
    (cons ':altitudeKd (altitudeKd-val msg))
    (cons ':yawKp (yawKp-val msg))
    (cons ':yawKi (yawKi-val msg))
    (cons ':yawKd (yawKd-val msg))
))
