; Auto-generated. Do not edit!


(in-package coax_msgs-msg)


;//! \htmlinclude Coax3DPose.msg.html

(defclass <Coax3DPose> (ros-message)
  ((x
    :reader x-val
    :initarg :x
    :type float
    :initform 0.0)
   (y
    :reader y-val
    :initarg :y
    :type float
    :initform 0.0)
   (z
    :reader z-val
    :initarg :z
    :type float
    :initform 0.0)
   (yawrate
    :reader yawrate-val
    :initarg :yawrate
    :type float
    :initform 0.0))
)
(defmethod serialize ((msg <Coax3DPose>) ostream)
  "Serializes a message object of type '<Coax3DPose>"
  (let ((bits (roslisp-utils:encode-single-float-bits (slot-value msg 'x))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream))
  (let ((bits (roslisp-utils:encode-single-float-bits (slot-value msg 'y))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream))
  (let ((bits (roslisp-utils:encode-single-float-bits (slot-value msg 'z))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream))
  (let ((bits (roslisp-utils:encode-single-float-bits (slot-value msg 'yawrate))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream))
)
(defmethod deserialize ((msg <Coax3DPose>) istream)
  "Deserializes a message object of type '<Coax3DPose>"
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (slot-value msg 'x) (roslisp-utils:decode-single-float-bits bits)))
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (slot-value msg 'y) (roslisp-utils:decode-single-float-bits bits)))
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (slot-value msg 'z) (roslisp-utils:decode-single-float-bits bits)))
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (slot-value msg 'yawrate) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(defmethod ros-datatype ((msg (eql '<Coax3DPose>)))
  "Returns string type for a message object of type '<Coax3DPose>"
  "coax_msgs/Coax3DPose")
(defmethod md5sum ((type (eql '<Coax3DPose>)))
  "Returns md5sum for a message object of type '<Coax3DPose>"
  "1ff9ebeac8f7c7a45d6bc0fd77076ef7")
(defmethod message-definition ((type (eql '<Coax3DPose>)))
  "Returns full string definition for message of type '<Coax3DPose>"
  (format nil "float32 x~%float32 y~%float32 z~%float32 yawrate~%~%~%~%"))
(defmethod serialization-length ((msg <Coax3DPose>))
  (+ 0
     4
     4
     4
     4
))
(defmethod ros-message-to-list ((msg <Coax3DPose>))
  "Converts a ROS message object to a list"
  (list '<Coax3DPose>
    (cons ':x (x-val msg))
    (cons ':y (y-val msg))
    (cons ':z (z-val msg))
    (cons ':yawrate (yawrate-val msg))
))
