; Auto-generated. Do not edit!


(in-package coax_msgs-msg)


;//! \htmlinclude CoaxTrimMode.msg.html

(defclass <CoaxTrimMode> (ros-message)
  ((trimMode
    :reader trimMode-val
    :initarg :trimMode
    :type fixnum
    :initform 0)
   (rollTrim
    :reader rollTrim-val
    :initarg :rollTrim
    :type float
    :initform 0.0)
   (pitchTrim
    :reader pitchTrim-val
    :initarg :pitchTrim
    :type float
    :initform 0.0))
)
(defmethod serialize ((msg <CoaxTrimMode>) ostream)
  "Serializes a message object of type '<CoaxTrimMode>"
    (write-byte (ldb (byte 8 0) (slot-value msg 'trimMode)) ostream)
  (let ((bits (roslisp-utils:encode-single-float-bits (slot-value msg 'rollTrim))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream))
  (let ((bits (roslisp-utils:encode-single-float-bits (slot-value msg 'pitchTrim))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream))
)
(defmethod deserialize ((msg <CoaxTrimMode>) istream)
  "Deserializes a message object of type '<CoaxTrimMode>"
  (setf (ldb (byte 8 0) (slot-value msg 'trimMode)) (read-byte istream))
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (slot-value msg 'rollTrim) (roslisp-utils:decode-single-float-bits bits)))
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (slot-value msg 'pitchTrim) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(defmethod ros-datatype ((msg (eql '<CoaxTrimMode>)))
  "Returns string type for a message object of type '<CoaxTrimMode>"
  "coax_msgs/CoaxTrimMode")
(defmethod md5sum ((type (eql '<CoaxTrimMode>)))
  "Returns md5sum for a message object of type '<CoaxTrimMode>"
  "0315cef579f853e633299fb30b9c35c1")
(defmethod message-definition ((type (eql '<CoaxTrimMode>)))
  "Returns full string definition for message of type '<CoaxTrimMode>"
  (format nil "# Trim mode, can be SB_TRIM_FROM_RC or SB_TRIM_SOFTWARE */~%uint8 trimMode ~%# Trim position for the roll axis */~%float32 rollTrim~%# Trim position for the pitch axis */~%float32 pitchTrim~%~%~%~%"))
(defmethod serialization-length ((msg <CoaxTrimMode>))
  (+ 0
     1
     4
     4
))
(defmethod ros-message-to-list ((msg <CoaxTrimMode>))
  "Converts a ROS message object to a list"
  (list '<CoaxTrimMode>
    (cons ':trimMode (trimMode-val msg))
    (cons ':rollTrim (rollTrim-val msg))
    (cons ':pitchTrim (pitchTrim-val msg))
))
