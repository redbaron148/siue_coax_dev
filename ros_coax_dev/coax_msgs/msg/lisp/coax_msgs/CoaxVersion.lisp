; Auto-generated. Do not edit!


(in-package coax_msgs-msg)


;//! \htmlinclude CoaxVersion.msg.html

(defclass <CoaxVersion> (ros-message)
  ((apiVersion
    :reader apiVersion-val
    :initarg :apiVersion
    :type fixnum
    :initform 0)
   (controllerVersion
    :reader controllerVersion-val
    :initarg :controllerVersion
    :type fixnum
    :initform 0)
   (imuVersion
    :reader imuVersion-val
    :initarg :imuVersion
    :type string
    :initform "")
   (compileTime
    :reader compileTime-val
    :initarg :compileTime
    :type string
    :initform ""))
)
(defmethod serialize ((msg <CoaxVersion>) ostream)
  "Serializes a message object of type '<CoaxVersion>"
    (write-byte (ldb (byte 8 0) (slot-value msg 'apiVersion)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'apiVersion)) ostream)
    (write-byte (ldb (byte 8 0) (slot-value msg 'controllerVersion)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'controllerVersion)) ostream)
  (let ((__ros_str_len (length (slot-value msg 'imuVersion))))
    (write-byte (ldb (byte 8 0) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 8) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 16) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 24) __ros_str_len) ostream))
  (map nil #'(lambda (c) (write-byte (char-code c) ostream)) (slot-value msg 'imuVersion))
  (let ((__ros_str_len (length (slot-value msg 'compileTime))))
    (write-byte (ldb (byte 8 0) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 8) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 16) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 24) __ros_str_len) ostream))
  (map nil #'(lambda (c) (write-byte (char-code c) ostream)) (slot-value msg 'compileTime))
)
(defmethod deserialize ((msg <CoaxVersion>) istream)
  "Deserializes a message object of type '<CoaxVersion>"
  (setf (ldb (byte 8 0) (slot-value msg 'apiVersion)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'apiVersion)) (read-byte istream))
  (setf (ldb (byte 8 0) (slot-value msg 'controllerVersion)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'controllerVersion)) (read-byte istream))
  (let ((__ros_str_len 0))
    (setf (ldb (byte 8 0) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 8) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 16) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 24) __ros_str_len) (read-byte istream))
    (setf (slot-value msg 'imuVersion) (make-string __ros_str_len))
    (dotimes (__ros_str_idx __ros_str_len msg)
      (setf (char (slot-value msg 'imuVersion) __ros_str_idx) (code-char (read-byte istream)))))
  (let ((__ros_str_len 0))
    (setf (ldb (byte 8 0) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 8) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 16) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 24) __ros_str_len) (read-byte istream))
    (setf (slot-value msg 'compileTime) (make-string __ros_str_len))
    (dotimes (__ros_str_idx __ros_str_len msg)
      (setf (char (slot-value msg 'compileTime) __ros_str_idx) (code-char (read-byte istream)))))
  msg
)
(defmethod ros-datatype ((msg (eql '<CoaxVersion>)))
  "Returns string type for a message object of type '<CoaxVersion>"
  "coax_msgs/CoaxVersion")
(defmethod md5sum ((type (eql '<CoaxVersion>)))
  "Returns md5sum for a message object of type '<CoaxVersion>"
  "614a2e1fe797a467bac6f721b97ec4c0")
(defmethod message-definition ((type (eql '<CoaxVersion>)))
  "Returns full string definition for message of type '<CoaxVersion>"
  (format nil "uint16 apiVersion~%uint16 controllerVersion~%string imuVersion~%string compileTime~%~%~%"))
(defmethod serialization-length ((msg <CoaxVersion>))
  (+ 0
     2
     2
     4 (length (slot-value msg 'imuVersion))
     4 (length (slot-value msg 'compileTime))
))
(defmethod ros-message-to-list ((msg <CoaxVersion>))
  "Converts a ROS message object to a list"
  (list '<CoaxVersion>
    (cons ':apiVersion (apiVersion-val msg))
    (cons ':controllerVersion (controllerVersion-val msg))
    (cons ':imuVersion (imuVersion-val msg))
    (cons ':compileTime (compileTime-val msg))
))
