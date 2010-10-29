; Auto-generated. Do not edit!


(in-package coax_msgs-srv)


;//! \htmlinclude CoaxConfigureComm-request.msg.html

(defclass <CoaxConfigureComm-request> (ros-message)
  ((commMode
    :reader commMode-val
    :initarg :commMode
    :type fixnum
    :initform 0)
   (frequency
    :reader frequency-val
    :initarg :frequency
    :type fixnum
    :initform 0)
   (numMessages
    :reader numMessages-val
    :initarg :numMessages
    :type fixnum
    :initform 0)
   (contents
    :reader contents-val
    :initarg :contents
    :type integer
    :initform 0))
)
(defmethod serialize ((msg <CoaxConfigureComm-request>) ostream)
  "Serializes a message object of type '<CoaxConfigureComm-request>"
    (write-byte (ldb (byte 8 0) (slot-value msg 'commMode)) ostream)
    (write-byte (ldb (byte 8 0) (slot-value msg 'frequency)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'frequency)) ostream)
    (write-byte (ldb (byte 8 0) (slot-value msg 'numMessages)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'numMessages)) ostream)
    (write-byte (ldb (byte 8 0) (slot-value msg 'contents)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'contents)) ostream)
  (write-byte (ldb (byte 8 16) (slot-value msg 'contents)) ostream)
  (write-byte (ldb (byte 8 24) (slot-value msg 'contents)) ostream)
)
(defmethod deserialize ((msg <CoaxConfigureComm-request>) istream)
  "Deserializes a message object of type '<CoaxConfigureComm-request>"
  (setf (ldb (byte 8 0) (slot-value msg 'commMode)) (read-byte istream))
  (setf (ldb (byte 8 0) (slot-value msg 'frequency)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'frequency)) (read-byte istream))
  (setf (ldb (byte 8 0) (slot-value msg 'numMessages)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'numMessages)) (read-byte istream))
  (setf (ldb (byte 8 0) (slot-value msg 'contents)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'contents)) (read-byte istream))
  (setf (ldb (byte 8 16) (slot-value msg 'contents)) (read-byte istream))
  (setf (ldb (byte 8 24) (slot-value msg 'contents)) (read-byte istream))
  msg
)
(defmethod ros-datatype ((msg (eql '<CoaxConfigureComm-request>)))
  "Returns string type for a service object of type '<CoaxConfigureComm-request>"
  "coax_msgs/CoaxConfigureCommRequest")
(defmethod md5sum ((type (eql '<CoaxConfigureComm-request>)))
  "Returns md5sum for a message object of type '<CoaxConfigureComm-request>"
  "0a4ffc01e802a3c95dac462243d2bddb")
(defmethod message-definition ((type (eql '<CoaxConfigureComm-request>)))
  "Returns full string definition for message of type '<CoaxConfigureComm-request>"
  (format nil "uint8 commMode~%uint16 frequency~%uint16 numMessages~%uint32 contents~%~%"))
(defmethod serialization-length ((msg <CoaxConfigureComm-request>))
  (+ 0
     1
     2
     2
     4
))
(defmethod ros-message-to-list ((msg <CoaxConfigureComm-request>))
  "Converts a ROS message object to a list"
  (list '<CoaxConfigureComm-request>
    (cons ':commMode (commMode-val msg))
    (cons ':frequency (frequency-val msg))
    (cons ':numMessages (numMessages-val msg))
    (cons ':contents (contents-val msg))
))
;//! \htmlinclude CoaxConfigureComm-response.msg.html

(defclass <CoaxConfigureComm-response> (ros-message)
  ((result
    :reader result-val
    :initarg :result
    :type fixnum
    :initform 0))
)
(defmethod serialize ((msg <CoaxConfigureComm-response>) ostream)
  "Serializes a message object of type '<CoaxConfigureComm-response>"
    (write-byte (ldb (byte 8 0) (slot-value msg 'result)) ostream)
)
(defmethod deserialize ((msg <CoaxConfigureComm-response>) istream)
  "Deserializes a message object of type '<CoaxConfigureComm-response>"
  (setf (ldb (byte 8 0) (slot-value msg 'result)) (read-byte istream))
  msg
)
(defmethod ros-datatype ((msg (eql '<CoaxConfigureComm-response>)))
  "Returns string type for a service object of type '<CoaxConfigureComm-response>"
  "coax_msgs/CoaxConfigureCommResponse")
(defmethod md5sum ((type (eql '<CoaxConfigureComm-response>)))
  "Returns md5sum for a message object of type '<CoaxConfigureComm-response>"
  "0a4ffc01e802a3c95dac462243d2bddb")
(defmethod message-definition ((type (eql '<CoaxConfigureComm-response>)))
  "Returns full string definition for message of type '<CoaxConfigureComm-response>"
  (format nil "int8 result~%~%~%~%"))
(defmethod serialization-length ((msg <CoaxConfigureComm-response>))
  (+ 0
     1
))
(defmethod ros-message-to-list ((msg <CoaxConfigureComm-response>))
  "Converts a ROS message object to a list"
  (list '<CoaxConfigureComm-response>
    (cons ':result (result-val msg))
))
(defmethod service-request-type ((msg (eql 'CoaxConfigureComm)))
  '<CoaxConfigureComm-request>)
(defmethod service-response-type ((msg (eql 'CoaxConfigureComm)))
  '<CoaxConfigureComm-response>)
(defmethod ros-datatype ((msg (eql 'CoaxConfigureComm)))
  "Returns string type for a service object of type '<CoaxConfigureComm>"
  "coax_msgs/CoaxConfigureComm")
