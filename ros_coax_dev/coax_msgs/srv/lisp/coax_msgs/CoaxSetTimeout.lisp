; Auto-generated. Do not edit!


(in-package coax_msgs-srv)


;//! \htmlinclude CoaxSetTimeout-request.msg.html

(defclass <CoaxSetTimeout-request> (ros-message)
  ((control_timeout_ms
    :reader control_timeout_ms-val
    :initarg :control_timeout_ms
    :type fixnum
    :initform 0)
   (watchdog_timeout_ms
    :reader watchdog_timeout_ms-val
    :initarg :watchdog_timeout_ms
    :type fixnum
    :initform 0))
)
(defmethod serialize ((msg <CoaxSetTimeout-request>) ostream)
  "Serializes a message object of type '<CoaxSetTimeout-request>"
    (write-byte (ldb (byte 8 0) (slot-value msg 'control_timeout_ms)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'control_timeout_ms)) ostream)
    (write-byte (ldb (byte 8 0) (slot-value msg 'watchdog_timeout_ms)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'watchdog_timeout_ms)) ostream)
)
(defmethod deserialize ((msg <CoaxSetTimeout-request>) istream)
  "Deserializes a message object of type '<CoaxSetTimeout-request>"
  (setf (ldb (byte 8 0) (slot-value msg 'control_timeout_ms)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'control_timeout_ms)) (read-byte istream))
  (setf (ldb (byte 8 0) (slot-value msg 'watchdog_timeout_ms)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'watchdog_timeout_ms)) (read-byte istream))
  msg
)
(defmethod ros-datatype ((msg (eql '<CoaxSetTimeout-request>)))
  "Returns string type for a service object of type '<CoaxSetTimeout-request>"
  "coax_msgs/CoaxSetTimeoutRequest")
(defmethod md5sum ((type (eql '<CoaxSetTimeout-request>)))
  "Returns md5sum for a message object of type '<CoaxSetTimeout-request>"
  "717279abbcbf88c43fdc682100d3c16d")
(defmethod message-definition ((type (eql '<CoaxSetTimeout-request>)))
  "Returns full string definition for message of type '<CoaxSetTimeout-request>"
  (format nil "uint16 control_timeout_ms~%uint16 watchdog_timeout_ms~%~%"))
(defmethod serialization-length ((msg <CoaxSetTimeout-request>))
  (+ 0
     2
     2
))
(defmethod ros-message-to-list ((msg <CoaxSetTimeout-request>))
  "Converts a ROS message object to a list"
  (list '<CoaxSetTimeout-request>
    (cons ':control_timeout_ms (control_timeout_ms-val msg))
    (cons ':watchdog_timeout_ms (watchdog_timeout_ms-val msg))
))
;//! \htmlinclude CoaxSetTimeout-response.msg.html

(defclass <CoaxSetTimeout-response> (ros-message)
  ((result
    :reader result-val
    :initarg :result
    :type fixnum
    :initform 0))
)
(defmethod serialize ((msg <CoaxSetTimeout-response>) ostream)
  "Serializes a message object of type '<CoaxSetTimeout-response>"
    (write-byte (ldb (byte 8 0) (slot-value msg 'result)) ostream)
)
(defmethod deserialize ((msg <CoaxSetTimeout-response>) istream)
  "Deserializes a message object of type '<CoaxSetTimeout-response>"
  (setf (ldb (byte 8 0) (slot-value msg 'result)) (read-byte istream))
  msg
)
(defmethod ros-datatype ((msg (eql '<CoaxSetTimeout-response>)))
  "Returns string type for a service object of type '<CoaxSetTimeout-response>"
  "coax_msgs/CoaxSetTimeoutResponse")
(defmethod md5sum ((type (eql '<CoaxSetTimeout-response>)))
  "Returns md5sum for a message object of type '<CoaxSetTimeout-response>"
  "717279abbcbf88c43fdc682100d3c16d")
(defmethod message-definition ((type (eql '<CoaxSetTimeout-response>)))
  "Returns full string definition for message of type '<CoaxSetTimeout-response>"
  (format nil "int8 result~%~%~%~%"))
(defmethod serialization-length ((msg <CoaxSetTimeout-response>))
  (+ 0
     1
))
(defmethod ros-message-to-list ((msg <CoaxSetTimeout-response>))
  "Converts a ROS message object to a list"
  (list '<CoaxSetTimeout-response>
    (cons ':result (result-val msg))
))
(defmethod service-request-type ((msg (eql 'CoaxSetTimeout)))
  '<CoaxSetTimeout-request>)
(defmethod service-response-type ((msg (eql 'CoaxSetTimeout)))
  '<CoaxSetTimeout-response>)
(defmethod ros-datatype ((msg (eql 'CoaxSetTimeout)))
  "Returns string type for a service object of type '<CoaxSetTimeout>"
  "coax_msgs/CoaxSetTimeout")
