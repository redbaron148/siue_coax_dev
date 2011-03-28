; Auto-generated. Do not edit!


(in-package coax_msgs-srv)


;//! \htmlinclude CoaxSetAckMode-request.msg.html

(defclass <CoaxSetAckMode-request> (ros-message)
  ((mode
    :reader mode-val
    :initarg :mode
    :type fixnum
    :initform 0))
)
(defmethod serialize ((msg <CoaxSetAckMode-request>) ostream)
  "Serializes a message object of type '<CoaxSetAckMode-request>"
    (write-byte (ldb (byte 8 0) (slot-value msg 'mode)) ostream)
)
(defmethod deserialize ((msg <CoaxSetAckMode-request>) istream)
  "Deserializes a message object of type '<CoaxSetAckMode-request>"
  (setf (ldb (byte 8 0) (slot-value msg 'mode)) (read-byte istream))
  msg
)
(defmethod ros-datatype ((msg (eql '<CoaxSetAckMode-request>)))
  "Returns string type for a service object of type '<CoaxSetAckMode-request>"
  "coax_msgs/CoaxSetAckModeRequest")
(defmethod md5sum ((type (eql '<CoaxSetAckMode-request>)))
  "Returns md5sum for a message object of type '<CoaxSetAckMode-request>"
  "4cc5eeb93a9b2000ef0e38bcc7d41dcc")
(defmethod message-definition ((type (eql '<CoaxSetAckMode-request>)))
  "Returns full string definition for message of type '<CoaxSetAckMode-request>"
  (format nil "uint8 mode~%~%"))
(defmethod serialization-length ((msg <CoaxSetAckMode-request>))
  (+ 0
     1
))
(defmethod ros-message-to-list ((msg <CoaxSetAckMode-request>))
  "Converts a ROS message object to a list"
  (list '<CoaxSetAckMode-request>
    (cons ':mode (mode-val msg))
))
;//! \htmlinclude CoaxSetAckMode-response.msg.html

(defclass <CoaxSetAckMode-response> (ros-message)
  ((result
    :reader result-val
    :initarg :result
    :type fixnum
    :initform 0))
)
(defmethod serialize ((msg <CoaxSetAckMode-response>) ostream)
  "Serializes a message object of type '<CoaxSetAckMode-response>"
    (write-byte (ldb (byte 8 0) (slot-value msg 'result)) ostream)
)
(defmethod deserialize ((msg <CoaxSetAckMode-response>) istream)
  "Deserializes a message object of type '<CoaxSetAckMode-response>"
  (setf (ldb (byte 8 0) (slot-value msg 'result)) (read-byte istream))
  msg
)
(defmethod ros-datatype ((msg (eql '<CoaxSetAckMode-response>)))
  "Returns string type for a service object of type '<CoaxSetAckMode-response>"
  "coax_msgs/CoaxSetAckModeResponse")
(defmethod md5sum ((type (eql '<CoaxSetAckMode-response>)))
  "Returns md5sum for a message object of type '<CoaxSetAckMode-response>"
  "4cc5eeb93a9b2000ef0e38bcc7d41dcc")
(defmethod message-definition ((type (eql '<CoaxSetAckMode-response>)))
  "Returns full string definition for message of type '<CoaxSetAckMode-response>"
  (format nil "int8 result~%~%~%"))
(defmethod serialization-length ((msg <CoaxSetAckMode-response>))
  (+ 0
     1
))
(defmethod ros-message-to-list ((msg <CoaxSetAckMode-response>))
  "Converts a ROS message object to a list"
  (list '<CoaxSetAckMode-response>
    (cons ':result (result-val msg))
))
(defmethod service-request-type ((msg (eql 'CoaxSetAckMode)))
  '<CoaxSetAckMode-request>)
(defmethod service-response-type ((msg (eql 'CoaxSetAckMode)))
  '<CoaxSetAckMode-response>)
(defmethod ros-datatype ((msg (eql 'CoaxSetAckMode)))
  "Returns string type for a service object of type '<CoaxSetAckMode>"
  "coax_msgs/CoaxSetAckMode")
