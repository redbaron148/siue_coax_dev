; Auto-generated. Do not edit!


(in-package coax_msgs-srv)


;//! \htmlinclude CoaxConfigureOAMode-request.msg.html

(defclass <CoaxConfigureOAMode-request> (ros-message)
  ((oavoidMode
    :reader oavoidMode-val
    :initarg :oavoidMode
    :type fixnum
    :initform 0))
)
(defmethod serialize ((msg <CoaxConfigureOAMode-request>) ostream)
  "Serializes a message object of type '<CoaxConfigureOAMode-request>"
    (write-byte (ldb (byte 8 0) (slot-value msg 'oavoidMode)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'oavoidMode)) ostream)
)
(defmethod deserialize ((msg <CoaxConfigureOAMode-request>) istream)
  "Deserializes a message object of type '<CoaxConfigureOAMode-request>"
  (setf (ldb (byte 8 0) (slot-value msg 'oavoidMode)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'oavoidMode)) (read-byte istream))
  msg
)
(defmethod ros-datatype ((msg (eql '<CoaxConfigureOAMode-request>)))
  "Returns string type for a service object of type '<CoaxConfigureOAMode-request>"
  "coax_msgs/CoaxConfigureOAModeRequest")
(defmethod md5sum ((type (eql '<CoaxConfigureOAMode-request>)))
  "Returns md5sum for a message object of type '<CoaxConfigureOAMode-request>"
  "c0668a37c3c443907202b387475fec72")
(defmethod message-definition ((type (eql '<CoaxConfigureOAMode-request>)))
  "Returns full string definition for message of type '<CoaxConfigureOAMode-request>"
  (format nil "uint16 oavoidMode~%~%"))
(defmethod serialization-length ((msg <CoaxConfigureOAMode-request>))
  (+ 0
     2
))
(defmethod ros-message-to-list ((msg <CoaxConfigureOAMode-request>))
  "Converts a ROS message object to a list"
  (list '<CoaxConfigureOAMode-request>
    (cons ':oavoidMode (oavoidMode-val msg))
))
;//! \htmlinclude CoaxConfigureOAMode-response.msg.html

(defclass <CoaxConfigureOAMode-response> (ros-message)
  ((result
    :reader result-val
    :initarg :result
    :type fixnum
    :initform 0))
)
(defmethod serialize ((msg <CoaxConfigureOAMode-response>) ostream)
  "Serializes a message object of type '<CoaxConfigureOAMode-response>"
    (write-byte (ldb (byte 8 0) (slot-value msg 'result)) ostream)
)
(defmethod deserialize ((msg <CoaxConfigureOAMode-response>) istream)
  "Deserializes a message object of type '<CoaxConfigureOAMode-response>"
  (setf (ldb (byte 8 0) (slot-value msg 'result)) (read-byte istream))
  msg
)
(defmethod ros-datatype ((msg (eql '<CoaxConfigureOAMode-response>)))
  "Returns string type for a service object of type '<CoaxConfigureOAMode-response>"
  "coax_msgs/CoaxConfigureOAModeResponse")
(defmethod md5sum ((type (eql '<CoaxConfigureOAMode-response>)))
  "Returns md5sum for a message object of type '<CoaxConfigureOAMode-response>"
  "c0668a37c3c443907202b387475fec72")
(defmethod message-definition ((type (eql '<CoaxConfigureOAMode-response>)))
  "Returns full string definition for message of type '<CoaxConfigureOAMode-response>"
  (format nil "int8 result~%~%~%~%"))
(defmethod serialization-length ((msg <CoaxConfigureOAMode-response>))
  (+ 0
     1
))
(defmethod ros-message-to-list ((msg <CoaxConfigureOAMode-response>))
  "Converts a ROS message object to a list"
  (list '<CoaxConfigureOAMode-response>
    (cons ':result (result-val msg))
))
(defmethod service-request-type ((msg (eql 'CoaxConfigureOAMode)))
  '<CoaxConfigureOAMode-request>)
(defmethod service-response-type ((msg (eql 'CoaxConfigureOAMode)))
  '<CoaxConfigureOAMode-response>)
(defmethod ros-datatype ((msg (eql 'CoaxConfigureOAMode)))
  "Returns string type for a service object of type '<CoaxConfigureOAMode>"
  "coax_msgs/CoaxConfigureOAMode")
