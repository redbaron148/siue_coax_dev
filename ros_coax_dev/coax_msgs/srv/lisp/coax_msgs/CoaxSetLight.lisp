; Auto-generated. Do not edit!


(in-package coax_msgs-srv)


;//! \htmlinclude CoaxSetLight-request.msg.html

(defclass <CoaxSetLight-request> (ros-message)
  ((percent
    :reader percent-val
    :initarg :percent
    :type fixnum
    :initform 0))
)
(defmethod serialize ((msg <CoaxSetLight-request>) ostream)
  "Serializes a message object of type '<CoaxSetLight-request>"
    (write-byte (ldb (byte 8 0) (slot-value msg 'percent)) ostream)
)
(defmethod deserialize ((msg <CoaxSetLight-request>) istream)
  "Deserializes a message object of type '<CoaxSetLight-request>"
  (setf (ldb (byte 8 0) (slot-value msg 'percent)) (read-byte istream))
  msg
)
(defmethod ros-datatype ((msg (eql '<CoaxSetLight-request>)))
  "Returns string type for a service object of type '<CoaxSetLight-request>"
  "coax_msgs/CoaxSetLightRequest")
(defmethod md5sum ((type (eql '<CoaxSetLight-request>)))
  "Returns md5sum for a message object of type '<CoaxSetLight-request>"
  "240f3dc9b05ebb42cdf3a9eb3ef29ad6")
(defmethod message-definition ((type (eql '<CoaxSetLight-request>)))
  "Returns full string definition for message of type '<CoaxSetLight-request>"
  (format nil "uint8 percent~%~%"))
(defmethod serialization-length ((msg <CoaxSetLight-request>))
  (+ 0
     1
))
(defmethod ros-message-to-list ((msg <CoaxSetLight-request>))
  "Converts a ROS message object to a list"
  (list '<CoaxSetLight-request>
    (cons ':percent (percent-val msg))
))
;//! \htmlinclude CoaxSetLight-response.msg.html

(defclass <CoaxSetLight-response> (ros-message)
  ((result
    :reader result-val
    :initarg :result
    :type fixnum
    :initform 0))
)
(defmethod serialize ((msg <CoaxSetLight-response>) ostream)
  "Serializes a message object of type '<CoaxSetLight-response>"
    (write-byte (ldb (byte 8 0) (slot-value msg 'result)) ostream)
)
(defmethod deserialize ((msg <CoaxSetLight-response>) istream)
  "Deserializes a message object of type '<CoaxSetLight-response>"
  (setf (ldb (byte 8 0) (slot-value msg 'result)) (read-byte istream))
  msg
)
(defmethod ros-datatype ((msg (eql '<CoaxSetLight-response>)))
  "Returns string type for a service object of type '<CoaxSetLight-response>"
  "coax_msgs/CoaxSetLightResponse")
(defmethod md5sum ((type (eql '<CoaxSetLight-response>)))
  "Returns md5sum for a message object of type '<CoaxSetLight-response>"
  "240f3dc9b05ebb42cdf3a9eb3ef29ad6")
(defmethod message-definition ((type (eql '<CoaxSetLight-response>)))
  "Returns full string definition for message of type '<CoaxSetLight-response>"
  (format nil "int8 result~%~%~%"))
(defmethod serialization-length ((msg <CoaxSetLight-response>))
  (+ 0
     1
))
(defmethod ros-message-to-list ((msg <CoaxSetLight-response>))
  "Converts a ROS message object to a list"
  (list '<CoaxSetLight-response>
    (cons ':result (result-val msg))
))
(defmethod service-request-type ((msg (eql 'CoaxSetLight)))
  '<CoaxSetLight-request>)
(defmethod service-response-type ((msg (eql 'CoaxSetLight)))
  '<CoaxSetLight-response>)
(defmethod ros-datatype ((msg (eql 'CoaxSetLight)))
  "Returns string type for a service object of type '<CoaxSetLight>"
  "coax_msgs/CoaxSetLight")
