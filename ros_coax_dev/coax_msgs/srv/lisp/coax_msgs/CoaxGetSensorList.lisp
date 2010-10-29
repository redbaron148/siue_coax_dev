; Auto-generated. Do not edit!


(in-package coax_msgs-srv)


;//! \htmlinclude CoaxGetSensorList-request.msg.html

(defclass <CoaxGetSensorList-request> (ros-message)
  ()
)
(defmethod serialize ((msg <CoaxGetSensorList-request>) ostream)
  "Serializes a message object of type '<CoaxGetSensorList-request>"
)
(defmethod deserialize ((msg <CoaxGetSensorList-request>) istream)
  "Deserializes a message object of type '<CoaxGetSensorList-request>"
  msg
)
(defmethod ros-datatype ((msg (eql '<CoaxGetSensorList-request>)))
  "Returns string type for a service object of type '<CoaxGetSensorList-request>"
  "coax_msgs/CoaxGetSensorListRequest")
(defmethod md5sum ((type (eql '<CoaxGetSensorList-request>)))
  "Returns md5sum for a message object of type '<CoaxGetSensorList-request>"
  "8707033dcf26046312062358fc767479")
(defmethod message-definition ((type (eql '<CoaxGetSensorList-request>)))
  "Returns full string definition for message of type '<CoaxGetSensorList-request>"
  (format nil "~%"))
(defmethod serialization-length ((msg <CoaxGetSensorList-request>))
  (+ 0
))
(defmethod ros-message-to-list ((msg <CoaxGetSensorList-request>))
  "Converts a ROS message object to a list"
  (list '<CoaxGetSensorList-request>
))
;//! \htmlinclude CoaxGetSensorList-response.msg.html

(defclass <CoaxGetSensorList-response> (ros-message)
  ((list
    :reader list-val
    :initarg :list
    :type integer
    :initform 0))
)
(defmethod serialize ((msg <CoaxGetSensorList-response>) ostream)
  "Serializes a message object of type '<CoaxGetSensorList-response>"
    (write-byte (ldb (byte 8 0) (slot-value msg 'list)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'list)) ostream)
  (write-byte (ldb (byte 8 16) (slot-value msg 'list)) ostream)
  (write-byte (ldb (byte 8 24) (slot-value msg 'list)) ostream)
)
(defmethod deserialize ((msg <CoaxGetSensorList-response>) istream)
  "Deserializes a message object of type '<CoaxGetSensorList-response>"
  (setf (ldb (byte 8 0) (slot-value msg 'list)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'list)) (read-byte istream))
  (setf (ldb (byte 8 16) (slot-value msg 'list)) (read-byte istream))
  (setf (ldb (byte 8 24) (slot-value msg 'list)) (read-byte istream))
  msg
)
(defmethod ros-datatype ((msg (eql '<CoaxGetSensorList-response>)))
  "Returns string type for a service object of type '<CoaxGetSensorList-response>"
  "coax_msgs/CoaxGetSensorListResponse")
(defmethod md5sum ((type (eql '<CoaxGetSensorList-response>)))
  "Returns md5sum for a message object of type '<CoaxGetSensorList-response>"
  "8707033dcf26046312062358fc767479")
(defmethod message-definition ((type (eql '<CoaxGetSensorList-response>)))
  "Returns full string definition for message of type '<CoaxGetSensorList-response>"
  (format nil "uint32 list~%~%~%"))
(defmethod serialization-length ((msg <CoaxGetSensorList-response>))
  (+ 0
     4
))
(defmethod ros-message-to-list ((msg <CoaxGetSensorList-response>))
  "Converts a ROS message object to a list"
  (list '<CoaxGetSensorList-response>
    (cons ':list (list-val msg))
))
(defmethod service-request-type ((msg (eql 'CoaxGetSensorList)))
  '<CoaxGetSensorList-request>)
(defmethod service-response-type ((msg (eql 'CoaxGetSensorList)))
  '<CoaxGetSensorList-response>)
(defmethod ros-datatype ((msg (eql 'CoaxGetSensorList)))
  "Returns string type for a service object of type '<CoaxGetSensorList>"
  "coax_msgs/CoaxGetSensorList")
