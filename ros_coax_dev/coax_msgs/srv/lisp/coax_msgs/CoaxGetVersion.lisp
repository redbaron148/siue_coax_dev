; Auto-generated. Do not edit!


(in-package coax_msgs-srv)


;//! \htmlinclude CoaxGetVersion-request.msg.html

(defclass <CoaxGetVersion-request> (ros-message)
  ()
)
(defmethod serialize ((msg <CoaxGetVersion-request>) ostream)
  "Serializes a message object of type '<CoaxGetVersion-request>"
)
(defmethod deserialize ((msg <CoaxGetVersion-request>) istream)
  "Deserializes a message object of type '<CoaxGetVersion-request>"
  msg
)
(defmethod ros-datatype ((msg (eql '<CoaxGetVersion-request>)))
  "Returns string type for a service object of type '<CoaxGetVersion-request>"
  "coax_msgs/CoaxGetVersionRequest")
(defmethod md5sum ((type (eql '<CoaxGetVersion-request>)))
  "Returns md5sum for a message object of type '<CoaxGetVersion-request>"
  "e6b62c09e386f0774fff1cbc20534440")
(defmethod message-definition ((type (eql '<CoaxGetVersion-request>)))
  "Returns full string definition for message of type '<CoaxGetVersion-request>"
  (format nil "~%"))
(defmethod serialization-length ((msg <CoaxGetVersion-request>))
  (+ 0
))
(defmethod ros-message-to-list ((msg <CoaxGetVersion-request>))
  "Converts a ROS message object to a list"
  (list '<CoaxGetVersion-request>
))
;//! \htmlinclude CoaxGetVersion-response.msg.html

(defclass <CoaxGetVersion-response> (ros-message)
  ((version
    :reader version-val
    :initarg :version
    :type coax_msgs-msg:<CoaxVersion>
    :initform (make-instance 'coax_msgs-msg:<CoaxVersion>)))
)
(defmethod serialize ((msg <CoaxGetVersion-response>) ostream)
  "Serializes a message object of type '<CoaxGetVersion-response>"
  (serialize (slot-value msg 'version) ostream)
)
(defmethod deserialize ((msg <CoaxGetVersion-response>) istream)
  "Deserializes a message object of type '<CoaxGetVersion-response>"
  (deserialize (slot-value msg 'version) istream)
  msg
)
(defmethod ros-datatype ((msg (eql '<CoaxGetVersion-response>)))
  "Returns string type for a service object of type '<CoaxGetVersion-response>"
  "coax_msgs/CoaxGetVersionResponse")
(defmethod md5sum ((type (eql '<CoaxGetVersion-response>)))
  "Returns md5sum for a message object of type '<CoaxGetVersion-response>"
  "e6b62c09e386f0774fff1cbc20534440")
(defmethod message-definition ((type (eql '<CoaxGetVersion-response>)))
  "Returns full string definition for message of type '<CoaxGetVersion-response>"
  (format nil "CoaxVersion version~%~%~%================================================================================~%MSG: coax_msgs/CoaxVersion~%uint16 apiVersion~%uint16 controllerVersion~%string imuVersion~%string compileTime~%~%~%"))
(defmethod serialization-length ((msg <CoaxGetVersion-response>))
  (+ 0
     (serialization-length (slot-value msg 'version))
))
(defmethod ros-message-to-list ((msg <CoaxGetVersion-response>))
  "Converts a ROS message object to a list"
  (list '<CoaxGetVersion-response>
    (cons ':version (version-val msg))
))
(defmethod service-request-type ((msg (eql 'CoaxGetVersion)))
  '<CoaxGetVersion-request>)
(defmethod service-response-type ((msg (eql 'CoaxGetVersion)))
  '<CoaxGetVersion-response>)
(defmethod ros-datatype ((msg (eql 'CoaxGetVersion)))
  "Returns string type for a service object of type '<CoaxGetVersion>"
  "coax_msgs/CoaxGetVersion")
