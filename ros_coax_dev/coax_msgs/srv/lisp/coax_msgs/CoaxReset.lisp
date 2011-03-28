; Auto-generated. Do not edit!


(in-package coax_msgs-srv)


;//! \htmlinclude CoaxReset-request.msg.html

(defclass <CoaxReset-request> (ros-message)
  ()
)
(defmethod serialize ((msg <CoaxReset-request>) ostream)
  "Serializes a message object of type '<CoaxReset-request>"
)
(defmethod deserialize ((msg <CoaxReset-request>) istream)
  "Deserializes a message object of type '<CoaxReset-request>"
  msg
)
(defmethod ros-datatype ((msg (eql '<CoaxReset-request>)))
  "Returns string type for a service object of type '<CoaxReset-request>"
  "coax_msgs/CoaxResetRequest")
(defmethod md5sum ((type (eql '<CoaxReset-request>)))
  "Returns md5sum for a message object of type '<CoaxReset-request>"
  "4414c67819626a1b8e0f043a9a0d6c9a")
(defmethod message-definition ((type (eql '<CoaxReset-request>)))
  "Returns full string definition for message of type '<CoaxReset-request>"
  (format nil "~%"))
(defmethod serialization-length ((msg <CoaxReset-request>))
  (+ 0
))
(defmethod ros-message-to-list ((msg <CoaxReset-request>))
  "Converts a ROS message object to a list"
  (list '<CoaxReset-request>
))
;//! \htmlinclude CoaxReset-response.msg.html

(defclass <CoaxReset-response> (ros-message)
  ((result
    :reader result-val
    :initarg :result
    :type fixnum
    :initform 0))
)
(defmethod serialize ((msg <CoaxReset-response>) ostream)
  "Serializes a message object of type '<CoaxReset-response>"
    (write-byte (ldb (byte 8 0) (slot-value msg 'result)) ostream)
)
(defmethod deserialize ((msg <CoaxReset-response>) istream)
  "Deserializes a message object of type '<CoaxReset-response>"
  (setf (ldb (byte 8 0) (slot-value msg 'result)) (read-byte istream))
  msg
)
(defmethod ros-datatype ((msg (eql '<CoaxReset-response>)))
  "Returns string type for a service object of type '<CoaxReset-response>"
  "coax_msgs/CoaxResetResponse")
(defmethod md5sum ((type (eql '<CoaxReset-response>)))
  "Returns md5sum for a message object of type '<CoaxReset-response>"
  "4414c67819626a1b8e0f043a9a0d6c9a")
(defmethod message-definition ((type (eql '<CoaxReset-response>)))
  "Returns full string definition for message of type '<CoaxReset-response>"
  (format nil "int8 result~%~%~%"))
(defmethod serialization-length ((msg <CoaxReset-response>))
  (+ 0
     1
))
(defmethod ros-message-to-list ((msg <CoaxReset-response>))
  "Converts a ROS message object to a list"
  (list '<CoaxReset-response>
    (cons ':result (result-val msg))
))
(defmethod service-request-type ((msg (eql 'CoaxReset)))
  '<CoaxReset-request>)
(defmethod service-response-type ((msg (eql 'CoaxReset)))
  '<CoaxReset-response>)
(defmethod ros-datatype ((msg (eql 'CoaxReset)))
  "Returns string type for a service object of type '<CoaxReset>"
  "coax_msgs/CoaxReset")
