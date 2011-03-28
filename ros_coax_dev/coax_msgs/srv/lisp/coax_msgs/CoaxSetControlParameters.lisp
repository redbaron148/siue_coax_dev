; Auto-generated. Do not edit!


(in-package coax_msgs-srv)


;//! \htmlinclude CoaxSetControlParameters-request.msg.html

(defclass <CoaxSetControlParameters-request> (ros-message)
  ((params
    :reader params-val
    :initarg :params
    :type coax_msgs-msg:<CoaxControlParameters>
    :initform (make-instance 'coax_msgs-msg:<CoaxControlParameters>)))
)
(defmethod serialize ((msg <CoaxSetControlParameters-request>) ostream)
  "Serializes a message object of type '<CoaxSetControlParameters-request>"
  (serialize (slot-value msg 'params) ostream)
)
(defmethod deserialize ((msg <CoaxSetControlParameters-request>) istream)
  "Deserializes a message object of type '<CoaxSetControlParameters-request>"
  (deserialize (slot-value msg 'params) istream)
  msg
)
(defmethod ros-datatype ((msg (eql '<CoaxSetControlParameters-request>)))
  "Returns string type for a service object of type '<CoaxSetControlParameters-request>"
  "coax_msgs/CoaxSetControlParametersRequest")
(defmethod md5sum ((type (eql '<CoaxSetControlParameters-request>)))
  "Returns md5sum for a message object of type '<CoaxSetControlParameters-request>"
  "4ced18bc984a7d80fa03ac2fadfc11ac")
(defmethod message-definition ((type (eql '<CoaxSetControlParameters-request>)))
  "Returns full string definition for message of type '<CoaxSetControlParameters-request>"
  (format nil "CoaxControlParameters params~%~%"))
(defmethod serialization-length ((msg <CoaxSetControlParameters-request>))
  (+ 0
     (serialization-length (slot-value msg 'params))
))
(defmethod ros-message-to-list ((msg <CoaxSetControlParameters-request>))
  "Converts a ROS message object to a list"
  (list '<CoaxSetControlParameters-request>
    (cons ':params (params-val msg))
))
;//! \htmlinclude CoaxSetControlParameters-response.msg.html

(defclass <CoaxSetControlParameters-response> (ros-message)
  ((result
    :reader result-val
    :initarg :result
    :type fixnum
    :initform 0))
)
(defmethod serialize ((msg <CoaxSetControlParameters-response>) ostream)
  "Serializes a message object of type '<CoaxSetControlParameters-response>"
    (write-byte (ldb (byte 8 0) (slot-value msg 'result)) ostream)
)
(defmethod deserialize ((msg <CoaxSetControlParameters-response>) istream)
  "Deserializes a message object of type '<CoaxSetControlParameters-response>"
  (setf (ldb (byte 8 0) (slot-value msg 'result)) (read-byte istream))
  msg
)
(defmethod ros-datatype ((msg (eql '<CoaxSetControlParameters-response>)))
  "Returns string type for a service object of type '<CoaxSetControlParameters-response>"
  "coax_msgs/CoaxSetControlParametersResponse")
(defmethod md5sum ((type (eql '<CoaxSetControlParameters-response>)))
  "Returns md5sum for a message object of type '<CoaxSetControlParameters-response>"
  "4ced18bc984a7d80fa03ac2fadfc11ac")
(defmethod message-definition ((type (eql '<CoaxSetControlParameters-response>)))
  "Returns full string definition for message of type '<CoaxSetControlParameters-response>"
  (format nil "int8 result~%~%~%================================================================================~%MSG: coax_msgs/CoaxControlParameters~%# Base thrust around which the control operates */~%float32 baseThrust~%# Difference of operating point between up and down motors */~%float32 yawOffset~%# Altitude gain: Kp*/~%float32 altitudeKp~%# Altitude gain: Ki*/~%float32 altitudeKi~%# Altitude gain: Kd*/~%float32 altitudeKd~%~%# Yaw gain: Kp*/~%float32 yawKp~%# Yaw gain: Ki*/~%float32 yawKi~%# Yaw gain: Kd*/~%float32 yawKd~%~%~%~%"))
(defmethod serialization-length ((msg <CoaxSetControlParameters-response>))
  (+ 0
     1
))
(defmethod ros-message-to-list ((msg <CoaxSetControlParameters-response>))
  "Converts a ROS message object to a list"
  (list '<CoaxSetControlParameters-response>
    (cons ':result (result-val msg))
))
(defmethod service-request-type ((msg (eql 'CoaxSetControlParameters)))
  '<CoaxSetControlParameters-request>)
(defmethod service-response-type ((msg (eql 'CoaxSetControlParameters)))
  '<CoaxSetControlParameters-response>)
(defmethod ros-datatype ((msg (eql 'CoaxSetControlParameters)))
  "Returns string type for a service object of type '<CoaxSetControlParameters>"
  "coax_msgs/CoaxSetControlParameters")
