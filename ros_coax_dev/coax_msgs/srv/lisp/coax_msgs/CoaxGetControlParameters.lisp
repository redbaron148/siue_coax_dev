; Auto-generated. Do not edit!


(in-package coax_msgs-srv)


;//! \htmlinclude CoaxGetControlParameters-request.msg.html

(defclass <CoaxGetControlParameters-request> (ros-message)
  ()
)
(defmethod serialize ((msg <CoaxGetControlParameters-request>) ostream)
  "Serializes a message object of type '<CoaxGetControlParameters-request>"
)
(defmethod deserialize ((msg <CoaxGetControlParameters-request>) istream)
  "Deserializes a message object of type '<CoaxGetControlParameters-request>"
  msg
)
(defmethod ros-datatype ((msg (eql '<CoaxGetControlParameters-request>)))
  "Returns string type for a service object of type '<CoaxGetControlParameters-request>"
  "coax_msgs/CoaxGetControlParametersRequest")
(defmethod md5sum ((type (eql '<CoaxGetControlParameters-request>)))
  "Returns md5sum for a message object of type '<CoaxGetControlParameters-request>"
  "4def45fbd3753b180c424ffdcb35116e")
(defmethod message-definition ((type (eql '<CoaxGetControlParameters-request>)))
  "Returns full string definition for message of type '<CoaxGetControlParameters-request>"
  (format nil "~%"))
(defmethod serialization-length ((msg <CoaxGetControlParameters-request>))
  (+ 0
))
(defmethod ros-message-to-list ((msg <CoaxGetControlParameters-request>))
  "Converts a ROS message object to a list"
  (list '<CoaxGetControlParameters-request>
))
;//! \htmlinclude CoaxGetControlParameters-response.msg.html

(defclass <CoaxGetControlParameters-response> (ros-message)
  ((params
    :reader params-val
    :initarg :params
    :type coax_msgs-msg:<CoaxControlParameters>
    :initform (make-instance 'coax_msgs-msg:<CoaxControlParameters>)))
)
(defmethod serialize ((msg <CoaxGetControlParameters-response>) ostream)
  "Serializes a message object of type '<CoaxGetControlParameters-response>"
  (serialize (slot-value msg 'params) ostream)
)
(defmethod deserialize ((msg <CoaxGetControlParameters-response>) istream)
  "Deserializes a message object of type '<CoaxGetControlParameters-response>"
  (deserialize (slot-value msg 'params) istream)
  msg
)
(defmethod ros-datatype ((msg (eql '<CoaxGetControlParameters-response>)))
  "Returns string type for a service object of type '<CoaxGetControlParameters-response>"
  "coax_msgs/CoaxGetControlParametersResponse")
(defmethod md5sum ((type (eql '<CoaxGetControlParameters-response>)))
  "Returns md5sum for a message object of type '<CoaxGetControlParameters-response>"
  "4def45fbd3753b180c424ffdcb35116e")
(defmethod message-definition ((type (eql '<CoaxGetControlParameters-response>)))
  "Returns full string definition for message of type '<CoaxGetControlParameters-response>"
  (format nil "CoaxControlParameters params~%~%~%================================================================================~%MSG: coax_msgs/CoaxControlParameters~%# Base thrust around which the control operates */~%float32 baseThrust~%# Difference of operating point between up and down motors */~%float32 yawOffset~%# Altitude gain: Kp*/~%float32 altitudeKp~%# Altitude gain: Ki*/~%float32 altitudeKi~%# Altitude gain: Kd*/~%float32 altitudeKd~%~%# Yaw gain: Kp*/~%float32 yawKp~%# Yaw gain: Ki*/~%float32 yawKi~%# Yaw gain: Kd*/~%float32 yawKd~%~%~%~%"))
(defmethod serialization-length ((msg <CoaxGetControlParameters-response>))
  (+ 0
     (serialization-length (slot-value msg 'params))
))
(defmethod ros-message-to-list ((msg <CoaxGetControlParameters-response>))
  "Converts a ROS message object to a list"
  (list '<CoaxGetControlParameters-response>
    (cons ':params (params-val msg))
))
(defmethod service-request-type ((msg (eql 'CoaxGetControlParameters)))
  '<CoaxGetControlParameters-request>)
(defmethod service-response-type ((msg (eql 'CoaxGetControlParameters)))
  '<CoaxGetControlParameters-response>)
(defmethod ros-datatype ((msg (eql 'CoaxGetControlParameters)))
  "Returns string type for a service object of type '<CoaxGetControlParameters>"
  "coax_msgs/CoaxGetControlParameters")
