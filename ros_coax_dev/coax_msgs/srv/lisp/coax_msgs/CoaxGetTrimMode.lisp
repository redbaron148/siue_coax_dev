; Auto-generated. Do not edit!


(in-package coax_msgs-srv)


;//! \htmlinclude CoaxGetTrimMode-request.msg.html

(defclass <CoaxGetTrimMode-request> (ros-message)
  ()
)
(defmethod serialize ((msg <CoaxGetTrimMode-request>) ostream)
  "Serializes a message object of type '<CoaxGetTrimMode-request>"
)
(defmethod deserialize ((msg <CoaxGetTrimMode-request>) istream)
  "Deserializes a message object of type '<CoaxGetTrimMode-request>"
  msg
)
(defmethod ros-datatype ((msg (eql '<CoaxGetTrimMode-request>)))
  "Returns string type for a service object of type '<CoaxGetTrimMode-request>"
  "coax_msgs/CoaxGetTrimModeRequest")
(defmethod md5sum ((type (eql '<CoaxGetTrimMode-request>)))
  "Returns md5sum for a message object of type '<CoaxGetTrimMode-request>"
  "ebb2d5a0e1488ca7b436e3a43853c58b")
(defmethod message-definition ((type (eql '<CoaxGetTrimMode-request>)))
  "Returns full string definition for message of type '<CoaxGetTrimMode-request>"
  (format nil "~%"))
(defmethod serialization-length ((msg <CoaxGetTrimMode-request>))
  (+ 0
))
(defmethod ros-message-to-list ((msg <CoaxGetTrimMode-request>))
  "Converts a ROS message object to a list"
  (list '<CoaxGetTrimMode-request>
))
;//! \htmlinclude CoaxGetTrimMode-response.msg.html

(defclass <CoaxGetTrimMode-response> (ros-message)
  ((mode
    :reader mode-val
    :initarg :mode
    :type coax_msgs-msg:<CoaxTrimMode>
    :initform (make-instance 'coax_msgs-msg:<CoaxTrimMode>)))
)
(defmethod serialize ((msg <CoaxGetTrimMode-response>) ostream)
  "Serializes a message object of type '<CoaxGetTrimMode-response>"
  (serialize (slot-value msg 'mode) ostream)
)
(defmethod deserialize ((msg <CoaxGetTrimMode-response>) istream)
  "Deserializes a message object of type '<CoaxGetTrimMode-response>"
  (deserialize (slot-value msg 'mode) istream)
  msg
)
(defmethod ros-datatype ((msg (eql '<CoaxGetTrimMode-response>)))
  "Returns string type for a service object of type '<CoaxGetTrimMode-response>"
  "coax_msgs/CoaxGetTrimModeResponse")
(defmethod md5sum ((type (eql '<CoaxGetTrimMode-response>)))
  "Returns md5sum for a message object of type '<CoaxGetTrimMode-response>"
  "ebb2d5a0e1488ca7b436e3a43853c58b")
(defmethod message-definition ((type (eql '<CoaxGetTrimMode-response>)))
  "Returns full string definition for message of type '<CoaxGetTrimMode-response>"
  (format nil "CoaxTrimMode mode~%~%================================================================================~%MSG: coax_msgs/CoaxTrimMode~%# Trim mode, can be SB_TRIM_FROM_RC or SB_TRIM_SOFTWARE */~%uint8 trimMode ~%# Trim position for the roll axis */~%float32 rollTrim~%# Trim position for the pitch axis */~%float32 pitchTrim~%~%~%~%"))
(defmethod serialization-length ((msg <CoaxGetTrimMode-response>))
  (+ 0
     (serialization-length (slot-value msg 'mode))
))
(defmethod ros-message-to-list ((msg <CoaxGetTrimMode-response>))
  "Converts a ROS message object to a list"
  (list '<CoaxGetTrimMode-response>
    (cons ':mode (mode-val msg))
))
(defmethod service-request-type ((msg (eql 'CoaxGetTrimMode)))
  '<CoaxGetTrimMode-request>)
(defmethod service-response-type ((msg (eql 'CoaxGetTrimMode)))
  '<CoaxGetTrimMode-response>)
(defmethod ros-datatype ((msg (eql 'CoaxGetTrimMode)))
  "Returns string type for a service object of type '<CoaxGetTrimMode>"
  "coax_msgs/CoaxGetTrimMode")
