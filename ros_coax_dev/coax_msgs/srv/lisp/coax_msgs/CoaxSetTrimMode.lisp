; Auto-generated. Do not edit!


(in-package coax_msgs-srv)


;//! \htmlinclude CoaxSetTrimMode-request.msg.html

(defclass <CoaxSetTrimMode-request> (ros-message)
  ((mode
    :reader mode-val
    :initarg :mode
    :type coax_msgs-msg:<CoaxTrimMode>
    :initform (make-instance 'coax_msgs-msg:<CoaxTrimMode>)))
)
(defmethod serialize ((msg <CoaxSetTrimMode-request>) ostream)
  "Serializes a message object of type '<CoaxSetTrimMode-request>"
  (serialize (slot-value msg 'mode) ostream)
)
(defmethod deserialize ((msg <CoaxSetTrimMode-request>) istream)
  "Deserializes a message object of type '<CoaxSetTrimMode-request>"
  (deserialize (slot-value msg 'mode) istream)
  msg
)
(defmethod ros-datatype ((msg (eql '<CoaxSetTrimMode-request>)))
  "Returns string type for a service object of type '<CoaxSetTrimMode-request>"
  "coax_msgs/CoaxSetTrimModeRequest")
(defmethod md5sum ((type (eql '<CoaxSetTrimMode-request>)))
  "Returns md5sum for a message object of type '<CoaxSetTrimMode-request>"
  "f44261c5d78b4d7c608cae2c0e15d49d")
(defmethod message-definition ((type (eql '<CoaxSetTrimMode-request>)))
  "Returns full string definition for message of type '<CoaxSetTrimMode-request>"
  (format nil "CoaxTrimMode mode~%~%"))
(defmethod serialization-length ((msg <CoaxSetTrimMode-request>))
  (+ 0
     (serialization-length (slot-value msg 'mode))
))
(defmethod ros-message-to-list ((msg <CoaxSetTrimMode-request>))
  "Converts a ROS message object to a list"
  (list '<CoaxSetTrimMode-request>
    (cons ':mode (mode-val msg))
))
;//! \htmlinclude CoaxSetTrimMode-response.msg.html

(defclass <CoaxSetTrimMode-response> (ros-message)
  ((result
    :reader result-val
    :initarg :result
    :type fixnum
    :initform 0))
)
(defmethod serialize ((msg <CoaxSetTrimMode-response>) ostream)
  "Serializes a message object of type '<CoaxSetTrimMode-response>"
    (write-byte (ldb (byte 8 0) (slot-value msg 'result)) ostream)
)
(defmethod deserialize ((msg <CoaxSetTrimMode-response>) istream)
  "Deserializes a message object of type '<CoaxSetTrimMode-response>"
  (setf (ldb (byte 8 0) (slot-value msg 'result)) (read-byte istream))
  msg
)
(defmethod ros-datatype ((msg (eql '<CoaxSetTrimMode-response>)))
  "Returns string type for a service object of type '<CoaxSetTrimMode-response>"
  "coax_msgs/CoaxSetTrimModeResponse")
(defmethod md5sum ((type (eql '<CoaxSetTrimMode-response>)))
  "Returns md5sum for a message object of type '<CoaxSetTrimMode-response>"
  "f44261c5d78b4d7c608cae2c0e15d49d")
(defmethod message-definition ((type (eql '<CoaxSetTrimMode-response>)))
  "Returns full string definition for message of type '<CoaxSetTrimMode-response>"
  (format nil "int8 result~%~%~%================================================================================~%MSG: coax_msgs/CoaxTrimMode~%# Trim mode, can be SB_TRIM_FROM_RC or SB_TRIM_SOFTWARE */~%uint8 trimMode ~%# Trim position for the roll axis */~%float32 rollTrim~%# Trim position for the pitch axis */~%float32 pitchTrim~%~%~%~%"))
(defmethod serialization-length ((msg <CoaxSetTrimMode-response>))
  (+ 0
     1
))
(defmethod ros-message-to-list ((msg <CoaxSetTrimMode-response>))
  "Converts a ROS message object to a list"
  (list '<CoaxSetTrimMode-response>
    (cons ':result (result-val msg))
))
(defmethod service-request-type ((msg (eql 'CoaxSetTrimMode)))
  '<CoaxSetTrimMode-request>)
(defmethod service-response-type ((msg (eql 'CoaxSetTrimMode)))
  '<CoaxSetTrimMode-response>)
(defmethod ros-datatype ((msg (eql 'CoaxSetTrimMode)))
  "Returns string type for a service object of type '<CoaxSetTrimMode>"
  "coax_msgs/CoaxSetTrimMode")
