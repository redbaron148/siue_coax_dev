; Auto-generated. Do not edit!


(in-package coax_msgs-srv)


;//! \htmlinclude Coax3DSetControlMode-request.msg.html

(defclass <Coax3DSetControlMode-request> (ros-message)
  ((mode
    :reader mode-val
    :initarg :mode
    :type fixnum
    :initform 0))
)
(defmethod symbol-codes ((msg-type (eql '<Coax3DSetControlMode-request>)))
  "Constants for message type '<Coax3DSetControlMode-request>"
  '((:stopMode . 0)
    (:nocontrolMode . 1)
    (:velocityMode . 2)
    (:positionMode . 3))
)
(defmethod serialize ((msg <Coax3DSetControlMode-request>) ostream)
  "Serializes a message object of type '<Coax3DSetControlMode-request>"
    (write-byte (ldb (byte 8 0) (slot-value msg 'mode)) ostream)
)
(defmethod deserialize ((msg <Coax3DSetControlMode-request>) istream)
  "Deserializes a message object of type '<Coax3DSetControlMode-request>"
  (setf (ldb (byte 8 0) (slot-value msg 'mode)) (read-byte istream))
  msg
)
(defmethod ros-datatype ((msg (eql '<Coax3DSetControlMode-request>)))
  "Returns string type for a service object of type '<Coax3DSetControlMode-request>"
  "coax_msgs/Coax3DSetControlModeRequest")
(defmethod md5sum ((type (eql '<Coax3DSetControlMode-request>)))
  "Returns md5sum for a message object of type '<Coax3DSetControlMode-request>"
  "89de998d8a18d3b85b052c5aba0c2fa0")
(defmethod message-definition ((type (eql '<Coax3DSetControlMode-request>)))
  "Returns full string definition for message of type '<Coax3DSetControlMode-request>"
  (format nil "uint8 stopMode = 0~%uint8 nocontrolMode = 1~%uint8 velocityMode = 2~%uint8 positionMode = 3~%uint8 mode~%~%"))
(defmethod serialization-length ((msg <Coax3DSetControlMode-request>))
  (+ 0
     1
))
(defmethod ros-message-to-list ((msg <Coax3DSetControlMode-request>))
  "Converts a ROS message object to a list"
  (list '<Coax3DSetControlMode-request>
    (cons ':mode (mode-val msg))
))
;//! \htmlinclude Coax3DSetControlMode-response.msg.html

(defclass <Coax3DSetControlMode-response> (ros-message)
  ((result
    :reader result-val
    :initarg :result
    :type fixnum
    :initform 0))
)
(defmethod serialize ((msg <Coax3DSetControlMode-response>) ostream)
  "Serializes a message object of type '<Coax3DSetControlMode-response>"
    (write-byte (ldb (byte 8 0) (slot-value msg 'result)) ostream)
)
(defmethod deserialize ((msg <Coax3DSetControlMode-response>) istream)
  "Deserializes a message object of type '<Coax3DSetControlMode-response>"
  (setf (ldb (byte 8 0) (slot-value msg 'result)) (read-byte istream))
  msg
)
(defmethod ros-datatype ((msg (eql '<Coax3DSetControlMode-response>)))
  "Returns string type for a service object of type '<Coax3DSetControlMode-response>"
  "coax_msgs/Coax3DSetControlModeResponse")
(defmethod md5sum ((type (eql '<Coax3DSetControlMode-response>)))
  "Returns md5sum for a message object of type '<Coax3DSetControlMode-response>"
  "89de998d8a18d3b85b052c5aba0c2fa0")
(defmethod message-definition ((type (eql '<Coax3DSetControlMode-response>)))
  "Returns full string definition for message of type '<Coax3DSetControlMode-response>"
  (format nil "int8 result~%~%~%"))
(defmethod serialization-length ((msg <Coax3DSetControlMode-response>))
  (+ 0
     1
))
(defmethod ros-message-to-list ((msg <Coax3DSetControlMode-response>))
  "Converts a ROS message object to a list"
  (list '<Coax3DSetControlMode-response>
    (cons ':result (result-val msg))
))
(defmethod service-request-type ((msg (eql 'Coax3DSetControlMode)))
  '<Coax3DSetControlMode-request>)
(defmethod service-response-type ((msg (eql 'Coax3DSetControlMode)))
  '<Coax3DSetControlMode-response>)
(defmethod ros-datatype ((msg (eql 'Coax3DSetControlMode)))
  "Returns string type for a service object of type '<Coax3DSetControlMode>"
  "coax_msgs/Coax3DSetControlMode")
