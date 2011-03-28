; Auto-generated. Do not edit!


(in-package coax_msgs-srv)


;//! \htmlinclude CoaxConfigureControl-request.msg.html

(defclass <CoaxConfigureControl-request> (ros-message)
  ((rollMode
    :reader rollMode-val
    :initarg :rollMode
    :type fixnum
    :initform 0)
   (pitchMode
    :reader pitchMode-val
    :initarg :pitchMode
    :type fixnum
    :initform 0)
   (yawMode
    :reader yawMode-val
    :initarg :yawMode
    :type fixnum
    :initform 0)
   (altitudeMode
    :reader altitudeMode-val
    :initarg :altitudeMode
    :type fixnum
    :initform 0))
)
(defmethod serialize ((msg <CoaxConfigureControl-request>) ostream)
  "Serializes a message object of type '<CoaxConfigureControl-request>"
    (write-byte (ldb (byte 8 0) (slot-value msg 'rollMode)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'rollMode)) ostream)
    (write-byte (ldb (byte 8 0) (slot-value msg 'pitchMode)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'pitchMode)) ostream)
    (write-byte (ldb (byte 8 0) (slot-value msg 'yawMode)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'yawMode)) ostream)
    (write-byte (ldb (byte 8 0) (slot-value msg 'altitudeMode)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'altitudeMode)) ostream)
)
(defmethod deserialize ((msg <CoaxConfigureControl-request>) istream)
  "Deserializes a message object of type '<CoaxConfigureControl-request>"
  (setf (ldb (byte 8 0) (slot-value msg 'rollMode)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'rollMode)) (read-byte istream))
  (setf (ldb (byte 8 0) (slot-value msg 'pitchMode)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'pitchMode)) (read-byte istream))
  (setf (ldb (byte 8 0) (slot-value msg 'yawMode)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'yawMode)) (read-byte istream))
  (setf (ldb (byte 8 0) (slot-value msg 'altitudeMode)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'altitudeMode)) (read-byte istream))
  msg
)
(defmethod ros-datatype ((msg (eql '<CoaxConfigureControl-request>)))
  "Returns string type for a service object of type '<CoaxConfigureControl-request>"
  "coax_msgs/CoaxConfigureControlRequest")
(defmethod md5sum ((type (eql '<CoaxConfigureControl-request>)))
  "Returns md5sum for a message object of type '<CoaxConfigureControl-request>"
  "3612d80e1bf1f78cf318a82336a878bf")
(defmethod message-definition ((type (eql '<CoaxConfigureControl-request>)))
  "Returns full string definition for message of type '<CoaxConfigureControl-request>"
  (format nil "uint16 rollMode~%uint16 pitchMode~%uint16 yawMode~%uint16 altitudeMode~%~%"))
(defmethod serialization-length ((msg <CoaxConfigureControl-request>))
  (+ 0
     2
     2
     2
     2
))
(defmethod ros-message-to-list ((msg <CoaxConfigureControl-request>))
  "Converts a ROS message object to a list"
  (list '<CoaxConfigureControl-request>
    (cons ':rollMode (rollMode-val msg))
    (cons ':pitchMode (pitchMode-val msg))
    (cons ':yawMode (yawMode-val msg))
    (cons ':altitudeMode (altitudeMode-val msg))
))
;//! \htmlinclude CoaxConfigureControl-response.msg.html

(defclass <CoaxConfigureControl-response> (ros-message)
  ((result
    :reader result-val
    :initarg :result
    :type fixnum
    :initform 0))
)
(defmethod serialize ((msg <CoaxConfigureControl-response>) ostream)
  "Serializes a message object of type '<CoaxConfigureControl-response>"
    (write-byte (ldb (byte 8 0) (slot-value msg 'result)) ostream)
)
(defmethod deserialize ((msg <CoaxConfigureControl-response>) istream)
  "Deserializes a message object of type '<CoaxConfigureControl-response>"
  (setf (ldb (byte 8 0) (slot-value msg 'result)) (read-byte istream))
  msg
)
(defmethod ros-datatype ((msg (eql '<CoaxConfigureControl-response>)))
  "Returns string type for a service object of type '<CoaxConfigureControl-response>"
  "coax_msgs/CoaxConfigureControlResponse")
(defmethod md5sum ((type (eql '<CoaxConfigureControl-response>)))
  "Returns md5sum for a message object of type '<CoaxConfigureControl-response>"
  "3612d80e1bf1f78cf318a82336a878bf")
(defmethod message-definition ((type (eql '<CoaxConfigureControl-response>)))
  "Returns full string definition for message of type '<CoaxConfigureControl-response>"
  (format nil "int8 result~%~%~%~%"))
(defmethod serialization-length ((msg <CoaxConfigureControl-response>))
  (+ 0
     1
))
(defmethod ros-message-to-list ((msg <CoaxConfigureControl-response>))
  "Converts a ROS message object to a list"
  (list '<CoaxConfigureControl-response>
    (cons ':result (result-val msg))
))
(defmethod service-request-type ((msg (eql 'CoaxConfigureControl)))
  '<CoaxConfigureControl-request>)
(defmethod service-response-type ((msg (eql 'CoaxConfigureControl)))
  '<CoaxConfigureControl-response>)
(defmethod ros-datatype ((msg (eql 'CoaxConfigureControl)))
  "Returns string type for a service object of type '<CoaxConfigureControl>"
  "coax_msgs/CoaxConfigureControl")
