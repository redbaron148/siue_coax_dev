; Auto-generated. Do not edit!


(in-package coax_msgs-srv)


;//! \htmlinclude CoaxSetVerbose-request.msg.html

(defclass <CoaxSetVerbose-request> (ros-message)
  ((verbose
    :reader verbose-val
    :initarg :verbose
    :type fixnum
    :initform 0)
   (channel
    :reader channel-val
    :initarg :channel
    :type fixnum
    :initform 0))
)
(defmethod serialize ((msg <CoaxSetVerbose-request>) ostream)
  "Serializes a message object of type '<CoaxSetVerbose-request>"
    (write-byte (ldb (byte 8 0) (slot-value msg 'verbose)) ostream)
    (write-byte (ldb (byte 8 0) (slot-value msg 'channel)) ostream)
)
(defmethod deserialize ((msg <CoaxSetVerbose-request>) istream)
  "Deserializes a message object of type '<CoaxSetVerbose-request>"
  (setf (ldb (byte 8 0) (slot-value msg 'verbose)) (read-byte istream))
  (setf (ldb (byte 8 0) (slot-value msg 'channel)) (read-byte istream))
  msg
)
(defmethod ros-datatype ((msg (eql '<CoaxSetVerbose-request>)))
  "Returns string type for a service object of type '<CoaxSetVerbose-request>"
  "coax_msgs/CoaxSetVerboseRequest")
(defmethod md5sum ((type (eql '<CoaxSetVerbose-request>)))
  "Returns md5sum for a message object of type '<CoaxSetVerbose-request>"
  "a5656957cc9c1010a24927dc8f7dc869")
(defmethod message-definition ((type (eql '<CoaxSetVerbose-request>)))
  "Returns full string definition for message of type '<CoaxSetVerbose-request>"
  (format nil "uint8 verbose~%uint8 channel~%~%"))
(defmethod serialization-length ((msg <CoaxSetVerbose-request>))
  (+ 0
     1
     1
))
(defmethod ros-message-to-list ((msg <CoaxSetVerbose-request>))
  "Converts a ROS message object to a list"
  (list '<CoaxSetVerbose-request>
    (cons ':verbose (verbose-val msg))
    (cons ':channel (channel-val msg))
))
;//! \htmlinclude CoaxSetVerbose-response.msg.html

(defclass <CoaxSetVerbose-response> (ros-message)
  ((result
    :reader result-val
    :initarg :result
    :type fixnum
    :initform 0))
)
(defmethod serialize ((msg <CoaxSetVerbose-response>) ostream)
  "Serializes a message object of type '<CoaxSetVerbose-response>"
    (write-byte (ldb (byte 8 0) (slot-value msg 'result)) ostream)
)
(defmethod deserialize ((msg <CoaxSetVerbose-response>) istream)
  "Deserializes a message object of type '<CoaxSetVerbose-response>"
  (setf (ldb (byte 8 0) (slot-value msg 'result)) (read-byte istream))
  msg
)
(defmethod ros-datatype ((msg (eql '<CoaxSetVerbose-response>)))
  "Returns string type for a service object of type '<CoaxSetVerbose-response>"
  "coax_msgs/CoaxSetVerboseResponse")
(defmethod md5sum ((type (eql '<CoaxSetVerbose-response>)))
  "Returns md5sum for a message object of type '<CoaxSetVerbose-response>"
  "a5656957cc9c1010a24927dc8f7dc869")
(defmethod message-definition ((type (eql '<CoaxSetVerbose-response>)))
  "Returns full string definition for message of type '<CoaxSetVerbose-response>"
  (format nil "int8 result~%~%~%"))
(defmethod serialization-length ((msg <CoaxSetVerbose-response>))
  (+ 0
     1
))
(defmethod ros-message-to-list ((msg <CoaxSetVerbose-response>))
  "Converts a ROS message object to a list"
  (list '<CoaxSetVerbose-response>
    (cons ':result (result-val msg))
))
(defmethod service-request-type ((msg (eql 'CoaxSetVerbose)))
  '<CoaxSetVerbose-request>)
(defmethod service-response-type ((msg (eql 'CoaxSetVerbose)))
  '<CoaxSetVerbose-response>)
(defmethod ros-datatype ((msg (eql 'CoaxSetVerbose)))
  "Returns string type for a service object of type '<CoaxSetVerbose>"
  "coax_msgs/CoaxSetVerbose")
