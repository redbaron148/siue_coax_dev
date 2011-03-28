; Auto-generated. Do not edit!


(in-package coax_msgs-srv)


;//! \htmlinclude CoaxReachNavState-request.msg.html

(defclass <CoaxReachNavState-request> (ros-message)
  ((desiredState
    :reader desiredState-val
    :initarg :desiredState
    :type fixnum
    :initform 0)
   (timeout
    :reader timeout-val
    :initarg :timeout
    :type float
    :initform 0.0))
)
(defmethod serialize ((msg <CoaxReachNavState-request>) ostream)
  "Serializes a message object of type '<CoaxReachNavState-request>"
    (write-byte (ldb (byte 8 0) (slot-value msg 'desiredState)) ostream)
  (let ((bits (roslisp-utils:encode-single-float-bits (slot-value msg 'timeout))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream))
)
(defmethod deserialize ((msg <CoaxReachNavState-request>) istream)
  "Deserializes a message object of type '<CoaxReachNavState-request>"
  (setf (ldb (byte 8 0) (slot-value msg 'desiredState)) (read-byte istream))
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (slot-value msg 'timeout) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(defmethod ros-datatype ((msg (eql '<CoaxReachNavState-request>)))
  "Returns string type for a service object of type '<CoaxReachNavState-request>"
  "coax_msgs/CoaxReachNavStateRequest")
(defmethod md5sum ((type (eql '<CoaxReachNavState-request>)))
  "Returns md5sum for a message object of type '<CoaxReachNavState-request>"
  "9dad7fac31028e676b30d95151fbf893")
(defmethod message-definition ((type (eql '<CoaxReachNavState-request>)))
  "Returns full string definition for message of type '<CoaxReachNavState-request>"
  (format nil "uint8 desiredState~%float32 timeout~%~%"))
(defmethod serialization-length ((msg <CoaxReachNavState-request>))
  (+ 0
     1
     4
))
(defmethod ros-message-to-list ((msg <CoaxReachNavState-request>))
  "Converts a ROS message object to a list"
  (list '<CoaxReachNavState-request>
    (cons ':desiredState (desiredState-val msg))
    (cons ':timeout (timeout-val msg))
))
;//! \htmlinclude CoaxReachNavState-response.msg.html

(defclass <CoaxReachNavState-response> (ros-message)
  ((result
    :reader result-val
    :initarg :result
    :type fixnum
    :initform 0))
)
(defmethod serialize ((msg <CoaxReachNavState-response>) ostream)
  "Serializes a message object of type '<CoaxReachNavState-response>"
    (write-byte (ldb (byte 8 0) (slot-value msg 'result)) ostream)
)
(defmethod deserialize ((msg <CoaxReachNavState-response>) istream)
  "Deserializes a message object of type '<CoaxReachNavState-response>"
  (setf (ldb (byte 8 0) (slot-value msg 'result)) (read-byte istream))
  msg
)
(defmethod ros-datatype ((msg (eql '<CoaxReachNavState-response>)))
  "Returns string type for a service object of type '<CoaxReachNavState-response>"
  "coax_msgs/CoaxReachNavStateResponse")
(defmethod md5sum ((type (eql '<CoaxReachNavState-response>)))
  "Returns md5sum for a message object of type '<CoaxReachNavState-response>"
  "9dad7fac31028e676b30d95151fbf893")
(defmethod message-definition ((type (eql '<CoaxReachNavState-response>)))
  "Returns full string definition for message of type '<CoaxReachNavState-response>"
  (format nil "int8 result~%~%~%"))
(defmethod serialization-length ((msg <CoaxReachNavState-response>))
  (+ 0
     1
))
(defmethod ros-message-to-list ((msg <CoaxReachNavState-response>))
  "Converts a ROS message object to a list"
  (list '<CoaxReachNavState-response>
    (cons ':result (result-val msg))
))
(defmethod service-request-type ((msg (eql 'CoaxReachNavState)))
  '<CoaxReachNavState-request>)
(defmethod service-response-type ((msg (eql 'CoaxReachNavState)))
  '<CoaxReachNavState-response>)
(defmethod ros-datatype ((msg (eql 'CoaxReachNavState)))
  "Returns string type for a service object of type '<CoaxReachNavState>"
  "coax_msgs/CoaxReachNavState")
