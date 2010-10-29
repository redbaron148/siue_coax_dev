; Auto-generated. Do not edit!


(in-package coax_msgs-msg)


;//! \htmlinclude CoaxKeepAlive.msg.html

(defclass <CoaxKeepAlive> (ros-message)
  ((header
    :reader header-val
    :initarg :header
    :type roslib-msg:<Header>
    :initform (make-instance 'roslib-msg:<Header>)))
)
(defmethod serialize ((msg <CoaxKeepAlive>) ostream)
  "Serializes a message object of type '<CoaxKeepAlive>"
  (serialize (slot-value msg 'header) ostream)
)
(defmethod deserialize ((msg <CoaxKeepAlive>) istream)
  "Deserializes a message object of type '<CoaxKeepAlive>"
  (deserialize (slot-value msg 'header) istream)
  msg
)
(defmethod ros-datatype ((msg (eql '<CoaxKeepAlive>)))
  "Returns string type for a message object of type '<CoaxKeepAlive>"
  "coax_msgs/CoaxKeepAlive")
(defmethod md5sum ((type (eql '<CoaxKeepAlive>)))
  "Returns md5sum for a message object of type '<CoaxKeepAlive>"
  "d7be0bb39af8fb9129d5a76e6b63a290")
(defmethod message-definition ((type (eql '<CoaxKeepAlive>)))
  "Returns full string definition for message of type '<CoaxKeepAlive>"
  (format nil "Header header~%~%================================================================================~%MSG: roslib/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(defmethod serialization-length ((msg <CoaxKeepAlive>))
  (+ 0
     (serialization-length (slot-value msg 'header))
))
(defmethod ros-message-to-list ((msg <CoaxKeepAlive>))
  "Converts a ROS message object to a list"
  (list '<CoaxKeepAlive>
    (cons ':header (header-val msg))
))
