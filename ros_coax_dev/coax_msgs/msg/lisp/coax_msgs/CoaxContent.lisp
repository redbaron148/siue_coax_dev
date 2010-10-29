; Auto-generated. Do not edit!


(in-package coax_msgs-msg)


;//! \htmlinclude CoaxContent.msg.html

(defclass <CoaxContent> (ros-message)
  ()
)
(defmethod serialize ((msg <CoaxContent>) ostream)
  "Serializes a message object of type '<CoaxContent>"
)
(defmethod deserialize ((msg <CoaxContent>) istream)
  "Deserializes a message object of type '<CoaxContent>"
  msg
)
(defmethod ros-datatype ((msg (eql '<CoaxContent>)))
  "Returns string type for a message object of type '<CoaxContent>"
  "coax_msgs/CoaxContent")
(defmethod md5sum ((type (eql '<CoaxContent>)))
  "Returns md5sum for a message object of type '<CoaxContent>"
  "d41d8cd98f00b204e9800998ecf8427e")
(defmethod message-definition ((type (eql '<CoaxContent>)))
  "Returns full string definition for message of type '<CoaxContent>"
  (format nil "# First all the constants~%~%~%~%~%~%~%~%"))
(defmethod serialization-length ((msg <CoaxContent>))
  (+ 0
))
(defmethod ros-message-to-list ((msg <CoaxContent>))
  "Converts a ROS message object to a list"
  (list '<CoaxContent>
))
