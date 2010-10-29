; Auto-generated. Do not edit!


(in-package coax_msgs-msg)


;//! \htmlinclude CoaxModes.msg.html

(defclass <CoaxModes> (ros-message)
  ((navigation
    :reader navigation-val
    :initarg :navigation
    :type fixnum
    :initform 0)
   (communication
    :reader communication-val
    :initarg :communication
    :type fixnum
    :initform 0)
   (oavoid
    :reader oavoid-val
    :initarg :oavoid
    :type fixnum
    :initform 0)
   (rollAxis
    :reader rollAxis-val
    :initarg :rollAxis
    :type fixnum
    :initform 0)
   (pitchAxis
    :reader pitchAxis-val
    :initarg :pitchAxis
    :type fixnum
    :initform 0)
   (yawAxis
    :reader yawAxis-val
    :initarg :yawAxis
    :type fixnum
    :initform 0)
   (altAxis
    :reader altAxis-val
    :initarg :altAxis
    :type fixnum
    :initform 0))
)
(defmethod serialize ((msg <CoaxModes>) ostream)
  "Serializes a message object of type '<CoaxModes>"
    (write-byte (ldb (byte 8 0) (slot-value msg 'navigation)) ostream)
    (write-byte (ldb (byte 8 0) (slot-value msg 'communication)) ostream)
    (write-byte (ldb (byte 8 0) (slot-value msg 'oavoid)) ostream)
    (write-byte (ldb (byte 8 0) (slot-value msg 'rollAxis)) ostream)
    (write-byte (ldb (byte 8 0) (slot-value msg 'pitchAxis)) ostream)
    (write-byte (ldb (byte 8 0) (slot-value msg 'yawAxis)) ostream)
    (write-byte (ldb (byte 8 0) (slot-value msg 'altAxis)) ostream)
)
(defmethod deserialize ((msg <CoaxModes>) istream)
  "Deserializes a message object of type '<CoaxModes>"
  (setf (ldb (byte 8 0) (slot-value msg 'navigation)) (read-byte istream))
  (setf (ldb (byte 8 0) (slot-value msg 'communication)) (read-byte istream))
  (setf (ldb (byte 8 0) (slot-value msg 'oavoid)) (read-byte istream))
  (setf (ldb (byte 8 0) (slot-value msg 'rollAxis)) (read-byte istream))
  (setf (ldb (byte 8 0) (slot-value msg 'pitchAxis)) (read-byte istream))
  (setf (ldb (byte 8 0) (slot-value msg 'yawAxis)) (read-byte istream))
  (setf (ldb (byte 8 0) (slot-value msg 'altAxis)) (read-byte istream))
  msg
)
(defmethod ros-datatype ((msg (eql '<CoaxModes>)))
  "Returns string type for a message object of type '<CoaxModes>"
  "coax_msgs/CoaxModes")
(defmethod md5sum ((type (eql '<CoaxModes>)))
  "Returns md5sum for a message object of type '<CoaxModes>"
  "6ddc58fd5953a0b8a93f392132f7417d")
(defmethod message-definition ((type (eql '<CoaxModes>)))
  "Returns full string definition for message of type '<CoaxModes>"
  (format nil "# Navigation mode: SB_NAV_... */~%uint8 navigation~%# Communication mode: SB_COM_... */~%uint8 communication~%# Obstacle avoidance mode: or of SB_OA_... */~%uint8 oavoid~%# Control mode for roll axis: SB_CTRL_... */~%uint8 rollAxis~%# Control mode for pitch axis: SB_CTRL_... */~%uint8 pitchAxis~%# Control mode for yaw axis: SB_CTRL_... */~%uint8 yawAxis~%# Control mode for altitude axis: SB_CTRL_... */~%uint8 altAxis~%~%~%"))
(defmethod serialization-length ((msg <CoaxModes>))
  (+ 0
     1
     1
     1
     1
     1
     1
     1
))
(defmethod ros-message-to-list ((msg <CoaxModes>))
  "Converts a ROS message object to a list"
  (list '<CoaxModes>
    (cons ':navigation (navigation-val msg))
    (cons ':communication (communication-val msg))
    (cons ':oavoid (oavoid-val msg))
    (cons ':rollAxis (rollAxis-val msg))
    (cons ':pitchAxis (pitchAxis-val msg))
    (cons ':yawAxis (yawAxis-val msg))
    (cons ':altAxis (altAxis-val msg))
))
