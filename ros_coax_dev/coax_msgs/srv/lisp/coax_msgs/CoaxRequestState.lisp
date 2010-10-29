; Auto-generated. Do not edit!


(in-package coax_msgs-srv)


;//! \htmlinclude CoaxRequestState-request.msg.html

(defclass <CoaxRequestState-request> (ros-message)
  ((contents
    :reader contents-val
    :initarg :contents
    :type integer
    :initform 0))
)
(defmethod serialize ((msg <CoaxRequestState-request>) ostream)
  "Serializes a message object of type '<CoaxRequestState-request>"
    (write-byte (ldb (byte 8 0) (slot-value msg 'contents)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'contents)) ostream)
  (write-byte (ldb (byte 8 16) (slot-value msg 'contents)) ostream)
  (write-byte (ldb (byte 8 24) (slot-value msg 'contents)) ostream)
)
(defmethod deserialize ((msg <CoaxRequestState-request>) istream)
  "Deserializes a message object of type '<CoaxRequestState-request>"
  (setf (ldb (byte 8 0) (slot-value msg 'contents)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'contents)) (read-byte istream))
  (setf (ldb (byte 8 16) (slot-value msg 'contents)) (read-byte istream))
  (setf (ldb (byte 8 24) (slot-value msg 'contents)) (read-byte istream))
  msg
)
(defmethod ros-datatype ((msg (eql '<CoaxRequestState-request>)))
  "Returns string type for a service object of type '<CoaxRequestState-request>"
  "coax_msgs/CoaxRequestStateRequest")
(defmethod md5sum ((type (eql '<CoaxRequestState-request>)))
  "Returns md5sum for a message object of type '<CoaxRequestState-request>"
  "c48ce683e12ad9d0e530beb095838004")
(defmethod message-definition ((type (eql '<CoaxRequestState-request>)))
  "Returns full string definition for message of type '<CoaxRequestState-request>"
  (format nil "uint32 contents~%~%"))
(defmethod serialization-length ((msg <CoaxRequestState-request>))
  (+ 0
     4
))
(defmethod ros-message-to-list ((msg <CoaxRequestState-request>))
  "Converts a ROS message object to a list"
  (list '<CoaxRequestState-request>
    (cons ':contents (contents-val msg))
))
;//! \htmlinclude CoaxRequestState-response.msg.html

(defclass <CoaxRequestState-response> (ros-message)
  ((state
    :reader state-val
    :initarg :state
    :type coax_msgs-msg:<CoaxState>
    :initform (make-instance 'coax_msgs-msg:<CoaxState>)))
)
(defmethod serialize ((msg <CoaxRequestState-response>) ostream)
  "Serializes a message object of type '<CoaxRequestState-response>"
  (serialize (slot-value msg 'state) ostream)
)
(defmethod deserialize ((msg <CoaxRequestState-response>) istream)
  "Deserializes a message object of type '<CoaxRequestState-response>"
  (deserialize (slot-value msg 'state) istream)
  msg
)
(defmethod ros-datatype ((msg (eql '<CoaxRequestState-response>)))
  "Returns string type for a service object of type '<CoaxRequestState-response>"
  "coax_msgs/CoaxRequestStateResponse")
(defmethod md5sum ((type (eql '<CoaxRequestState-response>)))
  "Returns md5sum for a message object of type '<CoaxRequestState-response>"
  "c48ce683e12ad9d0e530beb095838004")
(defmethod message-definition ((type (eql '<CoaxRequestState-response>)))
  "Returns full string definition for message of type '<CoaxRequestState-response>"
  (format nil "CoaxState state~%~%~%================================================================================~%MSG: coax_msgs/CoaxState~%Header header~%# Error status set by the helicopter */~%uint8 errorFlags~%# ~%#	 Affected content in this data structure (~%#    Use AND with the SBS_... flags above to check the content  ~%#    e.g: if (state.content & SBS_RPY) {  ~%#   			compute_odo(state.roll,state.pitch,state.yaw)  ~%#   		}  ~%#    This content should correspond to what has been configured in  ~%#    sbConfigureComm or requested in sbRequestState  *#~%uint32 content~%# timestamp of the last update, in ms since the initialisation of the~%#  helicopter. *#~%uint32 timeStamp~%# current control timeout (for sending command in SB_NAV_CTRLLED mode) */~%uint16 controlTimeout~%# current comm timeout, to bring the helicopter back to safety is~%#  communication is not maintained. *#~%uint16 watchdogTimeout~%# Various bit field to represent the system configuration*/~%CoaxModes mode~%~%# Current helicopter attitude */~%float32 roll~%float32 pitch~%float32 yaw~%# GYRO data */~%float32[3] gyro~%# Accelerometer data */~%float32[3] accel~%# Magnetometer data */~%float32[3] magneto~%# Temperature measured by IMU */~%float32 imutemp~%# Range measurement in the vertical direction */~%float32 zrange~%# Filtered altitude, as used by the altitude control in POS mode */~%float32 zfiltered~%# Output of pressure sensor */~%float32 pressure~%# Range measurements in the horizontal plane. Sensor placement is~%#  platform dependent *#~%float32[4] hranges~%# Distance to closest obstacle (if implemented) */~%float32 xrel~%float32 yrel~%# Battery voltage */~%float32 battery~%# Output of the remote control channel, normalised to [-1,1] */~%float32[8] rcChannel~%~%# symbols below may be suppressed in future version of the library */~%~%# Output of attitude control (semantic unclear) */~%float32[3] o_attitude~%# Output of altitude control, i.e. thrust to keep the helicopter affloat32  */~%float32 o_altitude~%# Output of take-off/landing control (semantic unclear) */~%float32 o_tol~%# ??? (semantic unclear) */~%float32[2] o_xy~%# ??? (semantic unclear) */~%float32[2] o_oavoid~%~%~%================================================================================~%MSG: roslib/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: coax_msgs/CoaxModes~%# Navigation mode: SB_NAV_... */~%uint8 navigation~%# Communication mode: SB_COM_... */~%uint8 communication~%# Obstacle avoidance mode: or of SB_OA_... */~%uint8 oavoid~%# Control mode for roll axis: SB_CTRL_... */~%uint8 rollAxis~%# Control mode for pitch axis: SB_CTRL_... */~%uint8 pitchAxis~%# Control mode for yaw axis: SB_CTRL_... */~%uint8 yawAxis~%# Control mode for altitude axis: SB_CTRL_... */~%uint8 altAxis~%~%~%"))
(defmethod serialization-length ((msg <CoaxRequestState-response>))
  (+ 0
     (serialization-length (slot-value msg 'state))
))
(defmethod ros-message-to-list ((msg <CoaxRequestState-response>))
  "Converts a ROS message object to a list"
  (list '<CoaxRequestState-response>
    (cons ':state (state-val msg))
))
(defmethod service-request-type ((msg (eql 'CoaxRequestState)))
  '<CoaxRequestState-request>)
(defmethod service-response-type ((msg (eql 'CoaxRequestState)))
  '<CoaxRequestState-response>)
(defmethod ros-datatype ((msg (eql 'CoaxRequestState)))
  "Returns string type for a service object of type '<CoaxRequestState>"
  "coax_msgs/CoaxRequestState")
