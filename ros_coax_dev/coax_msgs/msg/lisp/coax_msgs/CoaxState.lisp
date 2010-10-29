; Auto-generated. Do not edit!


(in-package coax_msgs-msg)


;//! \htmlinclude CoaxState.msg.html

(defclass <CoaxState> (ros-message)
  ((header
    :reader header-val
    :initarg :header
    :type roslib-msg:<Header>
    :initform (make-instance 'roslib-msg:<Header>))
   (errorFlags
    :reader errorFlags-val
    :initarg :errorFlags
    :type fixnum
    :initform 0)
   (content
    :reader content-val
    :initarg :content
    :type integer
    :initform 0)
   (timeStamp
    :reader timeStamp-val
    :initarg :timeStamp
    :type integer
    :initform 0)
   (controlTimeout
    :reader controlTimeout-val
    :initarg :controlTimeout
    :type fixnum
    :initform 0)
   (watchdogTimeout
    :reader watchdogTimeout-val
    :initarg :watchdogTimeout
    :type fixnum
    :initform 0)
   (mode
    :reader mode-val
    :initarg :mode
    :type coax_msgs-msg:<CoaxModes>
    :initform (make-instance 'coax_msgs-msg:<CoaxModes>))
   (roll
    :reader roll-val
    :initarg :roll
    :type float
    :initform 0.0)
   (pitch
    :reader pitch-val
    :initarg :pitch
    :type float
    :initform 0.0)
   (yaw
    :reader yaw-val
    :initarg :yaw
    :type float
    :initform 0.0)
   (gyro
    :reader gyro-val
    :initarg :gyro
    :type (vector float)
   :initform (make-array 3 :element-type 'float :initial-element 0.0))
   (accel
    :reader accel-val
    :initarg :accel
    :type (vector float)
   :initform (make-array 3 :element-type 'float :initial-element 0.0))
   (magneto
    :reader magneto-val
    :initarg :magneto
    :type (vector float)
   :initform (make-array 3 :element-type 'float :initial-element 0.0))
   (imutemp
    :reader imutemp-val
    :initarg :imutemp
    :type float
    :initform 0.0)
   (zrange
    :reader zrange-val
    :initarg :zrange
    :type float
    :initform 0.0)
   (zfiltered
    :reader zfiltered-val
    :initarg :zfiltered
    :type float
    :initform 0.0)
   (pressure
    :reader pressure-val
    :initarg :pressure
    :type float
    :initform 0.0)
   (hranges
    :reader hranges-val
    :initarg :hranges
    :type (vector float)
   :initform (make-array 4 :element-type 'float :initial-element 0.0))
   (xrel
    :reader xrel-val
    :initarg :xrel
    :type float
    :initform 0.0)
   (yrel
    :reader yrel-val
    :initarg :yrel
    :type float
    :initform 0.0)
   (battery
    :reader battery-val
    :initarg :battery
    :type float
    :initform 0.0)
   (rcChannel
    :reader rcChannel-val
    :initarg :rcChannel
    :type (vector float)
   :initform (make-array 8 :element-type 'float :initial-element 0.0))
   (o_attitude
    :reader o_attitude-val
    :initarg :o_attitude
    :type (vector float)
   :initform (make-array 3 :element-type 'float :initial-element 0.0))
   (o_altitude
    :reader o_altitude-val
    :initarg :o_altitude
    :type float
    :initform 0.0)
   (o_tol
    :reader o_tol-val
    :initarg :o_tol
    :type float
    :initform 0.0)
   (o_xy
    :reader o_xy-val
    :initarg :o_xy
    :type (vector float)
   :initform (make-array 2 :element-type 'float :initial-element 0.0))
   (o_oavoid
    :reader o_oavoid-val
    :initarg :o_oavoid
    :type (vector float)
   :initform (make-array 2 :element-type 'float :initial-element 0.0)))
)
(defmethod serialize ((msg <CoaxState>) ostream)
  "Serializes a message object of type '<CoaxState>"
  (serialize (slot-value msg 'header) ostream)
    (write-byte (ldb (byte 8 0) (slot-value msg 'errorFlags)) ostream)
    (write-byte (ldb (byte 8 0) (slot-value msg 'content)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'content)) ostream)
  (write-byte (ldb (byte 8 16) (slot-value msg 'content)) ostream)
  (write-byte (ldb (byte 8 24) (slot-value msg 'content)) ostream)
    (write-byte (ldb (byte 8 0) (slot-value msg 'timeStamp)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'timeStamp)) ostream)
  (write-byte (ldb (byte 8 16) (slot-value msg 'timeStamp)) ostream)
  (write-byte (ldb (byte 8 24) (slot-value msg 'timeStamp)) ostream)
    (write-byte (ldb (byte 8 0) (slot-value msg 'controlTimeout)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'controlTimeout)) ostream)
    (write-byte (ldb (byte 8 0) (slot-value msg 'watchdogTimeout)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'watchdogTimeout)) ostream)
  (serialize (slot-value msg 'mode) ostream)
  (let ((bits (roslisp-utils:encode-single-float-bits (slot-value msg 'roll))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream))
  (let ((bits (roslisp-utils:encode-single-float-bits (slot-value msg 'pitch))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream))
  (let ((bits (roslisp-utils:encode-single-float-bits (slot-value msg 'yaw))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream))
    (map nil #'(lambda (ele) (let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream)))(slot-value msg 'gyro))
    (map nil #'(lambda (ele) (let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream)))(slot-value msg 'accel))
    (map nil #'(lambda (ele) (let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream)))(slot-value msg 'magneto))
  (let ((bits (roslisp-utils:encode-single-float-bits (slot-value msg 'imutemp))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream))
  (let ((bits (roslisp-utils:encode-single-float-bits (slot-value msg 'zrange))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream))
  (let ((bits (roslisp-utils:encode-single-float-bits (slot-value msg 'zfiltered))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream))
  (let ((bits (roslisp-utils:encode-single-float-bits (slot-value msg 'pressure))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream))
    (map nil #'(lambda (ele) (let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream)))(slot-value msg 'hranges))
  (let ((bits (roslisp-utils:encode-single-float-bits (slot-value msg 'xrel))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream))
  (let ((bits (roslisp-utils:encode-single-float-bits (slot-value msg 'yrel))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream))
  (let ((bits (roslisp-utils:encode-single-float-bits (slot-value msg 'battery))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream))
    (map nil #'(lambda (ele) (let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream)))(slot-value msg 'rcChannel))
    (map nil #'(lambda (ele) (let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream)))(slot-value msg 'o_attitude))
  (let ((bits (roslisp-utils:encode-single-float-bits (slot-value msg 'o_altitude))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream))
  (let ((bits (roslisp-utils:encode-single-float-bits (slot-value msg 'o_tol))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream))
    (map nil #'(lambda (ele) (let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream)))(slot-value msg 'o_xy))
    (map nil #'(lambda (ele) (let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream)))(slot-value msg 'o_oavoid))
)
(defmethod deserialize ((msg <CoaxState>) istream)
  "Deserializes a message object of type '<CoaxState>"
  (deserialize (slot-value msg 'header) istream)
  (setf (ldb (byte 8 0) (slot-value msg 'errorFlags)) (read-byte istream))
  (setf (ldb (byte 8 0) (slot-value msg 'content)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'content)) (read-byte istream))
  (setf (ldb (byte 8 16) (slot-value msg 'content)) (read-byte istream))
  (setf (ldb (byte 8 24) (slot-value msg 'content)) (read-byte istream))
  (setf (ldb (byte 8 0) (slot-value msg 'timeStamp)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'timeStamp)) (read-byte istream))
  (setf (ldb (byte 8 16) (slot-value msg 'timeStamp)) (read-byte istream))
  (setf (ldb (byte 8 24) (slot-value msg 'timeStamp)) (read-byte istream))
  (setf (ldb (byte 8 0) (slot-value msg 'controlTimeout)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'controlTimeout)) (read-byte istream))
  (setf (ldb (byte 8 0) (slot-value msg 'watchdogTimeout)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'watchdogTimeout)) (read-byte istream))
  (deserialize (slot-value msg 'mode) istream)
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (slot-value msg 'roll) (roslisp-utils:decode-single-float-bits bits)))
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (slot-value msg 'pitch) (roslisp-utils:decode-single-float-bits bits)))
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (slot-value msg 'yaw) (roslisp-utils:decode-single-float-bits bits)))
  (setf (slot-value msg 'gyro) (make-array 3))
  (let ((vals (slot-value msg 'gyro)))
    (dotimes (i 3)
(let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (aref vals i) (roslisp-utils:decode-single-float-bits bits)))))
  (setf (slot-value msg 'accel) (make-array 3))
  (let ((vals (slot-value msg 'accel)))
    (dotimes (i 3)
(let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (aref vals i) (roslisp-utils:decode-single-float-bits bits)))))
  (setf (slot-value msg 'magneto) (make-array 3))
  (let ((vals (slot-value msg 'magneto)))
    (dotimes (i 3)
(let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (aref vals i) (roslisp-utils:decode-single-float-bits bits)))))
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (slot-value msg 'imutemp) (roslisp-utils:decode-single-float-bits bits)))
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (slot-value msg 'zrange) (roslisp-utils:decode-single-float-bits bits)))
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (slot-value msg 'zfiltered) (roslisp-utils:decode-single-float-bits bits)))
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (slot-value msg 'pressure) (roslisp-utils:decode-single-float-bits bits)))
  (setf (slot-value msg 'hranges) (make-array 4))
  (let ((vals (slot-value msg 'hranges)))
    (dotimes (i 4)
(let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (aref vals i) (roslisp-utils:decode-single-float-bits bits)))))
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (slot-value msg 'xrel) (roslisp-utils:decode-single-float-bits bits)))
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (slot-value msg 'yrel) (roslisp-utils:decode-single-float-bits bits)))
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (slot-value msg 'battery) (roslisp-utils:decode-single-float-bits bits)))
  (setf (slot-value msg 'rcChannel) (make-array 8))
  (let ((vals (slot-value msg 'rcChannel)))
    (dotimes (i 8)
(let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (aref vals i) (roslisp-utils:decode-single-float-bits bits)))))
  (setf (slot-value msg 'o_attitude) (make-array 3))
  (let ((vals (slot-value msg 'o_attitude)))
    (dotimes (i 3)
(let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (aref vals i) (roslisp-utils:decode-single-float-bits bits)))))
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (slot-value msg 'o_altitude) (roslisp-utils:decode-single-float-bits bits)))
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (slot-value msg 'o_tol) (roslisp-utils:decode-single-float-bits bits)))
  (setf (slot-value msg 'o_xy) (make-array 2))
  (let ((vals (slot-value msg 'o_xy)))
    (dotimes (i 2)
(let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (aref vals i) (roslisp-utils:decode-single-float-bits bits)))))
  (setf (slot-value msg 'o_oavoid) (make-array 2))
  (let ((vals (slot-value msg 'o_oavoid)))
    (dotimes (i 2)
(let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (aref vals i) (roslisp-utils:decode-single-float-bits bits)))))
  msg
)
(defmethod ros-datatype ((msg (eql '<CoaxState>)))
  "Returns string type for a message object of type '<CoaxState>"
  "coax_msgs/CoaxState")
(defmethod md5sum ((type (eql '<CoaxState>)))
  "Returns md5sum for a message object of type '<CoaxState>"
  "05663407b1d685a3132efbff03046970")
(defmethod message-definition ((type (eql '<CoaxState>)))
  "Returns full string definition for message of type '<CoaxState>"
  (format nil "Header header~%# Error status set by the helicopter */~%uint8 errorFlags~%# ~%#	 Affected content in this data structure (~%#    Use AND with the SBS_... flags above to check the content  ~%#    e.g: if (state.content & SBS_RPY) {  ~%#   			compute_odo(state.roll,state.pitch,state.yaw)  ~%#   		}  ~%#    This content should correspond to what has been configured in  ~%#    sbConfigureComm or requested in sbRequestState  *#~%uint32 content~%# timestamp of the last update, in ms since the initialisation of the~%#  helicopter. *#~%uint32 timeStamp~%# current control timeout (for sending command in SB_NAV_CTRLLED mode) */~%uint16 controlTimeout~%# current comm timeout, to bring the helicopter back to safety is~%#  communication is not maintained. *#~%uint16 watchdogTimeout~%# Various bit field to represent the system configuration*/~%CoaxModes mode~%~%# Current helicopter attitude */~%float32 roll~%float32 pitch~%float32 yaw~%# GYRO data */~%float32[3] gyro~%# Accelerometer data */~%float32[3] accel~%# Magnetometer data */~%float32[3] magneto~%# Temperature measured by IMU */~%float32 imutemp~%# Range measurement in the vertical direction */~%float32 zrange~%# Filtered altitude, as used by the altitude control in POS mode */~%float32 zfiltered~%# Output of pressure sensor */~%float32 pressure~%# Range measurements in the horizontal plane. Sensor placement is~%#  platform dependent *#~%float32[4] hranges~%# Distance to closest obstacle (if implemented) */~%float32 xrel~%float32 yrel~%# Battery voltage */~%float32 battery~%# Output of the remote control channel, normalised to [-1,1] */~%float32[8] rcChannel~%~%# symbols below may be suppressed in future version of the library */~%~%# Output of attitude control (semantic unclear) */~%float32[3] o_attitude~%# Output of altitude control, i.e. thrust to keep the helicopter affloat32  */~%float32 o_altitude~%# Output of take-off/landing control (semantic unclear) */~%float32 o_tol~%# ??? (semantic unclear) */~%float32[2] o_xy~%# ??? (semantic unclear) */~%float32[2] o_oavoid~%~%~%================================================================================~%MSG: roslib/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: coax_msgs/CoaxModes~%# Navigation mode: SB_NAV_... */~%uint8 navigation~%# Communication mode: SB_COM_... */~%uint8 communication~%# Obstacle avoidance mode: or of SB_OA_... */~%uint8 oavoid~%# Control mode for roll axis: SB_CTRL_... */~%uint8 rollAxis~%# Control mode for pitch axis: SB_CTRL_... */~%uint8 pitchAxis~%# Control mode for yaw axis: SB_CTRL_... */~%uint8 yawAxis~%# Control mode for altitude axis: SB_CTRL_... */~%uint8 altAxis~%~%~%"))
(defmethod serialization-length ((msg <CoaxState>))
  (+ 0
     (serialization-length (slot-value msg 'header))
     1
     4
     4
     2
     2
     (serialization-length (slot-value msg 'mode))
     4
     4
     4
     0 (reduce #'+ (slot-value msg 'gyro) :key #'(lambda (ele) (declare (ignorable ele)) (+ 4)))
     0 (reduce #'+ (slot-value msg 'accel) :key #'(lambda (ele) (declare (ignorable ele)) (+ 4)))
     0 (reduce #'+ (slot-value msg 'magneto) :key #'(lambda (ele) (declare (ignorable ele)) (+ 4)))
     4
     4
     4
     4
     0 (reduce #'+ (slot-value msg 'hranges) :key #'(lambda (ele) (declare (ignorable ele)) (+ 4)))
     4
     4
     4
     0 (reduce #'+ (slot-value msg 'rcChannel) :key #'(lambda (ele) (declare (ignorable ele)) (+ 4)))
     0 (reduce #'+ (slot-value msg 'o_attitude) :key #'(lambda (ele) (declare (ignorable ele)) (+ 4)))
     4
     4
     0 (reduce #'+ (slot-value msg 'o_xy) :key #'(lambda (ele) (declare (ignorable ele)) (+ 4)))
     0 (reduce #'+ (slot-value msg 'o_oavoid) :key #'(lambda (ele) (declare (ignorable ele)) (+ 4)))
))
(defmethod ros-message-to-list ((msg <CoaxState>))
  "Converts a ROS message object to a list"
  (list '<CoaxState>
    (cons ':header (header-val msg))
    (cons ':errorFlags (errorFlags-val msg))
    (cons ':content (content-val msg))
    (cons ':timeStamp (timeStamp-val msg))
    (cons ':controlTimeout (controlTimeout-val msg))
    (cons ':watchdogTimeout (watchdogTimeout-val msg))
    (cons ':mode (mode-val msg))
    (cons ':roll (roll-val msg))
    (cons ':pitch (pitch-val msg))
    (cons ':yaw (yaw-val msg))
    (cons ':gyro (gyro-val msg))
    (cons ':accel (accel-val msg))
    (cons ':magneto (magneto-val msg))
    (cons ':imutemp (imutemp-val msg))
    (cons ':zrange (zrange-val msg))
    (cons ':zfiltered (zfiltered-val msg))
    (cons ':pressure (pressure-val msg))
    (cons ':hranges (hranges-val msg))
    (cons ':xrel (xrel-val msg))
    (cons ':yrel (yrel-val msg))
    (cons ':battery (battery-val msg))
    (cons ':rcChannel (rcChannel-val msg))
    (cons ':o_attitude (o_attitude-val msg))
    (cons ':o_altitude (o_altitude-val msg))
    (cons ':o_tol (o_tol-val msg))
    (cons ':o_xy (o_xy-val msg))
    (cons ':o_oavoid (o_oavoid-val msg))
))
