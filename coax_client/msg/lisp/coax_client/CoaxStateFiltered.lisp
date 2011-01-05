; Auto-generated. Do not edit!


(in-package coax_client-msg)


;//! \htmlinclude CoaxStateFiltered.msg.html

(defclass <CoaxStateFiltered> (ros-message)
  ((header
    :reader header-val
    :initarg :header
    :type roslib-msg:<Header>
    :initform (make-instance 'roslib-msg:<Header>))
   (ranges
    :reader ranges-val
    :initarg :ranges
    :type (vector float)
   :initform (make-array 3 :element-type 'float :initial-element 0.0)))
)
(defmethod serialize ((msg <CoaxStateFiltered>) ostream)
  "Serializes a message object of type '<CoaxStateFiltered>"
  (serialize (slot-value msg 'header) ostream)
    (map nil #'(lambda (ele) (let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream)))(slot-value msg 'ranges))
)
(defmethod deserialize ((msg <CoaxStateFiltered>) istream)
  "Deserializes a message object of type '<CoaxStateFiltered>"
  (deserialize (slot-value msg 'header) istream)
  (setf (slot-value msg 'ranges) (make-array 3))
  (let ((vals (slot-value msg 'ranges)))
    (dotimes (i 3)
(let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (aref vals i) (roslisp-utils:decode-single-float-bits bits)))))
  msg
)
(defmethod ros-datatype ((msg (eql '<CoaxStateFiltered>)))
  "Returns string type for a message object of type '<CoaxStateFiltered>"
  "coax_client/CoaxStateFiltered")
(defmethod md5sum ((type (eql '<CoaxStateFiltered>)))
  "Returns md5sum for a message object of type '<CoaxStateFiltered>"
  "22ae7269eb624014d345204a967a1082")
(defmethod message-definition ((type (eql '<CoaxStateFiltered>)))
  "Returns full string definition for message of type '<CoaxStateFiltered>"
  (format nil "Header header~%~%float32[3] ranges ~%~%================================================================================~%MSG: roslib/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(defmethod serialization-length ((msg <CoaxStateFiltered>))
  (+ 0
     (serialization-length (slot-value msg 'header))
     0 (reduce #'+ (slot-value msg 'ranges) :key #'(lambda (ele) (declare (ignorable ele)) (+ 4)))
))
(defmethod ros-message-to-list ((msg <CoaxStateFiltered>))
  "Converts a ROS message object to a list"
  (list '<CoaxStateFiltered>
    (cons ':header (header-val msg))
    (cons ':ranges (ranges-val msg))
))
