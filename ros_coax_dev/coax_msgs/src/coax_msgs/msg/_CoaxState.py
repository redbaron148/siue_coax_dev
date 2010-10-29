"""autogenerated by genmsg_py from CoaxState.msg. Do not edit."""
import roslib.message
import struct

import coax_msgs.msg
import roslib.msg

class CoaxState(roslib.message.Message):
  _md5sum = "05663407b1d685a3132efbff03046970"
  _type = "coax_msgs/CoaxState"
  _has_header = True #flag to mark the presence of a Header object
  _full_text = """Header header
# Error status set by the helicopter */
uint8 errorFlags
# 
#	 Affected content in this data structure (
#    Use AND with the SBS_... flags above to check the content  
#    e.g: if (state.content & SBS_RPY) {  
#   			compute_odo(state.roll,state.pitch,state.yaw)  
#   		}  
#    This content should correspond to what has been configured in  
#    sbConfigureComm or requested in sbRequestState  *#
uint32 content
# timestamp of the last update, in ms since the initialisation of the
#  helicopter. *#
uint32 timeStamp
# current control timeout (for sending command in SB_NAV_CTRLLED mode) */
uint16 controlTimeout
# current comm timeout, to bring the helicopter back to safety is
#  communication is not maintained. *#
uint16 watchdogTimeout
# Various bit field to represent the system configuration*/
CoaxModes mode

# Current helicopter attitude */
float32 roll
float32 pitch
float32 yaw
# GYRO data */
float32[3] gyro
# Accelerometer data */
float32[3] accel
# Magnetometer data */
float32[3] magneto
# Temperature measured by IMU */
float32 imutemp
# Range measurement in the vertical direction */
float32 zrange
# Filtered altitude, as used by the altitude control in POS mode */
float32 zfiltered
# Output of pressure sensor */
float32 pressure
# Range measurements in the horizontal plane. Sensor placement is
#  platform dependent *#
float32[4] hranges
# Distance to closest obstacle (if implemented) */
float32 xrel
float32 yrel
# Battery voltage */
float32 battery
# Output of the remote control channel, normalised to [-1,1] */
float32[8] rcChannel

# symbols below may be suppressed in future version of the library */

# Output of attitude control (semantic unclear) */
float32[3] o_attitude
# Output of altitude control, i.e. thrust to keep the helicopter affloat32  */
float32 o_altitude
# Output of take-off/landing control (semantic unclear) */
float32 o_tol
# ??? (semantic unclear) */
float32[2] o_xy
# ??? (semantic unclear) */
float32[2] o_oavoid


================================================================================
MSG: roslib/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.secs: seconds (stamp_secs) since epoch
# * stamp.nsecs: nanoseconds since stamp_secs
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
# 0: no frame
# 1: global frame
string frame_id

================================================================================
MSG: coax_msgs/CoaxModes
# Navigation mode: SB_NAV_... */
uint8 navigation
# Communication mode: SB_COM_... */
uint8 communication
# Obstacle avoidance mode: or of SB_OA_... */
uint8 oavoid
# Control mode for roll axis: SB_CTRL_... */
uint8 rollAxis
# Control mode for pitch axis: SB_CTRL_... */
uint8 pitchAxis
# Control mode for yaw axis: SB_CTRL_... */
uint8 yawAxis
# Control mode for altitude axis: SB_CTRL_... */
uint8 altAxis

"""
  __slots__ = ['header','errorFlags','content','timeStamp','controlTimeout','watchdogTimeout','mode','roll','pitch','yaw','gyro','accel','magneto','imutemp','zrange','zfiltered','pressure','hranges','xrel','yrel','battery','rcChannel','o_attitude','o_altitude','o_tol','o_xy','o_oavoid']
  _slot_types = ['Header','uint8','uint32','uint32','uint16','uint16','coax_msgs/CoaxModes','float32','float32','float32','float32[3]','float32[3]','float32[3]','float32','float32','float32','float32','float32[4]','float32','float32','float32','float32[8]','float32[3]','float32','float32','float32[2]','float32[2]']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.
    
    The available fields are:
       header,errorFlags,content,timeStamp,controlTimeout,watchdogTimeout,mode,roll,pitch,yaw,gyro,accel,magneto,imutemp,zrange,zfiltered,pressure,hranges,xrel,yrel,battery,rcChannel,o_attitude,o_altitude,o_tol,o_xy,o_oavoid
    
    @param args: complete set of field values, in .msg order
    @param kwds: use keyword arguments corresponding to message field names
    to set specific fields. 
    """
    if args or kwds:
      super(CoaxState, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.header is None:
        self.header = roslib.msg._Header.Header()
      if self.errorFlags is None:
        self.errorFlags = 0
      if self.content is None:
        self.content = 0
      if self.timeStamp is None:
        self.timeStamp = 0
      if self.controlTimeout is None:
        self.controlTimeout = 0
      if self.watchdogTimeout is None:
        self.watchdogTimeout = 0
      if self.mode is None:
        self.mode = coax_msgs.msg.CoaxModes()
      if self.roll is None:
        self.roll = 0.
      if self.pitch is None:
        self.pitch = 0.
      if self.yaw is None:
        self.yaw = 0.
      if self.gyro is None:
        self.gyro = [0.,0.,0.]
      if self.accel is None:
        self.accel = [0.,0.,0.]
      if self.magneto is None:
        self.magneto = [0.,0.,0.]
      if self.imutemp is None:
        self.imutemp = 0.
      if self.zrange is None:
        self.zrange = 0.
      if self.zfiltered is None:
        self.zfiltered = 0.
      if self.pressure is None:
        self.pressure = 0.
      if self.hranges is None:
        self.hranges = [0.,0.,0.,0.]
      if self.xrel is None:
        self.xrel = 0.
      if self.yrel is None:
        self.yrel = 0.
      if self.battery is None:
        self.battery = 0.
      if self.rcChannel is None:
        self.rcChannel = [0.,0.,0.,0.,0.,0.,0.,0.]
      if self.o_attitude is None:
        self.o_attitude = [0.,0.,0.]
      if self.o_altitude is None:
        self.o_altitude = 0.
      if self.o_tol is None:
        self.o_tol = 0.
      if self.o_xy is None:
        self.o_xy = [0.,0.]
      if self.o_oavoid is None:
        self.o_oavoid = [0.,0.]
    else:
      self.header = roslib.msg._Header.Header()
      self.errorFlags = 0
      self.content = 0
      self.timeStamp = 0
      self.controlTimeout = 0
      self.watchdogTimeout = 0
      self.mode = coax_msgs.msg.CoaxModes()
      self.roll = 0.
      self.pitch = 0.
      self.yaw = 0.
      self.gyro = [0.,0.,0.]
      self.accel = [0.,0.,0.]
      self.magneto = [0.,0.,0.]
      self.imutemp = 0.
      self.zrange = 0.
      self.zfiltered = 0.
      self.pressure = 0.
      self.hranges = [0.,0.,0.,0.]
      self.xrel = 0.
      self.yrel = 0.
      self.battery = 0.
      self.rcChannel = [0.,0.,0.,0.,0.,0.,0.,0.]
      self.o_attitude = [0.,0.,0.]
      self.o_altitude = 0.
      self.o_tol = 0.
      self.o_xy = [0.,0.]
      self.o_oavoid = [0.,0.]

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    @param buff: buffer
    @type  buff: StringIO
    """
    try:
      _x = self
      buff.write(_struct_3I.pack(_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs))
      _x = self.header.frame_id
      length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_B2I2H7B3f.pack(_x.errorFlags, _x.content, _x.timeStamp, _x.controlTimeout, _x.watchdogTimeout, _x.mode.navigation, _x.mode.communication, _x.mode.oavoid, _x.mode.rollAxis, _x.mode.pitchAxis, _x.mode.yawAxis, _x.mode.altAxis, _x.roll, _x.pitch, _x.yaw))
      buff.write(_struct_3f.pack(*self.gyro))
      buff.write(_struct_3f.pack(*self.accel))
      buff.write(_struct_3f.pack(*self.magneto))
      _x = self
      buff.write(_struct_4f.pack(_x.imutemp, _x.zrange, _x.zfiltered, _x.pressure))
      buff.write(_struct_4f.pack(*self.hranges))
      _x = self
      buff.write(_struct_3f.pack(_x.xrel, _x.yrel, _x.battery))
      buff.write(_struct_8f.pack(*self.rcChannel))
      buff.write(_struct_3f.pack(*self.o_attitude))
      _x = self
      buff.write(_struct_2f.pack(_x.o_altitude, _x.o_tol))
      buff.write(_struct_2f.pack(*self.o_xy))
      buff.write(_struct_2f.pack(*self.o_oavoid))
    except struct.error, se: self._check_types(se)
    except TypeError, te: self._check_types(te)

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    @param str: byte array of serialized message
    @type  str: str
    """
    try:
      if self.header is None:
        self.header = roslib.msg._Header.Header()
      if self.mode is None:
        self.mode = coax_msgs.msg.CoaxModes()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs,) = _struct_3I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      self.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 32
      (_x.errorFlags, _x.content, _x.timeStamp, _x.controlTimeout, _x.watchdogTimeout, _x.mode.navigation, _x.mode.communication, _x.mode.oavoid, _x.mode.rollAxis, _x.mode.pitchAxis, _x.mode.yawAxis, _x.mode.altAxis, _x.roll, _x.pitch, _x.yaw,) = _struct_B2I2H7B3f.unpack(str[start:end])
      start = end
      end += 12
      self.gyro = _struct_3f.unpack(str[start:end])
      start = end
      end += 12
      self.accel = _struct_3f.unpack(str[start:end])
      start = end
      end += 12
      self.magneto = _struct_3f.unpack(str[start:end])
      _x = self
      start = end
      end += 16
      (_x.imutemp, _x.zrange, _x.zfiltered, _x.pressure,) = _struct_4f.unpack(str[start:end])
      start = end
      end += 16
      self.hranges = _struct_4f.unpack(str[start:end])
      _x = self
      start = end
      end += 12
      (_x.xrel, _x.yrel, _x.battery,) = _struct_3f.unpack(str[start:end])
      start = end
      end += 32
      self.rcChannel = _struct_8f.unpack(str[start:end])
      start = end
      end += 12
      self.o_attitude = _struct_3f.unpack(str[start:end])
      _x = self
      start = end
      end += 8
      (_x.o_altitude, _x.o_tol,) = _struct_2f.unpack(str[start:end])
      start = end
      end += 8
      self.o_xy = _struct_2f.unpack(str[start:end])
      start = end
      end += 8
      self.o_oavoid = _struct_2f.unpack(str[start:end])
      return self
    except struct.error, e:
      raise roslib.message.DeserializationError(e) #most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    @param buff: buffer
    @type  buff: StringIO
    @param numpy: numpy python module
    @type  numpy module
    """
    try:
      _x = self
      buff.write(_struct_3I.pack(_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs))
      _x = self.header.frame_id
      length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_B2I2H7B3f.pack(_x.errorFlags, _x.content, _x.timeStamp, _x.controlTimeout, _x.watchdogTimeout, _x.mode.navigation, _x.mode.communication, _x.mode.oavoid, _x.mode.rollAxis, _x.mode.pitchAxis, _x.mode.yawAxis, _x.mode.altAxis, _x.roll, _x.pitch, _x.yaw))
      buff.write(self.gyro.tostring())
      buff.write(self.accel.tostring())
      buff.write(self.magneto.tostring())
      _x = self
      buff.write(_struct_4f.pack(_x.imutemp, _x.zrange, _x.zfiltered, _x.pressure))
      buff.write(self.hranges.tostring())
      _x = self
      buff.write(_struct_3f.pack(_x.xrel, _x.yrel, _x.battery))
      buff.write(self.rcChannel.tostring())
      buff.write(self.o_attitude.tostring())
      _x = self
      buff.write(_struct_2f.pack(_x.o_altitude, _x.o_tol))
      buff.write(self.o_xy.tostring())
      buff.write(self.o_oavoid.tostring())
    except struct.error, se: self._check_types(se)
    except TypeError, te: self._check_types(te)

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    @param str: byte array of serialized message
    @type  str: str
    @param numpy: numpy python module
    @type  numpy: module
    """
    try:
      if self.header is None:
        self.header = roslib.msg._Header.Header()
      if self.mode is None:
        self.mode = coax_msgs.msg.CoaxModes()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs,) = _struct_3I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      self.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 32
      (_x.errorFlags, _x.content, _x.timeStamp, _x.controlTimeout, _x.watchdogTimeout, _x.mode.navigation, _x.mode.communication, _x.mode.oavoid, _x.mode.rollAxis, _x.mode.pitchAxis, _x.mode.yawAxis, _x.mode.altAxis, _x.roll, _x.pitch, _x.yaw,) = _struct_B2I2H7B3f.unpack(str[start:end])
      start = end
      end += 12
      self.gyro = numpy.frombuffer(str[start:end], dtype=numpy.float32, count=3)
      start = end
      end += 12
      self.accel = numpy.frombuffer(str[start:end], dtype=numpy.float32, count=3)
      start = end
      end += 12
      self.magneto = numpy.frombuffer(str[start:end], dtype=numpy.float32, count=3)
      _x = self
      start = end
      end += 16
      (_x.imutemp, _x.zrange, _x.zfiltered, _x.pressure,) = _struct_4f.unpack(str[start:end])
      start = end
      end += 16
      self.hranges = numpy.frombuffer(str[start:end], dtype=numpy.float32, count=4)
      _x = self
      start = end
      end += 12
      (_x.xrel, _x.yrel, _x.battery,) = _struct_3f.unpack(str[start:end])
      start = end
      end += 32
      self.rcChannel = numpy.frombuffer(str[start:end], dtype=numpy.float32, count=8)
      start = end
      end += 12
      self.o_attitude = numpy.frombuffer(str[start:end], dtype=numpy.float32, count=3)
      _x = self
      start = end
      end += 8
      (_x.o_altitude, _x.o_tol,) = _struct_2f.unpack(str[start:end])
      start = end
      end += 8
      self.o_xy = numpy.frombuffer(str[start:end], dtype=numpy.float32, count=2)
      start = end
      end += 8
      self.o_oavoid = numpy.frombuffer(str[start:end], dtype=numpy.float32, count=2)
      return self
    except struct.error, e:
      raise roslib.message.DeserializationError(e) #most likely buffer underfill

_struct_I = roslib.message.struct_I
_struct_B2I2H7B3f = struct.Struct("<B2I2H7B3f")
_struct_8f = struct.Struct("<8f")
_struct_2f = struct.Struct("<2f")
_struct_3I = struct.Struct("<3I")
_struct_4f = struct.Struct("<4f")
_struct_3f = struct.Struct("<3f")
