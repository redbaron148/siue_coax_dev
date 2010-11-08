"""autogenerated by genmsg_py from CoaxConfigureCommRequest.msg. Do not edit."""
import roslib.message
import struct


class CoaxConfigureCommRequest(roslib.message.Message):
  _md5sum = "8bef03ada508431e00edf6ad2d960a31"
  _type = "coax_msgs/CoaxConfigureCommRequest"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """uint8 commMode
uint16 frequency
uint16 numMessages
uint32 contents

"""
  __slots__ = ['commMode','frequency','numMessages','contents']
  _slot_types = ['uint8','uint16','uint16','uint32']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.
    
    The available fields are:
       commMode,frequency,numMessages,contents
    
    @param args: complete set of field values, in .msg order
    @param kwds: use keyword arguments corresponding to message field names
    to set specific fields. 
    """
    if args or kwds:
      super(CoaxConfigureCommRequest, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.commMode is None:
        self.commMode = 0
      if self.frequency is None:
        self.frequency = 0
      if self.numMessages is None:
        self.numMessages = 0
      if self.contents is None:
        self.contents = 0
    else:
      self.commMode = 0
      self.frequency = 0
      self.numMessages = 0
      self.contents = 0

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
      buff.write(_struct_B2HI.pack(_x.commMode, _x.frequency, _x.numMessages, _x.contents))
    except struct.error, se: self._check_types(se)
    except TypeError, te: self._check_types(te)

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    @param str: byte array of serialized message
    @type  str: str
    """
    try:
      end = 0
      _x = self
      start = end
      end += 9
      (_x.commMode, _x.frequency, _x.numMessages, _x.contents,) = _struct_B2HI.unpack(str[start:end])
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
      buff.write(_struct_B2HI.pack(_x.commMode, _x.frequency, _x.numMessages, _x.contents))
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
      end = 0
      _x = self
      start = end
      end += 9
      (_x.commMode, _x.frequency, _x.numMessages, _x.contents,) = _struct_B2HI.unpack(str[start:end])
      return self
    except struct.error, e:
      raise roslib.message.DeserializationError(e) #most likely buffer underfill

_struct_I = roslib.message.struct_I
_struct_B2HI = struct.Struct("<B2HI")
"""autogenerated by genmsg_py from CoaxConfigureCommResponse.msg. Do not edit."""
import roslib.message
import struct


class CoaxConfigureCommResponse(roslib.message.Message):
  _md5sum = "4414c67819626a1b8e0f043a9a0d6c9a"
  _type = "coax_msgs/CoaxConfigureCommResponse"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """int8 result



"""
  __slots__ = ['result']
  _slot_types = ['int8']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.
    
    The available fields are:
       result
    
    @param args: complete set of field values, in .msg order
    @param kwds: use keyword arguments corresponding to message field names
    to set specific fields. 
    """
    if args or kwds:
      super(CoaxConfigureCommResponse, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.result is None:
        self.result = 0
    else:
      self.result = 0

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
      buff.write(_struct_b.pack(self.result))
    except struct.error, se: self._check_types(se)
    except TypeError, te: self._check_types(te)

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    @param str: byte array of serialized message
    @type  str: str
    """
    try:
      end = 0
      start = end
      end += 1
      (self.result,) = _struct_b.unpack(str[start:end])
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
      buff.write(_struct_b.pack(self.result))
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
      end = 0
      start = end
      end += 1
      (self.result,) = _struct_b.unpack(str[start:end])
      return self
    except struct.error, e:
      raise roslib.message.DeserializationError(e) #most likely buffer underfill

_struct_I = roslib.message.struct_I
_struct_b = struct.Struct("<b")
class CoaxConfigureComm(roslib.message.ServiceDefinition):
  _type          = 'coax_msgs/CoaxConfigureComm'
  _md5sum = '0a4ffc01e802a3c95dac462243d2bddb'
  _request_class  = CoaxConfigureCommRequest
  _response_class = CoaxConfigureCommResponse