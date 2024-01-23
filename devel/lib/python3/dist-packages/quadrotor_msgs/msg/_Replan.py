# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from quadrotor_msgs/Replan.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import geometry_msgs.msg
import nav_msgs.msg
import std_msgs.msg

class Replan(genpy.Message):
  _md5sum = "73a17ace4b8b54cf9b4974fbea003b7f"
  _type = "quadrotor_msgs/Replan"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """#data structure
geometry_msgs/Vector3 start_velocity
geometry_msgs/Vector3 start_acceleration
nav_msgs/Path         plan
geometry_msgs/Vector3 stop_velocity
geometry_msgs/Vector3 stop_acceleration
float64               replan_time

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z
================================================================================
MSG: nav_msgs/Path
#An array of poses that represents a Path for a robot to follow
Header header
geometry_msgs/PoseStamped[] poses

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: geometry_msgs/PoseStamped
# A Pose with reference coordinate frame and timestamp
Header header
Pose pose

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
"""
  __slots__ = ['start_velocity','start_acceleration','plan','stop_velocity','stop_acceleration','replan_time']
  _slot_types = ['geometry_msgs/Vector3','geometry_msgs/Vector3','nav_msgs/Path','geometry_msgs/Vector3','geometry_msgs/Vector3','float64']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       start_velocity,start_acceleration,plan,stop_velocity,stop_acceleration,replan_time

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(Replan, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.start_velocity is None:
        self.start_velocity = geometry_msgs.msg.Vector3()
      if self.start_acceleration is None:
        self.start_acceleration = geometry_msgs.msg.Vector3()
      if self.plan is None:
        self.plan = nav_msgs.msg.Path()
      if self.stop_velocity is None:
        self.stop_velocity = geometry_msgs.msg.Vector3()
      if self.stop_acceleration is None:
        self.stop_acceleration = geometry_msgs.msg.Vector3()
      if self.replan_time is None:
        self.replan_time = 0.
    else:
      self.start_velocity = geometry_msgs.msg.Vector3()
      self.start_acceleration = geometry_msgs.msg.Vector3()
      self.plan = nav_msgs.msg.Path()
      self.stop_velocity = geometry_msgs.msg.Vector3()
      self.stop_acceleration = geometry_msgs.msg.Vector3()
      self.replan_time = 0.

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    :param buff: buffer, ``StringIO``
    """
    try:
      _x = self
      buff.write(_get_struct_6d3I().pack(_x.start_velocity.x, _x.start_velocity.y, _x.start_velocity.z, _x.start_acceleration.x, _x.start_acceleration.y, _x.start_acceleration.z, _x.plan.header.seq, _x.plan.header.stamp.secs, _x.plan.header.stamp.nsecs))
      _x = self.plan.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      length = len(self.plan.poses)
      buff.write(_struct_I.pack(length))
      for val1 in self.plan.poses:
        _v1 = val1.header
        _x = _v1.seq
        buff.write(_get_struct_I().pack(_x))
        _v2 = _v1.stamp
        _x = _v2
        buff.write(_get_struct_2I().pack(_x.secs, _x.nsecs))
        _x = _v1.frame_id
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
        _v3 = val1.pose
        _v4 = _v3.position
        _x = _v4
        buff.write(_get_struct_3d().pack(_x.x, _x.y, _x.z))
        _v5 = _v3.orientation
        _x = _v5
        buff.write(_get_struct_4d().pack(_x.x, _x.y, _x.z, _x.w))
      _x = self
      buff.write(_get_struct_7d().pack(_x.stop_velocity.x, _x.stop_velocity.y, _x.stop_velocity.z, _x.stop_acceleration.x, _x.stop_acceleration.y, _x.stop_acceleration.z, _x.replan_time))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      if self.start_velocity is None:
        self.start_velocity = geometry_msgs.msg.Vector3()
      if self.start_acceleration is None:
        self.start_acceleration = geometry_msgs.msg.Vector3()
      if self.plan is None:
        self.plan = nav_msgs.msg.Path()
      if self.stop_velocity is None:
        self.stop_velocity = geometry_msgs.msg.Vector3()
      if self.stop_acceleration is None:
        self.stop_acceleration = geometry_msgs.msg.Vector3()
      end = 0
      _x = self
      start = end
      end += 60
      (_x.start_velocity.x, _x.start_velocity.y, _x.start_velocity.z, _x.start_acceleration.x, _x.start_acceleration.y, _x.start_acceleration.z, _x.plan.header.seq, _x.plan.header.stamp.secs, _x.plan.header.stamp.nsecs,) = _get_struct_6d3I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.plan.header.frame_id = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.plan.header.frame_id = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.plan.poses = []
      for i in range(0, length):
        val1 = geometry_msgs.msg.PoseStamped()
        _v6 = val1.header
        start = end
        end += 4
        (_v6.seq,) = _get_struct_I().unpack(str[start:end])
        _v7 = _v6.stamp
        _x = _v7
        start = end
        end += 8
        (_x.secs, _x.nsecs,) = _get_struct_2I().unpack(str[start:end])
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          _v6.frame_id = str[start:end].decode('utf-8', 'rosmsg')
        else:
          _v6.frame_id = str[start:end]
        _v8 = val1.pose
        _v9 = _v8.position
        _x = _v9
        start = end
        end += 24
        (_x.x, _x.y, _x.z,) = _get_struct_3d().unpack(str[start:end])
        _v10 = _v8.orientation
        _x = _v10
        start = end
        end += 32
        (_x.x, _x.y, _x.z, _x.w,) = _get_struct_4d().unpack(str[start:end])
        self.plan.poses.append(val1)
      _x = self
      start = end
      end += 56
      (_x.stop_velocity.x, _x.stop_velocity.y, _x.stop_velocity.z, _x.stop_acceleration.x, _x.stop_acceleration.y, _x.stop_acceleration.z, _x.replan_time,) = _get_struct_7d().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self
      buff.write(_get_struct_6d3I().pack(_x.start_velocity.x, _x.start_velocity.y, _x.start_velocity.z, _x.start_acceleration.x, _x.start_acceleration.y, _x.start_acceleration.z, _x.plan.header.seq, _x.plan.header.stamp.secs, _x.plan.header.stamp.nsecs))
      _x = self.plan.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      length = len(self.plan.poses)
      buff.write(_struct_I.pack(length))
      for val1 in self.plan.poses:
        _v11 = val1.header
        _x = _v11.seq
        buff.write(_get_struct_I().pack(_x))
        _v12 = _v11.stamp
        _x = _v12
        buff.write(_get_struct_2I().pack(_x.secs, _x.nsecs))
        _x = _v11.frame_id
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
        _v13 = val1.pose
        _v14 = _v13.position
        _x = _v14
        buff.write(_get_struct_3d().pack(_x.x, _x.y, _x.z))
        _v15 = _v13.orientation
        _x = _v15
        buff.write(_get_struct_4d().pack(_x.x, _x.y, _x.z, _x.w))
      _x = self
      buff.write(_get_struct_7d().pack(_x.stop_velocity.x, _x.stop_velocity.y, _x.stop_velocity.z, _x.stop_acceleration.x, _x.stop_acceleration.y, _x.stop_acceleration.z, _x.replan_time))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      if self.start_velocity is None:
        self.start_velocity = geometry_msgs.msg.Vector3()
      if self.start_acceleration is None:
        self.start_acceleration = geometry_msgs.msg.Vector3()
      if self.plan is None:
        self.plan = nav_msgs.msg.Path()
      if self.stop_velocity is None:
        self.stop_velocity = geometry_msgs.msg.Vector3()
      if self.stop_acceleration is None:
        self.stop_acceleration = geometry_msgs.msg.Vector3()
      end = 0
      _x = self
      start = end
      end += 60
      (_x.start_velocity.x, _x.start_velocity.y, _x.start_velocity.z, _x.start_acceleration.x, _x.start_acceleration.y, _x.start_acceleration.z, _x.plan.header.seq, _x.plan.header.stamp.secs, _x.plan.header.stamp.nsecs,) = _get_struct_6d3I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.plan.header.frame_id = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.plan.header.frame_id = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.plan.poses = []
      for i in range(0, length):
        val1 = geometry_msgs.msg.PoseStamped()
        _v16 = val1.header
        start = end
        end += 4
        (_v16.seq,) = _get_struct_I().unpack(str[start:end])
        _v17 = _v16.stamp
        _x = _v17
        start = end
        end += 8
        (_x.secs, _x.nsecs,) = _get_struct_2I().unpack(str[start:end])
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          _v16.frame_id = str[start:end].decode('utf-8', 'rosmsg')
        else:
          _v16.frame_id = str[start:end]
        _v18 = val1.pose
        _v19 = _v18.position
        _x = _v19
        start = end
        end += 24
        (_x.x, _x.y, _x.z,) = _get_struct_3d().unpack(str[start:end])
        _v20 = _v18.orientation
        _x = _v20
        start = end
        end += 32
        (_x.x, _x.y, _x.z, _x.w,) = _get_struct_4d().unpack(str[start:end])
        self.plan.poses.append(val1)
      _x = self
      start = end
      end += 56
      (_x.stop_velocity.x, _x.stop_velocity.y, _x.stop_velocity.z, _x.stop_acceleration.x, _x.stop_acceleration.y, _x.stop_acceleration.z, _x.replan_time,) = _get_struct_7d().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_2I = None
def _get_struct_2I():
    global _struct_2I
    if _struct_2I is None:
        _struct_2I = struct.Struct("<2I")
    return _struct_2I
_struct_3d = None
def _get_struct_3d():
    global _struct_3d
    if _struct_3d is None:
        _struct_3d = struct.Struct("<3d")
    return _struct_3d
_struct_4d = None
def _get_struct_4d():
    global _struct_4d
    if _struct_4d is None:
        _struct_4d = struct.Struct("<4d")
    return _struct_4d
_struct_6d3I = None
def _get_struct_6d3I():
    global _struct_6d3I
    if _struct_6d3I is None:
        _struct_6d3I = struct.Struct("<6d3I")
    return _struct_6d3I
_struct_7d = None
def _get_struct_7d():
    global _struct_7d
    if _struct_7d is None:
        _struct_7d = struct.Struct("<7d")
    return _struct_7d
