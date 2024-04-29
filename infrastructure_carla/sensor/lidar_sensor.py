import weakref
import ctypes, struct
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import numpy as np
import carla

class LidarSensor(object):
  def __init__(self, parent_actor, carla_transform, lidar_params):
    self.sensor = None
    if isinstance(parent_actor, carla.Actor):
      self._parent = parent_actor
      world = self._parent.get_world()
    else:
      self._parent = None
      world = parent_actor
    self.transform = None
    self._ros_pointcloud = PointCloud2()

    self._DATATYPES = {
        PointField.INT8: ('b', 1),
        PointField.UINT8: ('B', 1),
        PointField.INT16: ('h', 2),
        PointField.UINT16: ('H', 2),
        PointField.INT32: ('i', 4),
        PointField.UINT32: ('I', 4),
        PointField.FLOAT32: ('f', 4),
        PointField.FLOAT64: ('d', 8)
    }

    self.fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1)
    ]

    bp = world.get_blueprint_library().find('sensor.lidar.ray_cast')
    
    for key, value in lidar_params.items():
        bp.set_attribute(key, str(value))

    self.sensor = world.spawn_actor(bp, carla_transform, attach_to=self._parent)
    weak_self = weakref.ref(self)
    self.sensor.listen(
        lambda lidar_data: LidarSensor._Lidar_callback(weak_self, lidar_data))

  @staticmethod
  def _Lidar_callback(weak_self, lidar_data):
    self = weak_self()
    if not self:
        return

    data = np.copy(np.frombuffer(lidar_data.raw_data, dtype=np.float32))
    data = np.reshape(data, (int(data.shape[0] / 4), 4))

    # We take the opposite of y axis as lidar point are express in left handed
    # coordinate system, and ROS needs right handed.
    data[:, 1] *= -1

    self._create_cloud(self.fields, data)

  def _create_cloud(self, fields, points):
    cloud_struct = struct.Struct(self._get_struct_fmt(False, fields))
    buff = ctypes.create_string_buffer(cloud_struct.size * len(points))
    point_step, pack_into = cloud_struct.size, cloud_struct.pack_into

    offset = 0
    for p in points:
        pack_into(buff, offset, *p)
        offset += point_step

    self._ros_pointcloud = PointCloud2(header=Header(), height=1, width=len(points),
        is_dense=False, is_bigendian=False, fields=fields, point_step=cloud_struct.size,
        row_step=cloud_struct.size * len(points), data=buff.raw)

  # http://docs.ros.org/indigo/api/sensor_msgs/html/point__cloud2_8py_source.html
  def _get_struct_fmt(self, is_bigendian, fields, field_names=None):
    fmt = '>' if is_bigendian else '<'

    offset = 0
    for field in (f for f in sorted(fields, key=lambda f: f.offset)
                  if field_names is None or f.name in field_names):
        if offset < field.offset:
            fmt += 'x' * (field.offset - offset)
            offset = field.offset
        if field.datatype not in self._DATATYPES:
            print('Skipping unknown PointField datatype [{}]' % field.datatype, file=sys.stderr)
        else:
            datatype_fmt, datatype_length = self._DATATYPES[field.datatype]
            fmt += field.count * datatype_fmt
            offset += field.count * datatype_length

    return fmt

  def get_ros_pointcloud(self):
    return self._ros_pointcloud
