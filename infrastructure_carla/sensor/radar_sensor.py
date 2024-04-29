import weakref
import ctypes, struct
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import numpy as np

class RadarSensor(object):
  def __init__(self, parent_actor, carla_transform, radar_params):
    self.sensor = None
    self._parent = parent_actor
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
        PointField(name='Range', offset=12, datatype=PointField.FLOAT32, count=1),
        PointField(name='Velocity', offset=16, datatype=PointField.FLOAT32, count=1),
        PointField(name='AzimuthAngle', offset=20, datatype=PointField.FLOAT32, count=1),
        PointField(name='ElevationAngle', offset=28, datatype=PointField.FLOAT32, count=1)]

    world = self._parent.get_world()
    bp = world.get_blueprint_library().find('sensor.other.radar')

    for key, value in radar_params.items():
        bp.set_attribute(key, str(value))

    self.sensor = world.spawn_actor(bp, carla_transform, attach_to=self._parent)
    # We need a weak reference to self to avoid circular reference.
    weak_self = weakref.ref(self)
    self.sensor.listen(
        lambda radar_data: RadarSensor._Radar_callback(weak_self, radar_data))

  @staticmethod
  def _Radar_callback(weak_self, radar_data):
    self = weak_self()
    if not self:
        return

    points = []
    for detection in radar_data:
        points.append([detection.depth * np.cos(detection.azimuth) * np.cos(-detection.altitude),
                       detection.depth * np.sin(-detection.azimuth) *
                       np.cos(detection.altitude),
                       detection.depth * np.sin(detection.altitude),
                       detection.depth, detection.velocity, detection.azimuth, detection.altitude])

    self.create_cloud(self.fields, points)

  def create_cloud(self, fields, points):
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
