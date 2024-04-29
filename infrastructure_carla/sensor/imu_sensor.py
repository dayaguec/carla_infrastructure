import weakref
import math

class IMUSensor(object):
  def __init__(self, parent_actor, carla_transform, imu_params):
    self.sensor = None
    self._parent = parent_actor
    self.accelerometer = (0.0, 0.0, 0.0)
    self.gyroscope = (0.0, 0.0, 0.0)
    self.compass = 0.0
    self.transform = None
    world = self._parent.get_world()
    bp = world.get_blueprint_library().find('sensor.other.imu')

    for key, value in imu_params.items():
      bp.set_attribute(key, str(value))

    self.sensor = world.spawn_actor(bp, carla_transform, attach_to=self._parent)
    # We need to pass the lambda a weak reference to self to avoid circular
    # reference.
    weak_self = weakref.ref(self)
    self.sensor.listen(
      lambda sensor_data: IMUSensor._IMU_callback(weak_self, sensor_data))

  @staticmethod
  def _IMU_callback(weak_self, sensor_data):
    self = weak_self()
    if not self:
      return
    limits = (-99.9, 99.9)
    self.accelerometer = (
      max(limits[0], min(limits[1], sensor_data.accelerometer.x)),
      max(limits[0], min(limits[1], sensor_data.accelerometer.y)),
      max(limits[0], min(limits[1], sensor_data.accelerometer.z)))
    self.gyroscope = (
      max(limits[0], min(limits[1], math.degrees(sensor_data.gyroscope.x))),
      max(limits[0], min(limits[1], math.degrees(sensor_data.gyroscope.y))),
      max(limits[0], min(limits[1], math.degrees(sensor_data.gyroscope.z))))
    self.compass = math.degrees(sensor_data.compass)
    self.transform = sensor_data.transform
