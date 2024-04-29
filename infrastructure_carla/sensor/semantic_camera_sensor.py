import weakref
import math
from cv_bridge import CvBridge
import numpy as np
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Header
import carla

class SemanticCameraSensor(object):
  def __init__(self, parent_actor, carla_transform, rgb_camera_params):
    self.sensor = None
    if isinstance(parent_actor, carla.Actor):
      self._parent = parent_actor
      world = self._parent.get_world()
    else:
      self._parent = None
      world = parent_actor
    self.transform = None
    self._camera_info = CameraInfo()
    self._ros_data = Image()
    self.cv_bridge = CvBridge()

    bp = world.get_blueprint_library().find('sensor.camera.semantic_segmentation')
    
    for key, value in rgb_camera_params.items():
      bp.set_attribute(key, str(value))

    self.sensor = world.spawn_actor(bp, carla_transform, attach_to=self._parent)
    # We need to pass the lambda a weak reference to self to avoid circular
    # reference.
    weak_self = weakref.ref(self)
    self._build_camera_info()
    self.sensor.listen(
      lambda sensor_data: SemanticCameraSensor._RGB_callback(weak_self, sensor_data))

  def _build_camera_info(self):
    camera_info = CameraInfo()
    camera_info.header = Header()
    camera_info.width = int(self.sensor.attributes['image_size_x'])
    camera_info.height = int(self.sensor.attributes['image_size_y'])
    camera_info.distortion_model = 'plumb_bob'
    cx = camera_info.width / 2.0
    cy = camera_info.height / 2.0
    fx = camera_info.width / (
      2.0 * math.tan(float(self.sensor.attributes['fov']) * math.pi / 360.0))
    fy = fx
    camera_info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
    camera_info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
    camera_info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    camera_info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
    self._camera_info = camera_info

  def get_ros_image(self):
    return self._ros_data

  def get_camera_info(self):
    return self._camera_info

  @staticmethod
  def _RGB_callback(weak_self, sensor_data):
    self = weak_self()
    if not self:
      return
    if ((sensor_data.height != self._camera_info.height) or
       (sensor_data.width != self._camera_info.width)):
        print('Camera received image not matching configuration')

    sensor_data.convert(carla.ColorConverter.CityScapesPalette)
    image_data_array = np.ndarray(shape=(sensor_data.height, sensor_data.width, 4),
      dtype=np.uint8, buffer=sensor_data.raw_data)
    img_msg = self.cv_bridge.cv2_to_imgmsg(image_data_array, encoding='bgra8')
    img_msg.header = Header()
    self._ros_data = img_msg
    self.transform = sensor_data.transform