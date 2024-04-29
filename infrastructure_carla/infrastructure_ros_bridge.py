import rclpy
from rclpy.node import Node

from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
from tf_transformations import quaternion_from_euler

import yaml
from yaml.loader import SafeLoader

import carla
import datetime

from std_msgs.msg import (Empty, Header)
from geometry_msgs.msg import (Pose, Point, Quaternion)
from sensor_msgs.msg import (NavSatFix,
  CameraInfo, Image, PointCloud2, PointField)
from carla_infrastructure.msg import Perception
from visualization_msgs.msg import (Marker, MarkerArray)

from .localization.transform import (carla_transform_to_ros_transform, dict_to_transform)

class InfrastructureROSNode(Node):
  def __init__(self, world):
    """Initialize the ROS Node in charge of generating a bridge between Carla and ROS"""
    super().__init__('infrastructure_ros_node')
    
    self._timer = self.create_timer(0.02, self.timer_callback)
    self._tf_publisher = StaticTransformBroadcaster(self)
    
    self._node_ready = False
    self._is_game_quit = False

    # Carla simulator world
    self._carla_world = world

    # Declare this node params to use later
    self.declare_parameters(
      namespace='',
      parameters=[
        ('quit_simulation_topic', 'quit_simulation'),
        ('ground_truth_topic', 'ground_truth'),
        ('global_frame_id', 'world'),
        ('sensor_params', '')
      ]
    )

    self._global_frame_id = self.get_parameter('global_frame_id').get_parameter_value().string_value

    # Parse sensor yaml file for sensors configuration
    try:
      sensor_config_file_path = self.get_parameter('sensor_params').get_parameter_value().string_value
      self._sensors_data = None
      with open(sensor_config_file_path) as f:
        self._sensors_data = yaml.load(f, Loader=SafeLoader)
        self._sensors_data = self._sensors_data['sensors']
    except FileNotFoundError:
      self.get_logger().error(
        'Sensor config file provided: {} does not exist, exiting...'.format(sensor_config_file_path))
      self._is_game_quit = True

    # Parse frames, topics, types and params. If key sensor is not found in yalm,
    # it generates an empty vector
    # GPS
    try:
      gps_frames, self._gps_topics, gps_parameters, gps_transforms = \
        [gps_item['frame_id'] for gps_item in self._sensors_data['gps']],\
        [gps_item['topic'] for gps_item in self._sensors_data['gps']],\
        [gps_item['params'] for gps_item in self._sensors_data['gps']],\
        [dict_to_transform(gps_item["transform"])\
          for gps_item in self._sensors_data["gps"]]
    except KeyError:
      gps_frames, self._gps_topics, gps_parameters, gps_transforms = [], [], [], []
      self.get_logger().warn(
        'GPS sensor configuration not found, GPS will not spawn; localization may not work properly!')

    # RGB Camera
    try:
      rgb_camera_frames, self._rgb_camera_topics, rgb_camera_parameters, rgb_camera_transforms = \
        [rgb_camera_item["frame_id"] for rgb_camera_item in self._sensors_data["rgb_camera"]],\
        [rgb_camera_item["topic"] for rgb_camera_item in self._sensors_data["rgb_camera"]],\
        [rgb_camera_item["params"] for rgb_camera_item in self._sensors_data["rgb_camera"]],\
        [dict_to_transform(rgb_camera_item["transform"])\
          for rgb_camera_item in self._sensors_data["rgb_camera"]]
    except KeyError:
      rgb_camera_frames, self._rgb_camera_topics, rgb_camera_parameters, rgb_camera_transforms = \
        [], [], [], []
      self.get_logger().warn("RBG Camera sensor configuration not found, RBG Camera will not spawn!")

    # Semantic camera
    try:
      sem_camera_frames, self._sem_camera_topics, sem_camera_parameters, sem_camera_transforms = \
        [sem_camera_item["frame_id"] for sem_camera_item in self._sensors_data["semantic_camera"]],\
        [sem_camera_item["topic"] for sem_camera_item in self._sensors_data["semantic_camera"]],\
        [sem_camera_item["params"] for sem_camera_item in self._sensors_data["semantic_camera"]],\
        [dict_to_transform(sem_camera_item["transform"]) for sem_camera_item in self._sensors_data["semantic_camera"]]
    except KeyError:
      sem_camera_frames, self._sem_camera_topics, sem_camera_parameters, sem_camera_transforms = \
        [], [], [], []
      self.get_logger().warn("Semantic camera sensor configuration not found, Semantic Camera will not spawn!")

    # Ray cast LiDARs
    try:
      lidar_frames, self._lidar_topics, lidar_parameters, lidar_transforms = \
        [lidar_item["frame_id"] for lidar_item in self._sensors_data["lidar"]],\
        [lidar_item["topic"] for lidar_item in self._sensors_data["lidar"]],\
        [lidar_item["params"] for lidar_item in self._sensors_data["lidar"]],\
        [dict_to_transform(lidar_item["transform"]) for lidar_item in self._sensors_data["lidar"]]
    except KeyError:
      lidar_frames, self._lidar_topics, lidar_parameters, lidar_transforms = \
        [], [], [], []
      self.get_logger().warn("LiDAR sensor configuration not found, LiDAR will not spawn!")

    # Semantic LiDARs
    try:
      sem_lidar_frames, self._sem_lidar_topics, sem_lidar_parameters, sem_lidar_transforms = \
        [sem_lidar_item["frame_id"] for sem_lidar_item in self._sensors_data["semantic_lidar"]],\
        [sem_lidar_item["topic"] for sem_lidar_item in self._sensors_data["semantic_lidar"]],\
        [sem_lidar_item["params"] for sem_lidar_item in self._sensors_data["semantic_lidar"]],\
        [dict_to_transform(sem_lidar_item["transform"]) for sem_lidar_item in self._sensors_data["semantic_lidar"]]
    except KeyError:
      sem_lidar_frames, self._sem_lidar_topics, sem_lidar_parameters, sem_lidar_transforms = \
        [], [], [], []
      self.get_logger().warn("Semantic LiDAR sensor configuration not found, Semantic LiDAR will not spawn!")

    self._gps_tf, self._rgb_camera_tf, self._sem_camera_tf, self._lidar_tf, self._sem_lidar_tf =\
      [], [], [], [], []
    self._gps_msgs, self._rgb_camera_msgs, self._sem_camera_msgs, self._lidar_msgs, self._sem_lidar_msgs =\
      [], [], [], [], []

    for item in zip(gps_transforms, gps_frames):
      tf_ros = TransformStamped()
      tf_ros.header.frame_id = self._global_frame_id
      tf_ros.child_frame_id = item[1]
      tf_ros.transform = carla_transform_to_ros_transform(item[0])
      self._gps_tf.append(tf_ros)
      self._gps_msgs.append(NavSatFix(header=Header(frame_id=item[1])))

    for item in zip(rgb_camera_transforms, rgb_camera_frames):
      tf_ros = TransformStamped()
      tf_ros.header.frame_id = self._global_frame_id
      tf_ros.child_frame_id = item[1]
      tf_ros.transform = carla_transform_to_ros_transform(item[0])
      self._rgb_camera_tf.append(tf_ros)
      self._rgb_camera_msgs.append([CameraInfo(header=Header(frame_id=item[1])),
        Image(header=Header(frame_id=item[1]))])

    for item in zip(sem_camera_transforms, sem_camera_frames):
      tf_ros = TransformStamped()
      tf_ros.header.frame_id = self._global_frame_id
      tf_ros.child_frame_id = item[1]
      tf_ros.transform = carla_transform_to_ros_transform(item[0])
      self._sem_camera_tf.append(tf_ros)
      self._sem_camera_msgs.append([CameraInfo(header=Header(frame_id=item[1])),
        Image(header=Header(frame_id=item[1]))])

    for item in zip(lidar_transforms, lidar_frames):
      tf_ros = TransformStamped()
      tf_ros.header.frame_id = self._global_frame_id
      tf_ros.child_frame_id = item[1]
      tf_ros.transform = carla_transform_to_ros_transform(item[0])
      self._lidar_tf.append(tf_ros)
      self._lidar_msgs.append(PointCloud2(header=Header(frame_id=item[1])))

    for item in zip(sem_lidar_transforms, sem_lidar_frames):
      tf_ros = TransformStamped()
      tf_ros.header.frame_id = self._global_frame_id
      tf_ros.child_frame_id = item[1]
      tf_ros.transform = carla_transform_to_ros_transform(item[0])
      self._sem_lidar_tf.append(tf_ros)
      self._sem_lidar_msgs.append(PointCloud2(header=Header(frame_id=item[1])))

    # Aggregate data for easy processing
    sensor_params = {
      "gps" : (gps_transforms, gps_parameters),
      "rgb_camera" : (rgb_camera_transforms, rgb_camera_parameters),
      "semantic_camera" : (sem_camera_transforms, sem_camera_parameters),
      "lidar" : (lidar_transforms, lidar_parameters),
      "semantic_lidar" : (sem_lidar_transforms, sem_lidar_parameters)
    }

    # Spawn sensors proccesed in Carla world
    self._carla_world.spawn_sensors(sensor_params)

    # ROS comunication
    self.create_subcribers()
    self.create_publishers()

    self._node_ready = True

  def create_subcribers(self):
    """Create ROS Subscribers."""
    topic = self.get_parameter('quit_simulation_topic').get_parameter_value().string_value
    self._quit_simulation_sub = self.create_subscription(Empty, topic,
      self.quit_simulation_callback, 1)

  def create_publishers(self):
    """Create ROS Publishers."""
    # Individual topic publishers
    topic = self.get_parameter('ground_truth_topic').get_parameter_value().string_value
    self._detections_pub = self.create_publisher(Perception, topic, 1)
    self._detections_markers_pub = self.create_publisher(MarkerArray, (topic + "_markers"), 1)

    # List sensor data publishers
    self._gps_publishers = [self.create_publisher(NavSatFix, topic, 1)\
      for topic in self._gps_topics]
    self._rgb_camera_publishers = [(self.create_publisher(CameraInfo, topic + '_camera_info', 1),\
      self.create_publisher(Image, topic + '/image', 1)) for topic in self._rgb_camera_topics]
    self._sem_camera_publishers = [(self.create_publisher(CameraInfo, topic + '_camera_info', 1),\
      self.create_publisher(Image, topic + '/image', 1)) for topic in self._sem_camera_topics]
    self._lidar_publishers = [self.create_publisher(PointCloud2, topic, 1)\
      for topic in self._lidar_topics]
    self._sem_lidar_publishers = [self.create_publisher(PointCloud2, topic, 1)\
      for topic in self._sem_lidar_topics]

  def publish_messages(self):
    """Publish every ROS Message."""
    self._detections_pub.publish(self._detection_msg) # Detections
    self._detections_markers_pub.publish(self._detection_markers_msg) # Detections

    # List publishers (GPS, IMU, LiDAR and RGB_Camera)
    for gps in zip(self._gps_publishers, self._gps_msgs):
      gps[1].header.stamp = self.get_clock().now().to_msg()
      gps[0].publish(gps[1])

    for rgb_camera in zip(self._rgb_camera_publishers, self._rgb_camera_msgs):
      rgb_camera[1][0].header.stamp = self.get_clock().now().to_msg()
      rgb_camera[1][1].header.stamp = self.get_clock().now().to_msg()
      rgb_camera[0][0].publish(rgb_camera[1][0])
      rgb_camera[0][1].publish(rgb_camera[1][1])

    for sem_camera in zip(self._sem_camera_publishers, self._sem_camera_msgs):
      sem_camera[1][0].header.stamp = self.get_clock().now().to_msg()
      sem_camera[1][1].header.stamp = self.get_clock().now().to_msg()
      sem_camera[0][0].publish(sem_camera[1][0])
      sem_camera[0][1].publish(sem_camera[1][1])

    for lidar in zip(self._lidar_publishers, self._lidar_msgs):
      lidar[1].header.stamp = self.get_clock().now().to_msg()
      lidar[0].publish(lidar[1])

    for lidar in zip(self._sem_lidar_publishers, self._sem_lidar_msgs):
      lidar[1].header.stamp = self.get_clock().now().to_msg()
      lidar[0].publish(lidar[1])

    # Publish sensors tfs
    tf_list = [
      self._gps_tf,
      self._rgb_camera_tf,
      self._sem_camera_tf,
      self._lidar_tf,
      self._sem_lidar_tf
    ]
    for tf_item in tf_list:
      for item in tf_item:
        item.header.stamp = self.get_clock().now().to_msg()
        self._tf_publisher.sendTransform(item)

  def timer_callback(self):
    """Callback to tick in a constant rate."""
    if not self._node_ready:
      self.get_logger().warn('Node is not prepared!')
      return

    # GPS Data conversion
    for ii in range(len(self._gps_msgs)):
      gps_object = self._carla_world.gnss_sensors[ii]
      gps_ros_msg = self._gps_msgs[ii]

      gps_ros_msg.latitude = gps_object.lat
      gps_ros_msg.longitude = gps_object.lon
      gps_ros_msg.altitude = gps_object.alt

      # Add sensor covariance extracted from sensor noise model
      gps_ros_msg.position_covariance_type = 2 # COVARIANCE_TYPE_DIAGONAL_KNOWN
      gps_ros_msg.position_covariance[0] = 0.00019 # float(gps_object.sensor.attributes['noise_lat_stddev'])**2
      gps_ros_msg.position_covariance[4] = 0.00019 # float(gps_object.sensor.attributes['noise_lon_stddev'])**2
      gps_ros_msg.position_covariance[8] = 0.00019 # float(gps_object.sensor.attributes['noise_alt_stddev'])**2

      gps_ros_msg.status.status = 2  # With augmentated fix
      gps_ros_msg.status.service = 1 # GPS signal normal

    # RGB Camera data conversion, assume camera info header is the same as camera RGB data
    for ii in range(len(self._rgb_camera_msgs)):
      last_header = self._rgb_camera_msgs[ii][0].header
      self._rgb_camera_msgs[ii][0] = self._carla_world.rgb_camera_sensors[ii].get_camera_info()
      self._rgb_camera_msgs[ii][1] = self._carla_world.rgb_camera_sensors[ii].get_ros_image()
      self._rgb_camera_msgs[ii][0].header = last_header
      self._rgb_camera_msgs[ii][1].header = last_header

    # Semantic Camera data conversion, assume camera info header is the same as camera Semantic data
    for ii in range(len(self._sem_camera_msgs)):
      last_header = self._sem_camera_msgs[ii][0].header
      self._sem_camera_msgs[ii][0] = self._carla_world.sem_camera_sensors[ii].get_camera_info()
      self._sem_camera_msgs[ii][1] = self._carla_world.sem_camera_sensors[ii].get_ros_image()
      self._sem_camera_msgs[ii][0].header = last_header
      self._sem_camera_msgs[ii][1].header = last_header

    # Lidar Data conversion
    for ii in range(len(self._lidar_msgs)):
      last_header = self._lidar_msgs[ii].header
      self._lidar_msgs[ii] = self._carla_world.lidar_sensors[ii].get_ros_pointcloud()
      self._lidar_msgs[ii].header = last_header

    # Semantic Lidar Data conversion
    for ii in range(len(self._sem_lidar_msgs)):
      last_header = self._sem_lidar_msgs[ii].header
      self._sem_lidar_msgs[ii] = self._carla_world.sem_lidar_sensors[ii].get_ros_pointcloud()
      self._sem_lidar_msgs[ii].header = last_header

    # Near player vehicles for gound truth
    self._detection_msg = Perception()
    self._carla_world.get_near_player_vehicles(self._detection_msg, self._carla_world.gnss_sensors[0].sensor)
    self._detection_msg.header = Header(frame_id=self._global_frame_id, stamp=self.get_clock().now().to_msg())

    self._detection_markers_msg = MarkerArray()
    for detection in self._detection_msg.detections:
      marker = Marker()
      marker.header = Header(frame_id=self._global_frame_id, stamp=self.get_clock().now().to_msg())
      marker.type = marker.CUBE
      marker.id = detection.id
      marker.action = marker.ADD
      marker.scale = detection.bounding_box.size
      marker.color.r = 0.2
      marker.color.g = 0.7
      marker.color.b = 1.0
      marker.color.a = 1.0
      marker.pose = detection.bounding_box.center
      self._detection_markers_msg.markers.append(marker)

    self.publish_messages()

  def quit_simulation_callback(self, msg):
    """Check if the high level interface wants to finish the simulation."""
    self._is_game_quit = True

  def is_game_quit(self):
    """Checks if the simulation finished."""
    return self._is_game_quit

class InfrastructureROSBridge(object):
  def __init__(self, world):
    """Initilize the ROS Bridge Node."""
    self._carla_node = InfrastructureROSNode(world)

    self._executor = rclpy.executors.SingleThreadedExecutor()
    self._executor.add_node(self._carla_node)

  def tick(self, clock):
    """Calback for every Carla World tick."""
    self._executor.spin_once()

  def is_game_quit(self):
    """Returns to the high level if the simulation needs to finish."""
    return self._carla_node.is_game_quit()

  def destroy(self):
    """Destroy every object created."""
    self._executor.shutdown()
    self._carla_node.destroy_node()
