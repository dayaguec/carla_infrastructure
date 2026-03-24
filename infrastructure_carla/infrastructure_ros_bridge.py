import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped

import yaml
from yaml.loader import SafeLoader

import carla
import threading

from std_msgs.msg import (Empty, Header)
from sensor_msgs.msg import (NavSatFix,
  CameraInfo, Image, PointCloud2)
from perception_interfaces.msg import GroundTruthPerception
from cooperation_interfaces.msg import CooperativeAwarenessMessageArray
from visualization_msgs.msg import (Marker, MarkerArray)

from carla_interfaces.srv import (CarlaSpawnTraffic, CarlaCleanTraffic,
  CarlaChangeLayer, CarlaChangeWeather)

from .localization.transform import (carla_transform_to_ros_transform, dict_to_transform)

class InfrastructureROSNode(Node):
  def __init__(self, world, is_sync_mode, standalone=True):
    """Initialize the ROS Node in charge of generating a bridge between Carla and ROS"""
    super().__init__('infrastructure_ros_node')

    self._carla_traffic_manager = None
    self._carla_lock = threading.Lock()

    self._tf_publisher = StaticTransformBroadcaster(self)

    # Carla simulator world
    self._carla_world = world
    self._world_tick_id = None
    self._is_sync_mode = is_sync_mode

    self._is_game_quit = False

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

    # RSU for V2X
    try:
      rsu_frames, self._rsu_topics, rsu_parameters, rsu_transforms = \
        [rsu_item["frame_id"] for rsu_item in self._sensors_data["rsu"]],\
        [rsu_item["topic"] for rsu_item in self._sensors_data["rsu"]],\
        [rsu_item["params"] for rsu_item in self._sensors_data["rsu"]],\
        [dict_to_transform(rsu_item["transform"]) for rsu_item in self._sensors_data["rsu"]]
    except KeyError:
      rsu_frames, self._rsu_topics, rsu_parameters, rsu_transforms = \
        [], [], [], []
      self.get_logger().warn("RSU sensor configuration not found, RSU will not spawn!")

    self._gps_tf, self._rgb_camera_tf, self._sem_camera_tf, self._lidar_tf, self._sem_lidar_tf, self._rsu_tf =\
      [], [], [], [], [], []
    self._gps_msgs, self._rgb_camera_msgs, self._sem_camera_msgs, self._lidar_msgs, self._sem_lidar_msgs, self._rsu_msgs =\
      [], [], [], [], [], []

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

    for item in zip(rsu_transforms, rsu_frames):
      tf_ros = TransformStamped()
      tf_ros.header.frame_id = self._global_frame_id
      tf_ros.child_frame_id = item[1]
      tf_ros.transform = carla_transform_to_ros_transform(item[0])
      self._rsu_tf.append(tf_ros)
      self._rsu_msgs.append(CooperativeAwarenessMessageArray(header=Header(frame_id=item[1])))

    # Aggregate data for easy processing
    sensor_params = {
      "gps" : (gps_transforms, gps_parameters),
      "rgb_camera" : (rgb_camera_transforms, rgb_camera_parameters),
      "semantic_camera" : (sem_camera_transforms, sem_camera_parameters),
      "lidar" : (lidar_transforms, lidar_parameters),
      "semantic_lidar" : (sem_lidar_transforms, sem_lidar_parameters),
      "rsu" : (rsu_transforms, rsu_parameters)
    }

    # Spawn sensors proccesed in Carla world
    self._carla_world.spawn_sensors(sensor_params)

    # ROS comunication
    self._cb_group = MutuallyExclusiveCallbackGroup()
    self.create_subcribers()
    self.create_publishers()
    if standalone:
      self.create_services()

    if not (self._is_sync_mode and standalone):
      self._world_tick_id = self._carla_world.world.on_tick(self.on_tick)

    self._cache_timer = self.create_timer(2.0, self.update_vehicle_cache, self._cb_group)
    self.update_vehicle_cache()

    mode_str = "Synchronous" if self._is_sync_mode else "Asynchronous"
    standalone_str = "STANDALONE" if standalone else "SUBORDINATE"
    self.get_logger().info("ROS 2 Carla Bridge started in {} {} mode. Waiting for stop signal or Ctrl+C.".format(mode_str, standalone_str))

  def create_subcribers(self):
    """Create ROS Subscribers."""
    topic = self.get_parameter('quit_simulation_topic').get_parameter_value().string_value
    self._quit_simulation_sub = self.create_subscription(Empty, topic,
      self.quit_simulation_callback, 1, callback_group=self._cb_group)

  def create_publishers(self):
    """Create ROS Publishers."""
    # Individual topic publishers
    topic = self.get_parameter('ground_truth_topic').get_parameter_value().string_value
    self._detections_pub = self.create_publisher(GroundTruthPerception, topic, 1)
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
    self._rsu_publishers = [self.create_publisher(CooperativeAwarenessMessageArray, topic, 1)\
      for topic in self._rsu_topics]

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

    for rsu in zip(self._rsu_publishers, self._rsu_msgs):
      rsu[1].header.stamp = self.get_clock().now().to_msg()
      rsu[0].publish(rsu[1])

    # Publish sensors tfs
    tf_list = [
      self._gps_tf,
      self._rgb_camera_tf,
      self._sem_camera_tf,
      self._lidar_tf,
      self._sem_lidar_tf,
      self._rsu_tf,
    ]
    for tf_item in tf_list:
      for item in tf_item:
        item.header.stamp = self.get_clock().now().to_msg()
        self._tf_publisher.sendTransform(item)

  def create_services(self):
    """Create ROS Services."""
    self._traffic_manager_spawn_service = self.create_service(
      CarlaSpawnTraffic, 'carla_spawn_traffic', self.spawn_traffic,
      callback_group=self._cb_group)
    self._traffic_manager_clean_service = self.create_service(
      CarlaCleanTraffic, 'carla_clean_traffic', self.clean_traffic,
      callback_group=self._cb_group)
    self._carla_world_change_weather = self.create_service(
      CarlaChangeWeather, 'carla_change_weather', self.change_wheather,
      callback_group=self._cb_group)
    self._carla_world_change_layer = self.create_service(
      CarlaChangeLayer, 'carla_change_map_layer', self.change_layer,
      callback_group=self._cb_group)

  def update_vehicle_cache(self):
    """Use Carla world to update nearby vehicle cache with a low rate RPC Call."""
    self._carla_world.update_vehicle_cache()

  def on_tick(self, snapshot):
    """Callback to tick in a constant rate, ensure lock adquire if call in sync mode."""
    if self._is_game_quit:
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

    # RSU Data conversion
    for ii in range(len(self._rsu_msgs)):
      last_header = self._rsu_msgs[ii].header
      self._rsu_msgs[ii] = self._carla_world.rsu_sensors[ii].get_ros_message()
      self._rsu_msgs[ii].header = last_header

    # Near player vehicles for ground truth
    self._detection_msg = GroundTruthPerception()
    self._carla_world.get_nearby_vehicles(self._detection_msg,
      self._carla_world.gnss_sensors[0].sensor.get_location(), 50.0, snapshot)
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
      marker.lifetime.sec = 1
      marker.pose = detection.bounding_box.center.pose
      detection.header = Header(frame_id=self._global_frame_id, stamp=self.get_clock().now().to_msg())
      self._detection_markers_msg.markers.append(marker)

    self.publish_messages()

  def quit_simulation_callback(self, msg):
    """Check if the high level interface wants to finish the simulation."""
    self.get_logger().info("Stop signal received! Initiating shutdown...")
    self._is_game_quit = True

  def is_game_quit(self):
    """Checks if the simulation finished."""
    return self._is_game_quit

  def cleanup(self):
    """Remove the tick callback if aynch mode is on"""
    if self._world_tick_id:
      self._carla_world.world.remove_on_tick(self._world_tick_id)

  def spawn_traffic(self, request, response):
    """ROS service callback to spawn traffic controller with Carla Traffic Manager."""
    if self._carla_traffic_manager is None:
      response.result = False
      response.message = "Traffic Manager is not enabled in this client, cannot spawn traffic..."
      return response

    with self._carla_lock:
      self._carla_traffic_manager.setup_parameters(
        request.n_vehicles, request.n_walkers,
        request.random_seed, request.hybrid_mode, request.hybrid_radius)

      response.result, response.message = self._carla_traffic_manager.spawn_traffic()

    return response

  def clean_traffic(self, request, response):
    """ROS service callback to remove every Carla Actor related with traffic."""
    if self._carla_traffic_manager is None:
      response.result = False
      response.message = "Traffic Manager is not enabled in this client, cannot clean traffic..."
      return response

    with self._carla_lock:
      response.result, response.message = self._carla_traffic_manager.clean_traffic()

    return response

  def change_wheather(self, request, response):
    """ROS service callback to change Carla world weather."""
    with self._carla_lock:
      response.result, response.message = self._carla_world.change_wheather(request.weather)
    return response

  def change_layer(self, request, response):
    """ROS service callback to change a Carla Map layer."""
    with self._carla_lock:
      if request.action == CarlaChangeLayer.Request.LOAD: # Load Layer
        response.result, response.message = self._carla_world.change_map_layer(request.layer)
      else: # Unload Layer
        response.result, response.message = self._carla_world.change_map_layer(request.layer, False)
    return response

  def set_traffic_manager(self, traffic_manager):
    """Just set the traffic manager object to deal with traffic."""
    self._carla_traffic_manager = traffic_manager

class InfrastructureROSBridge(object):
  def __init__(self, world, is_sync_mode, standalone=True):
    """Initilize the ROS Bridge Node and execution mode."""
    self._carla_node = InfrastructureROSNode(world, is_sync_mode, standalone)

    self._executor = MultiThreadedExecutor(num_threads=4)
    self._executor.add_node(self._carla_node)

    self._executor_thread = threading.Thread(target=self._executor.spin, daemon=True)
    self._executor_thread.start()

  def on_tick(self, snapshot):
    """Calback for every Carla World tick. Ensure adquire lock if sync mode"""
    self._carla_node.on_tick(snapshot)

  def is_game_quit(self):
    """Returns to the high level if the simulation needs to finish."""
    return self._carla_node.is_game_quit()

  def set_traffic_manager(self, traffic_manager):
    """Just set the traffic manager object to deal with traffic at Node level."""
    self._carla_node.set_traffic_manager(traffic_manager)

  def get_lock(self):
    """Just return the lock to avoid concurrency problems in synch simulations."""
    return self._carla_node._carla_lock

  def destroy(self):
    """Destroy every object created."""
    self._executor.shutdown()
    self._carla_node.cleanup()
    self._executor_thread.join(timeout=1.0)
    self._carla_node.destroy_node()
