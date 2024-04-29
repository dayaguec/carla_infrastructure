from .localization.geo_converter import GeoConverter

from .sensor.gnss_sensor import GnssSensor
from .sensor.lidar_sensor import LidarSensor
from .sensor.semantic_lidar_sensor import SemanticLidarSensor
from .sensor.rgb_camera_sensor import RGBCameraSensor
from .sensor.semantic_camera_sensor import SemanticCameraSensor

from carla_infrastructure.msg import (Detection, ClassType, BoundingBox3D)
from geometry_msgs.msg import (Pose, Point, Quaternion, Vector3)

import carla
import random
import numpy as np
import pygame
import time

from tf_transformations import quaternion_from_euler

class InfrastructureWorld(object):
  def __init__(self, carla_world, args):
    """Initialize Infrastructure World to hold Carla Infrastructure World and Map."""
    self.world = carla_world
    self.sync = args.sync
    try:
      self.map = self.world.get_map()
    except RuntimeError as error:
      print('RuntimeError: {}'.format(error))
      print('  The server could not send the OpenDRIVE (.xodr) file:')
      print('  Make sure it exists, has the same name of your town, and is correct.')
      sys.exit(1)
    self.gnss_sensors = None
    self.imu_sensors = None
    self.lidar_sensors = None
    self.sem_lidar_sensors = None
    self.radar_sensors = None
    self.rgb_camera_sensors = None
    self.sem_camera_sensors = None
    # For Geolocation to Local coordinates from Carla Infrastructure World
    self.geo_converter = GeoConverter(args.geo, 0.0)

    if self.sync:
      self.world.tick()
    else:
      self.world.wait_for_tick()

  def spawn_sensors(self, sensor_params):
    """Spawn every sensor attached to the Infrastructure"""
    self.gnss_sensors = [GnssSensor(self.world, tf_item, param_item) for tf_item, param_item in \
      zip(sensor_params["gps"][0], sensor_params["gps"][1])]
    self.rgb_camera_sensors = [RGBCameraSensor(self.world, tf_item, param_item) for tf_item, param_item in \
      zip(sensor_params["rgb_camera"][0], sensor_params["rgb_camera"][1])]
    self.sem_camera_sensors = [SemanticCameraSensor(self.world, tf_item, param_item) for tf_item, param_item in \
      zip(sensor_params["semantic_camera"][0], sensor_params["semantic_camera"][1])]
    self.lidar_sensors = [LidarSensor(self.world, tf_item, param_item) for tf_item, param_item in \
      zip(sensor_params["lidar"][0], sensor_params["lidar"][1])]
    self.sem_lidar_sensors = [SemanticLidarSensor(self.world, tf_item, param_item) for tf_item, param_item in \
      zip(sensor_params["semantic_lidar"][0], sensor_params["semantic_lidar"][1])]

  def get_class(self, actor_id):
    """Get the detection Class given an Carla actor_id."""
    if "walker" in actor_id:
      return ClassType.PEDESTRIAN, ClassType.WALKER
    elif actor_id == "vehicle.harley-davidson.low_rider"\
      or actor_id == "vehicle.vespa.zx125"\
      or actor_id == "kawasaki.ninja"\
      or actor_id == "vehicle.yamaha.yzf":
      return ClassType.VEHICLE, ClassType.MOTORCYCLE
    elif actor_id == "vehicle.ford.ambulance"\
      or actor_id == "vehicle.volkswagen.t2"\
      or actor_id == "vehicle.carlamotors.carlacola":
      return ClassType.VEHICLE, ClassType.VAN
    elif actor_id == "vehicle.carlamotors.firetruck":
      return ClassType.VEHICLE, ClassType.TRUCK
    elif actor_id == "vehicle.diamondback.century"\
      or actor_id == "vehicle.gazelle.omafiets"\
      or actor_id == "vehicle.bh.crossbike":
      return ClassType.VEHICLE, ClassType.BIKE
    else:
      return ClassType.VEHICLE, ClassType.CAR

  def get_near_player_vehicles(self, detection_msg, actor, max_distance=50):
    """Return as a Detection msg the set of nearest vehicles within a distance
       from the actor."""
    vehicles = self.world.get_actors().filter('vehicle.*')
    t = actor.get_transform()

    distance = lambda l: np.sqrt((l.x - t.location.x)**2 + (l.y - t.location.y)**2 + (l.z - t.location.z)**2)
    vehicles = [(distance(x.get_location()), x) for x in vehicles if x.id != actor.id]
    for d, vehicle in sorted(vehicles, key=lambda vehicles: vehicles[0]):
      if d > max_distance:
        break
      transform = vehicle.get_transform()
      vehicle_geolocation = self.map.transform_to_geolocation(transform.location)
      enu_location = self.geo_converter.toENU(vehicle_geolocation)
      yaw = -np.radians(transform.rotation.yaw)
      v = vehicle.get_velocity()

      detection = Detection()
      detection.id = vehicle.id
      detection.velocity = Vector3(x=v.x, y=v.y, z=v.z)
      v_ype, v_class = self.get_class(vehicle.type_id)
      detection.type = ClassType(type=v_ype, class_detection=v_class)
      quat = quaternion_from_euler(0.0, 0.0, yaw)
      detection.bounding_box.center = Pose(
        position=Point(x=enu_location.position.x, y=enu_location.position.y, z=0.0),
        orientation=Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3]))
      detection.bounding_box.size = Vector3(
        x=vehicle.bounding_box.extent.x * 2.0,
        y=vehicle.bounding_box.extent.y * 2.0,
        z=vehicle.bounding_box.extent.z * 2.0)

      detection_msg.detections.append(detection)

  def destroy(self):
    """Once the simulation finish destroy every object spawned."""
    # Destroy all world items
    sensor_lists = [
      self.gnss_sensors,
      self.imu_sensors,
      self.lidar_sensors,
      self.sem_lidar_sensors,
      self.radar_sensors,
      self.rgb_camera_sensors,
      self.sem_camera_sensors
    ]
    for sensor_list in sensor_lists:
      if sensor_list is not None:
        for ss in sensor_list:
          if ss.sensor is not None:
            ss.sensor.stop()
            ss.sensor.destroy()
