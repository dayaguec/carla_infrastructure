from .sensor.gnss_sensor import GnssSensor
from .sensor.lidar_sensor import LidarSensor
from .sensor.semantic_lidar_sensor import SemanticLidarSensor
from .sensor.rgb_camera_sensor import RGBCameraSensor
from .sensor.semantic_camera_sensor import SemanticCameraSensor
from .sensor.rsu_sensor import RSUSensor

from .localization.transform import carla_transform_to_ros_pose

from perception_interfaces.msg import (GroundTruthDetection, ClassType)
from geometry_msgs.msg import Vector3

from infrastructure_carla.utils import find_weather_presets

import numpy as np
from dataclasses import dataclass
import carla

@dataclass
class VehicleProfile:
  """Stores static data that doesn't change during the simulation."""
  extent: carla.Vector3D
  speed: carla.Vector3D
  type_id: str

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
    self.lidar_sensors = None
    self.sem_lidar_sensors = None
    self.rgb_camera_sensors = None
    self.sem_camera_sensors = None
    self.rsu_sensors = None

    self._weather_presets = find_weather_presets()
    self._map_layers = [
      carla.MapLayer.NONE,
      carla.MapLayer.Buildings,
      carla.MapLayer.Decals,
      carla.MapLayer.Foliage,
      carla.MapLayer.Ground,
      carla.MapLayer.ParkedVehicles,
      carla.MapLayer.Particles,
      carla.MapLayer.Props,
      carla.MapLayer.StreetLights,
      carla.MapLayer.Walls,
      carla.MapLayer.All
    ]
    self._loaded_map_layers = set(self._map_layers)

    self._near_vehicles_cache = {}
    self._vehicle_id_list = []

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
    self.rsu_sensors = [RSUSensor(self.world, tf_item, param_item) for tf_item, param_item in \
      zip(sensor_params["rsu"][0], sensor_params["rsu"][1])]

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

  def update_vehicle_cache(self):
    """Low-frequency RPC call to get all vehicle in the map."""
    vehicles = self.world.get_actors().filter('vehicle.*')
    current_ids = [v.id for v in vehicles]

    # Clean up bbox cache for vehicles that no longer exist
    self._near_vehicles_cache = {id: profile for id, profile in self._near_vehicles_cache.items() if id in current_ids}

    # Add new vehicles to the cache
    for v in vehicles:
      if v.id not in self._near_vehicles_cache:
        self._near_vehicles_cache[v.id] = VehicleProfile(
          extent=v.bounding_box.extent,
          speed=v.get_velocity(),
          type_id=v.type_id
        )

    self._vehicle_id_list = current_ids

  def get_nearby_vehicles(self, detection_msg, target_location, radius, snapshot):
    """
    High-frequency, ZERO-RPC call. 
    Calculates nearby vehicles using local snapshot memory.
    
    :param target_location: carla.Location (the custom point to check from)
    :param radius: float (search radius in meters)
    :param snapshot: carla.WorldSnapshot (from world.get_snapshot())
    :return: list of actor IDs within the radius
    """
    radius_sq = radius ** 2  # Squared radius for faster math

    for actor_id in self._vehicle_id_list:
      # Fetch the actor's state directly from the local snapshot
      actor_snapshot = snapshot.find(actor_id)
      
      # If it returns None, the vehicle was destroyed since our last cache update
      if actor_snapshot is None:
        continue 

      transform = actor_snapshot.get_transform()
      loc = transform.location

      dist_sq = (loc.x - target_location.x)**2 + (loc.y - target_location.y)**2

      if dist_sq <= radius_sq:
        profile = self._near_vehicles_cache.get(actor_id)
        if not profile:
          continue

        detection = GroundTruthDetection()
        detection.id = actor_id
        detection.velocity = Vector3(x=profile.speed.x, y=profile.speed.y, z=profile.speed.z)
        v_ype, v_class = self.get_class(profile.type_id)
        detection.type = ClassType(type=v_ype, class_detection=v_class)
        detection.bounding_box.center.pose = carla_transform_to_ros_pose(transform)
        detection.bounding_box.center.covariance = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        detection.bounding_box.size = Vector3(
          x=profile.extent.x * 2.0,
          y=profile.extent.y * 2.0,
          z=profile.extent.z * 2.0)

        detection_msg.detections.append(detection)

  def change_wheather(self, weather_index):
    """Query Carla World to change current weather with a preset."""
    try:
      preset = self._weather_presets[weather_index]
    except IndexError:
      return False, "Weather not recognized, out of bounds!"

    if self.world.get_weather() != preset[0]:
      self.world.set_weather(preset[0])
      return True, "Weather changed succesfully!"
    return False, "Unable to change Weather, already in that preset!"

  def change_map_layer(self, layer_index, load=True):
    """Query Carla World to load/unload the current map layer in the simulation."""
    map_name = self.world.get_map().name
    if "_Opt" in map_name: # Only available in layered maps
      try:
        target_layer = self._map_layers[layer_index]
      except IndexError:
        return False, "Layer not recognized, out of bounds!"

      if load:
        if target_layer in self._loaded_map_layers:
          return True, "Layer {} already loaded. Skipping.".format(target_layer)
        else:
          self.world.load_map_layer(target_layer)
          self._loaded_map_layers.add(target_layer)
          return True, "Successfully loaded {}.".format(target_layer)
      else:
        if target_layer not in self._loaded_map_layers:
          return True, "Layer {} already unloaded. Skipping.".format(target_layer)
        else:
          self.world.unload_map_layer(target_layer)
          self._loaded_map_layers.discard(target_layer)
          return True, "Successfully unloaded {}.".format(target_layer)

    return False, "Unable to load Layer, map is not layered!"

  def get_traffic_lights(self, tl_list, actor, max_distance=50):
    """Return as a Light msg the set of nearest traffic lights within a distance
       from the actor."""
    traffic_lights = self.map.get_all_landmarks_of_type('1000001')
    t = actor.get_transform()

    distance = lambda l: np.sqrt((l.x - t.location.x)**2 + (l.y - t.location.y)**2 + (l.z - t.location.z)**2)
    traffic_lights = [(distance(x.transform.location), x) for x in traffic_lights]
    for d, traffic_light_landmark in sorted(traffic_lights, key=lambda traffic_lights: traffic_lights[0]):
      if d > max_distance:
        break

      traffic_light = self.world.get_traffic_light(traffic_light_landmark)
      tl_list.append(traffic_light)

  def destroy(self):
    """Once the simulation finish destroy every object spawned."""
    # Destroy all world items
    sensor_lists = [
      self.gnss_sensors,
      self.rsu_sensors,
      self.lidar_sensors,
      self.sem_lidar_sensors,
      self.rgb_camera_sensors,
      self.sem_camera_sensors
    ]
    for sensor_list in sensor_lists:
      if sensor_list is not None:
        for sensor in sensor_list:
            sensor.destroy()
