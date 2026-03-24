import carla

import numpy as np
from carla_infrastructure.msg import Perception
from rclpy import time
from std_msgs.msg import Header

class TrafficLightManager(object):
  def __init__(self, world):
    """Initialize Traffic Light Manager to manage Traffic Lights in a specific emplacement."""
    # InfrastructureWorld to get access to Carla World
    self._carla_world = world
    # Traffic lights objects for emplacement management
    self._traffic_lights = []
    # Store traffic light objects
    self.get_traffic_lights()
    # Nearby vehicles
    self._detection_msg = Perception()

  def update_control_loop(self):
    self.gather_nearby_actors()
    # Update traffic light state to control the emplacement
    #....

  def get_traffic_lights(self):
    """Get near traffic lights for management."""
    self._traffic_lights.clear()
    self._carla_world.get_traffic_lights(self._traffic_lights, self._carla_world.gnss_sensors[0].sensor)

    # Set all traffic light in red
    for tl in self._traffic_lights:
      # Draw id in server to see configuration
      self._carla_world.world.debug.draw_string(tl.get_light_boxes()[-1].location,\
        tl.get_opendrive_id(), False, carla.Color(255,0,0), 100)
      tl.set_state(carla.TrafficLightState.Red)
    # Freeze traffic light update to delegate management
    self.freeze_traffic_lights(True)

  def freeze_traffic_lights(self, freeze):
    """Freeze traffic lights to avoid automatic update"""
    for tl in self._traffic_lights:
      tl.freeze(freeze)

  def update_traffic_light_by_pos(self, new_state, stop_position):
    """Set desired color state of a traffic light based on the nearest location"""
    distance = 9999.9
    traffic_light_to_update = None
    for tl in self._traffic_lights:
      stop_waypoints = tl.get_stop_waypoints()
      stop_waypoint = stop_waypoints[int(np.trunc(len(stop_waypoints)/2))]
      stop_location = stop_waypoint.transform.location
      distance_to_tl = np.sqrt((stop_location.x - stop_position.x)**2 \
        + (stop_location.y - stop_position.y)**2 + (stop_location.z - stop_position.z)**2)
      if distance_to_tl < distance:
        traffic_light_to_update = tl
    traffic_light_to_update.set_state(new_state)

  def update_traffic_light_by_id(self, new_state, opendrive_id):
    """Set desired color state of a traffic light based on the opendrive_id"""
    for tl in self._traffic_lights:
      if tl.get_opendrive_id() == str(opendrive_id):
        tl.set_state(new_state)

  def gather_nearby_actors(self):
    self._detection_msg = Perception()
    self._carla_world.get_near_player_vehicles(self._detection_msg, self._carla_world.gnss_sensors[0].sensor)
    self._detection_msg.header = Header(frame_id='TrafficLightManager', stamp=time.Time().to_msg())
