import carla
import weakref
import time
import rclpy

from cooperation_interfaces.msg import (CooperativeAwarenessMessageArray, CooperativeAwarenessMessage)

from dataclasses import dataclass
import threading

@dataclass
class CacheEntry:
    last_seen_time: float
    message: CooperativeAwarenessMessage

class RSUSensor(object):
  def __init__(self, parent_actor, carla_transform, rsu_params, timeout_seconds=2.0):
    self.sensor = None
    if isinstance(parent_actor, carla.Actor):
      self._parent = parent_actor
      self.world = self._parent.get_world()
    else:
      self._parent = None
      self.world = parent_actor

    self.timeout_seconds = timeout_seconds
    self.cam_cache = {}
    self.lock = threading.Lock()

    self._is_active = True
    self._cleanup_thread = threading.Thread(target=self._background_cleanup, daemon=True)

    bp = self.world.get_blueprint_library().find('sensor.other.v2x')
    for key, value in rsu_params.items():
      bp.set_attribute(key, str(value))

    # In Carla 0.9.16 this sensor parent needs to be an actor, if not an exception is generated
    # this is a wacky workaround that needs to be fixed in next version
    blueprint = self.world.get_blueprint_library().find('vehicle.carlamotors.firetruck')
    carla_transform.location.z = 2.0
    self._actor = self.world.try_spawn_actor(blueprint, carla_transform)

    self.sensor = self.world.spawn_actor(bp, carla_transform, attach_to=self._actor)
    # We need to pass the lambda a weak reference to self to avoid circular reference.
    weak_self = weakref.ref(self)
    self.sensor.listen(lambda event: RSUSensor._on_rsu_event(weak_self, event))

    # Start the background janitor
    self._cleanup_thread.start()

  def _background_cleanup(self):
    """Periodically removes stale entries even if no new messages arrive."""
    while self._is_active:
      # Use simulation time for consistency
      snapshot = self.world.get_snapshot()
      if snapshot:
        current_sim_time = snapshot.timestamp.elapsed_seconds
        stale_ids = []
        
        with self.lock:
          for v_id, entry in self.cam_cache.items():
            if (current_sim_time - entry.last_seen_time) > self.timeout_seconds:
              stale_ids.append(v_id)
          
          for v_id in stale_ids:
            del self.cam_cache[v_id]

      # Sleep in real-time seconds to avoid maxing out a CPU core
      time.sleep(0.5)

  def destroy(self):
    """Clean shutdown of both the sensor and the background thread."""
    self._is_active = False
    if self.sensor:
      self.sensor.stop()
      self.sensor.destroy()
    if self._actor:
      self._actor.destroy()
    if self._cleanup_thread.is_alive():
      self._cleanup_thread.join(timeout=1.0)

  def get_ros_message(self):
    """Returns the currently valid messages from the cleaned cache."""
    with self.lock:
      ros_data = CooperativeAwarenessMessageArray()
      ros_data.cams = [entry.message for entry in self.cam_cache.values()]
      return ros_data

  def get_vehicle_role(self, role):
    """Get the detection role fiven a cam role."""
    if role == "Public Transport":
      return CooperativeAwarenessMessage.PUBLIC_TRANSPORT 
    elif role == "Special Transport":
      return CooperativeAwarenessMessage.SPECIAL_TRANSPORT 
    elif role == "Dangerous Goods":
      return CooperativeAwarenessMessage.DANGEROUS_GOODS 
    elif role == "Road Work":
      return CooperativeAwarenessMessage.ROADWORK 
    elif role == "Rescue":
      return CooperativeAwarenessMessage.RESCUE 
    elif role == "Emergency":
      return CooperativeAwarenessMessage.EMERGENCY 
    elif role == "Safety Car":
      return CooperativeAwarenessMessage.SAFETY_CAR 
    elif role == "Agriculture":
      return CooperativeAwarenessMessage.AGRICULTURE 
    elif role == "Commercial":
      return CooperativeAwarenessMessage.COMMERCIAL 
    elif role == "Military":
      return CooperativeAwarenessMessage.MILITARY 
    elif role == "Road Operator":
      return CooperativeAwarenessMessage.ROAD_OPERATOR 
    elif role == "Taxi":
      return CooperativeAwarenessMessage.TAXI 
    elif role == "Reserved1":
      return CooperativeAwarenessMessage.RESERVED_1 
    elif role == "Reserved2":
      return CooperativeAwarenessMessage.RESERVED_2
    elif role == "Reserved3":
      return CooperativeAwarenessMessage.RESERVED_3
    else:
      return CooperativeAwarenessMessage.DEFAULT 

  def get_station_type(self, station_type):
    """Get the detection type fiven a cam station type."""
    if station_type == "Pedestrian":
      return CooperativeAwarenessMessage.PEDESTRIAN 
    elif station_type == "Cyclist":
      return CooperativeAwarenessMessage.CYCLIST 
    elif station_type == "Moped":
      return CooperativeAwarenessMessage.MOPED 
    elif station_type == "Motorcycle":
      return CooperativeAwarenessMessage.MOTORCYCLE 
    elif station_type == "Passenger Car":
      return CooperativeAwarenessMessage.PASSENGER_CAR 
    elif station_type == "Bus":
      return CooperativeAwarenessMessage.BUS 
    elif station_type == "Light Truck":
      return CooperativeAwarenessMessage.LIGHT_TRUCK 
    elif station_type == "Heavy Truck":
      return CooperativeAwarenessMessage.HEAVY_TRUCK 
    elif station_type == "Trailer":
      return CooperativeAwarenessMessage.TRAILER 
    elif station_type == "Special Vehicles":
      return CooperativeAwarenessMessage.SPECIAL_VEHICLES 
    elif station_type == "Tram":
      return CooperativeAwarenessMessage.TRAM 
    elif station_type == "Road Side Unit":
      return CooperativeAwarenessMessage.ROAD_SIDE_UNIT 
    else:
      return CooperativeAwarenessMessage.UNKNOWN    

  @staticmethod
  def _on_rsu_event(weak_self, event):
    self = weak_self()
    if not self:
        return

    # Extract sim time from the sensor data timestamp
    current_sim_time = event.timestamp

    # Read Message information
    msg = event[0].get()
    message_dict = msg['Message']['Message']['CAM Parameters']
    basic_container = message_dict['Basic Container']

    power = msg['Power']
    station_id = msg['Message']['Header']['Station ID']
    latitude = basic_container['Reference Position']['Latitude'] * 1e-7
    longitude = basic_container['Reference Position']['Longitude'] * 1e-7
    station_type = basic_container['Station Type']

    try:
      high_container = message_dict['High Frequency Container']['Basic Vehicle Container High Frequency']
      speed = high_container['Speed']['Value'] * 0.01
      # 90 is to correct to global coordinates, not Unreal local
      heading = 90 - (high_container['Heading']['Value'] * 0.1)
      length = high_container['Vehicle Length']['Value'] * 0.001
      width = high_container['Vehicle Width'] * 0.001
    except KeyError:
      speed = 0.0    # If unavailable: 16383
      heading = 0.0  # If unavailable: 3601
      length = 0.0   # If unavailable: 1023
      width = 0.0    # If unavailable: 62

    try:
      low_container = message_dict['Low Frequency Container']['Basic Vehicle Low Frequency']
      vehicle_role = low_container['Vehicle Role']
    except KeyError:
      vehicle_role = 'Default'

    cam_msg = CooperativeAwarenessMessage(
      station_id=station_id,
      power=power,
      latitude=latitude,
      longitude=longitude,
      speed=speed,
      heading=heading,
      length=length,
      width=width,
      vehicle_role=self.get_vehicle_role(vehicle_role),
      station_type=self.get_station_type(station_type)
    )

    with self.lock:
      self.cam_cache[cam_msg.station_id] = CacheEntry(
        last_seen_time=current_sim_time,
        message=cam_msg
      )
