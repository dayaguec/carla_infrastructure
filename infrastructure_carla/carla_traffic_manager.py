import carla
from carla import VehicleLightState

from .utils import get_actor_blueprints

import random
import time
import rclpy

class CarlaTrafficManager(object):
  def __init__(self, client, tm_port='8000', sync=False, safe=True, hybrid=True,
               filterv='vehicle.*', generationv='All', filterw='walker.pedestrian.*',
               generationw='2'):
    self._client = client
    self._traffic_manager = self._client.get_trafficmanager(tm_port)
    self._traffic_manager.set_global_distance_to_leading_vehicle(2.5)
    self._synchronous_master = False

    self._vehicles_list = []
    self._walker_list = []
    self._all_id = []
    self._all_actors = []
    self._tf_clean = True

    if sync:
      self._synchronous_master = True
      self._traffic_manager.set_synchronous_mode(True)

    if hybrid:
      self._traffic_manager.set_hybrid_physics_mode(True)
      self._traffic_manager.set_hybrid_physics_radius(50.0)

    self._world = self._client.get_world()

    self._blueprints = get_actor_blueprints(self._world, filterv, generationv)
    self._blueprints_walkers = get_actor_blueprints(self._world, filterw, generationw)

    if safe:
      self._blueprints = [x for x in self._blueprints if x.get_attribute('base_type') == 'car']

    self._blueprints = sorted(self._blueprints, key=lambda bp: bp.id)

  def is_traffic_manager_clean(self):
    return self._tf_clean

  def generate_traffic(self, n_vehicles=10, n_walkers=0, seed=None, seedw=None):
    
    random.seed(seed if seed is not None else int(time.time()))

    spawn_points = self._world.get_map().get_spawn_points()
    number_of_spawn_points = len(spawn_points)

    if n_vehicles < number_of_spawn_points:
      random.shuffle(spawn_points)
    elif n_vehicles > number_of_spawn_points:
      rclpy.logging.get_logger('traffic_manager').info(
        "Requested {} vehicles, but could only find {} spawn points".format(
          n_vehicles, number_of_spawn_points))
      n_vehicles = number_of_spawn_points

    batch = []
    for n, transform in enumerate(spawn_points):
      if n >= n_vehicles:
        break
      blueprint = random.choice(self._blueprints)
      if blueprint.has_attribute('color'):
        color = random.choice(blueprint.get_attribute('color').recommended_values)
        blueprint.set_attribute('color', color)
      if blueprint.has_attribute('driver_id'):
        driver_id = random.choice(blueprint.get_attribute('driver_id').recommended_values)
        blueprint.set_attribute('driver_id', driver_id)

      blueprint.set_attribute('role_name', 'autopilot')

      # prepare the light state of the cars to spawn
      light_state = VehicleLightState.Position | VehicleLightState.LowBeam | VehicleLightState.LowBeam

      # spawn the cars and set their autopilot and light state all together
      batch.append(carla.command.SpawnActor(blueprint, transform)
        .then(carla.command.SetAutopilot(carla.command.FutureActor, True, self._traffic_manager.get_port()))
        .then(carla.command.SetVehicleLightState(carla.command.FutureActor, light_state)))

    # Example of how to use Traffic Manager parameters
    self._traffic_manager.global_percentage_speed_difference(30.0)

    for response in self._client.apply_batch_sync(batch, self._synchronous_master):
      if response.error:
        rclpy.logging.get_logger('traffic_manager').info(response.error)
      else:
        self._vehicles_list.append(response.actor_id)

    # -------------
    # Spawn Walkers
    # -------------
    # some settings
    percentagePedestriansRunning = 0.0      # how many pedestrians will run
    percentagePedestriansCrossing = 0.0     # how many pedestrians will walk through the road
    if seedw is not None:
      self._world.set_pedestrians_seed(seedw)
      random.seed(seedw)
    # 1. Take all the random locations to spawn
    spawn_points = []
    for i in range(n_walkers):
      spawn_point = carla.Transform()
      loc = self._world.get_random_location_from_navigation()
      if (loc is not None):
        spawn_point.location = loc
        spawn_points.append(spawn_point)
    # 2. Spawn the walker object
    batch = []
    walker_speed = []
    for spawn_point in spawn_points:
      walker_bp = random.choice(self._blueprints_walkers)
      if walker_bp.has_attribute('is_invincible'):
        walker_bp.set_attribute('is_invincible', 'false')
      if walker_bp.has_attribute('speed'):
        if (random.random() > percentagePedestriansRunning):
          walker_speed.append(walker_bp.get_attribute('speed').recommended_values[1])
        else:
          walker_speed.append(walker_bp.get_attribute('speed').recommended_values[2])
      else:
        walker_speed.append(0.0)
      batch.append(carla.command.SpawnActor(walker_bp, spawn_point))
    results = self._client.apply_batch_sync(batch, self._synchronous_master)
    walker_speed2 = []
    for i in range(len(results)):
      if results[i].error:
        rclpy.logging.get_logger('traffic_manager').info(results[i].error)
      else:
        self._walker_list.append({"id": results[i].actor_id})
        walker_speed2.append(walker_speed[i])
    walker_speed = walker_speed2
    # 3. Spawn the walker controller
    batch = []
    walker_controller_bp = self._world.get_blueprint_library().find('controller.ai.walker')
    for i in range(len(self._walker_list)):
      batch.append(carla.command.SpawnActor(
        walker_controller_bp, carla.Transform(), self._walker_list[i]["id"]))
    results = self._client.apply_batch_sync(batch, self._synchronous_master)
    for i in range(len(results)):
      if results[i].error:
        rclpy.logging.get_logger('traffic_manager').info(results[i].error)
      else:
        self._walker_list[i]["con"] = results[i].actor_id
    # 4. Put together the walkers and controllers id to get the objects from their id
    for i in range(len(self._walker_list)):
       self._all_id.append(self._walker_list[i]["con"])
       self._all_id.append(self._walker_list[i]["id"])
    self._all_actors = self._world.get_actors(self._all_id)

    # Wait for a tick to ensure client receives the last transform
    # of the walkers we have just created
    if not self._synchronous_master:
      self._world.wait_for_tick()
    else:
      self._world.tick()

    # 5. Initialize each controller and set target to walk to
    #    (list is [controler, actor, controller, actor ...])
    #    set how many pedestrians can cross the road
    self._world.set_pedestrians_cross_factor(percentagePedestriansCrossing)
    for i in range(0, len(self._all_id), 2):
      self._all_actors[i].start()
      self._all_actors[i].go_to_location(self._world.get_random_location_from_navigation())
      self._all_actors[i].set_max_speed(float(walker_speed[int(i/2)]))

    self._tf_clean = False

  def clean_traffic(self):
    if not self._tf_clean:
      self._client.apply_batch([carla.command.DestroyActor(x) for x in self._vehicles_list])

      # Stop walker controllers (list is [controller, actor, controller, actor ...])
      for i in range(0, len(self._all_id), 2):
        self._all_actors[i].stop()

      self._client.apply_batch([carla.command.DestroyActor(x) for x in self._all_id])
      
      self._vehicles_list = []
      self._walker_list = []
      self._all_id = []
      self._all_actors = []
      self._tf_clean = True
