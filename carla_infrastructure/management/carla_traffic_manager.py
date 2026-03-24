import carla
import random
import rclpy

class CarlaTrafficManager(object):
  def __init__(self, client, tm_port=8000):
    self.client = client
    self.world = self.client.get_world()
    self.traffic_manager = self.client.get_trafficmanager(tm_port)
    self.tm_port = tm_port
    
    # State tracking for cleanup
    self.vehicles_list = []
    self.walkers_list = []
    self.all_id = []
    
    # Configuration
    self.num_vehicles = 0
    self.num_walkers = 0
    self.hybrid_physics = False

    self.synchronous_mode = self.world.get_settings().synchronous_mode

    self.tf_clean = True

  def setup_parameters(self, num_vehicles, num_walkers,
    seed=None, hybrid_physics=False, hybrid_radius=50.0):
    """Configures traffic generation parameters and TM settings."""
    self.num_vehicles = num_vehicles
    self.num_walkers = num_walkers
    self.hybrid_physics = hybrid_physics
    
    # Set seeds for deterministic generation if provided
    if seed is not None:
      self.traffic_manager.set_random_device_seed(seed)

    # Configure Hybrid Physics
    if self.hybrid_physics:
      self.traffic_manager.set_hybrid_physics_mode(True)
      self.traffic_manager.set_hybrid_physics_radius(hybrid_radius)
    else:
      self.traffic_manager.set_hybrid_physics_mode(False)

    # General TM settings
    self.traffic_manager.set_global_distance_to_leading_vehicle(2.5)
    self.traffic_manager.global_percentage_speed_difference(30.0)
    if self.synchronous_mode:
      self.traffic_manager.set_synchronous_mode(True)

  def spawn_traffic(self):
    if not self.tf_clean:
      return False, "Traffic Manager is not clean, clean it first before spawning new actors!"

    """Spawns vehicles and walkers based on current parameters."""
    blueprints = self.world.get_blueprint_library()

    # 1. Spawn Vehicles (force safe)
    vehicle_bps = blueprints.filter('vehicle.*')
    vehicle_bps = [bp for bp in vehicle_bps if bp.get_attribute('base_type') == 'car']
    spawn_points = self.world.get_map().get_spawn_points()

    max_spawn_points = len(spawn_points)
    if self.num_vehicles > max_spawn_points:
      actual_num_vehicles = max_spawn_points
    else:
      actual_num_vehicles = self.num_vehicles

    random.shuffle(spawn_points)
    
    SpawnActor = carla.command.SpawnActor
    SetAutopilot = carla.command.SetAutopilot
    FutureActor = carla.command.FutureActor

    batch = []
    for i, transform in enumerate(spawn_points[:actual_num_vehicles]):
      blueprint = random.choice(vehicle_bps)
      
      # If hybrid physics is enabled, we need a "hero" vehicle for the radius calculation
      if self.hybrid_physics and i == 0:
        blueprint.set_attribute('role_name', 'hero')
      else:
        blueprint.set_attribute('role_name', 'autopilot')

      if blueprint.has_attribute('color'):
        color = random.choice(blueprint.get_attribute('color').recommended_values)
        blueprint.set_attribute('color', color)

      # Spawn and hand over to Traffic Manager
      batch.append(SpawnActor(blueprint, transform)
                   .then(SetAutopilot(FutureActor, True, self.tm_port)))

    # Execute vehicle batch
    for response in self.client.apply_batch_sync(batch, self.synchronous_mode):
      if response.error:
        logging.error(response.error)
      else:
        self.vehicles_list.append(response.actor_id)

    # 2. Spawn Walkers @todo: Adjust perceptage of speed/crosswalk/use_wheelchair
    walker_bps = blueprints.filter('walker.pedestrian.*')
    walker_controller_bp = blueprints.find('controller.ai.walker')
    
    batch = []
    walker_locs = []
    for _ in range(self.num_walkers):
      spawn_point = carla.Transform()
      loc = self.world.get_random_location_from_navigation()
      if loc:
        spawn_point.location = loc
        walker_locs.append(spawn_point)
        batch.append(SpawnActor(random.choice(walker_bps), spawn_point))

    # Execute walker batch
    walker_ids = []
    for response in self.client.apply_batch_sync(batch, self.synchronous_mode):
      if not response.error:
        walker_ids.append(response.actor_id)

    # 3. Spawn Walker Controllers
    batch = []
    for walker_id in walker_ids:
      batch.append(SpawnActor(walker_controller_bp, carla.Transform(), walker_id))
        
    controller_ids = []
    for response in self.client.apply_batch_sync(batch, self.synchronous_mode):
      if not response.error:
        controller_ids.append(response.actor_id)

    self.walkers_list.extend(walker_ids)
    self.walkers_list.extend(controller_ids)
    self.all_id = self.vehicles_list + self.walkers_list

    # 4. Initialize Walker AI (Requires a world tick first!)
    if self.synchronous_mode:
      self.world.tick()
    else:
      self.world.wait_for_tick()

    # Start the controllers and give them a random target
    world_actors = self.world.get_actors()
    controllers = world_actors.filter('controller.ai.walker')
    for controller in controllers:
      controller.start()
      controller.go_to_location(self.world.get_random_location_from_navigation())
      controller.set_max_speed(1 + random.random()) # Random speed

    self.tf_clean = False

    return True, 'Spawned {} vehicles and {} walkers.'.format(len(self.vehicles_list), len(walker_ids))

  def clean_traffic(self):
    """Destroys all spawned actors to clean the simulation."""
    if self.tf_clean:
      return False, 'Traffic Manager is already clean...'

    # Stop walker controllers first
    controllers = self.world.get_actors().filter('controller.ai.walker')
    for controller in controllers:
      controller.stop()

    # Batch destroy to prevent simulation hang
    batch = [carla.command.DestroyActor(x) for x in self.all_id]
    
    self.client.apply_batch_sync(batch, self.synchronous_mode)
    
    self.vehicles_list.clear()
    self.walkers_list.clear()
    self.all_id.clear()

    self.tf_clean = True
    
    return True, 'Traffic successfully cleaned.'
