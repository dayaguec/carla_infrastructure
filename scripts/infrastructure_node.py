#!/usr/bin/env python3

# Infrastructure world integration with CARLA and ROS for arbitration
# implemented by dayaguec@inf.uc3m.es.

# ==============================================================================
# -- imports -------------------------------------------------------------------
# ==============================================================================

import carla

import argparse
import rclpy
import time

from infrastructure_carla.infrastructure_ros_bridge import InfrastructureROSBridge
from infrastructure_carla.infrastructure_world import InfrastructureWorld

# ==============================================================================
# -- game_loop() ---------------------------------------------------------------
# ==============================================================================

def game_loop(args, ros_args):
  world = None
  ros_bridge = None
  original_settings = None
  sim_world = None
  traffic_manager = None
  traffic_light_manager = None
  sync_opt = False

  try:
    global_logger = rclpy.logging.get_logger('carla_infrastructure_node')
    global_logger.info("Connecting to CARLA...")

    # Connect to Carla Server
    client = carla.Client(args.host, args.port)
    client.set_timeout(float(args.timeout))

    if args.standalone:
      # Parse World and set if needed in Carla Server
      if (args.world).lower() != (client.get_world().get_map().name.split('/')[-1]).lower():
        sim_world = client.load_world(args.world)
      else:
        sim_world = client.get_world()

      original_settings = sim_world.get_settings()
      settings = sim_world.get_settings()

      # Enabled/Disabled rendering in server side to improve performance
      if not args.render:
        settings.no_rendering_mode = True

      # Sync mode in Carla to wait for sensor readings in each tick
      sync_opt = args.sync
      settings.synchronous_mode = sync_opt
      settings.fixed_delta_seconds = 1.0 / args.fps if sync_opt else None

      sim_world.apply_settings(settings)
    else:
      sim_world = client.get_world()
      sync_opt = sim_world.get_settings().synchronous_mode

    rclpy.init(args=ros_args)
    world = InfrastructureWorld(sim_world, args)
    ros_bridge = InfrastructureROSBridge(world, sync_opt, args.standalone)
    
    if args.enable_tm and args.standalone:
      from infrastructure_carla.management.carla_traffic_manager import CarlaTrafficManager
      traffic_manager = CarlaTrafficManager(client=client, tm_port=args.tm_port)
      ros_bridge.set_traffic_manager(traffic_manager)

    if args.enable_tfr and args.standalone:
      from infrastructure_carla.management.traffic_light_manager import TrafficLightManager
      traffic_light_manager = TrafficLightManager(world)

    if args.standalone:
      if sync_opt:
        while rclpy.ok():
          with ros_bridge.get_lock():
            if ros_bridge.is_game_quit():
              break

            _ = sim_world.tick()
            snapshot = sim_world.get_snapshot()
            ros_bridge.on_tick(snapshot)

            if args.enable_tfr:
              traffic_light_manager.update_control_loop()

          time.sleep(0.01)
      else:
        while rclpy.ok():
          if args.enable_tfr:
            traffic_light_manager.update_control_loop()

          if ros_bridge.is_game_quit():
            break

          time.sleep(0.01)
    else:
      # On tick callback already defined at node level, no need to control tick in subordinate mode
      while rclpy.ok():
        if ros_bridge.is_game_quit():
          break
        time.sleep(0.01)
  except KeyboardInterrupt:
    global_logger.info("Shutdown requested by user!")
  finally:
    if original_settings:
      global_logger.info("Restoring CARLA settings...")
      sim_world.apply_settings(original_settings)

    if world is not None:
      global_logger.info("Destroying World...")
      world.destroy()
    
    if ros_bridge is not None:
      global_logger.info("Shutting down ROS...")
      ros_bridge.destroy()

    if traffic_manager is not None:
      traffic_manager.clean_traffic()

    rclpy.shutdown()

# ==============================================================================
# -- main() --------------------------------------------------------------------
# ==============================================================================
def main():
  argparser = argparse.ArgumentParser(
    description='CARLA-ROS Infrastructure Client')
  argparser.add_argument(
    '-v', '--verbose',
    action='store_true',
    dest='debug',
    help='print debug information')
  argparser.add_argument(
    '--host',
    metavar='H',
    default='127.0.0.1',
    help='IP of the host server (default: 127.0.0.1)')
  argparser.add_argument(
    '-p', '--port',
    metavar='P',
    default=2000,
    type=int,
    help='TCP port to listen to (default: 2000)')
  argparser.add_argument(
    '--tm-port',
    metavar='P',
    default=8000,
    type=int,
    help='Port to communicate with TM (default: 8000)')
  argparser.add_argument(
    '--timeout',
    metavar='T',
    default='20.0',
    help='Waiting time for the server to be up (default: 20.0)')
  argparser.add_argument(
    '--sync',
    action='store_true',
    help='Activate synchronous mode execution')
  argparser.add_argument(
    '--standalone',
    action='store_true',
    help='Run the node as a standalone package')
  argparser.add_argument(
    '--render',
    dest='render',
    action='store_true',
    help='Render graphics on Carla server')
  argparser.add_argument(
    '--no-render',
    dest='render',
    action='store_false',
    help='Do not render graphics on Carla server')
  argparser.set_defaults(render=True)
  argparser.add_argument(
    '--world',
    metavar='WORLD',
    default='Town10_Opt',
    help='Carla world to generate (default: "Town10_Opt")')
  argparser.add_argument(
    '--enable-tm',
    dest='enable_tm',
    action='store_true',
    help='Do not connect node to Traffic Manager')
  argparser.set_defaults(enable_tm=False)
  argparser.add_argument(
    '--enable-tfr',
    dest='enable_tfr',
    action='store_true',
    help='Do not enable Traffic Light Rescheduling')
  argparser.set_defaults(enable_tfr=False)
  argparser.add_argument('--fps', type=float, default=60.0, help="Target FPS")

  # ROS introduces two additional unknown arguments, thus use parse_known_args
  args, ros_args = argparser.parse_known_args()

  game_loop(args, ros_args)

if __name__ == '__main__':
  main()
