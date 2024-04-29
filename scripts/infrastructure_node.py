#!/usr/bin/env python3

# Infrastructure world integration with CARLA and ROS for arbitration
# implemented by dayaguec@inf.uc3m.es.

# ==============================================================================
# -- imports -------------------------------------------------------------------
# ==============================================================================

import carla

import argparse
import pygame
import numpy as np
import rclpy

from infrastructure_carla.infrastructure_ros_bridge import InfrastructureROSBridge
from infrastructure_carla.infrastructure_world import InfrastructureWorld
from infrastructure_carla.carla_traffic_manager import CarlaTrafficManager
from infrastructure_carla.utils import get_actor_blueprints

# ==============================================================================
# -- game_loop() ---------------------------------------------------------------
# ==============================================================================

def game_loop(args):
  pygame.init()
  rclpy.init(args=None)

  world = None
  ros_bridge = None
  original_settings = None
  sim_world = None
  traffic_manager = None
  sync_opt = False

  try:
    # Connect to Carla Server
    client = carla.Client(args.host, args.port)
    client.set_timeout(float(args.timeout))

    if args.standalone:
      # Parse World and set if needed in Carla Server
      if args.world != client.get_world().get_map().name.split('/')[-1]:
        sim_world = client.load_world(args.world)
      else:
        sim_world = client.get_world()

      original_settings = sim_world.get_settings()

      # Enabled/Disabled rendering in server side to improve performance
      if not args.render:
        settings = sim_world.get_settings()
        settings.no_rendering_mode = True
        sim_world.apply_settings(settings)

      # Sync mode in Carla to wait for sensor readings in each tick
      sync_opt = args.sync
      if sync_opt:
        settings = sim_world.get_settings()
        if not settings.synchronous_mode:
          settings.synchronous_mode = True
          settings.fixed_delta_seconds = 0.05

        sim_world.apply_settings(settings)
    else:
      sim_world = client.get_world()

    world = InfrastructureWorld(sim_world, args)
    ros_bridge = InfrastructureROSBridge(world)
    traffic_manager = CarlaTrafficManager(client=client, tm_port=args.tm_port,
      sync=sync_opt, safe=True, hybrid=False)

    if args.sync:
      sim_world.tick()
    else:
      sim_world.wait_for_tick()

    traffic_manager.generate_traffic()

    # Game loop for PyGame and InfrastructureROSBridge
    clock = pygame.time.Clock()
    loop_flag = False
    while not loop_flag:
      if args.sync:
        sim_world.tick()
      clock.tick_busy_loop(60)
      ros_bridge.tick(clock)
      loop_flag = ros_bridge.is_game_quit()
  finally:
    if original_settings:
      sim_world.apply_settings(original_settings)

    if world is not None:
      world.destroy()
    
    if ros_bridge is not None:
      ros_bridge.destroy()

    if traffic_manager is not None:
      traffic_manager.clean_traffic()

    pygame.quit()
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
    '--geo',
    metavar='GEO_PROJECTION',
    default='+proj=tmerc +lat_0=40.354550084445 +lon_0=-3.7463664011244586 +k=1 +x_0=0 +y_0=0 +datum=WGS84 +units=m +geoidgrids=egm96_15.gtx +vunits=m +no_defs',
    help='Geo projection for the GPS fix conversion')

  # ROS introduces two additional unknown arguments, thus use parse_known_args
  args, unknown = argparser.parse_known_args()

  try:
    game_loop(args)
  except KeyboardInterrupt:
    print('\nCancelled by user. Bye!')

if __name__ == '__main__':
  main()
