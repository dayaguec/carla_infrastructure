#!/usr/bin/env python3

"""
Script that renders Carla spectator position. The Spectator is a special actor that spawns to control
Server camera view.

Use W to enable/Disable Render.
Use ESC to Quit PyGame window and shutdown.
"""

import carla
import argparse
import os

import pygame
from pygame.locals import K_ESCAPE
from pygame.locals import K_q
from pygame.locals import K_w

# ==============================================================================
# -- game_loop() ---------------------------------------------------------------
# ==============================================================================
def game_loop(args):
  pygame.init()
  pygame.font.init()

  # PyGame font to display sensor names
  font = pygame.font.Font(pygame.font.get_default_font(), 24)
  font_name = 'courier' if os.name == 'nt' else 'mono'
  fonts = [x for x in pygame.font.get_fonts() if font_name in x]
  default_font = 'ubuntumono'
  mono = default_font if default_font in fonts else fonts[0]
  mono = pygame.font.match_font(mono)
  font_mono = pygame.font.Font(mono, 18 if os.name == 'nt' else 20)

  world = None
  original_settings = None

  try:
    # Connect to Carla Server
    client = carla.Client(args.host, args.port)
    client.set_timeout(float(args.timeout))

    # Get world and main spectator
    world = client.get_world()

    spectator = world.get_spectator()

    original_settings = world.get_settings()

    # Sync mode in Carla to wait for sensor readings in each tick
    if args.sync:
      settings = world.get_settings()
      if not settings.synchronous_mode:
          settings.synchronous_mode = True
          settings.fixed_delta_seconds = 0.05
      world.apply_settings(settings)

      traffic_manager = client.get_trafficmanager()
      traffic_manager.set_synchronous_mode(True)

    # Instantiate all necessary objects
    display = pygame.display.set_mode(
      (args.width, args.height),
      pygame.HWSURFACE | pygame.DOUBLEBUF)
    display.fill((0,0,0))
    pygame.display.flip()

    # Simulation loop
    call_exit = False
    clock = pygame.time.Clock()
    while True:
      # Carla Tick
      if args.sync:
        world.tick()
      else:
        world.wait_for_tick()
      clock.tick_busy_loop(60)

      # Get spectator pose in Carla world
      spectator_transform = spectator.get_transform()
      carla_location = spectator_transform.location
      carla_rotation = spectator_transform.rotation

      # Pose to string conversion
      actor_pose_loc = '(x: ' + "{:.3f}".format(carla_location.x) + ' y: ' + "{:.3f}".format(carla_location.y) + ' z: ' + "{:.3f}".format(carla_location.z) + ')'
      actor_pose_rot = '(yaw: ' + "{:.3f}".format(carla_rotation.yaw) + ' pitch: '+ "{:.3f}".format(carla_rotation.pitch) + ' roll: ' + "{:.3f}".format(carla_rotation.roll) + ')'

      # Clear Pygame surface
      display.fill((0,0,0))

      # Display Location and Orientation on Pygame surface
      text_loc = font_mono.render(actor_pose_loc, True, (0, 255, 0), (0, 0, 150))
      text_rot = font_mono.render(actor_pose_rot, True, (0, 255, 0), (0, 0, 150))
      coords_loc = text_loc.get_rect(center = (args.width / 2, args.height / 2))
      coords_rot = text_rot.get_rect(center = (args.width / 2 + 10, args.height / 2 + 50))
      display.blit(text_loc, coords_loc)
      display.blit(text_rot, coords_rot)
      pygame.display.flip()

      # Print on Terminal for easy copy...
      print(actor_pose_loc)
      print(actor_pose_rot)

      # Parse PyGame keyboard events
      for event in pygame.event.get():
        if event.type == pygame.QUIT:
          call_exit = True
        elif event.type == pygame.KEYDOWN:
          if event.key == K_ESCAPE or event.key == K_q:
            call_exit = True
            break
          elif event.key == K_w:
            settings = world.get_settings()
            if not settings.no_rendering_mode:
              settings.no_rendering_mode = True
            else:
              settings.no_rendering_mode = False
            world.apply_settings(settings)
      if call_exit:
        break
  finally:
    if original_settings:
      world.apply_settings(original_settings)

    pygame.quit()

# ==============================================================================
# -- main() --------------------------------------------------------------------
# ==============================================================================
def main():
  argparser = argparse.ArgumentParser(
    description='CARLA Spectator Position')
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
    '--timeout',
    metavar='T',
    default='20.0',
    help='Waiting time for the server to be up (default: 20.0)')
  argparser.add_argument(
    '--sync',
    action='store_true',
    help='Activate synchronous mode execution')
  argparser.add_argument(
    '--res',
    metavar='WIDTHxHEIGHT',
    default='1280x720',
    help='window resolution (default: 1280x720)')

  # ROS introduce two additional unknown arguments, thus use parse_known_args
  args, unknown = argparser.parse_known_args()

  args.width, args.height = [int(x) for x in args.res.split('x')]

  try:
    game_loop(args)
  except KeyboardInterrupt:
    print('\nCancelled by user. Bye!')

if __name__ == '__main__':
  main()
