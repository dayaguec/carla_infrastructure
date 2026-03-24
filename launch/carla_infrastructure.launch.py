from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, RegisterEventHandler, LogInfo, EmitEvent)
from launch.substitutions import (LaunchConfiguration, TextSubstitution,
  PathJoinSubstitution)
from launch_ros.substitutions import FindPackageShare
from launch.events import Shutdown
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_path
from launch.conditions import IfCondition

def generate_launch_description():
  host = LaunchConfiguration('host')
  host_arg = DeclareLaunchArgument(name='host', default_value=TextSubstitution(text='localhost'),
    description='Network location of the machine where Carla server is running')
  port = LaunchConfiguration('port')
  port_arg = DeclareLaunchArgument(name='port', default_value=TextSubstitution(text='2000'),
    description='Port for the communication with Carla server')
  timeout = LaunchConfiguration('timeout') 
  timeout_arg = DeclareLaunchArgument(name='timeout', default_value=TextSubstitution(text='20.0'),
    description='Time to wait for Carla server response to the client')
  target_fps = LaunchConfiguration('target_fps') 
  target_fps_arg = DeclareLaunchArgument(name='target_fps', default_value=TextSubstitution(text='60.0'),
    description='Target FPS for the simulation')
  carla_world = LaunchConfiguration('carla_world') 
  carla_world_arg = DeclareLaunchArgument(name='carla_world', default_value=TextSubstitution(text='Town10_Opt'),
    description='Carla world to generate, available worlds: Town01, Town02, Town03, Town04, Town05, Town06 \
    Town07, Town10. Use "_Opt" for Layered version')
  sync = LaunchConfiguration('sync')
  sync_arg = DeclareLaunchArgument(name='sync', default_value='', choices=['--sync', ''],
    description='Run Simulation on sync mode')
  standalone = LaunchConfiguration('standalone')
  standalone_arg = DeclareLaunchArgument(name='standalone', default_value='', choices=['--standalone', ''],
    description='Run Simulation standalone')
  render = LaunchConfiguration('render')
  render_arg = DeclareLaunchArgument(name='render', default_value='--render', choices=['--render', '--no-render'],
    description='Render graphics on Carla server side, this is overrided if a GPU sensor is spawned')
  sensor_params_path = LaunchConfiguration('sensor_params_path',
    default=(get_package_share_path('carla_infrastructure') / 'config/infrastructure_params.yaml'))
  enable_tm = LaunchConfiguration('enable_tm')
  enable_tm_arg = DeclareLaunchArgument(name='enable_tm', default_value='', choices=['--enable-tm', ''],
    description='Run Simulation with Traffic Manager')
  enable_tfr = LaunchConfiguration('enable_tfr')
  enable_tfr_arg = DeclareLaunchArgument(name='enable_tfr', default_value='', choices=['--enable-tfr', ''],
    description='Run Simulation with Traffic Lights Manager')
  image_transport_on = LaunchConfiguration('image_transport_on') 
  image_transport_on_arg = DeclareLaunchArgument(name='image_transport_on', default_value='true',
    choices=['true', 'false'],
    description='Enable Image Transport for RGB Images')

  # Global parameters from top level launch
  quit_simulation_topic = LaunchConfiguration('quit_simulation_topic', default='quit_simulation')
  ground_truth_topic = LaunchConfiguration('ground_truth_topic', default='ground_truth')
  global_frame_id = LaunchConfiguration('global_frame_id', default='world')

  # CARLA infrastructure ROS Bridge
  infrastructure_node = Node(
    package='carla_infrastructure',
    executable='infrastructure_node.py',
    name='infrastructure_node',
    output='screen',
    arguments=['--host', host, '--port', port, '--timeout', timeout,
      '--world', carla_world, '--fps', target_fps, sync, standalone, render, enable_tm, enable_tfr],
    parameters=[
      {'quit_simulation_topic': quit_simulation_topic},
      {'ground_truth_topic': ground_truth_topic},
      {'global_frame_id': global_frame_id},
      {'sensor_params': sensor_params_path},
    ]
  )

  image_transport_node = Node(
    package='carla_infrastructure',
    executable='image_transport_node',
    name='image_transport_node',
    condition=IfCondition(image_transport_on),
    output='screen',
    parameters=[
      # Private params
      {'sensor_params': sensor_params_path},
      {'in_transport': "raw"}
    ]
  )

  # Handlers for shutdown
  infrastructure_node_handler = RegisterEventHandler(
    OnProcessExit(
      target_action=infrastructure_node,
        on_exit=[
          LogInfo(msg='Carla-ROS infrastructure node is required!'),
            EmitEvent(event=Shutdown(
              reason='Carla-ROS infrastructure node exited'))
      ]
    )
  )

  return LaunchDescription([
    host_arg,
    port_arg,
    timeout_arg,
    target_fps_arg,
    carla_world_arg,
    enable_tm_arg,
    enable_tfr_arg,
    sync_arg,
    image_transport_on_arg,
    standalone_arg,
    render_arg,
    infrastructure_node,
    image_transport_node,
    infrastructure_node_handler
  ])
