from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, RegisterEventHandler, LogInfo, EmitEvent)
from launch.substitutions import (LaunchConfiguration, TextSubstitution,
  PathJoinSubstitution)
from launch_ros.substitutions import FindPackageShare
from launch.events import Shutdown
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_path

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
  carla_world = LaunchConfiguration('carla_world') 
  carla_world_arg = DeclareLaunchArgument(name='carla_world', default_value=TextSubstitution(text='Town10_Opt'),
    description='Carla world to generate, available worlds: Town01, Town02, Town03, Town04, Town05, Town06 \
    Town07, Town10. Use "_Opt" for Layered version')
  sync = LaunchConfiguration('sync')
  sync_arg = DeclareLaunchArgument(name='sync', default_value='', choices=['--sync', ''],
    description='Run Simulation on sync mode')
  standalone = LaunchConfiguration('standalone')
  standalone_arg = DeclareLaunchArgument(name='standalone', default_value='', choices=['--standalone', ''],
    description='Run Simulation on sync mode')
  render = LaunchConfiguration('render')
  render_arg = DeclareLaunchArgument(name='render', default_value='--render', choices=['--render', '--no-render'],
    description='Render graphics on Carla server side, this is overrided if a GPU sensor is spawned')
  geo_projection = LaunchConfiguration('geo_projection') 
  geo_projection_arg = DeclareLaunchArgument(name='geo_projection',
    default_value=TextSubstitution(text='+proj=tmerc +lat_0=40.354550084445 +lon_0=-3.7463664011244586 +k=1 +x_0=0 +y_0=0 +datum=WGS84 +units=m +geoidgrids=egm96_15.gtx +vunits=m +no_defs'),
    description='Geo projection for the GPS fix conversion')
  sensor_params_path = LaunchConfiguration('sensor_params_path',
    default=(get_package_share_path('carla_infrastructure') / 'config/infrastructure_params.yaml'))

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
      '--world', carla_world, '--geo', geo_projection, sync, standalone, render],
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
    carla_world_arg,
    geo_projection_arg,
    sync_arg,
    standalone_arg,
    render_arg,
    infrastructure_node,
    image_transport_node,
    infrastructure_node_handler
  ])