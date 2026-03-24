from ament_index_python.packages import get_package_share_path

from launch_ros.actions import (PushRosNamespace, Node)
from launch import LaunchDescription
from launch.actions import (IncludeLaunchDescription, DeclareLaunchArgument, GroupAction, TimerAction, Shutdown, ExecuteProcess)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (LaunchConfiguration, PathJoinSubstitution,
  TextSubstitution)
from launch_ros.substitutions import FindPackageShare

import yaml
from yaml.loader import SafeLoader

def generate_launch_description():

  namespace = LaunchConfiguration('namespace')
  namespace_arg = DeclareLaunchArgument(name='namespace', default_value=TextSubstitution(text='infrastructure'),
    description='Name for node namespace')

  # Parse global yaml config file
  global_params_yaml = get_package_share_path('carla_infrastructure') / 'config/global_params.yaml'
  global_params = None
  with open(global_params_yaml) as f:
    global_params = yaml.load(f, Loader=SafeLoader)

  ############################################################
  ########################### CARLA ##########################
  ############################################################
  sensor_params_yaml = PathJoinSubstitution([
    FindPackageShare('carla_infrastructure'), 'config/infrastructure_params.yaml'
  ])
  # Carla ROS bridge and infrastrucutre simulation
  infrastructure_control_launch = GroupAction(
    actions = [
      PushRosNamespace(namespace),
      IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
          PathJoinSubstitution([
            FindPackageShare('carla_infrastructure'), 'launch', 'carla_infrastructure.launch.py'
            ])
          ]),
          launch_arguments = {
            # Launch parameters
            'carla_world' : global_params.get('hd_map', 'Town10_Opt'),
            'sync' : '--sync',                  # '--sync' or ''
            'standalone' : '--standalone',      # '--standalone' or ''
            'render' : '--render',  # '--no-render' or '--render',
            'enable_tm' : '', # '--enable-tm' (Traffic Manager) or ''
            'enable_tfr' : '',# '--enable-tfr' (Traffic Light Manager) or '',
            'image_transport_on' : 'true',
            'sensor_params_path' : sensor_params_yaml,
            'target_fps' : str(global_params.get('target_fps', 60.0)),
            # Global parameters from yaml
            'global_frame_id' : global_params.get('map_frame', 'world'),
            'quit_simulation_topic' : global_params.get('quit_simulation_topic', 'quit_simulation'),
            'infrastructure_perception_event_topic' : global_params.get('perception_event_topic', 'infrastructure_perception_event'),
            'infrastructure_traffic_lights_topic' : global_params.get('traffic_lights_topic', 'infrastructure_traffic_lights_info'),
          }.items()
      )
    ]
  )

  ##################################################################
  ########################## VISUALIZATION #########################
  ##################################################################

  rviz_config_path = PathJoinSubstitution([
    FindPackageShare('carla_infrastructure'), 'rviz', 'carla_infrastructure.rviz'
  ])
  rviz_node = TimerAction(
    period = 1.0,
    actions = [
      Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path]
      )
    ]
  )

  return LaunchDescription([
    namespace_arg,
    infrastructure_control_launch,
    rviz_node
  ])
