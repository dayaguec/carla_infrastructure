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

  # Carla ROS bridge and infrastrucutre simulation
  carla_infrastructure_node_launch = GroupAction(
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
            'sync' : '',                        # '--sync' or ''
            'standalone' : '--standalone',      # '--standalone' or ''
            'render' : '--render',  # '--no-render' or '--render',
            'geo' : global_params.get('geo_projection',
              '+proj=tmerc +lat_0=40.354550084445 +lon_0=-3.7463664011244586 +k=1 +x_0=0 +y_0=0 +datum=WGS84 +units=m +geoidgrids=egm96_15.gtx +vunits=m +no_defs'),
            # Global parameters from yaml
            'global_frame_id' : global_params.get('map_frame', 'world'),
            'quit_simulation_topic' : global_params.get('quit_simulation_topic', 'quit_simulation'),
            'ground_truth_topic' : global_params.get('ground_truth_topic', 'ground_truth'),
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

  config_fps = TimerAction(
    period = 6.0,
    actions = [
      ExecuteProcess(
        cmd=[["python3 ~/Carla/PythonAPI/util/config.py --fps 60.0"]],
        shell=True
        )
    ]
  )

  return LaunchDescription([
    namespace_arg,
    carla_infrastructure_node_launch,
    config_fps,
    rviz_node
  ])
