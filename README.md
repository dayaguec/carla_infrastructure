
# Carla Infrastructure
Carla Infrastructure Simulation framework for the generation of custom synthetic data. This package allows to simulate any kind of sensor in Carla deployed in custom infrastructure positions. All sensors readings are converted to ROS Messages in order to generate custom bagfiles.

# ROS Messages
The package contains a folder named **_msg_** with the following ROS interfaces to store sensor readings from Carla Server.
1. **BoundingBox3D**: Msg defining a 3D bounding box.
2. **ClassType**: Msg defining a class for detection systems. Vehicle, pedestrian, etc.
3. **Detection**: Msg defining a 3D detection.
4. **Perception**: Msgs defining a detection array.

# Sensors

Every sensor class that can be instantiated to deploy in simulation is inside the folder **_infrastructure_carla_**. Each class contains a callback for the readings that come from Carla server and a method to convert those to ROS Messages. The available sensors are:
1. **GNSS**: Produces [NavSatFix](https://docs.ros2.org/latest/api/sensor_msgs/msg/NavSatFix.html) ROS Messages.
2. **IMU**: Produces [IMU](https://docs.ros2.org/foxy/api/sensor_msgs/msg/Imu.html) ROS messages.
3. **LiDAR (Raycast)**: Produces [PointCloud2](https://docs.ros2.org/latest/api/sensor_msgs/msg/PointCloud.html) ROS messages.
4. **Radar**: Produces [PointCloud2](https://docs.ros2.org/latest/api/sensor_msgs/msg/PointCloud.html) ROS messages.
5. **RGB Camera**: Produces [Image](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Image.html) ROS messages or [CompressedImage](https://docs.ros2.org/galactic/api/sensor_msgs/msg/CompressedImage.html) if image transport is active.
6. **Semantic Camera**:  Produces [Image](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Image.html) ROS messages or [CompressedImage](https://docs.ros2.org/galactic/api/sensor_msgs/msg/CompressedImage.html) if image transport is active.
7. **Semantic LiDAR**: Produces [PointCloud2](https://docs.ros2.org/latest/api/sensor_msgs/msg/PointCloud.html) ROS messages.

# ROS-Carla Client Node
The Node-Client is composed of three items: the World, the Traffic Manager, and the actual ROS Bridge.
1. **World**: Interacts with the Carla server and extract information from the current map the server is simulating. It spawn the sensors required from configuration files.
2. **Traffic Manager**: Encapsulates the Traffic Manager from Carla to generate custom traffic in the current simulation environment.
3. **ROS-Carla Bridge**: Connects the Carla client to ROS environment by implementing a custom ROS node. Needs the following parameters:
	- **quit_simulation_topic**: Topic at which the node listen to finish the bridge.
	- **ground_truth_topic**: Topic to publish ground truth information in the form of Detection messages.
	- **global_frame_id**: Global frame id from with the Transformation tree is generated.
	- **sensor_params**: Configuration file in yaml format to spawn sensors.

# Configuration
In the **_config_** folder the configuration files for the package can be found. There are two:
1. **global_params**: stablishes all global params for the bridge like the custom topics, the map or the GNSS origin.
2. **infrastructure_params**: Contains all sensors set up to spawn in simulation. Each sensors is composed of its frame_id for the TF tree, the topic to publish its readings, the transform (Carla transform), that is the position in the world, and all its params, like resolution, channels, etc. An arbitrary amount of sensors from the same type can be instantiated by adding multiple entries to each type of sensor.

# Utilities
The package contains several tools:
1. **camera_pose.py**: Spawns a pygame window to extract the position of the spectator camera inside the Carla server. The camera can be moved with WASD.
2. **image_transport_node.cpp**: Images produces by Carla sensors are RAW and uncompressed. In order to deal with a huge number of images, this node just compress all image readings.

# Usage
The package provides launch files to launch the bridge. Follow these steps to launch everything:
1. Launch Carla with ``carla`` or ``carla_low``. Please, note this is a bash alias that execute Carla Binary launch script.
2. Launch the Bridge with ``ros2 launch carla_infrastructure infrastructure.launch``. This will launch everything, including RVIZ, the visualization tool from ROS from which the ground truth and sensor readings can be seen.


