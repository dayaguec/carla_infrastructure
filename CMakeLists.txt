cmake_minimum_required(VERSION 3.8)
project(carla_infrastructure)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(image_transport REQUIRED)

set(msg_files
  "msg/BoundingBox3D.msg"
  "msg/ClassType.msg"
  "msg/Detection.msg"
  "msg/Perception.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES geometry_msgs std_msgs
  ADD_LINTER_TESTS
)

include_directories(
  include
)

add_executable(image_transport_node src/image_transport_node.cpp)
ament_target_dependencies(image_transport_node rclcpp image_transport yaml-cpp sensor_msgs)
target_link_libraries(image_transport_node ${YAML_CPP_LIBRARIES})

ament_python_install_package(infrastructure_carla)

# Install Python executables
install(PROGRAMS
  scripts/infrastructure_node.py
  scripts/camera_pose.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  rviz
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  image_transport_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
