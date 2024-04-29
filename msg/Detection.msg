# Msg defining a 3D fused detection
std_msgs/Header header                               # Header
int64 id                                             # Unique Id of the detection
carla_infrastructure/BoundingBox3D bounding_box     # Bounding box
carla_infrastructure/ClassType type                 # Class type of the dectection
geometry_msgs/Vector3 velocity                       # Velocity computed from the detection

