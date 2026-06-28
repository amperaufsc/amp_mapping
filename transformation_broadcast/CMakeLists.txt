cmake_minimum_required(VERSION 3.5)
project(transformation_broadcast)

# Enable Python
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

# Install Python modules and scripts
install(
  PROGRAMS
    ros2/dynamic_transformation.py
    ros2/orbslam_tf_bridge.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  PROGRAMS
    ros2/tf_unifier_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)
