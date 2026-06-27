cmake_minimum_required(VERSION 3.8)
project(mapper)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()


find_package(ament_cmake REQUIRED)

find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(fs_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(lifecycle_msgs REQUIRED)




install(
  PROGRAMS
    ros/mapper_node.py
    ros/sim_track_node.py
    ros/odometry_message.py
    ros/pose_message.py
    ros/mapper_node_life.py
  DESTINATION lib/${PROJECT_NAME}
)

 install(
        PROGRAMS
        src/mapper_function.py
        src/map.py
        src/obstacle.py
        src/vehicle_state.py
        DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
