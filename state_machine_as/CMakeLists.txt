cmake_minimum_required(VERSION 3.5)
project(state_machine_as)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ros_timer_client REQUIRED)
find_package(smacc2 REQUIRED)
find_package(Boost COMPONENTS thread REQUIRED)
find_package(lifecycle_msgs REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  ros_timer_client
  smacc2
  lifecycle_msgs
  rclcpp
)

include_directories(
  include
)

add_executable(state_node
  src/state_machine_as/state_node.cpp
)
add_executable(publisher_tester
  src/state_machine_as/publisher_tester.cpp
)
add_executable(publisher_tester2
  src/state_machine_as/publisher_tester2.cpp
)

target_link_libraries(state_node
  ${smacc2_LIBRARIES}
  ${ros_timer_client_LIBRARIES}
  ${Boost_LIBRARIES}
)

ament_target_dependencies(state_node ${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_target_dependencies(publisher_tester rclcpp std_msgs)
ament_target_dependencies(publisher_tester2 rclcpp std_msgs)


install(
  DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY
  config launch
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  state_node
  publisher_tester
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS
  state_node
  publisher_tester2
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
