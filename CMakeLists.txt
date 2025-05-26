cmake_minimum_required(VERSION 3.5)
project(joint_state_publisher

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(unitree_hg REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

find_package(rclcpp REQUIRED)

include_directories(
  include
)

add_executable(joint_state_publisher src/joint_state_publisher.cpp)

ament_target_dependencies(joint_state_publisher
  rclcpp
  unitree_hg
  sensor_msgs
  geometry_msgs
  std_msgs
)

install(TARGETS
  joint_state_publisher
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  robots
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()