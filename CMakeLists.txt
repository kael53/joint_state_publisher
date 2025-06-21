cmake_minimum_required(VERSION 3.5)
project(unitree_g1_dex3_stack)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

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
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(unitree_hg REQUIRED)
find_package(urdf REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(trac_ik_lib REQUIRED)
find_package(ompl REQUIRED)
find_package(fcl REQUIRED)
find_package(geometric_shapes REQUIRED)
find_package(resource_retriever REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(bboxes_ex_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(message_filters REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(ament_index_cpp REQUIRED)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OMPL_INCLUDE_DIRS}
  ${FCL_INCLUDE_DIRS}
  ${TRAC_IK_INCLUDE_DIRS}
)

add_executable(joint_state_publisher src/joint_state_publisher.cpp)
add_executable(dex3_controller src/dex3_controller.cpp)
add_executable(joint_trajectory_executor src/joint_trajectory_executor.cpp)
add_executable(ik_fcl_ompl_planner src/ik_fcl_ompl_planner.cpp)
add_executable(project_to_3d_node src/project_to_3d_node.cpp)
add_executable(detection_to_goal_node src/detection_to_goal_node.cpp)

ament_target_dependencies(joint_state_publisher
  rclcpp unitree_hg sensor_msgs geometry_msgs std_msgs urdf kdl_parser
)
ament_target_dependencies(dex3_controller
  rclcpp unitree_hg sensor_msgs geometry_msgs std_msgs urdf
)
ament_target_dependencies(joint_trajectory_executor
  rclcpp unitree_hg sensor_msgs geometry_msgs std_msgs trajectory_msgs urdf
)
ament_target_dependencies(ik_fcl_ompl_planner
  rclcpp sensor_msgs geometry_msgs std_msgs trajectory_msgs urdf kdl_parser trac_ik_lib ompl fcl geometric_shapes resource_retriever vision_msgs ament_index_cpp
)
ament_target_dependencies(project_to_3d_node
  rclcpp vision_msgs bboxes_ex_msgs sensor_msgs geometry_msgs std_msgs cv_bridge image_transport message_filters pcl_conversions
)
ament_target_dependencies(detection_to_goal_node
  rclcpp vision_msgs bboxes_ex_msgs sensor_msgs geometry_msgs std_msgs
)

target_link_libraries(ik_fcl_ompl_planner
  ${OMPL_LIBRARIES}
  ${FCL_LIBRARIES}
  ${TRAC_IK_LIBRARIES}
)

target_link_libraries(project_to_3d_node
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
)

install(TARGETS
  joint_state_publisher
  dex3_controller
  joint_trajectory_executor
  ik_fcl_ompl_planner
  project_to_3d_node
  detection_to_goal_node
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