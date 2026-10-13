cmake_minimum_required(VERSION 3.8)
project(jo_detector)

if(POLICY CMP0144)
  cmake_policy(SET CMP0144 NEW)
endif()
set(CMAKE_SUPPRESS_DEVELOPER_WARNINGS 1 CACHE INTERNAL "Suppress third-party CMake developer warnings")

# --------------------------------------------------
# Compile options
# --------------------------------------------------
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-O3 -Wall)

# Set RPATH so binary can find libraries
set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}")
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

# --------------------------------------------------
# Find dependencies
# --------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(image_transport REQUIRED)
find_package(vision_msgs REQUIRED)
unset(FLANN_ROOT)
find_package(PCL REQUIRED)

find_package(pcl_conversions REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(jo_msgs REQUIRED)

# gtsam / gtsam_points: used by calibration_icp_node for GICP-based
# depth<->lidar registration (same registration stack already required by
# the glim submodule, see glim/CMakeLists.txt), replacing the former Open3D
# dependency.
find_package(GTSAM 4.2 REQUIRED)
find_package(gtsam_points 1.2.0 REQUIRED)

# For services
find_package(rosidl_default_generators REQUIRED)

# --------------------------------------------------
# Service definition
# --------------------------------------------------
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GetDynamicObstacles.srv"
  DEPENDENCIES geometry_msgs
)

# --------------------------------------------------
# Include directories
# --------------------------------------------------
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

# --------------------------------------------------
# Library
# --------------------------------------------------
add_library(${PROJECT_NAME}_lib
  include/${PROJECT_NAME}/dynamicDetector.cpp
  include/${PROJECT_NAME}/uvDetector.cpp
  include/${PROJECT_NAME}/dbscan.cpp
  include/${PROJECT_NAME}/kalmanFilter.cpp
  include/${PROJECT_NAME}/lidarDetector.cpp
)

ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  visualization_msgs
  cv_bridge
  message_filters
  image_transport
  vision_msgs
  pcl_conversions
  tf2
  tf2_geometry_msgs
  jo_msgs
)

target_link_libraries(${PROJECT_NAME}_lib
  ${PCL_LIBRARIES}
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)

target_link_libraries(${PROJECT_NAME}_lib
  "${cpp_typesupport_target}")

# --------------------------------------------------
# Executable
# --------------------------------------------------
add_executable(detector_node src/detector_node.cpp)

target_link_libraries(detector_node ${PROJECT_NAME}_lib)
ament_target_dependencies(detector_node
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  visualization_msgs
  cv_bridge
  message_filters
  image_transport
  vision_msgs
  pcl_conversions
  tf2
  tf2_geometry_msgs
  jo_msgs
)

# Calibration node executable
add_executable(calibration_node src/calibration_node.cpp)

ament_target_dependencies(calibration_node
  rclcpp
  sensor_msgs
  cv_bridge
  message_filters
  image_transport
  pcl_conversions
  tf2
  tf2_geometry_msgs
)

target_link_libraries(calibration_node
  ${PCL_LIBRARIES}
)

# Calibration ICP node executable
add_executable(calibration_icp_node src/calibration_icp_node.cpp)

ament_target_dependencies(calibration_icp_node
  rclcpp
  geometry_msgs
  sensor_msgs
  cv_bridge
  message_filters
  image_transport
  pcl_conversions
  tf2
  tf2_geometry_msgs
)

target_link_libraries(calibration_icp_node
  ${PCL_LIBRARIES}
  gtsam
  gtsam_points::gtsam_points
)

# Wall detector node executable (standalone, under scripts/wall_detector/)
add_executable(wall_detector_node
  scripts/wall_detector/wall_detector_node.cpp
  scripts/wall_detector/wallDetector.cpp
)
target_include_directories(wall_detector_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/scripts/wall_detector
)
target_link_libraries(wall_detector_node ${PCL_LIBRARIES})
ament_target_dependencies(wall_detector_node
  rclcpp
  geometry_msgs
  sensor_msgs
  std_msgs
  visualization_msgs
  pcl_conversions
  cv_bridge
  image_transport
  tf2
  tf2_ros
  tf2_geometry_msgs
)


# Image restamp node (generic image_transport decode + header.stamp fix —
# see scripts/image_restamper/image_restamp_node.cpp for why this exists)
add_executable(image_restamp_node
  scripts/image_restamper/image_restamp_node.cpp
)
ament_target_dependencies(image_restamp_node
  rclcpp
  sensor_msgs
  image_transport
)

# Odometry TF publisher node
add_executable(odometry_tf_publisher_node src/odometry_tf_publisher.cpp)
ament_target_dependencies(odometry_tf_publisher_node
  rclcpp
  geometry_msgs
  nav_msgs
  tf2_ros
)

# --------------------------------------------------
# Install
# --------------------------------------------------
install(TARGETS
  ${PROJECT_NAME}_lib
  detector_node
  calibration_node
  calibration_icp_node
  wall_detector_node
  image_restamp_node
  odometry_tf_publisher_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY cfg/
  DESTINATION share/${PROJECT_NAME}/cfg
)

install(DIRECTORY rviz/
  DESTINATION share/${PROJECT_NAME}/rviz
)

# Install Python node entrypoints as executables
install(PROGRAMS
  scripts/yolo_detector/yolo_detector_node.py
  scripts/yolo_detector/yolov11_detector_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install all scripts (for weights, modules, etc.)
install(DIRECTORY scripts/
  DESTINATION share/${PROJECT_NAME}/scripts
  USE_SOURCE_PERMISSIONS
)

# --------------------------------------------------
# Export
# --------------------------------------------------
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_lib)
ament_package()