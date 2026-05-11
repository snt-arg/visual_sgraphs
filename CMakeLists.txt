cmake_minimum_required(VERSION 3.5)
project(vs_graphs)

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall   -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3")

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

find_package(OpenCV 4.2)
if(NOT OpenCV_FOUND)
  message(FATAL_ERROR "OpenCV 4.2 not found.")
endif()

MESSAGE("OPENCV VERSION:")
MESSAGE(${OpenCV_VERSION})

# Find required packages
find_package(PCL REQUIRED)
find_package(tf2 REQUIRED)
find_package(GLEW REQUIRED)
find_package(rclpy REQUIRED)
find_package(OpenGL REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Pangolin REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(Eigen3 3.1.0 REQUIRED)
find_package(backward_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(segmenter_ros REQUIRED)
find_package(image_transport REQUIRED)
find_package(message_filters REQUIRED)
find_package(rviz_visual_tools REQUIRED)
find_package(rosidl_default_runtime REQUIRED)
find_package(situational_graphs_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
# find_package(aruco_ros REQUIRED)
# find_package(aruco_msgs REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Generate services
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SaveMap.srv"
  "msg/VSGraphsWallData.msg"
  "msg/VSGraphsRoomData.msg"
  "msg/VSGraphsAllWallsData.msg"
  "msg/VSGraphsAllDetectdetRooms.msg"
  DEPENDENCIES std_msgs geometry_msgs sensor_msgs
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_SOURCE_DIR}/include
  ${PROJECT_SOURCE_DIR}/core
  ${PROJECT_SOURCE_DIR}/core/include
  ${PROJECT_SOURCE_DIR}/core/include/CameraModels
  ${PROJECT_SOURCE_DIR}/core/Thirdparty
  ${PROJECT_SOURCE_DIR}/core/Thirdparty/Sophus
  ${EIGEN3_INCLUDE_DIR}
  ${Pangolin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${segmenter_ros_INCLUDE_DIRS}
)
include_directories(${OPENGL_INCLUDE_DIRS})

# ROS2 uses ament_target_dependencies instead of catkin_INCLUDE_DIRS
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Include third-party CMakeLists
include(${PROJECT_SOURCE_DIR}/core/Thirdparty/DBoW2/CMakeLists.txt)
include(${PROJECT_SOURCE_DIR}/core/Thirdparty/g2o/CMakeLists.txt)

# Set library output directory
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/core/lib)

# Create the main library
add_library(${PROJECT_NAME}_lib SHARED
  core/src/System.cc
  core/src/Tracking.cc
  core/src/LocalMapping.cc
  core/src/LoopClosing.cc
  core/src/ORBextractor.cc
  core/src/ORBmatcher.cc
  core/src/FrameDrawer.cc
  core/src/Converter.cc
  core/src/MapPoint.cc
  core/src/KeyFrame.cc
  core/src/Atlas.cc
  core/src/Map.cc
  core/src/MapDrawer.cc
  core/src/Optimizer.cc
  core/src/Frame.cc
  core/src/KeyFrameDatabase.cc
  core/src/Sim3Solver.cc
  core/src/Viewer.cc
  core/src/ImuTypes.cc
  core/src/G2oTypes.cc
  core/src/CameraModels/Pinhole.cpp
  core/src/CameraModels/KannalaBrandt8.cpp
  core/src/OptimizableTypes.cpp
  core/src/MLPnPsolver.cpp
  core/src/GeometricTools.cc
  core/src/TwoViewReconstruction.cc
  core/src/Utils.cc
  core/src/Config.cc
  core/src/Settings.cc
  core/src/GeoSemHelpers.cc
  core/src/Semantic/Room.cc
  core/src/Semantic/Floor.cc
  core/src/DatabaseParser.cc
  core/src/Semantic/Marker.cc
  core/src/Geometric/Plane.cc
  core/src/Semantic/Passage.cc
  core/src/Types/SystemParams.cc
  core/src/SemanticSegmentation.cc
  core/src/SemanticsManager.cc
  core/include/System.h
  core/include/Tracking.h
  core/include/LocalMapping.h
  core/include/LoopClosing.h
  core/include/ORBextractor.h
  core/include/ORBmatcher.h
  core/include/FrameDrawer.h
  core/include/Converter.h
  core/include/MapPoint.h
  core/include/KeyFrame.h
  core/include/Atlas.h
  core/include/Map.h
  core/include/MapDrawer.h
  core/include/Optimizer.h
  core/include/Frame.h
  core/include/KeyFrameDatabase.h
  core/include/Sim3Solver.h
  core/include/Viewer.h
  core/include/ImuTypes.h
  core/include/G2oTypes.h
  core/include/CameraModels/GeometricCamera.h
  core/include/CameraModels/Pinhole.h
  core/include/CameraModels/KannalaBrandt8.h
  core/include/OptimizableTypes.h
  core/include/MLPnPsolver.h
  core/include/GeometricTools.h
  core/include/TwoViewReconstruction.h
  core/include/SerializationUtils.h
  core/include/Config.h
  core/include/Utils.h
  core/include/Settings.h
  core/include/GeoSemHelpers.h
  core/include/Semantic/Room.h
  core/include/Semantic/Floor.h
  core/include/DatabaseParser.h
  core/include/Semantic/Marker.h
  core/include/Geometric/Plane.h
  core/include/Semantic/Passage.h
  core/include/Types/SystemParams.h
  core/include/SemanticSegmentation.h
  core/include/SemanticsManager.h
)

target_link_libraries(${PROJECT_NAME}_lib 
  ${OpenCV_LIBS}
  ${EIGEN3_LIBS}
  ${Pangolin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${PROJECT_SOURCE_DIR}/core/Thirdparty/DBoW2/lib/libDBoW2.so
  ${PROJECT_SOURCE_DIR}/core/Thirdparty/g2o/lib/libg2o.so
  -lboost_system
  -lboost_serialization
  -lcrypto
)

# ROS2 specific: Use ament_target_dependencies for ROS2 packages
ament_target_dependencies(${PROJECT_NAME}_lib 
  rclcpp
  std_msgs
  cv_bridge
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  pcl_ros
  nav_msgs
  message_filters
  image_transport
  rviz_visual_tools
  situational_graphs_msgs
)

# Common source files
set(COMMON_SOURCES
  src/common.cc
)

# Common dependencies
set(COMMON_DEPENDENCIES
  tf2
  rclcpp
  tf2_ros
  nav_msgs
  std_msgs
  cv_bridge
  sensor_msgs
  geometry_msgs
  segmenter_ros
  message_filters
  situational_graphs_msgs
)

# ROS2 nodes
function(add_ros2_node target_name cpp_file)
  add_executable(${target_name}
    ${COMMON_SOURCES}
    ${cpp_file}
  )
  target_link_libraries(${target_name}
    ${PROJECT_NAME}_lib
  )
  ament_target_dependencies(${target_name}
    ${COMMON_DEPENDENCIES}
  )
  rosidl_target_interfaces(${target_name} ${PROJECT_NAME} "rosidl_typesupport_cpp")

  install(TARGETS ${target_name}
    DESTINATION lib/${PROJECT_NAME}
  )
endfunction()

# ROS2 nodes
add_ros2_node(ros_rgbd          src/ros_rgbd.cc)
add_ros2_node(ros_rgbd_inertial src/ros_rgbd_inertial.cc)

# Install other resources
install(
  DIRECTORY launch config core/Vocabulary
  DESTINATION share/${PROJECT_NAME}
)

# Install the library
install(TARGETS ${PROJECT_NAME}_lib
  DESTINATION lib/${PROJECT_NAME}
)

# Ament package configuration
ament_package()