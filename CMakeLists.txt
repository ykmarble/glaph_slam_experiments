cmake_minimum_required(VERSION 2.8.3)
project(ptx)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake")

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

set (CATKIN_DEPS
  roscpp
  roslib
  tf
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
  tf2_sensor_msgs
  pluginlib
  nodelet
  pcl_ros
  pcl_conversions
  laser_geometry
  fast_gicp

  std_msgs
  std_srvs
  geometry_msgs
  nav_msgs
)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS ${CATKIN_DEPS})

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

find_package(Eigen3 REQUIRED)
find_package(G2O REQUIRED)
find_package(PCL REQUIRED COMPONENTS common features filters io segmentation surface)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
   INCLUDE_DIRS include
   CATKIN_DEPENDS ${CATKIN_DEPS}
   DEPENDS Boost EIGEN3 PCL
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${G2O_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

# Declare a C++ library
add_library(graph_slam src/graph_slam.cpp)
target_link_libraries(graph_slam
  ${PCL_LIBRARIES}
  ${G2O_TYPES_DATA}
  ${G2O_CORE_LIBRARY}
  ${G2O_STUFF_LIBRARY}
  ${G2O_SOLVER_PCG}
  ${G2O_SOLVER_CSPARSE}   # be aware of that CSPARSE is released under LGPL
  ${G2O_SOLVER_CHOLMOD}   # be aware of that cholmod is released under GPL
  ${G2O_TYPES_SLAM2D}
)

add_library(graph_slam_ros_nodelet
  src/graph_slam_ros_nodelet.cpp
  src/graph_slam_ros.cpp
)
add_dependencies(graph_slam_ros_nodelet ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(graph_slam_ros_nodelet
  ${catkin_LIBRARIES}
  graph_slam
)

## Declare a C++ executable
add_executable(graph_slam_ros_node
  src/graph_slam_ros_node.cpp
  src/graph_slam_ros.cpp
)
add_dependencies(graph_slam_ros_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(graph_slam_ros_node
  ${catkin_LIBRARIES}
  graph_slam
)

add_library(registration_factory
  src/registration_factory.cpp
  src/registration_factories/fast_gicp.cpp
)
add_dependencies(registration_factory ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(registration_factory
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_library(points_insert_criteria_factory
  src/points_insert_criteria_factory.cpp
  src/points_insert_criteria_factories/manual_trigger.cpp
  src/points_insert_criteria_factories/elapsed_time.cpp
  src/points_insert_criteria_factories/traveled_distance.cpp
)
add_dependencies(points_insert_criteria_factory ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(points_insert_criteria_factory
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_executable(lidar_odometry
  src/lidar_odometry.cpp
  src/queued_points_map.cpp
)
add_dependencies(lidar_odometry ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(lidar_odometry
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  registration_factory
  points_insert_criteria_factory
)

add_executable(scan2cloud_node
  src/scan2cloud_node.cpp
)
add_dependencies(scan2cloud_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(scan2cloud_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_executable(pcd_registration
  src/pcd_registration.cpp
)
add_dependencies(pcd_registration ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pcd_registration
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(TARGETS
  lidar_odometry
  scan2cloud_node
  graph_slam
  graph_slam_ros_nodelet
  graph_slam_ros_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

# Mark other files (FILES) and directories (DIRECTORY) for installation (e.g. launch and bag files, etc.)
install(DIRECTORY
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES
  nodelet.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

# Add gtest based cpp test target and link libraries
set(BUILD_TEST 0)
if(BUILD_TEST)
#  catkin_add_executable_with_gtest(test test/test.cpp)
#  target_link_libraries(test
#    ${catkin_LIBRARIES}
#    ${Boost_LIBRARIES}
#  )
endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
