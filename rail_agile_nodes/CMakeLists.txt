cmake_minimum_required(VERSION 2.8.3)
project(rail_agile_nodes)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  agile_grasp_base
  cmake_modules
  eigen_conversions
  geometry_msgs
  interactive_markers
  pcl_conversions
  pcl_ros
  rail_agile_grasp_msgs
  rail_manipulation_msgs
  remote_manipulation_markers
  roscpp
  std_srvs
  tf_conversions
  visualization_msgs
)

find_package(Eigen REQUIRED)

################################################
## Declare ROS messages, services and actions ##
################################################

###################################################
## Declare things to be passed to other projects ##
###################################################

## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package()

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
)

add_executable(point_cloud_filter
  src/PointCloudFilter.cpp
)
target_link_libraries(point_cloud_filter
  ${catkin_LIBRARIES}
)

add_executable(grasp_sampler
  src/GraspSampler.cpp src/PoseWithHeuristic.cpp
)
add_dependencies(grasp_sampler
  agile_grasp_base_gencpp
  rail_manipulation_msgs_gencpp
)
target_link_libraries(grasp_sampler
  ${catkin_LIBRARIES}
  ${EIGEN_INCLUDE_DIRS}
)

add_executable(point_cloud_interleaver
  src/PointCloudInterleaver.cpp
  )
target_link_libraries(point_cloud_interleaver
  ${catkin_LIBRARIES}
)

add_executable(point_cloud_clicker
  src/PointCloudClicker.cpp
)
add_dependencies(point_cloud_clicker
  rail_agile_grasp_msgs_gencpp
)
target_link_libraries(point_cloud_clicker
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS point_cloud_filter
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
