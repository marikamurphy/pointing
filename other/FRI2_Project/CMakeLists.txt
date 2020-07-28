cmake_minimum_required(VERSION 2.8.3)
project(fri2_pkg)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  geometry_msgs
  std_msgs
  tf
  cv_bridge
  image_transport
  sensor_msgs
  openpose_ros_msgs
  darknet_ros_msgs
  visualization_msgs
)
find_package(
  OpenCV
)
#find_package(
#Eigen REQUIRED
#)


## Generate messages in the 'msg' folder
# add_message_files()

## Generate services in the 'srv' folder
# add_service_files()

## Generate actions in the 'action' folder
# add-action_files)

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
	#  INCLUDE_DIRS include
#  CATKIN_DEPENDS roscpp std_msgs tf
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  #${EIGEN_INCLUDE_DIRS}
  /usr/include/eigen3/
)

## Declare a C++ Executable
add_executable(pointing_main
  src/pointing_main.cpp
  src/ImageProcessor.cpp
  src/Storage.cpp
  src/MakeLine.cpp
  src/Mapping.cpp
  src/point_math.cpp
#  others
)

## Specify libraries to link a library or executable target against
target_link_libraries(pointing_main
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
)

#target_link_libraries(Mapping
#	${catkin_LIBRARIES}
#)

#install(FILES
#  launch/kinect
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#)
