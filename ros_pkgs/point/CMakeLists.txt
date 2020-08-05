cmake_minimum_required(VERSION 2.8.3)
project(point)

## Compile as C++11, supported in ROS Kinetic and newer
#add_compile_options(-std=c++11 -g)
add_compile_options(-std=c++11)

#set(OpenPose_DIR /home/users/ljernigan/openpose)
#set(OpenPose_INCLUDE_DIRS ${OpenPose_DIR}/include)
#set(OpenPose_LINK_LIBRARY ${OpenPose_DIR}/build/src/openpose/libopenpose.so)
set (EIGEN3_INCLUDE_DIR = /usr/include/eigen3)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules")
#list(APPEND CMAKE_INCLUDE_PATH "/usr/include/eigen3")
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  cv_bridge
  tf2
  tf2_ros
  
  rospy
  geometry_msgs
  image_transport
  sensor_msgs
  visualization_msgs
)

find_package(
  OpenCV
  #Eigen3 REQUIRED
)
#find_package(
  #OpenPose REQUIRED
  
#)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES point
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  #${OpenPose_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)


## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/point.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


add_executable(${PROJECT_NAME}_node
  src/ProjectiveUtil.cpp
  src/point_identification_node.cpp
  src/SubscribeToKinect.cpp
  src/TFBroadcastPR.cpp
  src/Client.cpp
)

 target_link_libraries(${PROJECT_NAME}_node
    ${OpenCV_LIBS}   
    ${catkin_LIBRARIES}
    #${OpenPose_LINK_LIBRARY}
 )