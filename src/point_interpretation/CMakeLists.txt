cmake_minimum_required(VERSION 3.0)  # CMake version check

# Set the project name
project(point_interpretation)               # Create project "simple_example"

set(CMAKE_CXX_STANDARD 14)            # Enable c++14 standard
 
# Load settings for an external project
find_package (
Eigen3 3.3 REQUIRED)
find_package(
    OpenCV
)
find_package(
  roscpp
  #message_filters
  #std_msgs
  # cv_bridge
  # tf2
  # tf2_ros
  
  # rospy
  # geometry_msgs
  # image_transport
  #sensor_msgs
  # visualization_msgs
)

include_directories(
  include
  ${OpenCV_LIBS}
)

# Add executable target with source files listed in SOURCE_FILES variable
add_executable(
    point_interpretation 
    src/point_interpretation.cpp
    src/Client.cpp
    src/SubscribeToKinect.cpp
)

target_link_libraries (point_interpretation ${OpenCV_LIBS} Eigen3::Eigen    )
    
