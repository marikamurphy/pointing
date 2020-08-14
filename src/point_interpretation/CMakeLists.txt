cmake_minimum_required(VERSION 3.0)  # CMake version check

project(point_interpretation)               # Create project "simple_example"

set(CMAKE_CXX_STANDARD 14)            # Enable c++14 standard
 
find_package (
Eigen3 3.3 REQUIRED)
find_package(
    OpenCV
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
    src/Image.cpp
)

target_link_libraries (point_interpretation ${OpenCV_LIBS} Eigen3::Eigen    )
    
