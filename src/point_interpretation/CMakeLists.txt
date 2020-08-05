cmake_minimum_required(VERSION 3.0)  # CMake version check

project(point_interpretation)               # Create project "simple_example"

set(CMAKE_CXX_STANDARD 14)            # Enable c++14 standard
 
find_package (Eigen3 3.3 REQUIRED)

# Add executable target with source files listed in SOURCE_FILES variable
add_executable(
    point_interpretation 
    point_interpretation.cpp
)

target_link_libraries (point_interpretation Eigen3::Eigen opencv_core opencv_highgui opencv_imgproc)
