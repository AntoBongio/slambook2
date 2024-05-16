cmake_minimum_required(VERSION 3.8)

set(CMAKE_BUILD_TYPE "Release")
# 添加c++ 11标准支持
set(CMAKE_CXX_FLAGS "-std=c++17 -O2")

# Eigen
include_directories("/usr/include/eigen3")

find_package(fmt REQUIRED)
set(FMT_LIBRARIES fmt::fmt)

find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS} ${FMT_LIBRARIES})

add_subdirectory(imageBasics)
add_subdirectory(stereo)
add_subdirectory(rgbd)
