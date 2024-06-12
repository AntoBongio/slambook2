cmake_minimum_required(VERSION 3.8)
project(vo1)

set(CMAKE_BUILD_TYPE "Release")
add_definitions("-DENABLE_SSE")
set(CMAKE_CXX_FLAGS "-std=c++17 -O2 ${SSE_FLAGS} -msse4")
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

find_package(OpenCV 4 REQUIRED)
find_package(G2O REQUIRED)
find_package(Sophus REQUIRED)
find_package(Pangolin REQUIRED)


# fmt
find_package(fmt REQUIRED)
set(FMT_LIBRARIES fmt::fmt)

include_directories(
        ${OpenCV_INCLUDE_DIRS}
        ${G2O_INCLUDE_DIRS}
        ${Sophus_INCLUDE_DIRS}
        "/usr/include/eigen3/"
)

add_executable(orb_cv orb_cv.cpp)
target_link_libraries(orb_cv ${OpenCV_LIBS} ${FMT_LIBRARIES})

add_executable(orb_self orb_self.cpp)
target_link_libraries(orb_self ${OpenCV_LIBS} ${FMT_LIBRARIES})

# add_executable( pose_estimation_2d2d pose_estimation_2d2d.cpp extra.cpp ) # use this if in OpenCV2 
add_executable(pose_estimation_2d2d pose_estimation_2d2d.cpp)
target_link_libraries(pose_estimation_2d2d ${OpenCV_LIBS} ${FMT_LIBRARIES})

# # add_executable( triangulation triangulation.cpp extra.cpp) # use this if in opencv2
add_executable(triangulation triangulation.cpp)
target_link_libraries(triangulation ${OpenCV_LIBS} ${FMT_LIBRARIES})

add_executable(pose_estimation_3d2d pose_estimation_3d2d.cpp)
target_link_libraries(pose_estimation_3d2d
        g2o_core g2o_stuff
        ${OpenCV_LIBS}
        ${FMT_LIBRARIES})

add_executable(pose_estimation_3d3d pose_estimation_3d3d.cpp)
target_link_libraries(pose_estimation_3d3d
        g2o_core g2o_stuff
        ${OpenCV_LIBS}
        ${FMT_LIBRARIES})

add_executable(slam_2d2d slam_2d2d.cpp)
target_link_libraries(slam_2d2d
        g2o_core g2o_stuff
        ${OpenCV_LIBS}
        ${Pangolin_LIBRARIES}
        ${FMT_LIBRARIES})
