cmake_minimum_required(VERSION 2.8.3)
project(pose_utils)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES pose_utils
)

find_package(Armadillo REQUIRED)

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${ARMADILLO_INCLUDE_DIRS}
    include
    )

add_library(pose_utils
   ${ARMADILLO_LIBRARIES}
   src/pose_utils.cpp)
