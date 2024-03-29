cmake_minimum_required(VERSION 3.1)
project(robot_collision_checking)

set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  roslib
  eigen_conversions
  geometric_shapes
  octomap_msgs
  costmap_2d
)

set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake/")

find_package(CCD REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(octomap REQUIRED)

if(EXISTS "${CMAKE_SOURCE_DIR}/cmake/FindFCL.cmake")
  find_package(FCL REQUIRED)
else()
  set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/${PROJECT_NAME}/cmake/")
  find_package(FCL REQUIRED)
endif()

find_library(EIGEN_CDDLIB_LIB eigen-cddlib)

###########
## Build ##
###########
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES fcl_interface
  CATKIN_DEPENDS roscpp rospy roslib std_msgs shape_msgs geometry_msgs geometric_shapes octomap_msgs costmap_2d
  DEPENDS Eigen3 CCD FCL
)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "-std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "-std=c++0x")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler. Suggested solution: update the pkg build-essential ")
endif()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
  ${FCL_INCLUDE_DIRS}
  ${CCD_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
)

add_library(fcl_interface
  src/fcl_interface.cpp
  src/static_fcl_interface.cpp
  include/${PROJECT_NAME}/fcl_interface.h
)
add_dependencies(fcl_interface
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(fcl_interface 
  ${catkin_LIBRARIES}
  ${Eigen3_LIBRARIES}
  ${FCL_LIBRARIES}
  ${CCD_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
)

add_executable(test_file tests/test_file.cpp)
add_dependencies(test_file ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_file 
  fcl_interface
)

add_executable(test_inter_fcl tests/test_inter_fcl.cpp)
add_dependencies(test_inter_fcl ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_inter_fcl 
  fcl_interface
)