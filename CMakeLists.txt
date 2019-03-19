cmake_minimum_required(VERSION 2.8.3)
project(robot_collision_checking)



set(MSG_DEPS
  std_msgs
  shape_msgs
  geometry_msgs
)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  roslib
  eigen_conversions
  geometric_shapes
  )


set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake/")

find_package(CCD REQUIRED)
find_package(Eigen3 REQUIRED)

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
   CATKIN_DEPENDS roscpp rospy roslib   ${MSG_DEPS}  geometric_shapes
   DEPENDS
  EIGEN3
  CCD
  FCL
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
  /usr/include/eigen3/
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${PCL_LIBRARY_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${FCL_INCLUDE_DIRS}
  ${CCD_INCLUDE_DIRS}
)

add_definitions(${PCL_DEFINITIONS})

add_library(fcl_interface
   src/fcl_interface.cpp
   include/${PROJECT_NAME}/fcl_interface.h
)
add_executable(testing ./src/test_file.cpp)
add_executable(test_fcl_inter ./src/fcl_interface.cpp)

target_link_libraries(testing ${catkin_LIBRARIES}
                      ${FCL_LIBRARIES}
                       ${CCD_LIBRARIES})
target_link_libraries(test_fcl_inter
                        ${catkin_LIBRARIES}
                        ${FCL_LIBRARIES}
                        ${CCD_LIBRARIES})

add_dependencies(fcl_interface
		  ${${PROJECT_NAME}_EXPORTED_TARGETS}
		  ${catkin_EXPORTED_TARGETS}
		  #${FCL_LIBRARIES}
		  ${CCD_LIBRARIES})
