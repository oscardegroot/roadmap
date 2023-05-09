cmake_minimum_required(VERSION 3.0.2)
project(roadmap)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-fstack-protector)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wno-deprecated-declarations")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roslib
  roscpp
  std_msgs
  asr_rapidxml
  roadmap_msgs
  ros_tools
)

## System dependencies are found with CMake's conventions
include_directories(
  include/roadmap 
  include/spline
  )

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
  CATKIN_DEPENDS geometry_msgs roslib roscpp std_msgs asr_rapidxml roadmap_msgs ros_tools
  INCLUDE_DIRS include include/${PROJECT_NAME} include/spline
  LIBRARIES roadmap
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
	include
	include/roadmap
	include/spline
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(roadmap
   src/roadmap.cpp
   src/configuration.cpp
   src/reader.cpp
   src/spline_converter.cpp
   src/spline/spline.cpp
   src/spline/Clothoid.cpp
)
add_dependencies(roadmap ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


add_executable(roadmap_node src/roadmap_node.cpp)
target_link_libraries(roadmap_node roadmap ${catkin_LIBRARIES})
add_dependencies(roadmap_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

#############
## Install ##
#############


# install(FILES "${PROJECT_NAME}Config.cmake" "${PROJECT_NAME}ConfigVersion.cmake"
#   DESTINATION "share/${PROJECT_NAME}/cmake")