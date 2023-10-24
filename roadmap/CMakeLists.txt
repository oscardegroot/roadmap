cmake_minimum_required(VERSION 3.5)
project(roadmap)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-parameter)
endif()
add_compile_options(-DCMAKE_BUILD_TYPE=Release)

## DEPENDENCIES ##
set(DEPENDENCIES
rclcpp
geometry_msgs
std_msgs
asr_rapidxml
roadmap_msgs
ros_tools
)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

find_package(ros_tools REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(roadmap_msgs REQUIRED)

find_package(asr_rapidxml REQUIRED)
find_package(ros_tools REQUIRED)

# find_package(eigen3_cmake_module REQUIRED)
# find_package(Eigen3 REQUIRED)
# add_definitions(${EIGEN_DEFINITIONS})

###########
## Build ##
###########

include_directories(
  include/${PROJECT_NAME}
  include/spline
)

## Declare a C++ library
add_library(${PROJECT_NAME}
   src/roadmap.cpp
   src/configuration.cpp
   src/reader.cpp
   src/spline_converter.cpp
   src/spline/spline.cpp
   src/spline/Clothoid.cpp
)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
ament_target_dependencies(${PROJECT_NAME}
  ${DEPENDENCIES}
)

add_executable(${PROJECT_NAME}_node src/roadmap_node.cpp)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME})

## INSTALL ##
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/ 
  DESTINATION include/${PROJECT_NAME})

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY maps
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME})

# ament_export_include_directories(include/${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${DEPENDENCIES})

ament_package()