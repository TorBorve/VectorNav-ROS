cmake_minimum_required(VERSION 2.8.3)
project(vectornav)
# Compile options so compiles with ROS Kinetic
# Future releases may want to 
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    roscpp
    tf2 
    tf2_ros
    message_generation
)

add_message_files(
    FILES
    Ins.msg
    InsStatus.msg
)

generate_messages(
    DEPENDENCIES
    std_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES vectornav
   CATKIN_DEPENDS roscpp sensor_msgs geometry_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

#include(vnproglib-1.1/cpp/CMakeLists.txt)
add_subdirectory(vnproglib-1.2.0.0/cpp)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(vnproglib-1.2.0.0/cpp/include 
    include
    ${catkin_INCLUDE_DIRS})

## Declare a cpp library
## Declare a cpp executable
add_executable(vnpub src/main.cpp)
add_dependencies(vnpub ${PROJECT_NAME}_generate_messages)

## Specify libraries to link a library or executable target against
target_link_libraries(vnpub
  libvncxx
  ${catkin_LIBRARIES}
)

## Mark executables and/or libraries for installation
install(TARGETS vnpub
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Declare vn300 executable
add_executable(vn300 src/vn300.cpp src/VnRos.cpp src/utilities.cpp)
add_dependencies(vn300 ${PROJECT_NAME}_generate_messages)
target_link_libraries(vn300 libvncxx ${catkin_LIBRARIES})