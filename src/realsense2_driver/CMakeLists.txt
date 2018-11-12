cmake_minimum_required(VERSION 2.8.3)
project(realsense2_driver)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
    message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport
  tf
  rgbd_srv
)

# MUST use OpenCV in ROS !
# the original OpenCV at /usr/local/share/OpenCV do not include the Aruco modules 
# this line shoudld be default if you have successfully installed ROS, and sourced your workspace.
SET(OpenCV_DIR /opt/ros/kinetic/share/OpenCV-3.3.1-dev)
SET(realsense2_DIR /usr/lib/x86_64-linux-gnu/cmake/realsense2)


find_package(OpenCV 3 REQUIRED)
find_package(realsense2 REQUIRED)
find_package(PCL REQUIRED)

message(STATUS "########################  OpenCV library status ###############################" )
message(STATUS "                     version: ${OpenCV_VERSION}")
message(STATUS "                     include path: ${OpenCV_INCLUDE_DIRS}")
message(STATUS "                     libraries: ${OpenCV_LIBS}")

message(STATUS "########################  realsense2 library status ###############################" )
message(STATUS "                     version: ${realsense_VERSION_MAJOR}")
message(STATUS "                     include path: ${realsense_INCLUDE_DIR}")
message(STATUS "                     libraries: ${realsense2_LIBRARY}")



################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )



###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES realsense_driver
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
include_directories(${realsense_INCLUDE_DIR})
#include_directories("/usr/include/librealsense2/")



link_directories(${OpenCV_LIBRARY_DIR})
link_directories(${PCL_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})

add_executable(realsense2_driver src/realsense2_driver.cpp)
add_executable(realsense2_server src/realsense2_server.cpp)
#add_executable(grab_pointcloud_demo src/grab_pointcloud_demo.cpp)

add_dependencies(realsense2_server rgbd_srv_gencpp)

target_link_libraries(realsense2_driver
   ${catkin_LIBRARIES}
   ${OpenCV_LIBS}
   ${PCL_LIBRARIES}
   ${realsense2_LIBRARY}  # /usr/lib/x86_64-linux-gnu/librealsense2.so
   
)
target_link_libraries(realsense2_server
   ${catkin_LIBRARIES}
   ${OpenCV_LIBS}
   ${PCL_LIBRARIES}
)
