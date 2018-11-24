cmake_minimum_required(VERSION 2.8.3)
project(kinect_driver)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}  -std=c++0x")

############### dependencies ######################
# 添加cmake模块
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}")
#PCL
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})
#Opencv
find_package(OpenCV REQUIRED )
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIR})
#ros
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  rospy
  sensor_msgs
  std_msgs
  rgbd_srv
)
catkin_package()
include_directories(${catkin_INCLUDE_DIRS})
#self file
include_directories(../../devel/include)
include_directories(include/kinect_driver)
# include_directories(include)
include_directories( ${PROJECT_SOURCE_DIR}/include )
############### execute ######################
# file(GLOB srcs "${PROJECT_SOURCE_DIR}/src/*.cpp")
set(srcs src/main.cpp src/OpenNI2Interface.cpp)
add_executable(kinect_driver
               ${srcs}
)
add_executable(kinect_server
               src/kinect_server.cpp
)

add_dependencies(kinect_server rgbd_srv_gencpp)

target_link_libraries(kinect_driver
		      ${catkin_LIBRARIES}
          ${OPENNI2_LIBRARY}
 		      ${OpenCV_LIBS}
		      ${PCL_LIBRARIES}
)
target_link_libraries(kinect_server
          ${catkin_LIBRARIES}
          ${OPENNI2_LIBRARY}
          ${OpenCV_LIBS}
          ${PCL_LIBRARIES}
)

