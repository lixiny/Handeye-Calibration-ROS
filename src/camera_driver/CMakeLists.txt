cmake_minimum_required(VERSION 2.8)
project(camera_driver)

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

############### dependencies ######################
# 添加cmake模块
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}")

#PCL
find_package(PCL REQUIRED)
add_definitions(${PCL_DEFINITIONS})
#Opencv
SET(OpenCV_DIR /usr/local/share/OpenCV)
find_package(OpenCV 3 REQUIRED)
# cv_bridge
# SET(cv_bridge_DIR /usr/local/share/cv_bridge)
# find_package(cv_bridge REQUIRED )
#realsense
find_package(realsense2 REQUIRED)
#ros
find_package(catkin REQUIRED COMPONENTS
    cv_bridge
    image_transport
    message_generation
    roscpp
    rospy
    sensor_msgs
    std_msgs
    tf
    rgbd_srv
)
catkin_package()


############### Include path ######################
include_directories(
    ${realsense2_INCLUDE_DIR}
    ${OpenCV_INCLUDE_DIRS}
    # ${cv_bridge_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
)

## Link 3rdparty directory
link_directories(${OpenCV_LIBRARY_DIR})
link_directories(${PCL_LIBRARY_DIRS})

############### executable ######################
add_executable(realsense2_driver src/realsense2_driver.cpp)
add_executable(realsense2_server src/realsense2_server.cpp)

add_dependencies(realsense2_driver rgbd_srv_gencpp)
add_dependencies(realsense2_server rgbd_srv_gencpp)

target_link_libraries(realsense2_driver
   ${OpenCV_LIBS}
   ${PCL_LIBRARIES}
   ${realsense2_LIBRARY}  
#    ${cv_bridge_LIBRARIES}
   ${catkin_LIBRARIES}
   
)
target_link_libraries(realsense2_server
   ${OpenCV_LIBS}
   ${PCL_LIBRARIES}
#    ${cv_bridge_LIBRARIES}
   ${catkin_LIBRARIES}
)
