cmake_minimum_required(VERSION 2.8)
# add_compile_options(-std=c++11)
project(handeye_calib_sphere)


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
# 添加cmake模块以使用ceres库
list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules/ )

# PCL
find_package(PCL 1.7 REQUIRED)
add_definitions(${PCL_DEFINITIONS})
# Opencv
SET(OpenCV_DIR /usr/local/share/OpenCV)
find_package(OpenCV 3 REQUIRED)
# # cv_bridge
SET(cv_bridge_DIR /usr/local/share/cv_bridge)
find_package(cv_bridge REQUIRED )
# ceres-solver
find_package(Ceres REQUIRED )
# sophus
find_package(Sophus REQUIRED)
# ros
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

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
    # include
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    # ${cv_bridge_INCLUDE_DIRS}
    ${CERES_INCLUDE_DIRS}
    ${Sophus_INCLUDE_DIRS}
    ${PROJECT_SOURCE_DIR}/include 
)


## Link 3rdparty directory
link_directories(${OpenCV_LIBRARY_DIR})
link_directories(${PCL_LIBRARY_DIRS})



############### executable ######################

add_executable(grab_data_node
    src/grab_data_node.cpp
    src/CalibrationTool.cpp
    src/ArucoPlane.cpp)

add_dependencies(grab_data_node rgbd_srv_gencpp)


target_link_libraries(grab_data_node
	${OpenCV_LIBS}
	${PCL_LIBRARIES}
    ${Sophus_LIBRARIES}
    # ${cv_bridge_LIBRARIES}
    ${catkin_LIBRARIES}
)


add_executable(eyeinhand_calib_node
    src/eyeinhand_calib_node.cpp
    src/CalibrationTool.cpp)

target_link_libraries(eyeinhand_calib_node
	${catkin_LIBRARIES}
	${OpenCV_LIBS}
	${PCL_LIBRARIES}
	${CERES_LIBRARIES}
    ${Sophus_LIBRARIES}
    # ${cv_bridge_LIBRARIES}
)


add_executable(grab_tf_node
    src/grab_tf_node.cpp)

target_link_libraries(grab_tf_node
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
)


