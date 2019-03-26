cmake_minimum_required(VERSION 2.8)
#add_compile_options(-std=c++11)
project(handeye_calib_sphere)

set( CMAKE_BUILD_TYPE "Release" )
set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )

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


find_package(OpenMP REQUIRED)
if(OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

# 添加cmake模块以使用ceres库
list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules/ )

find_package(catkin REQUIRED COMPONENTS
    image_transport
    cv_bridge
    roscpp
    rospy
    sensor_msgs
    std_msgs
    message_generation
    tf
    rgbd_srv
    )


# MUST use OpenCV in ROS !
# the original OpenCV at /usr/local/share/OpenCV do not include the Aruco modules 
# this line shoudld be default if you have successfully installed ROS, and sourced your workspace.
SET(OpenCV_DIR /opt/ros/kinetic/share/OpenCV-3.3.1-dev)


find_package(OpenCV 3 REQUIRED )
find_package(PCL 1.7 REQUIRED)
find_package(Sophus REQUIRED)
find_package(Ceres REQUIRED )


# Generate added messages and services with any dependencies listed here
#generate_messages(
#    DEPENDENCIES
#    sensor_msgs
#    std_msgs
#    )

catkin_package(
    #  INCLUDE_DIRS include
    #  LIBRARIES rgbd_stremmer
    #  CATKIN_DEPENDS roscpp rospy sensor_msgs std_msgs
    #  CATKIN_DEPENDS message_runtime
    #  DEPENDS system_lib
    )

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
    # include
    ${catkin_INCLUDE_DIRS}
    
    )


include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
include_directories(${Sophus_INCLUDE_DIRS})
include_directories(${CERES_INCLUDE_DIRS} )
include_directories( ${PROJECT_SOURCE_DIR}/include )


## Link 3rdparty directory
link_directories(${OpenCV_LIBRARY_DIR})
link_directories(${PCL_LIBRARY_DIRS})


#add_definitions(${PCL_DEFINITIONS})


add_executable(grab_data_node
    src/grab_data_node.cpp
    src/CalibrationTool.cpp
    src/ArucoPlane.cpp)

add_dependencies(grab_data_node rgbd_srv_gencpp)


target_link_libraries(grab_data_node
	${catkin_LIBRARIES}
	${OpenCV_LIBS}
	${PCL_LIBRARIES}
	${Sophus_LIBRARIES}
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
	)


add_executable(grab_tf_node
    src/grab_tf_node.cpp)

target_link_libraries(grab_tf_node
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
    )




################### test node ######################
	
add_executable(generate_fake_data
    test/generate_fake_data.cpp)

target_link_libraries(generate_fake_data
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
    ${Sophus_LIBRARIES}
        ${CERES_LIBRARIES}
    )


add_executable(generate_fake_data_v2
    test/generate_fake_data_v2.cpp)

target_link_libraries(generate_fake_data_v2
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
    ${Sophus_LIBRARIES}
        ${CERES_LIBRARIES}
    )

add_executable(generate_fake_data_v3
    test/generate_fake_data_v3.cpp)

target_link_libraries(generate_fake_data_v3
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
    ${Sophus_LIBRARIES}
        ${CERES_LIBRARIES}
    )


add_executable(fit_sphere
    test/fit_sphere.cpp
    src/CalibrationTool.cpp)

target_link_libraries(fit_sphere
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
    ${Sophus_LIBRARIES}
    ${CERES_LIBRARIES}
    ${PCL_LIBRARIES}
    )


############### validation node ###################

#add_executable(validation_node
#    src/validation.cpp
#    src/calibration_tool.cpp
#    )

#target_link_libraries(validation_node
#	${catkin_LIBRARIES}
#	${OpenCV_LIBS}
#	${PCL_LIBRARIES}
#	${Sophus_LIBRARIES}
#	/usr/local/lib/librealsense2.so

#	)

#add_executable(validation_offline_node
#    src/validation_offline.cpp
#    src/calibration_tool.cpp
#    )

#target_link_libraries(validation_offline_node
#	${catkin_LIBRARIES}
#	${OpenCV_LIBS}
#	${PCL_LIBRARIES}
#	${Sophus_LIBRARIES}

#	)

#add_executable(validation_offline2_node
#    src/validation_offline2.cpp
#    src/calibration_tool.cpp
#    )

#target_link_libraries(validation_offline2_node
#	${catkin_LIBRARIES}
#	${OpenCV_LIBS}
#	${PCL_LIBRARIES}
#	${Sophus_LIBRARIES}

#	)

