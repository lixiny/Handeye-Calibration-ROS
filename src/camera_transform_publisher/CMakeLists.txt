cmake_minimum_required(VERSION 2.8.3)
project(camera_transform_publisher)


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


## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

############### dependencies ######################
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

# opencv
SET(OpenCV_DIR /usr/local/share/OpenCV)
find_package(OpenCV 3 REQUIRED )
# cv_bridge
SET(cv_bridge_DIR /usr/local/share/cv_bridge)
find_package(cv_bridge REQUIRED )
# eigen
find_package(Eigen3 REQUIRED)
# sophus
find_package(Sophus REQUIRED)

#ros
find_package(catkin REQUIRED COMPONENTS
    cv_bridge
    image_transport
    message_generation
    roscpp
    sensor_msgs
    std_msgs
    tf
)
catkin_package()

############### Include path ######################
include_directories(
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${OpenCV_INCLUDE_DIRS}
    ${cv_bridge_INCLUDE_DIRS}
    ${Sophus_INCLUDE_DIRS}
    ${PROJECT_SOURCE_DIR}/include 
)
link_directories(${OpenCV_LIBRARY_DIR})


############### executable ######################
add_executable(chessboard_publisher
    src/chessboard_publisher.cpp
    src/aruco_plane.cpp)

add_executable(aruco_publisher
    src/aruco_publisher.cpp
    src/aruco_plane.cpp
)



target_link_libraries(chessboard_publisher
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
    ${EIGEN3_LIBS}
    ${cv_bridge_LIBRARIES}
    ${Sophus_LIBRARIES}
)

target_link_libraries(aruco_publisher
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
    ${EIGEN3_LIBS}
    ${cv_bridge_LIBRARIES}
    ${Sophus_LIBRARIES}
)

