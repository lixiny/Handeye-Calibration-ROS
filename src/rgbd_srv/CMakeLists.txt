cmake_minimum_required(VERSION 2.8.3)
project(rgbd_srv)


find_package(catkin REQUIRED COMPONENTS
    image_transport
    cv_bridge
    roscpp
    rospy
    sensor_msgs
    std_msgs
    message_generation
    tf
)
find_package(OpenCV 3 REQUIRED )
find_package(PCL 1.7 REQUIRED)

# Generate services in the 'srv' folder
add_service_files(
    FILES
    rgbd.srv
)


# Generate added messages and services with any dependencies listed here
generate_messages(
    DEPENDENCIES
    sensor_msgs
    std_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES rgb_depth
#  CATKIN_DEPENDS roscpp rospy sensor_msgs std_msgs
  CATKIN_DEPENDS message_runtime
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


