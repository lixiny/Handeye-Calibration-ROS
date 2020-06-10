cmake_minimum_required(VERSION 2.8.3)
project(handeye_calib_marker)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/")

# Enable C++11
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
        message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support, or our tests failed to detect it correctly. Please use a different C++ compiler or report this problem to the developers.")
endif()


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  tf_conversions
  roscpp
  tf
  trajectory_msgs
  #vrep_common
)
find_package(Gflags REQUIRED)
find_package(Glog REQUIRED)

message("Gflags found = ${GFLAGS_FOUND}")
message("Glog found = ${GLOG_FOUND}")
SET(Gflags_FOUND ${GFLAGS_FOUND})
SET(Glog_FOUND ${GLOG_FOUND})

find_package(Threads QUIET)
find_package(Ceres QUIET REQUIRED)
SET(GFLAGS_LIBRARY ${CMAKE_THREAD_LIBS_INIT})
## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


SET(OpenCV_DIR /usr/local/share/OpenCV)
find_package(OpenCV 3 REQUIRED)

if(OpenCV_FOUND)
	message(STATUS "OpenCV version: ${OpenCV_VERSION}")
    if(NOT OpenCV_VERSION VERSION_LESS "3.0.0")
        add_definitions(-DHAVE_OPENCV3)
		message(STATUS "defined HAVE_OPENCV3")

        set(CMAKE_REQUIRED_INCLUDES ${CMAKE_REQUIRED_INCLUDES} ${OPENCV_INCLUDE_DIRS})
        include(CheckIncludeFileCXX)

        check_include_file_cxx(opencv2/face/facerec.hpp HAVE_OPENCV_CONTRIB)
        if(HAVE_OPENCV_CONTRIB)
            add_definitions(-DHAVE_OPENCV_CONTRIB)
        elseif()
            message(STATUS "OPENCV_CONTRIB NOT PRESENT, DISABLING LOTS OF FUNCTIONALITY, SEE https://github.com/opencv/opencv_contrib")
        endif()

        check_include_file_cxx(opencv2/xfeatures2d/nonfree.hpp HAVE_OPENCV_XFEATURES2D_NONFREE)
        if(HAVE_OPENCV_XFEATURES2D_NONFREE)
            add_definitions(-DHAVE_OPENCV_XFEATURES2D_NONFREE)
        elseif()
            message(STATUS "OPENCV_XFEATURES2D_NONFREE NOT PRESENT, DISABLING LOTS OF FUNCTIONALITY, SEE https://github.com/opencv/opencv_contrib")
        endif()

        check_include_file_cxx(opencv2/cudafeatures2d.hpp HAVE_OPENCV_CUDAFEATURES2D)
        if(HAVE_OPENCV_CUDAFEATURES2D)
            add_definitions(-DHAVE_OPENCV_CUDAFEATURES2D)
        elseif()
            message(STATUS "OPENCV_CUDAFEATURES2D NOT PRESENT, DISABLING LOTS OF FUNCTIONALITY")
        endif()

    else()
        add_definitions(-DHAVE_OPENCV2)
    endif()
endif()

# enable GPU enhanced SURF features
# if BOTH CUDA and the OPENCV contrib cuda features are available
if(CUDA_FOUND AND HAVE_OPENCV_CUDAFEATURES2D)
    add_definitions(-DHAVE_CUDA)
    message(STATUS "defined HAVE_CUDA")

    set(CUDA_CUDART_LIBRARY_OPTIONAL ${CUDA_CUDART_LIBRARY})
endif()

# OSX RPATH
if(APPLE)
   set(CMAKE_MACOSX_RPATH ON)
endif()


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES handeye_calib_camodocal
   CATKIN_DEPENDS
   eigen
   roscpp
   tf
   trajectory_msgs
  #vrep_common
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include
)

## Declare a C++ library
# add_library(handeye_calib_camodocal
#   src/${PROJECT_NAME}/handeye_calib_camodocal.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(handeye_calib_camodocal ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
add_executable(handeye_calib_camodocal
  src/handeye_calibration.cpp
  src/HandEyeCalibration.cc)


## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(handeye_calib_camodocal_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
message("GLOG = ${GLOG_LIBRARIES}  ${GLOG_LIB}  ${GLOG_LIBS}")
target_link_libraries(handeye_calib_camodocal
  ${catkin_LIBRARIES} ${GLOG_LIBRARIES}  ${GLOG_LIB}  ${GLOG_LIBS} ${OpenCV_LIBRARIES} ${CERES_LIBRARIES}
)

