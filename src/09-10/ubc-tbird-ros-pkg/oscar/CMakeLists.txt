cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})


# OpenCV junk
FIND_PACKAGE(OpenCV2 REQUIRED)
INCLUDE_DIRECTORIES(${OPENCV_INCLUDE_DIR})
#INCLUDE_DIRECTORIES(${cv_bridge})
#INCLUDE_DIRECTORIES(${opencv2})
#INCLUDE_DIRECTORIES(${image_transport})


rosbuild_add_executable(visionCommander src/visionCommander.cpp)
rosbuild_add_executable(imageReceive src/imageReceive2.cpp)
rosbuild_add_library(${PROJECT_NAME} src/vision.cpp)
target_link_libraries(visionCommander ${PROJECT_NAME} )
rosbuild_add_library(${PROJECT_NAME} src/cvblob/cvblob.cpp)
rosbuild_add_library(${PROJECT_NAME} src/cvblob/cvcontour.cpp)
rosbuild_add_library(${PROJECT_NAME} src/cvblob/cvaux.cpp)
rosbuild_add_library(${PROJECT_NAME} src/cvblob/cvtrack.cpp)
rosbuild_add_library(${PROJECT_NAME} src/cvblob/cvlabel.cpp)
rosbuild_add_library(${PROJECT_NAME} src/cvblob/cvcolor.cpp)
rosbuild_add_library(${PROJECT_NAME} src/imageReceive2.cpp)
target_link_libraries(imageReceive ${PROJECT_NAME} )
rosbuild_add_executable(speedControl src/speedControl.cpp)
rosbuild_add_executable(eStop src/eStop.cpp)
