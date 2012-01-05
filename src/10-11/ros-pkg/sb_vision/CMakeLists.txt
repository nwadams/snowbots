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

#Nick
rosbuild_add_executable(simple_vision src/simple_vision.cpp)
rosbuild_add_executable(stereo_vision src/stereoVision.cpp)
#rosbuild_add_executable(stop_sign src/stop_sign.cpp)
rosbuild_add_executable(videoWriter src/videoWriter.cpp)

#rosbuild_add_executable(circle_detect_demo src/circle_detect.cpp)
#rosbuild_add_executable(detect_rect src/detectrect.cpp)
#rosbuild_add_executable(traffic_ros_demo src/traffic_ros.cpp)
#rosbuild_add_executable(bright_test src/bright.cpp)
#rosbuild_add_executable(traffic_detect_demo src/traffic_detect.cpp)
#rosbuild_add_executable(circle_API src/circle_detect_API2.cpp)
