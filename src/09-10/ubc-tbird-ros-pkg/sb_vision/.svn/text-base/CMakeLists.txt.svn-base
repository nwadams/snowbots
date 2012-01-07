cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE Debug)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

add_definitions( -DDEBUG )
#rosbuild_add_compile_flags(${PROJECT_NAME} -Werror)

# Choose the correct directory for OpenCV below
#set( OpenCV_DIR "/usr/local/")
set( OpenCV_DIR "/opt/ros/boxturtle/stacks/vision_opencv/opencv2/opencv")

include_directories( "${OpenCV_DIR}/include" )
link_directories( "${OpenCV_DIR}/lib/" )
INCLUDE_DIRECTORIES(${cv_bridge})
INCLUDE_DIRECTORIES(${image_transport})

rosbuild_add_boost_directories()

#rosbuild_add_executable( stopSignDetector src/stopSignDetector.cpp)
#target_link_libraries(   stopSignDetector cv highgui)

#rosbuild_add_library(${PROJECT_NAME} src/stopSignDetector2.cpp)
#target_link_libraries(stopSignDetector2 ${PROJECT_NAME})

rosbuild_add_executable( ian_nav src/ian_nav.cpp )
target_link_libraries(   ian_nav cv highgui )
rosbuild_link_boost(     ian_nav program_options)

#rosbuild_add_executable( nnet_train src/nnet_train.cpp )
#target_link_libraries(   nnet_train cv highgui ml )

rosbuild_add_executable( oscar_stop_sign src/oscar_stop_sign.cpp )
target_link_libraries(   oscar_stop_sign cv highgui )
rosbuild_link_boost(     oscar_stop_sign program_options)
