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

rosbuild_add_executable(cone_wall src/cone_identification.cpp)
rosbuild_add_executable(track_wall_lidar src/cone_id_withTrackWall.cpp)
rosbuild_add_executable(simple_lidar src/simple_lidar.cpp)
rosbuild_add_library(${PROJECT_NAME} src/TrackWall.cpp)
rosbuild_add_library(${PROJECT_NAME} src/ConeNode.cpp)
target_link_libraries(track_wall_lidar ${PROJECT_NAME})

rosbuild_add_executable(pathing src/pathing_node.cpp)
rosbuild_add_library(${PROJECT_NAME} src/ConePosition.cpp)
rosbuild_add_library(${PROJECT_NAME} src/NormalizeWall.cpp)
rosbuild_add_library(${PROJECT_NAME} src/LongestLineNav.cpp)
rosbuild_add_library(${PROJECT_NAME} src/VectorPath.cpp)
target_link_libraries(pathing ${PROJECT_NAME})

rosbuild_add_executable(pathing2 src/pathing_node2.cpp)
target_link_libraries(pathing2 ${PROJECT_NAME})


#old comp code
#rosbuild_add_executable(blizzard_lidar src/blizzard_lidar.cpp)
