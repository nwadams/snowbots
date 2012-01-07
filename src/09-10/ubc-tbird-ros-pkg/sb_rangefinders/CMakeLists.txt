cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE Debug)

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

rosbuild_add_boost_directories()
#rosbuild_add_compile_flags(${PROJECT_NAME} -Werror)
rosbuild_add_library(${PROJECT_NAME} src/Rangefinder.cpp)

#rosbuild_add_executable(coneHugger src/coneHugger.cpp)
#rosbuild_add_executable(irTester src/irTester.cpp)

rosbuild_add_executable(ir_extractor src/ir_extractor.cpp)
target_link_libraries(ir_extractor ${PROJECT_NAME})

rosbuild_add_executable(lidar_cone_hugger src/lidar_cone_hugger3.cpp)
target_link_libraries(lidar_cone_hugger ${PROJECT_NAME})

rosbuild_add_executable(lidar_cone_hugger_sim src/lidar_cone_hugger_sim2.cpp)
target_link_libraries(lidar_cone_hugger_sim ${PROJECT_NAME})

rosbuild_link_boost(ir_extractor program_options)

rosbuild_add_executable(blizzard_lidar src/blizzard_lidar.cpp)
#target_link_libraries (oscar_lidar ${PROJECT_NAME})
