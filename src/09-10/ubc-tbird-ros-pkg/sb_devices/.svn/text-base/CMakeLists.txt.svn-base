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
rosbuild_add_boost_directories()
rosbuild_add_library(${PROJECT_NAME} 
	src/Odometer.cpp 
	src/ServoController.cpp 
	src/SpeedController.cpp
	src/Accelerometer.cpp
	src/AccelSpeedController.cpp )
rosbuild_add_compile_flags(${PROJECT_NAME} -Werror)

#rosbuild_add_executable(speed_control src/speed_control.cpp)
#target_link_libraries(speed_control ${PROJECT_NAME})

# The servo_control node executable is prepended with an underscore.  This
# is because the binary executable should not be run directly -- instead,
# the src/servo_control shell script should be run instead with the name 
# of the servo you want to control (eg, "throttle").  This will run the
# actual node with all of the correct remappings set up.  See the shell
# script itself for more info.
#rosbuild_add_executable(_servo_control src/servo_control.cpp)
#target_link_libraries(_servo_control ${PROJECT_NAME})

rosbuild_add_executable(bot_hal src/bot_hal.cpp)
target_link_libraries(bot_hal ${PROJECT_NAME} sb_rangefinders)

rosbuild_add_executable(oscar_hal src/oscar_hal.cpp)
target_link_libraries(oscar_hal ${PROJECT_NAME} sb_rangefinders)

rosbuild_link_boost(${PROJECT_NAME} program_options)
