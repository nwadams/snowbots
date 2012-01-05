FILE(REMOVE_RECURSE
  "CMakeFiles/ROSBUILD_genmsg_cpp"
  "msg_gen/cpp/include/sb_msgs/RobotState.h"
  "msg_gen/cpp/include/sb_msgs/VisionNav.h"
  "msg_gen/cpp/include/sb_msgs/IMU.h"
  "msg_gen/cpp/include/sb_msgs/CarCommand.h"
  "msg_gen/cpp/include/sb_msgs/ServoCommand.h"
  "msg_gen/cpp/include/sb_msgs/TurretCommand.h"
  "msg_gen/cpp/include/sb_msgs/LidarNav.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
