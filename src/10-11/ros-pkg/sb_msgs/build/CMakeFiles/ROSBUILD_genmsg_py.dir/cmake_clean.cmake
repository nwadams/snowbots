FILE(REMOVE_RECURSE
  "../msg_gen"
  "../src/sb_msgs/msg"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/sb_msgs/msg/__init__.py"
  "../src/sb_msgs/msg/_VisionNav.py"
  "../src/sb_msgs/msg/_CarCommand.py"
  "../src/sb_msgs/msg/_IMU.py"
  "../src/sb_msgs/msg/_LidarNav.py"
  "../src/sb_msgs/msg/_ServoCommand.py"
  "../src/sb_msgs/msg/_TurretCommand.py"
  "../src/sb_msgs/msg/_RobotState.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
