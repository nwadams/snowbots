FILE(REMOVE_RECURSE
  "../src/sb_msgs/msg"
  "../msg_gen"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_lisp"
  "../msg_gen/lisp/RobotState.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_RobotState.lisp"
  "../msg_gen/lisp/VisionNav.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_VisionNav.lisp"
  "../msg_gen/lisp/IMU.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_IMU.lisp"
  "../msg_gen/lisp/CarCommand.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_CarCommand.lisp"
  "../msg_gen/lisp/ServoCommand.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_ServoCommand.lisp"
  "../msg_gen/lisp/TurretCommand.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_TurretCommand.lisp"
  "../msg_gen/lisp/LidarNav.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_LidarNav.lisp"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_lisp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
