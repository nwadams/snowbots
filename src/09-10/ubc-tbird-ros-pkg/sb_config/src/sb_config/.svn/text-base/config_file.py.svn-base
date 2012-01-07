"""
Exports the L{find_config_file} function, which can be used to locate
the config file for the robot.

@author: Ian Phillips <ianfp@freeshell.org>
"""

from optparse import OptionParser
import os

def find_config_file(path=None):
  """ 
  Searches for the robot config file in these three places, in this
  order:

    - as specified by the path parameter
    - as specified by the SB_CONFIG_FILE environment variable
    - at sb_config/cfg/robot.cfg

  @param path: the path to the config file.  This parameter can be
    None, in which case this function will check the environment
    and default location.
  @return: the full path to the default config file for the robot, 
    or None if the file is not found.
  """
  return _check_arg(path) or _check_env() or _check_default()


def _check_arg(filename):
  if filename and os.path.isfile(filename):
    return filename
  return None


def _check_env():
  if 'SB_CONFIG_FILE' in os.environ:
    filename = os.environ['SB_CONFIG_FILE']
    if filename and os.path.isfile(filename):
      return filename
  return None


def _check_default():
  package_path = os.environ['ROS_PACKAGE_PATH']
  stacks = package_path.split(":")
  for stack in stacks:
    if os.path.isdir(stack):
      packages = os.listdir(stack)
      for package in packages:
        fullpath = os.path.join(stack, package)
        if os.path.isdir(fullpath) and package == "sb_config":
          fullpath = os.path.join(fullpath, "cfg", "robot.cfg")
          if os.path.isfile(fullpath):
            return fullpath
  return None


