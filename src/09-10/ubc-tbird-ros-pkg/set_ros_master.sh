#!/bin/bash

## Really simple script to set the host/ip address that the
## ROS master is running on.  Use this script to run a node
## on a different machine than the one the master is running
## on.
##
## Author: Ian Phillips

if [ $1 ]; then
  echo "Setting ROS master to $1"
  export ROS_MASTER_URI="http://$1:11311"
else
  echo "Resetting ROS master to localhost"
  export ROS_MASTER_URI="http://localhost:11311"
fi
env | grep ROS_MASTER
