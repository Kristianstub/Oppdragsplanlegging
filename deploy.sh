#!/bin/bash
set -e

ssh ubuntu@192.168.50.${1} "
export ROS_MASTER_URI=http://192.168.50.84:11311
export ROS_IP=192.168.50.${1}
export TURTLEBOT3_MODEL=waffle_pi
roslaunch turtlebot3_bringup turtlebot3_robot.launch
"