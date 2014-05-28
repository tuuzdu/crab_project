#!/bin/bash
  source /opt/ros/hydro/setup.bash
  source /home/ubuntu/catkin_ws/devel/setup.bash
#Get custom packages (this script will launch with 'sudo', so it'll be the root user
  export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/home/ubuntu/ros_ws 
#Home directory of your user for logging
  export HOME="/home/ubuntu"
#Home directory for your user for logging
  export ROS_HOME="/home/ubuntu"
 
#I run ROS on a headless machine, so I want to be able to connect to it remotely
#  export ROS_IP="192.168.1.10"

 
  roslaunch crab_description gait_kinematics_imu.launch
 
