#!/bin/bash
 
# this file is listed in /etc/rc.local to be executed on boot as user root
 
 
if [ "$(whoami)" != "root" ]; then
        echo "Run me as super user!"
        exit 1
fi
 
 
# we source this because of ROS_IP etc:
source /home/demo/.bashrc
# but as ~-relative paths from that won't work as we are root,
#  make sure we have ros sourced:
source /opt/ros/hydro/setup.bash
 
echo "waiting for rostopic.."
while ! rostopic list ; do
	sleep 1
done
echo "roscore ready"
 
 
echo "Checking whether any ps3joy(_node) is already running.."
if !(pgrep ps3joy.py > /dev/null || pgrep ps3joy_node > /dev/null); then
	echo "Error! ps3joy(_node) is already running! Quitting starter"
	exit 1
else
	echo "Starting ps3joy_node..."
	/opt/ros/hydro/lib/ps3joy/ps3joy.py 
	#/home/demo/catkin_ws/src/joystick_drivers/ps3joy/scripts/ps3joy_node.py #--inactivity-timeout=300 # > /var/log/rc_local-ps3joy_node.log 2>&1 &
	exit 0
fi
