#!/bin/bash     
cd ~/src/Firmware
gnome-terminal -e './gazeboVmWork.sh "make posix_sitl_default gazebo_iris_opt_flow"'
sleep 10s
cd ~/src/Firmware
gnome-terminal -e './rosLaunchDrone.sh'
sleep 6s
gnome-terminal -e 'rosrun tera_2_pkg tera_2_node'
sleep 3s
cd ~/Desktop/GDP/POT_FIELDS/06_06_2018_EA/
gnome-terminal -e './main_06_06_2018.py'
sleep 140s
killall -15 "gazeboVmWork.sh"
killall -15 "gzserver"
killall -15 "make"
killall -15 "ninja"
killall -15 "sitl_run.sh"
killall -15 "px4"
killall -15 "roslaunch"
killall -15 "rosmaster"
killall -15 "rosout"
killall -15 "tera_2_node"
killall -15 "mavros_node"
killall -15 "rosLaunchDrone.sh"
killall -15 "sh"
# Last but not least, kill Navigation Algorithm
pgrep -f main_06_06_2018.py # will give you its pid
pkill -9 -f main_06_06_2018.py # kills the matching pid
exit 0


