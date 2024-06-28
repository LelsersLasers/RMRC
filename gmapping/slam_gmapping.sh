#!/bin/bash

echo "granting 777 permission to USB0 port, where Lidar should be connected..."

sudo chmod 777 /dev/ttyUSB0

echo "sourcing catkin_ws/devel/setup.bash so all commands are recognized..."

source /home/student/Desktop/catkin_ws/devel/setup.bash

echo "sourcing .imu_venv to activate with all imu dependencies..."

source .imu_venv/bin/activate

echo "starting all slam processes in parallel with 2-sec delay"

cat slam_commands.txt | parallel --delay 2

killall -9 rosmaster

echo "all ros processes should have ended."
