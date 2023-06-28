#!/bin/bash


# also starts main realsense node
roslaunch launch_t265_d435i.launch &

sleep 10

# unset because the commands writes then uses an env var but it can't overwrite
unset GSCAM_CONFIG
roslaunch launch_c270.launch &

sleep 10

# should be fine to reuse the same env var in the same terminal...
unset GSCAM_CONFIG
roslaunch launch_ir.launch &

sleep 10

rviz -d fourCameras.rviz


jobs

# close 3 the roslaunch commands
kill %3
kill %2
kill %1

jobs