#!/bin/bash


echo "Starting launching t265 and d435i"
sleep 1

# also starts main realsense node
roslaunch launch_t265_d435i.launch &
sleep 9
echo "Launched t265 and d435i"


echo "Starting launching c270"
sleep 1
# unset because the commands writes then uses an env var but it can't overwrite
unset GSCAM_CONFIG
roslaunch launch_c270.launch &
sleep 9
echo "Launched c270"


echo "Starting launching ir"
# should be fine to reuse the same env var in the same terminal...
unset GSCAM_CONFIG
roslaunch launch_ir.launch &
sleep 9
echo "Launched ir"


echo "Starting RVIZ"
rviz -d fourCameras.rviz
echo "Closing RVIZ"


sleep 5


echo "Closing background processes"
jobs

sleep 1

kill %%
# kill %3
# kill %2
# kill %1

jobs

echo "Finishing..."
sleep 10
echo "Finished cleanup"
