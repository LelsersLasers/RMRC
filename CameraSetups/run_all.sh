#!/bin/bash

printInColor() {
    BLUE='\033[0;34m'
    NC='\033[0m' # no color

    echo -e "${BLUE} $1 ${NC}"
}


printInColor "STARTING! Will take ~30 seconds to launch everything."
printInColor "(note: this script doesn't play super nice with control-c)\n"

printInColor "Starting launching t265 and d435i"
sleep 1

# also starts main realsense node
roslaunch launch_t265_d435i.launch &

sleep 9



printInColor "Starting launching c270"
sleep 1

# unset because the commands writes then uses an env var but it can't overwrite
unset GSCAM_CONFIG
roslaunch launch_c270.launch &

sleep 9



printInColor "Starting launching ir"
sleep 1

# should be fine to reuse the same env var in the same terminal...
unset GSCAM_CONFIG
roslaunch launch_ir.launch &

sleep 9



printInColor "Starting RVIZ"
rviz -d fourCameras.rviz
printInColor "Closing RVIZ"


sleep 5


printInColor "Starting closing background processes"
printInColor "Currently running backgrounds processes:"
jobs -r

sleep 1

# kill $(jobs -p) # this could fail if $(jobs -p) is empty
jobs -p | xargs kill

sleep 15
jobs

printInColor "Finished cleanup"


printInColor "Exiting"
