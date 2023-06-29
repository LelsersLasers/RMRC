#!/bin/bash

# Prints my output in blue so it can be seen easier and different from the outputs from the commands
printInColor() {
    BLUE='\033[0;34m'
    NC='\033[0m' # no color

    echo -e "${BLUE} $1 ${NC}"
}


# start: t265 & d435i --------------------------------------------------------#
printInColor "STARTING! Will take ~30 seconds to launch everything."
printInColor "(note: this script doesn't play super nice with control-c)\n"

printInColor "Starting launching t265 and d435i"
sleep 1

# also starts main realsense node
roslaunch launch_t265_d435i.launch &

# Likely could be ~3 sec, but this reduces yellow/red text (and thus unneeded stress)
sleep 9
# end: t265 & d435i ----------------------------------------------------------#


# start: c270 ----------------------------------------------------------------#
printInColor "Starting launching c270"
sleep 1

# unset because the commands writes then uses an env var but it can't overwrite
unset GSCAM_CONFIG
roslaunch launch_c270.launch &

sleep 9
# end: c270 ------------------------------------------------------------------#


# start: ir ------------------------------------------------------------------#
printInColor "Starting launching ir"
sleep 1

unset GSCAM_CONFIG
roslaunch launch_ir.launch &

sleep 9
# end: ir --------------------------------------------------------------------#


# start: rviz ----------------------------------------------------------------#
printInColor "Starting RVIZ (try to close it by the 'X' rather than control-c)"
rviz -d fourCameras.rviz
printInColor "Closing RVIZ"

sleep 2
# end: rviz ------------------------------------------------------------------#


# start: cleanup -------------------------------------------------------------#
printInColor "Starting closing background processes"
printInColor "Currently running backgrounds processes:"
jobs -r

sleep 1

# Take output of running background jobs and pass them to kill
jobs -p | xargs kill

sleep 15
jobs

printInColor "Finished cleanup"
# end: cleanup ---------------------------------------------------------------#


printInColor "Exiting"