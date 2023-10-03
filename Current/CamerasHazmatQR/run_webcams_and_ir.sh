#!/bin/bash

# Prints my output in blue so it can be seen easier and different from the outputs from the commands
BLUE='\033[0;34m'
NC='\033[0m' # no color
printInColor() {
    echo -e "${BLUE} $1 ${NC}"
}


printInColor "STARTING!"
printInColor "(note: this script doesn't play super nice with control-c)\n"


# request info from ir camera
# it never works the first time, so this is the first time so it works in rviz
# printInColor "Starting fake ir camera request"
# timeout 3 gst-launch-1.0 v4l2src device=/dev/v4l/by-id/usb-GroupGets_PureThermal__fw:v1.3.0__8003000b-5113-3238-3233-393800000000-video-index0 ! videoconvert ! xvimagesink
# sleep 3
# timeout 3 gst-launch-1.0 v4l2src device=/dev/v4l/by-id/usb-GroupGets_PureThermal__fw:v1.3.0__8003000b-5113-3238-3233-393800000000-video-index0 ! videoconvert ! xvimagesink


# start: webcam 1 ------------------------------------------------------------#
printInColor "Starting webcam 1"

# gst-launch-1.0 v4l2src device=/dev/v4l/by-id/usb-046d_C270_HD_WEBCAM_2D4AA0A0-video-index0 ! videoconvert ! video/x-raw,format=UYVY ! videoscale ! video/x-raw,width=320,height=240 ! videoconvert ! xvimagesink &
# python3 webcam.py &
python3 webcam_live_hazmat_qr_multiprocessing.py &

sleep 1
# end: webcam 1 --------------------------------------------------------------#


# start: webcam 2 ------------------------------------------------------------#
printInColor "Starting webcam 2"

gst-launch-1.0 v4l2src device=/dev/v4l/by-id/usb-046d_C270_HD_WEBCAM_348E60A0-video-index0 ! videoconvert ! video/x-raw,format=UYVY ! videoscale ! video/x-raw,width=320,height=240 ! videoconvert ! xvimagesink &

sleep 1
# end: webcam 2 --------------------------------------------------------------#


# start: ir ------------------------------------------------------------------#
printInColor "Starting launching ir"

gst-launch-1.0 v4l2src device=/dev/v4l/by-id/usb-GroupGets_PureThermal__fw:v1.3.0__8003000b-5113-3238-3233-393800000000-video-index0 ! videoconvert ! xvimagesink &
# bash run_ir.sh &

sleep 1
# end: ir --------------------------------------------------------------------#


# start: wait for quit requested ---------------------------------------------#
printInColor "Finished startup Press K to exit"
while true; do 
read -rsn1 input
if [ "$input" = "k" ]; then
    printInColor "K key pressed"
    break
fi
done 
# end: wait for quit requested -----------------------------------------------#


# start: cleanup -------------------------------------------------------------#
printInColor "Starting closing background processes"
printInColor "Currently running backgrounds processes:"

echo -e "${BLUE}"
jobs -r
echo -e "${NC}"

sleep 1

# Take output of running background jobs and pass them to kill
jobs -p | xargs kill

sleep 5 # give time for the procceses to end

echo -e "${BLUE}"
jobs
echo -e "${NC}"

printInColor "Finished cleanup"
# end: cleanup ---------------------------------------------------------------#


printInColor "Exiting"
