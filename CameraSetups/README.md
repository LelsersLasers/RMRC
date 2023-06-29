# Running

Should be able to just `./run_all.sh` from this directory.

## Issues

- If something doesn't work
    - Try closing rviz and rerunning the script
- Realsense cameras sometimes don't connect
    - Try lower power mode (15W 6 core instead of 20W)
        - Allows more power for the cameras
    - Other possible solution:
        - Keep rerunning script and/or rebooting until it works
- Don't press control-c while the script is running
    - It must finish to correctly close all the background realsense nodes/etc that it started
    - If this happens, just reboot to be able to run the script again
- Finishes/exits but then prints a couple more lines starting with `escalating to SIGTERM` or similar
    - Once it says `done` you can press control-c to get back to the terminal prompt
    - (The script already finished and closed all of the background proccesses)

# Cameras

- To get camera ids/symlinks (should be stable): `ls /dev/v4l/by-id`
- If the output doesn't contain the following two entries, update the respective launch files (`launch_ir.launch` and `launch_c270.launch`)
```
usb-046d_C270_HD_WEBCAM_200901010001-video-index0
usb-GroupGets_PureThermal__fw:v1.3.0__8003001e-5119-3038-3532-373600000000-video-index0
```

To run the proccesses individually (for either better troubleshooting or if `run_all.sh` isn't working properly):
- Run each of the following sections in their own terminal
    - 4 total (3 launchs and 1 for rviz)'
- Run in the order listed (t265 & d435i first and rviz last)

## t265 & d435i

```
roslaunch launch_t265_d435i.launch
```

## c270


Note: assumes `device=/dev/v4l/by-id/usb-046d_C270_HD_WEBCAM_200901010001-video-index0`
```
export GSCAM_CONFIG="gst-launch-1.0 v4l2src device=/dev/v4l/by-id/usb-046d_C270_HD_WEBCAM_200901010001-video-index0 ! videoconvert ! ffmpegcolorspace"
rosrun gscam gscam
```
Or:
```
roslaunch launch_c270.launch
```

## IR

Note: assumes `device=/dev/v4l/by-id/usb-GroupGets_PureThermal__fw:v1.3.0__8003001e-5119-3038-3532-373600000000-video-index0`
```
export GSCAM_CONFIG="gst-launch-1.0 v4l2src device=/dev/v4l/by-id/usb-GroupGets_PureThermal__fw:v1.3.0__8003001e-5119-3038-3532-373600000000-video-index0 ! videoconvert ! ffmpegcolorspace"
rosrun gscam gscam
```
Or
```
roslaunch launch_ir.launch
```

### Issues

- If the ir camera is having issues/the window is doing the weird see through thing, make sure the black piece is fully fully pushed into the board.
- The first time info is requested from it it does not work

# RVIZ

Can edit layout in the open GUI window then press `CONTROL-S` to save.
```
rviz -d fourCameras.rviz
```