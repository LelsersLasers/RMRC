# Cameras

To get camera ids/symlinks (should be stable): `ls /dev/v4l/by-id`
```
usb-046d_C270_HD_WEBCAM_200901010001-video-index0
usb-GroupGets_PureThermal__fw:v1.3.0__8003001e-5119-3038-3532-373600000000-video-index0
```

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

If the ir camera is having issues/the window is doing the weird see through thing, make sure the black piece is fully fully pushed into the board.

# RVIZ

Can edit layout in the open GUI window then press `CONTROL-S` to save.
```
rviz -d fourCameras.rviz
```