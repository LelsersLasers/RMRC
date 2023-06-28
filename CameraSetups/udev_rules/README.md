# udev_rules

## NOTE: THESE ARE NOT USED

`/dev/v4l/by-id/` should have stable symlinks. Use those instead.

## Example from Dr. J

Link: <https://github.com/TBSDrJ/Xavier-jp45-setup/blob/main/install_sudo_step_2.sh>
```
# Set up udev rule so that U2D2 is readable/writeable by the student user automatically
echo 'SUBSYSTEM=="usb", ACTION=="add", ATTR{idVendor}=="0403", ATTR{idProduct}=="6014", SYMLINK+="dynamixel", OWNER="student", GROUP="student", MODE="0664"' > 80-dynamixel-controller-U2D2.rules
chown root 80-dynamixel-controller-U2D2.rules
chgrp root 80-dynamixel-controller-U2D2.rules
mv 80-dynamixel-controller-U2D2.rules /etc/udev/rules.d
```