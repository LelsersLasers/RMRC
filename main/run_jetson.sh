#!/bin/bash

# jetson_clocks without any options: set CPU, GPU, and EMC clocks to maximum
sudo jetson_clocks

chmod 777 /dev/ttyUSB0
bash -c "echo 1 > /sys/bus/usb-serial/devices/ttyUSB0/latency_timer"

tegrastats --interval 20 --logfile tegrastats.log --start # run in background
sleep 1

python3 main_jetson.py "$@"
sleep 1

tegrastats --stop # close background process
rm -v tegrastats.log # don't care about old data