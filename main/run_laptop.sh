#!/bin/bash

sudo chmod 777 /dev/ttyUSB0
sudo bash -c "echo 1 > /sys/bus/usb-serial/devices/ttyUSB0/latency_timer"

python3 main_laptop.py "$@"