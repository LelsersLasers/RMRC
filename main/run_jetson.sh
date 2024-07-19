#!/bin/bash

# jetson_clocks without any options: set CPU, GPU, and EMC clocks to maximum
sudo jetson_clocks

# high power mode
sudo nvpmodel -m 0

# In main_jetson.py instead (so can be argparse) [NEVERMIND]
sudo chmod 777 /dev/ttyUSB0
sudo bash -c "echo 1 > /sys/bus/usb-serial/devices/ttyUSB0/latency_timer"

tegrastats --interval 20 --logfile tegrastats.log --start # run in background
sleep 1

export LD_PRELOAD=/usr/local/lib/python3.8/dist-packages/torch.libs/libgomp-4dbbc2f2.so.1.0.0
echo -e "LD_PRELOAD=$LD_PRELOAD\n"

python3 main_jetson.py "$@"
sleep 1

tegrastats --stop # close background process
rm -v tegrastats.log # don't care about old data
