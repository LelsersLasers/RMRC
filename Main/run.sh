#!/bin/bash

chmod 777 /dev/ttyUSB0

tegrastats --interval 20 --logfile tegrastats.log --start # run in background
sleep 1

# -t: responses from motors (mostly testing/debuging purposes)
python3 main.py -t
sleep 1

tegrastats --stop # close background process
rm -v tegrastats.log # don't care about old data