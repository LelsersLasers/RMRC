#!/bin/bash

tegrastats --interval 20 --logfile tegrastats.log --start # run in background
sleep 1

python3 main.py
sleep 1

tegrastats --stop # close background process
rm -v tegrastats.log # don't care about old data