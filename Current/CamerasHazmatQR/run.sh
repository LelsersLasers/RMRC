#!/bin/bash

tegrastats --interval 50 --logfile tegrastats.log --start
sleep 1

python3 main.py
sleep 1

tegrastats --stop