#!/bin/bash

tegrastats --interval 50 --logfile tegrastats.log --start

python3 main.py

tegrastats --stop