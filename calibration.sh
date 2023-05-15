#!/bin/bash

pidfile="$HOME/MayTest/.lock"
if [ -f "$pidfile" ] && kill -0 "$(cat "$pidfile")" 2>/dev/null; then
    echo still running
    exit 1
fi
echo $$ > $pidfile

cd ~/MayTest || exit
export PYTHONPATH=.

venv/bin/python3 main/calibration.py
