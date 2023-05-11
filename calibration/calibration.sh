#!/bin/bash

cd ~/MayTest || exit
export PYTHONPATH=.

venv/bin/python3 calibration/calibration.py
