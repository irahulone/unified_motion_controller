#!/bin/bash

sudo apt-get update && sudo apt-get install raspi-config i2c-tools -y 
pip install lgpio adafruit-circuitpython-gps RPi.GPIO adafruit-circuitpython-bno055 --break-system-packages
sudo raspi-config
