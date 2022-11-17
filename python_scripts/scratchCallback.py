#!/usr/bin/env python
# -*- coding: utf-8 -*-

HOST = "localhost"
PORT = 4223
UID = "Hfe" # Change XYZ to the UID of your Industrial Dual 0-20mA Bricklet 2.0
UID_master1 = "6EG9ps"
UID_master2 = "6s6yLQ"

from tinkerforge.ip_connection import IPConnection
from tinkerforge.bricklet_industrial_dual_0_20ma_v2 import BrickletIndustrialDual020mAV2
from tinkerforge.brick_master import BrickMaster

# Callback function for current callback
def cb_current(channel, current):
    if channel == 0:
        print("Channel: " + str(channel))
        print("Current: " + str(current/1000000.0) + " mA")
        print("")

if __name__ == "__main__":
    ipcon = IPConnection() # Create IP connection
    id020 = BrickletIndustrialDual020mAV2(UID, ipcon) # Create device object
    master1 = BrickMaster(UID_master1, ipcon) 
    master2 = BrickMaster(UID_master2, ipcon) 

    ipcon.connect(HOST, PORT) # Connect to brickd
    # Don't use device before ipcon is connected

    # Register current callback to function cb_current
    id020.register_callback(id020.CALLBACK_CURRENT, cb_current)

    # Set period for current (channel 0) callback to 1s (1000ms) without a threshold
    id020.set_current_callback_configuration(0, 1000, False, "x", 0, 0)
    id020.set_current_callback_configuration(1, 1000, False, "x", 0, 0)

    input("Press key to exit\n") # Use raw_input() in Python 2
    ipcon.disconnect()