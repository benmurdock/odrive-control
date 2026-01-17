#!/usr/bin/env python3
"""
ODrive Motor Controller Interface

Basic script to connect to an ODrive v3.x and demonstrate communication.
"""

import odrive
from odrive.enums import *  # Imports constants like AXIS_STATE_IDLE, etc.


def find_odrive():
    """
    Searches for an ODrive connected via USB.
    This will block (wait) until an ODrive is found.
    """
    print("Looking for ODrive...")
    odrv = odrive.find_any()
    print(f"Found ODrive! Serial: {odrv.serial_number}")
    return odrv


def print_status(odrv):
    """Print basic status information from the ODrive."""
    print("\n--- ODrive Status ---")
    print(f"Hardware version: {odrv.hw_version_major}.{odrv.hw_version_minor}")
    print(f"Firmware version: {odrv.fw_version_major}.{odrv.fw_version_minor}.{odrv.fw_version_revision}")
    print(f"Voltage: {odrv.vbus_voltage:.2f}V")

    # Check each axis (ODrive v3 has 2 axes: axis0 and axis1)
    for i, axis in enumerate([odrv.axis0, odrv.axis1]):
        print(f"\nAxis {i}:")
        print(f"  State: {axis.current_state}")
        print(f"  Error: {axis.error}")


def main():
    # Find and connect to ODrive
    odrv = find_odrive()

    # Show current status
    print_status(odrv)

    print("\n--- Ready ---")
    print("The 'odrv' object is available for interaction.")
    print("Example commands:")
    print("  odrv.axis0.requested_state = AXIS_STATE_IDLE")
    print("  odrv.axis0.controller.input_vel = 2  # turns/sec")

    # Return the odrive object so you can interact with it
    # if running in interactive mode (ipython -i main.py)
    return odrv


if __name__ == "__main__":
    odrv = main()
