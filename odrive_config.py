#!/usr/bin/env python3
"""
ODrive Configuration Script

Configures motor, encoder, and controller settings for both axes.
Designed for easy GUI integration later - all config values are in
a single dictionary that can be edited and applied.

Hardware:
- Motor: D5065 270KV (7 pole pairs)
- Encoder: CUI AMT102-V (8192 CPR)
- Current limit: 10A (conservative)

Usage:
    python odrive_config.py              # Interactive menu
    python odrive_config.py --apply      # Apply config to both axes
    python odrive_config.py --read       # Read current config from axis0
    python odrive_config.py --copy       # Copy axis0 config to axis1
"""

import sys
import odrive
from odrive.enums import (
    AXIS_STATE_IDLE,
    AXIS_STATE_FULL_CALIBRATION_SEQUENCE,
    MOTOR_TYPE_PMSM_CURRENT_CONTROL,
    ENCODER_MODE_INCREMENTAL,
)

# =============================================================================
# CONFIGURATION VALUES
# =============================================================================
# Edit these values to match your hardware.
# This dictionary structure is designed for easy GUI editing later.

DEFAULT_CONFIG = {
    # Motor configuration
    "motor": {
        "pole_pairs": 7,                    # D5065 has 7 pole pairs
        "motor_type": MOTOR_TYPE_PMSM_CURRENT_CONTROL,
        "current_lim": 10.0,                # Amps - conservative limit
        "current_lim_margin": 8.0,          # Amps - margin above current_lim
        "calibration_current": 5.0,         # Amps - reduced for low-inductance D5065
        "resistance_calib_max_voltage": 2.0,  # Volts - reduced for low-resistance motor
        "requested_current_range": 25.0,    # Amps - ADC range (should be > current_lim)
        "torque_constant": 8.27 / 270,      # Nm/A = 8.27 / KV
    },

    # Encoder configuration
    "encoder": {
        "mode": ENCODER_MODE_INCREMENTAL,
        "cpr": 8192,                        # CUI AMT102-V counts per revolution
        "bandwidth": 1000,                  # Hz - encoder filter bandwidth
        "use_index": False,                 # Set True if using index pulse
    },

    # Controller configuration
    "controller": {
        "vel_limit": 20.0,                  # turns/sec - velocity limit
        "vel_limit_tolerance": 1.2,         # Multiplier for vel_limit before error
        "pos_gain": 20.0,                   # Position control gain
        "vel_gain": 0.16,                   # Velocity control gain
        "vel_integrator_gain": 0.32,        # Velocity integrator gain
    },

    # Axis configuration
    "axis": {
        "watchdog_timeout": 0.0,            # 0 = disabled
    },

    # Brake resistor (if installed)
    "brake_resistor": {
        "enable": True,
        "resistance": 2.0,                  # Ohms - common value
    },
}


# =============================================================================
# CONFIGURATION FUNCTIONS
# =============================================================================

def apply_motor_config(axis, config):
    """Apply motor configuration to an axis."""
    motor = config["motor"]
    axis.motor.config.pole_pairs = motor["pole_pairs"]
    axis.motor.config.motor_type = motor["motor_type"]
    axis.motor.config.current_lim = motor["current_lim"]
    axis.motor.config.current_lim_margin = motor["current_lim_margin"]
    axis.motor.config.calibration_current = motor["calibration_current"]
    axis.motor.config.resistance_calib_max_voltage = motor["resistance_calib_max_voltage"]
    axis.motor.config.requested_current_range = motor["requested_current_range"]
    axis.motor.config.torque_constant = motor["torque_constant"]
    print(f"  Motor: {motor['pole_pairs']} poles, {motor['current_lim']}A limit")


def apply_encoder_config(axis, config):
    """Apply encoder configuration to an axis."""
    encoder = config["encoder"]
    axis.encoder.config.mode = encoder["mode"]
    axis.encoder.config.cpr = encoder["cpr"]
    axis.encoder.config.bandwidth = encoder["bandwidth"]
    axis.encoder.config.use_index = encoder["use_index"]
    print(f"  Encoder: {encoder['cpr']} CPR, bandwidth {encoder['bandwidth']} Hz")


def apply_controller_config(axis, config):
    """Apply controller configuration to an axis."""
    ctrl = config["controller"]
    axis.controller.config.vel_limit = ctrl["vel_limit"]
    axis.controller.config.vel_limit_tolerance = ctrl["vel_limit_tolerance"]
    axis.controller.config.pos_gain = ctrl["pos_gain"]
    axis.controller.config.vel_gain = ctrl["vel_gain"]
    axis.controller.config.vel_integrator_gain = ctrl["vel_integrator_gain"]
    print(f"  Controller: vel_limit {ctrl['vel_limit']} t/s, pos_gain {ctrl['pos_gain']}")


def apply_axis_config(axis, config):
    """Apply axis-level configuration."""
    ax = config["axis"]
    axis.config.watchdog_timeout = ax["watchdog_timeout"]


def apply_brake_resistor_config(odrv, config):
    """Apply brake resistor configuration."""
    brake = config["brake_resistor"]
    odrv.config.enable_brake_resistor = brake["enable"]
    odrv.config.brake_resistance = brake["resistance"]
    print(f"  Brake resistor: {'enabled' if brake['enable'] else 'disabled'}, "
          f"{brake['resistance']} ohms")


def apply_config_to_axis(axis, config, axis_name="axis"):
    """Apply full configuration to a single axis."""
    print(f"\nApplying configuration to {axis_name}...")
    apply_motor_config(axis, config)
    apply_encoder_config(axis, config)
    apply_controller_config(axis, config)
    apply_axis_config(axis, config)
    print(f"{axis_name} configuration complete.")


def apply_config_to_odrive(odrv, config):
    """Apply configuration to both axes and ODrive-level settings."""
    print("\n" + "=" * 50)
    print("APPLYING CONFIGURATION")
    print("=" * 50)

    # ODrive-level config
    print("\nODrive settings:")
    apply_brake_resistor_config(odrv, config)

    # Both axes
    apply_config_to_axis(odrv.axis0, config, "axis0")
    apply_config_to_axis(odrv.axis1, config, "axis1")

    print("\n" + "=" * 50)
    print("Configuration applied to both axes.")
    print("Run 'save_configuration()' to persist to flash.")
    print("=" * 50)


def read_config_from_axis(axis):
    """Read current configuration from an axis into a config dict."""
    return {
        "motor": {
            "pole_pairs": axis.motor.config.pole_pairs,
            "motor_type": axis.motor.config.motor_type,
            "current_lim": axis.motor.config.current_lim,
            "current_lim_margin": axis.motor.config.current_lim_margin,
            "calibration_current": axis.motor.config.calibration_current,
            "resistance_calib_max_voltage": axis.motor.config.resistance_calib_max_voltage,
            "requested_current_range": axis.motor.config.requested_current_range,
            "torque_constant": axis.motor.config.torque_constant,
        },
        "encoder": {
            "mode": axis.encoder.config.mode,
            "cpr": axis.encoder.config.cpr,
            "bandwidth": axis.encoder.config.bandwidth,
            "use_index": axis.encoder.config.use_index,
        },
        "controller": {
            "vel_limit": axis.controller.config.vel_limit,
            "vel_limit_tolerance": axis.controller.config.vel_limit_tolerance,
            "pos_gain": axis.controller.config.pos_gain,
            "vel_gain": axis.controller.config.vel_gain,
            "vel_integrator_gain": axis.controller.config.vel_integrator_gain,
        },
        "axis": {
            "watchdog_timeout": axis.config.watchdog_timeout,
        },
    }


def read_brake_resistor_config(odrv):
    """Read brake resistor configuration from ODrive."""
    return {
        "brake_resistor": {
            "enable": odrv.config.enable_brake_resistor,
            "resistance": odrv.config.brake_resistance,
        }
    }


def print_config(config, title="Configuration"):
    """Pretty-print a configuration dictionary."""
    print(f"\n{title}")
    print("-" * 40)
    for section, values in config.items():
        print(f"\n[{section}]")
        for key, value in values.items():
            print(f"  {key}: {value}")


def copy_axis0_to_axis1(odrv):
    """Copy all configuration from axis0 to axis1."""
    print("\nReading configuration from axis0...")
    config = read_config_from_axis(odrv.axis0)
    config.update(read_brake_resistor_config(odrv))

    print("Applying configuration to axis1...")
    apply_config_to_axis(odrv.axis1, config, "axis1")
    print("\naxis1 now matches axis0 configuration.")


def save_configuration(odrv):
    """Save configuration to ODrive flash memory."""
    print("\nSaving configuration to flash...")
    try:
        odrv.save_configuration()
        print("Configuration saved successfully!")
        print("Note: ODrive will reboot after save.")
    except Exception as e:
        print(f"Save failed: {e}")
        print("You may need to reconnect after a reboot.")


# =============================================================================
# INTERACTIVE MENU
# =============================================================================

def interactive_menu(odrv):
    """Run an interactive configuration menu."""
    while True:
        print("\n" + "=" * 50)
        print("ODRIVE CONFIGURATION MENU")
        print("=" * 50)
        print("1. Apply default config to both axes")
        print("2. Copy axis0 config to axis1")
        print("3. Read and display axis0 config")
        print("4. Read and display axis1 config")
        print("5. Save configuration to flash")
        print("6. Run calibration on axis0")
        print("7. Run calibration on axis1")
        print("8. Run calibration on both axes (sequential)")
        print("0. Exit")
        print("-" * 50)

        try:
            choice = input("Enter choice: ").strip()
        except (EOFError, KeyboardInterrupt):
            print("\nExiting...")
            break

        if choice == "1":
            apply_config_to_odrive(odrv, DEFAULT_CONFIG)

        elif choice == "2":
            copy_axis0_to_axis1(odrv)

        elif choice == "3":
            config = read_config_from_axis(odrv.axis0)
            config.update(read_brake_resistor_config(odrv))
            print_config(config, "Axis 0 Configuration")

        elif choice == "4":
            config = read_config_from_axis(odrv.axis1)
            print_config(config, "Axis 1 Configuration")

        elif choice == "5":
            confirm = input("This will reboot the ODrive. Continue? (y/n): ")
            if confirm.lower() == 'y':
                save_configuration(odrv)
                print("ODrive is rebooting. Please reconnect.")
                break

        elif choice == "6":
            print("Starting axis0 calibration...")
            odrv.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
            print("Calibration started. Watch for motor movement.")
            print("Check axis0.error after calibration completes.")

        elif choice == "7":
            print("Starting axis1 calibration...")
            odrv.axis1.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
            print("Calibration started. Watch for motor movement.")
            print("Check axis1.error after calibration completes.")

        elif choice == "8":
            print("Starting axis0 calibration...")
            odrv.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
            input("Press Enter when axis0 calibration is complete...")
            print("Starting axis1 calibration...")
            odrv.axis1.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
            print("Calibration started. Watch for motor movement.")

        elif choice == "0":
            print("Exiting without saving.")
            break

        else:
            print("Invalid choice. Please try again.")


# =============================================================================
# MAIN
# =============================================================================

def main():
    """Main entry point."""
    print("ODrive Configuration Utility")
    print("=" * 50)
    print("\nSearching for ODrive...")

    try:
        odrv = odrive.find_any(timeout=10)
        serial = hex(odrv.serial_number).upper()
        print(f"Found ODrive: {serial}")
        print(f"Firmware: {odrv.fw_version_major}.{odrv.fw_version_minor}.{odrv.fw_version_revision}")
        print(f"Voltage: {odrv.vbus_voltage:.1f}V")
    except Exception as e:
        print(f"Failed to find ODrive: {e}")
        sys.exit(1)

    # Check command line arguments
    if len(sys.argv) > 1:
        arg = sys.argv[1]

        if arg == "--apply":
            apply_config_to_odrive(odrv, DEFAULT_CONFIG)
            confirm = input("\nSave to flash? (y/n): ")
            if confirm.lower() == 'y':
                save_configuration(odrv)

        elif arg == "--read":
            config = read_config_from_axis(odrv.axis0)
            config.update(read_brake_resistor_config(odrv))
            print_config(config, "Axis 0 Current Configuration")

        elif arg == "--copy":
            copy_axis0_to_axis1(odrv)
            confirm = input("\nSave to flash? (y/n): ")
            if confirm.lower() == 'y':
                save_configuration(odrv)

        else:
            print(f"Unknown argument: {arg}")
            print("Usage: python odrive_config.py [--apply|--read|--copy]")

    else:
        # Interactive mode
        interactive_menu(odrv)


if __name__ == "__main__":
    main()
