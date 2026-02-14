# ODrive Control Project

## Overview
Python interface for ODrive v3.x motor controller with PyQt5 GUI for testing and development. Ultimate goal is position-dependent torque control with rapid communication.

## Hardware
- ODrive v3.x with two axes (axis0, axis1)
- USB Native protocol (direct Python object access, not ASCII serial)
- Encoder feedback for position/velocity
- Fedora Linux with udev rules for USB access

## Architecture

### Threading Model
All ODrive USB calls MUST happen in the background QThread, never from the GUI thread. Blocking USB calls from the GUI thread cause freezes.

**Pattern that works:**
```python
# GUI thread sets flags/values
self.reader_thread.request_state(AXIS_STATE_IDLE)
self.reader_thread.set_velocity(velocity)

# Background thread checks flags and executes
if self.requested_state is not None:
    self.axis.requested_state = self.requested_state
    self.requested_state = None
```

### Update Rates
- Control loop: ~1000 Hz (1ms sleep in thread)
- GUI updates: ~50 Hz (throttled via counter, every 20 loops)
- Error checks: ~10 Hz (every 100 loops)
- Plot updates: 20 Hz (QTimer)

High-frequency signal emissions overwhelm the Qt event queue - always throttle.

## Key Files
- `exercise_gui.py` - Main exercise machine GUI (load-side units: lbs, inches)
- `cable_geometry.py` - Geometry conversions between motor and load domains
- `odrive_gui.py` - Low-level motor test GUI (raw motor units)
- `main.py` - Simple connection test script
- `91-odrive.rules` - udev rules for USB permissions

## Cable Geometry

The system uses a V-pulley configuration:
```
        [Load point]
           /\
          /  \
         / h  \     h = 24" to 80" above pulleys
        /      \
   [P1]──48"──[P2]  Pulleys 24" from center each
       ↓       ↓
      (both cables to single 3" spool, 20:1 gearbox)
```

Key relationships (in `cable_geometry.py`):
- `sin(θ) = h / sqrt(576 + h²)` - cable angle factor
- Force: `motor_torque = load_force * spool_radius / (2 * sin(θ))`
- Position: `cable_length = 2 * sqrt(576 + h²)`

At lower heights, cables are at steeper angles → need more motor torque per lb.
At higher heights, cables are more vertical → need less motor torque per lb.

### Calibration
User enters current height in inches, clicks "Set" to sync encoder position.
The geometry module uses this reference to convert all positions.

### Motor Direction
`motor_direction` parameter in CableGeometry (+1 or -1) handles motor mounting direction.
When adding second motor (opposite mounting), set its direction to -1.

## Lessons Learned

### GUI Freezing
1. Never call `self.axis.*` from GUI callbacks - always route through background thread
2. Throttle signal emissions to prevent event queue overflow
3. Use request/flag pattern for thread-safe communication

### ODrive API
- `axis.requested_state` - set to change state (IDLE, CLOSED_LOOP_CONTROL, CALIBRATION)
- `axis.controller.input_vel` - velocity command (turns/sec)
- `axis.encoder.pos_estimate` - position (turns)
- `axis.encoder.vel_estimate` - velocity (turns/sec)
- Error codes are bitmasks - decode with bitwise AND

## Next Steps
- Add axis1 support (extend ExerciseReader to handle both axes)
- Add velocity/position control modes in load-side units
- Test force compensation at different heights

## User Notes
Ben is a "late beginner" programmer - explain changes and walk through code.
