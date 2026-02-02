"""
Cable Geometry Module

Handles conversions between motor-side units and load-side units for a V-pulley
cable system where two cables run from a single spool to pulleys, then meet at
a load attachment point.

Physical setup:
    - Two pulleys spaced 48" apart (24" from center each)
    - Load point moves vertically from 24" to 80" above the pulleys
    - Both cables wind on a single 1.5" diameter spool
    - Motor encoder tracks spool rotation in turns

Coordinate conventions:
    - Height (h): vertical distance from pulley level to load point, in inches
    - Positive motor direction: configurable via motor_direction sign
    - Positive force: upward at load point
"""

import math


class CableGeometry:
    """Converts between motor units (turns, Nm) and load units (inches, lbs)."""

    # Physical constants
    HALF_PULLEY_SPACING = 24.0  # inches (half of 48" total spacing)
    SPOOL_DIAMETER = 1.5  # inches
    SPOOL_RADIUS_INCHES = SPOOL_DIAMETER / 2  # 0.75 inches
    SPOOL_RADIUS_METERS = SPOOL_RADIUS_INCHES * 0.0254  # ~0.01905 m
    SPOOL_CIRCUMFERENCE = math.pi * SPOOL_DIAMETER  # ~4.712 inches

    # Unit conversion constants
    NEWTONS_PER_LBF = 4.44822  # 1 pound-force = 4.448 N
    INCHES_TO_METERS = 0.0254

    # Height limits (for bounds checking)
    MIN_HEIGHT = 24.0  # inches
    MAX_HEIGHT = 80.0  # inches

    def __init__(self, motor_direction=1):
        """
        Initialize geometry with calibration defaults.

        Args:
            motor_direction: +1 or -1, determines which motor rotation direction
                           causes the load to rise. Default +1 means positive
                           motor turns = load rises.
        """
        # Calibration reference point
        self.reference_height = self.MIN_HEIGHT  # inches
        self.reference_motor_turns = 0.0

        # Motor direction sign (+1 or -1)
        self.motor_direction = motor_direction

    def set_reference(self, height_inches, motor_turns):
        """
        Set the calibration reference point.

        Call this when the user specifies the current load height.
        The system will use this to calculate heights from encoder readings.

        Args:
            height_inches: Current height of load above pulleys
            motor_turns: Current motor encoder reading in turns
        """
        self.reference_height = height_inches
        self.reference_motor_turns = motor_turns

    def _cable_length_at_height(self, h):
        """
        Calculate total cable length from both pulleys to load point.

        Args:
            h: Height in inches above pulley level

        Returns:
            Total cable length in inches (sum of both cable segments)
        """
        # Each cable: sqrt(24^2 + h^2), total is 2x that
        return 2.0 * math.sqrt(self.HALF_PULLEY_SPACING ** 2 + h ** 2)

    def _height_from_cable_length(self, cable_length):
        """
        Calculate height from total cable length.

        Args:
            cable_length: Total cable length in inches

        Returns:
            Height in inches above pulley level
        """
        # cable_length = 2 * sqrt(24^2 + h^2)
        # cable_length/2 = sqrt(576 + h^2)
        # (cable_length/2)^2 = 576 + h^2
        # h^2 = (cable_length/2)^2 - 576
        half_cable = cable_length / 2.0
        h_squared = half_cable ** 2 - self.HALF_PULLEY_SPACING ** 2

        if h_squared < 0:
            # Cable is too short - would be below pulley level
            # Clamp to minimum
            return self.MIN_HEIGHT

        return math.sqrt(h_squared)

    def _sin_theta(self, h):
        """
        Calculate sin(theta) where theta is cable angle from horizontal.

        This factor appears in both force and velocity conversions.
        At steeper angles (higher h), sin(theta) approaches 1.
        At shallower angles (lower h), sin(theta) is smaller.

        Args:
            h: Height in inches

        Returns:
            sin(theta) value between 0 and 1
        """
        return h / math.sqrt(self.HALF_PULLEY_SPACING ** 2 + h ** 2)

    def motor_turns_to_height(self, motor_turns):
        """
        Convert motor encoder position to load height.

        Args:
            motor_turns: Motor encoder reading in turns

        Returns:
            Height of load in inches above pulley level
        """
        # Calculate cable length at reference height
        ref_cable = self._cable_length_at_height(self.reference_height)

        # Calculate how much cable has changed since reference
        # Positive motor turns (in our convention) = more cable paid out = higher load
        turns_delta = (motor_turns - self.reference_motor_turns) * self.motor_direction
        cable_delta = turns_delta * self.SPOOL_CIRCUMFERENCE

        # Current cable length
        current_cable = ref_cable + cable_delta

        # Convert back to height
        height = self._height_from_cable_length(current_cable)

        return height

    def height_to_motor_turns(self, height_inches):
        """
        Convert desired load height to motor encoder position.

        Args:
            height_inches: Desired height in inches

        Returns:
            Motor encoder position in turns
        """
        ref_cable = self._cable_length_at_height(self.reference_height)
        target_cable = self._cable_length_at_height(height_inches)

        cable_delta = target_cable - ref_cable
        turns_delta = cable_delta / self.SPOOL_CIRCUMFERENCE

        return self.reference_motor_turns + (turns_delta * self.motor_direction)

    def motor_velocity_to_load_velocity(self, motor_vel_turns_per_sec, height_inches):
        """
        Convert motor velocity to load velocity.

        Args:
            motor_vel_turns_per_sec: Motor velocity in turns/sec
            height_inches: Current height (needed for angle calculation)

        Returns:
            Load velocity in inches/sec (positive = rising)
        """
        # Motor velocity -> cable velocity
        cable_vel = motor_vel_turns_per_sec * self.motor_direction * self.SPOOL_CIRCUMFERENCE

        # Cable velocity -> load velocity
        # d(cable)/dt = 2 * sin(theta) * d(height)/dt
        # So: d(height)/dt = d(cable)/dt / (2 * sin(theta))
        sin_theta = self._sin_theta(height_inches)

        if sin_theta < 0.1:
            # Avoid division by very small numbers at extreme angles
            sin_theta = 0.1

        load_vel = cable_vel / (2.0 * sin_theta)

        return load_vel

    def load_velocity_to_motor_velocity(self, load_vel_inches_per_sec, height_inches):
        """
        Convert desired load velocity to motor velocity.

        Args:
            load_vel_inches_per_sec: Desired load velocity in inches/sec
            height_inches: Current height (needed for angle calculation)

        Returns:
            Motor velocity in turns/sec
        """
        sin_theta = self._sin_theta(height_inches)

        # Load velocity -> cable velocity
        cable_vel = load_vel_inches_per_sec * 2.0 * sin_theta

        # Cable velocity -> motor velocity
        motor_vel = cable_vel / self.SPOOL_CIRCUMFERENCE * self.motor_direction

        return motor_vel

    def motor_torque_to_load_force(self, motor_torque_nm, height_inches):
        """
        Convert motor torque to vertical load force.

        Args:
            motor_torque_nm: Motor torque in Newton-meters
            height_inches: Current height (needed for angle calculation)

        Returns:
            Vertical force at load in pounds (positive = upward)
        """
        # Motor torque -> cable tension
        # T_cable = T_motor / spool_radius
        cable_tension_n = motor_torque_nm / self.SPOOL_RADIUS_METERS

        # Cable tension -> vertical force
        # F_vertical = 2 * T_cable * sin(theta)
        sin_theta = self._sin_theta(height_inches)
        vertical_force_n = 2.0 * cable_tension_n * sin_theta

        # Convert to pounds
        vertical_force_lbs = vertical_force_n / self.NEWTONS_PER_LBF

        return vertical_force_lbs

    def load_force_to_motor_torque(self, force_lbs, height_inches):
        """
        Convert desired vertical load force to motor torque.

        Args:
            force_lbs: Desired vertical force at load in pounds
            height_inches: Current height (needed for angle calculation)

        Returns:
            Required motor torque in Newton-meters
        """
        # Convert force to Newtons
        force_n = force_lbs * self.NEWTONS_PER_LBF

        # Vertical force -> cable tension
        # F_vertical = 2 * T_cable * sin(theta)
        # T_cable = F_vertical / (2 * sin(theta))
        sin_theta = self._sin_theta(height_inches)

        if sin_theta < 0.1:
            # Avoid division by very small numbers
            sin_theta = 0.1

        cable_tension_n = force_n / (2.0 * sin_theta)

        # Cable tension -> motor torque
        # T_motor = T_cable * spool_radius
        motor_torque_nm = cable_tension_n * self.SPOOL_RADIUS_METERS

        return motor_torque_nm

    def get_mechanical_advantage(self, height_inches):
        """
        Get the current mechanical advantage at a given height.

        This is useful for display/debugging. Higher values mean less
        motor effort needed per pound of load force.

        Args:
            height_inches: Current height

        Returns:
            Mechanical advantage ratio (dimensionless)
        """
        sin_theta = self._sin_theta(height_inches)
        # MA = 2 * sin(theta) * (spool_radius / 1 inch) approximately
        # But more directly: ratio of load force to cable tension
        return 2.0 * sin_theta
