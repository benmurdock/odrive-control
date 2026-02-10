#!/usr/bin/env python3
"""
Exercise Machine GUI - V-Pulley Cable System

Controls an ODrive motor connected to a cable system with the following geometry:
- Two pulleys spaced 48" apart horizontally
- Both cables wind on a single 1.5" diameter spool
- Load attachment point moves vertically from 24" to 80" above pulleys

The GUI operates in load-side units:
- Force in pounds (lbs)
- Height in inches
- Motor stats shown for debugging

Control paradigm:
- Torque control mode with force conversion based on cable geometry
- When load is below "home height", motor applies constant vertical force
- When load is at or above home height, motor applies zero force (freewheel)

Calibration:
- Enter the current load height and click "Set" to sync with encoder
- The geometry module handles all motor <-> load unit conversions

Requirements:
pip install odrive PyQt5 pyqtgraph

Usage:
python exercise_gui.py
"""

import sys
import time
from collections import deque
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout,
                             QHBoxLayout, QLabel, QPushButton, QLineEdit,
                             QCheckBox, QFrame, QDoubleSpinBox)
from PyQt5.QtCore import QTimer, Qt, QThread, pyqtSignal
from PyQt5.QtGui import QDoubleValidator
import pyqtgraph as pg

import odrive
from odrive.enums import (AXIS_STATE_IDLE, AXIS_STATE_CLOSED_LOOP_CONTROL,
                          AXIS_STATE_FULL_CALIBRATION_SEQUENCE,
                          CONTROL_MODE_TORQUE_CONTROL)
from cable_geometry import CableGeometry

# ODrive error code descriptions (for firmware 0.5.x)
AXIS_ERRORS = {
    0x0001: "INVALID_STATE",
    0x0002: "WATCHDOG_TIMER_EXPIRED",
    0x0004: "MIN_ENDSTOP_PRESSED",
    0x0008: "MAX_ENDSTOP_PRESSED",
    0x0010: "ESTOP_REQUESTED",
    0x0020: "HOMING_WITHOUT_ENDSTOP",
    0x0040: "OVER_TEMP",
    0x0100: "UNKNOWN_POSITION",
}

MOTOR_ERRORS = {
    0x0001: "PHASE_RESISTANCE_OUT_OF_RANGE",
    0x0002: "PHASE_INDUCTANCE_OUT_OF_RANGE",
    0x0004: "DRV_FAULT",
    0x0008: "CONTROL_DEADLINE_MISSED",
    0x0010: "MODULATION_MAGNITUDE",
    0x0040: "CURRENT_SENSE_SATURATION",
    0x0100: "CURRENT_LIMIT_VIOLATION",
    0x0200: "MODULATION_IS_NAN",
    0x0400: "MOTOR_THERMISTOR_OVER_TEMP",
    0x0800: "FET_THERMISTOR_OVER_TEMP",
    0x1000: "TIMER_UPDATE_MISSED",
    0x2000: "CURRENT_MEASUREMENT_UNAVAILABLE",
}

ENCODER_ERRORS = {
    0x0001: "UNSTABLE_GAIN",
    0x0002: "CPR_POLEPAIRS_MISMATCH",
    0x0004: "NO_RESPONSE",
    0x0008: "UNSUPPORTED_ENCODER_MODE",
    0x0010: "ILLEGAL_HALL_STATE",
    0x0020: "INDEX_NOT_FOUND_YET",
    0x0040: "ABS_SPI_TIMEOUT",
    0x0080: "ABS_SPI_COM_FAIL",
    0x0100: "ABS_SPI_NOT_READY",
}

CONTROLLER_ERRORS = {
    0x0001: "OVERSPEED",
    0x0002: "INVALID_INPUT_MODE",
    0x0004: "UNSTABLE_GAIN",
    0x0008: "INVALID_MIRROR_AXIS",
    0x0010: "INVALID_LOAD_ENCODER",
    0x0020: "INVALID_ESTIMATE",
}


def decode_errors(error_code, error_dict):
    """Convert error code bitmask to list of error names"""
    errors = []
    for bit, name in error_dict.items():
        if error_code & bit:
            errors.append(name)
    return errors


class ExerciseReader(QThread):
    """Background thread for ODrive communication in exercise mode"""
    # Signal: motor_pos (turns), motor_vel (turns/s), current Iq (A), voltage (V), loop_time (ms), axis0_state (int), axis1_state (int), load_height (in)
    data_received = pyqtSignal(float, float, float, float, float, int, int, float)
    # Signal: axis_error, motor_error, encoder_error, controller_error
    errors_received = pyqtSignal(int, int, int, int)
    connection_lost = pyqtSignal()

    def __init__(self, odrv, axis, axis1, geometry):
        super().__init__()
        self.odrv = odrv
        self.axis = axis
        self.axis1 = axis1
        self.geometry = geometry
        self.running = True

        # Home height - the "floor" where the weight rests (in inches)
        self.home_height = 24.0  # inches (at pulley level minimum)

        # Load force to apply (simulated weight) in pounds
        self.load_force_lbs = 5.0  # lbs - force applied when below home height

        self.requested_state = None
        self.requested_state_axis1 = None
        self.enabled = False
        self.clear_errors_requested = False

    def set_home_height(self, height_inches):
        """Thread-safe way to update home height in inches"""
        self.home_height = height_inches

    def set_load_force(self, force_lbs):
        """Thread-safe way to update load force in pounds"""
        print(f"[DEBUG] Load force updated to {force_lbs:.1f} lbs")
        self.load_force_lbs = force_lbs


    def request_state(self, state):
        """Thread-safe way to request a state change for axis0"""
        print(f"[DEBUG] Requesting axis0 state: {state}")
        self.requested_state = state
        if state == AXIS_STATE_CLOSED_LOOP_CONTROL:
            self.enabled = True
        elif state == AXIS_STATE_IDLE:
            self.enabled = False

    def request_state_axis1(self, state):
        """Thread-safe way to request a state change for axis1"""
        print(f"[DEBUG] Requesting axis1 state: {state}")
        self.requested_state_axis1 = state

    def request_clear_errors(self):
        """Thread-safe way to request error clearing"""
        print("[DEBUG] Requesting clear errors")
        self.clear_errors_requested = True

    def run(self):
        """Fast polling loop for exercise mode"""
        print("[DEBUG] ExerciseReader thread started")
        error_check_counter = 0
        gui_update_counter = 0

        while self.running:
            loop_start = time.perf_counter()

            try:
                # Handle pending state change for axis0
                if self.requested_state is not None:
                    print(f"[DEBUG] Applying axis0 state change: {self.requested_state}")
                    self.axis.requested_state = self.requested_state
                    self.requested_state = None
                    print("[DEBUG] Axis0 state change applied")

                # Handle pending state change for axis1
                if self.requested_state_axis1 is not None:
                    print(f"[DEBUG] Applying axis1 state change: {self.requested_state_axis1}")
                    self.axis1.requested_state = self.requested_state_axis1
                    self.requested_state_axis1 = None
                    print("[DEBUG] Axis1 state change applied")

                # Handle clear errors
                if self.clear_errors_requested:
                    print("[DEBUG] Clearing errors")
                    self.axis.error = 0
                    self.axis.motor.error = 0
                    self.axis.encoder.error = 0
                    self.axis.controller.error = 0
                    self.clear_errors_requested = False
                    print("[DEBUG] Errors cleared")

                # Read current state for both axes
                current_state = self.axis.current_state
                current_state_axis1 = self.axis1.current_state

                # During calibration, don't poll - just wait
                # Check if either axis is calibrating
                axis0_busy = current_state not in (AXIS_STATE_IDLE, AXIS_STATE_CLOSED_LOOP_CONTROL)
                axis1_busy = current_state_axis1 not in (AXIS_STATE_IDLE, AXIS_STATE_CLOSED_LOOP_CONTROL)
                if axis0_busy or axis1_busy:
                    # Still emit state updates during calibration so GUI can track progress
                    self.data_received.emit(0, 0, 0, self.odrv.vbus_voltage,
                                            0, current_state, current_state_axis1, 24.0)
                    time.sleep(0.5)  # Check state twice per second during calibration
                    continue

                # Normal operation - read all values
                motor_position = self.axis.encoder.pos_estimate
                motor_velocity = self.axis.encoder.vel_estimate
                current_iq = self.axis.motor.current_control.Iq_measured
                voltage = self.odrv.vbus_voltage

                # Convert motor position to load height using geometry
                load_height = self.geometry.motor_turns_to_height(motor_position)

                # Apply torque control when in closed loop
                if current_state == AXIS_STATE_CLOSED_LOOP_CONTROL:
                    # Apply force when below home height, zero when at or above
                    if load_height < self.home_height:
                        # Convert desired load force (lbs) to motor torque (Nm)
                        torque_cmd = self.geometry.load_force_to_motor_torque(
                            self.load_force_lbs, load_height)
                    else:
                        torque_cmd = 0.0

                    self.axis.controller.input_torque = torque_cmd

                    # Debug: print command periodically (every ~1 second)
                    if gui_update_counter == 0:
                        print(f"[DEBUG] height={load_height:.1f}in, home={self.home_height:.1f}in, "
                              f"force={self.load_force_lbs:.1f}lbs, torque={torque_cmd:.3f}Nm")

                # Throttle GUI updates to ~50Hz
                gui_update_counter += 1
                if gui_update_counter >= 20:
                    gui_update_counter = 0
                    loop_time = (time.perf_counter() - loop_start) * 1000
                    self.data_received.emit(motor_position, motor_velocity, current_iq, voltage,
                                            loop_time, current_state, current_state_axis1, load_height)

                # Check errors less frequently
                error_check_counter += 1
                if error_check_counter >= 100:
                    error_check_counter = 0
                    self.errors_received.emit(
                        self.axis.error,
                        self.axis.motor.error,
                        self.axis.encoder.error,
                        self.axis.controller.error
                    )

            except Exception as e:
                print(f"[DEBUG] ODrive read error: {e}")
                self.connection_lost.emit()
                break

            time.sleep(0.001)

        print("[DEBUG] ExerciseReader thread stopped")

    def stop(self):
        print("[DEBUG] ExerciseReader stop requested")
        self.running = False


class ExerciseGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Exercise Machine - Cable System")
        self.setGeometry(100, 100, 1000, 800)

        # Cable geometry for unit conversions
        self.geometry = CableGeometry(motor_direction=1)

        # ODrive connection
        self.odrv = None
        self.axis = None
        self.axis1 = None
        self.reader_thread = None

        # Data buffers for plotting
        self.max_points = 1000
        self.time_data = deque(maxlen=self.max_points)
        self.height_data = deque(maxlen=self.max_points)  # Load height in inches
        self.current_data = deque(maxlen=self.max_points)
        self.start_time = None

        # Motor state (raw values from ODrive)
        self.motor_position = 0.0  # turns
        self.motor_velocity = 0.0  # turns/sec
        self.current_iq = 0.0  # amps
        self.voltage = 0.0  # volts

        # Load state (converted via geometry)
        self.load_height = 24.0  # inches
        self.home_height = 24.0  # inches - where the "weight" wants to return to
        self.load_force_lbs = 5.0  # lbs - simulated weight force

        # Error tracking
        self.has_errors = False
        self.calibrating = False
        self.calibrating_axis1 = False
        self.current_state = AXIS_STATE_IDLE

        self.init_ui()

        # Plot update timer
        self.plot_timer = QTimer()
        self.plot_timer.timeout.connect(self.update_plots)

    def init_ui(self):
        # Dark theme
        self.setStyleSheet("""
            QMainWindow, QWidget {
                background-color: #1e1e1e;
                color: #e0e0e0;
                font-family: 'Segoe UI', Arial, sans-serif;
                font-size: 11pt;
            }
            QLabel { color: #e0e0e0; padding: 2px; }
            QPushButton {
                background-color: #2d2d2d;
                color: #e0e0e0;
                border: 1px solid #3d3d3d;
                border-radius: 6px;
                padding: 8px 16px;
            }
            QPushButton:hover { background-color: #3d3d3d; }
            QPushButton:pressed { background-color: #252525; }
            QPushButton#connectButton {
                background-color: #0d7377;
                border: 1px solid #14b8a6;
            }
            QPushButton#connectButton:hover { background-color: #14b8a6; }
            QPushButton#stopButton {
                background-color: #dc2626;
                border: 1px solid #ef4444;
                font-weight: bold;
                font-size: 14pt;
                padding: 12px 24px;
            }
            QPushButton#stopButton:hover { background-color: #ef4444; }
            QPushButton#zeroButton {
                background-color: #7c3aed;
                border: 1px solid #8b5cf6;
            }
            QPushButton#zeroButton:hover { background-color: #8b5cf6; }
            QLabel#sectionHeader {
                font-size: 13pt;
                font-weight: 600;
                color: #14b8a6;
                padding: 8px 0px;
            }
            QLabel#valueDisplay {
                font-size: 18pt;
                font-weight: 600;
                color: #14b8a6;
            }
            QLineEdit, QDoubleSpinBox {
                background-color: #2d2d2d;
                color: #e0e0e0;
                border: 1px solid #3d3d3d;
                border-radius: 6px;
                padding: 8px 12px;
                font-size: 14pt;
            }
            QCheckBox { color: #e0e0e0; }
            QCheckBox::indicator {
                width: 18px;
                height: 18px;
            }
        """)

        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)
        main_layout.setSpacing(15)
        main_layout.setContentsMargins(20, 20, 20, 20)

        # === Connection Panel ===
        connection_panel = QWidget()
        connection_panel.setStyleSheet("background-color: #252525; border-radius: 10px;")
        connection_layout = QHBoxLayout(connection_panel)
        connection_layout.setContentsMargins(15, 10, 15, 10)

        connection_label = QLabel("ODrive Connection")
        connection_label.setObjectName("sectionHeader")
        connection_layout.addWidget(connection_label)

        self.connect_btn = QPushButton("Connect")
        self.connect_btn.setObjectName("connectButton")
        self.connect_btn.clicked.connect(self.toggle_connection)
        connection_layout.addWidget(self.connect_btn)

        self.status_label = QLabel("Disconnected")
        self.status_label.setStyleSheet("color: #ef4444; font-weight: bold;")
        connection_layout.addWidget(self.status_label)

        connection_layout.addStretch()

        self.voltage_label = QLabel("-- V")
        self.voltage_label.setStyleSheet("color: #a0a0a0;")
        connection_layout.addWidget(self.voltage_label)

        main_layout.addWidget(connection_panel)

        # === Error Panel ===
        self.error_panel = QWidget()
        self.error_panel.setStyleSheet("background-color: #1a4d1a; border-radius: 10px;")
        error_panel_layout = QHBoxLayout(self.error_panel)
        error_panel_layout.setContentsMargins(15, 8, 15, 8)

        self.error_status_label = QLabel("No Errors")
        self.error_status_label.setStyleSheet("color: #4ade80; font-weight: bold;")
        error_panel_layout.addWidget(self.error_status_label)

        error_panel_layout.addStretch()

        self.clear_errors_btn = QPushButton("Clear Errors")
        self.clear_errors_btn.setStyleSheet("background-color: #7c3aed; border: 1px solid #8b5cf6;")
        self.clear_errors_btn.clicked.connect(self.clear_errors)
        self.clear_errors_btn.setEnabled(False)
        self.clear_errors_btn.setVisible(False)
        error_panel_layout.addWidget(self.clear_errors_btn)

        main_layout.addWidget(self.error_panel)

        # Error details panel
        self.error_details_panel = QWidget()
        self.error_details_panel.setStyleSheet("background-color: #3b1a1a; border-radius: 10px;")
        self.error_details_panel.setVisible(False)
        error_details_layout = QVBoxLayout(self.error_details_panel)
        error_details_layout.setContentsMargins(15, 10, 15, 10)

        self.error_detail_label = QLabel("")
        self.error_detail_label.setStyleSheet("color: #fca5a5;")
        self.error_detail_label.setWordWrap(True)
        error_details_layout.addWidget(self.error_detail_label)

        main_layout.addWidget(self.error_details_panel)

        # === Control Panel ===
        control_panel = QWidget()
        control_panel.setStyleSheet("background-color: #252525; border-radius: 10px;")
        control_layout = QVBoxLayout(control_panel)
        control_layout.setContentsMargins(15, 15, 15, 15)
        control_layout.setSpacing(15)

        # Header row with title and stop button
        header_layout = QHBoxLayout()

        header_label = QLabel("Exercise Control")
        header_label.setObjectName("sectionHeader")
        header_layout.addWidget(header_label)

        header_layout.addStretch()

        self.status_msg_label = QLabel("")
        self.status_msg_label.setStyleSheet("color: #f59e0b;")
        header_layout.addWidget(self.status_msg_label)

        header_layout.addStretch()

        self.stop_btn = QPushButton("STOP")
        self.stop_btn.setObjectName("stopButton")
        self.stop_btn.clicked.connect(self.emergency_stop)
        header_layout.addWidget(self.stop_btn)

        control_layout.addLayout(header_layout)

        # Axis control row
        axis_control_layout = QHBoxLayout()

        # Axis 0 controls
        axis0_label = QLabel("Axis 0:")
        axis0_label.setStyleSheet("font-weight: bold; color: #14b8a6;")
        axis_control_layout.addWidget(axis0_label)

        self.enable_checkbox = QCheckBox("Enable")
        self.enable_checkbox.setChecked(False)
        self.enable_checkbox.stateChanged.connect(self.on_enable_changed)
        self.enable_checkbox.setEnabled(False)
        axis_control_layout.addWidget(self.enable_checkbox)

        self.calibrate_btn = QPushButton("Calibrate")
        self.calibrate_btn.setStyleSheet("background-color: #2563eb; border: 1px solid #3b82f6;")
        self.calibrate_btn.clicked.connect(self.run_calibration)
        self.calibrate_btn.setEnabled(False)
        axis_control_layout.addWidget(self.calibrate_btn)

        axis_control_layout.addSpacing(40)

        # Axis 1 controls
        axis1_label = QLabel("Axis 1:")
        axis1_label.setStyleSheet("font-weight: bold; color: #f59e0b;")
        axis_control_layout.addWidget(axis1_label)

        self.enable_checkbox_axis1 = QCheckBox("Enable")
        self.enable_checkbox_axis1.setChecked(False)
        self.enable_checkbox_axis1.stateChanged.connect(self.on_enable_changed_axis1)
        self.enable_checkbox_axis1.setEnabled(False)
        axis_control_layout.addWidget(self.enable_checkbox_axis1)

        self.calibrate_btn_axis1 = QPushButton("Calibrate")
        self.calibrate_btn_axis1.setStyleSheet("background-color: #2563eb; border: 1px solid #3b82f6;")
        self.calibrate_btn_axis1.clicked.connect(self.run_calibration_axis1)
        self.calibrate_btn_axis1.setEnabled(False)
        axis_control_layout.addWidget(self.calibrate_btn_axis1)

        axis_control_layout.addStretch()

        control_layout.addLayout(axis_control_layout)

        # Separator
        separator1 = QFrame()
        separator1.setFrameShape(QFrame.HLine)
        separator1.setStyleSheet("background-color: #3d3d3d;")
        control_layout.addWidget(separator1)

        # Height calibration row
        height_cal_layout = QHBoxLayout()

        height_cal_label = QLabel("Current Height:")
        height_cal_label.setStyleSheet("font-size: 12pt;")
        height_cal_layout.addWidget(height_cal_label)

        self.height_input = QLineEdit()
        self.height_input.setPlaceholderText("inches")
        self.height_input.setFixedWidth(80)
        self.height_input.setValidator(QDoubleValidator(24.0, 80.0, 1))
        height_cal_layout.addWidget(self.height_input)

        self.set_height_btn = QPushButton("Set")
        self.set_height_btn.setStyleSheet("background-color: #2563eb; border: 1px solid #3b82f6;")
        self.set_height_btn.clicked.connect(self.on_set_height)
        self.set_height_btn.setEnabled(False)
        height_cal_layout.addWidget(self.set_height_btn)

        height_cal_layout.addSpacing(30)

        # Load force setting
        force_label = QLabel("Load Force:")
        force_label.setStyleSheet("font-size: 12pt;")
        height_cal_layout.addWidget(force_label)

        self.force_input = QDoubleSpinBox()
        self.force_input.setRange(0.5, 100.0)
        self.force_input.setValue(5.0)
        self.force_input.setSingleStep(1.0)
        self.force_input.setDecimals(1)
        self.force_input.setSuffix(" lbs")
        self.force_input.setFixedWidth(100)
        self.force_input.valueChanged.connect(self.on_force_changed)
        height_cal_layout.addWidget(self.force_input)

        height_cal_layout.addStretch()

        # Home height button
        self.set_home_btn = QPushButton("Set Home Height")
        self.set_home_btn.setObjectName("zeroButton")
        self.set_home_btn.clicked.connect(self.set_home_position)
        self.set_home_btn.setEnabled(False)
        height_cal_layout.addWidget(self.set_home_btn)

        control_layout.addLayout(height_cal_layout)

        # Separator
        separator2 = QFrame()
        separator2.setFrameShape(QFrame.HLine)
        separator2.setStyleSheet("background-color: #3d3d3d;")
        control_layout.addWidget(separator2)

        # Measured values display - Load side
        measured_layout = QHBoxLayout()

        # Load Height display
        height_container = QVBoxLayout()
        height_label = QLabel("Load Height")
        height_label.setStyleSheet("color: #a0a0a0;")
        height_label.setAlignment(Qt.AlignCenter)
        height_container.addWidget(height_label)

        self.height_display = QLabel("24.0")
        self.height_display.setObjectName("valueDisplay")
        self.height_display.setStyleSheet("color: #f59e0b; font-size: 24pt;")
        self.height_display.setAlignment(Qt.AlignCenter)
        height_container.addWidget(self.height_display)

        height_unit = QLabel("inches")
        height_unit.setStyleSheet("color: #a0a0a0;")
        height_unit.setAlignment(Qt.AlignCenter)
        height_container.addWidget(height_unit)

        measured_layout.addLayout(height_container)

        measured_layout.addStretch()

        # Current display (motor side, but useful feedback)
        current_container = QVBoxLayout()
        current_label = QLabel("Motor Current")
        current_label.setStyleSheet("color: #a0a0a0;")
        current_label.setAlignment(Qt.AlignCenter)
        current_container.addWidget(current_label)

        self.current_display = QLabel("0.00")
        self.current_display.setObjectName("valueDisplay")
        self.current_display.setStyleSheet("color: #10b981; font-size: 24pt;")
        self.current_display.setAlignment(Qt.AlignCenter)
        current_container.addWidget(self.current_display)

        current_unit = QLabel("Amps")
        current_unit.setStyleSheet("color: #a0a0a0;")
        current_unit.setAlignment(Qt.AlignCenter)
        current_container.addWidget(current_unit)

        measured_layout.addLayout(current_container)

        measured_layout.addStretch()

        # Home height display
        home_container = QVBoxLayout()
        home_label = QLabel("Home Height")
        home_label.setStyleSheet("color: #a0a0a0;")
        home_label.setAlignment(Qt.AlignCenter)
        home_container.addWidget(home_label)

        self.home_display = QLabel("24.0")
        self.home_display.setObjectName("valueDisplay")
        self.home_display.setStyleSheet("color: #8b5cf6; font-size: 24pt;")
        self.home_display.setAlignment(Qt.AlignCenter)
        home_container.addWidget(self.home_display)

        home_unit = QLabel("inches")
        home_unit.setStyleSheet("color: #a0a0a0;")
        home_unit.setAlignment(Qt.AlignCenter)
        home_container.addWidget(home_unit)

        measured_layout.addLayout(home_container)

        control_layout.addLayout(measured_layout)

        # Separator for motor stats
        separator3 = QFrame()
        separator3.setFrameShape(QFrame.HLine)
        separator3.setStyleSheet("background-color: #3d3d3d;")
        control_layout.addWidget(separator3)

        # Motor stats (raw values for debugging)
        motor_stats_layout = QHBoxLayout()
        motor_stats_label = QLabel("Motor Stats:")
        motor_stats_label.setStyleSheet("color: #666666; font-size: 10pt;")
        motor_stats_layout.addWidget(motor_stats_label)

        self.motor_pos_label = QLabel("Pos: 0.00 turns")
        self.motor_pos_label.setStyleSheet("color: #666666; font-size: 10pt;")
        motor_stats_layout.addWidget(self.motor_pos_label)

        self.motor_vel_label = QLabel("Vel: 0.0 t/s")
        self.motor_vel_label.setStyleSheet("color: #666666; font-size: 10pt;")
        motor_stats_layout.addWidget(self.motor_vel_label)

        self.mech_adv_label = QLabel("MA: 1.00")
        self.mech_adv_label.setStyleSheet("color: #666666; font-size: 10pt;")
        motor_stats_layout.addWidget(self.mech_adv_label)

        motor_stats_layout.addStretch()

        control_layout.addLayout(motor_stats_layout)

        main_layout.addWidget(control_panel)

        # === Plot Section ===
        pg.setConfigOptions(antialias=True)
        pg.setConfigOption('background', '#1e1e1e')
        pg.setConfigOption('foreground', '#e0e0e0')

        self.plot_widget = pg.GraphicsLayoutWidget()
        self.plot_widget.setBackground('#1e1e1e')
        main_layout.addWidget(self.plot_widget)

        # Height plot
        self.height_plot = self.plot_widget.addPlot(
            title="<span style='color: #e0e0e0;'>Load Height</span>")
        self.height_plot.setLabel('left', 'Height', units='inches')
        self.height_plot.setLabel('bottom', 'Time', units='s')
        self.height_plot.showGrid(x=True, y=True, alpha=0.3)
        self.height_curve = self.height_plot.plot(pen=pg.mkPen('#f59e0b', width=2))

        # Current plot
        self.plot_widget.nextRow()
        self.current_plot = self.plot_widget.addPlot(
            title="<span style='color: #e0e0e0;'>Motor Current (Force)</span>")
        self.current_plot.setLabel('left', 'Current', units='A')
        self.current_plot.setLabel('bottom', 'Time', units='s')
        self.current_plot.showGrid(x=True, y=True, alpha=0.3)
        self.current_curve = self.current_plot.plot(pen=pg.mkPen('#10b981', width=2))

    def toggle_connection(self):
        if self.odrv is not None:
            self.disconnect_odrive()
        else:
            self.connect_odrive()

    def connect_odrive(self):
        self.status_label.setText("Searching...")
        self.status_label.setStyleSheet("color: #f59e0b; font-weight: bold;")
        QApplication.processEvents()

        try:
            self.odrv = odrive.find_any(timeout=10)
            self.axis = self.odrv.axis0
            self.axis1 = self.odrv.axis1

            # Set both axes to idle state initially
            self.axis.requested_state = AXIS_STATE_IDLE
            self.axis1.requested_state = AXIS_STATE_IDLE

            # Note: control_mode and input_mode are set when enabling the motor
            # to avoid interfering with calibration

            serial = hex(self.odrv.serial_number).upper()
            self.status_label.setText(f"Connected: {serial}")
            self.status_label.setStyleSheet("color: #14b8a6; font-weight: bold;")
            self.connect_btn.setText("Disconnect")
            self.enable_checkbox.setEnabled(True)
            self.calibrate_btn.setEnabled(True)
            self.enable_checkbox_axis1.setEnabled(True)
            self.calibrate_btn_axis1.setEnabled(True)
            self.set_height_btn.setEnabled(True)
            self.set_home_btn.setEnabled(True)
            self.clear_errors_btn.setEnabled(True)

            # Start reader thread with geometry
            self.reader_thread = ExerciseReader(self.odrv, self.axis, self.axis1, self.geometry)
            self.reader_thread.load_force_lbs = self.load_force_lbs
            self.reader_thread.home_height = self.home_height
            self.reader_thread.data_received.connect(self.on_data_received)
            self.reader_thread.errors_received.connect(self.on_errors_received)
            self.reader_thread.connection_lost.connect(self.on_connection_lost)
            self.reader_thread.start()

            # Start plot updates
            self.start_time = time.time()
            self.plot_timer.start(50)

        except Exception as e:
            self.status_label.setText(f"Error: {str(e)[:30]}")
            self.status_label.setStyleSheet("color: #ef4444; font-weight: bold;")
            self.odrv = None

    def disconnect_odrive(self):
        self.enable_checkbox.setChecked(False)

        if self.reader_thread:
            self.reader_thread.stop()
            self.reader_thread.wait()
            self.reader_thread = None

        self.plot_timer.stop()
        self.odrv = None
        self.axis = None
        self.axis1 = None

        self.status_label.setText("Disconnected")
        self.status_label.setStyleSheet("color: #ef4444; font-weight: bold;")
        self.connect_btn.setText("Connect")
        self.enable_checkbox.setEnabled(False)
        self.enable_checkbox.setChecked(False)
        self.calibrate_btn.setEnabled(False)
        self.enable_checkbox_axis1.setEnabled(False)
        self.enable_checkbox_axis1.setChecked(False)
        self.calibrate_btn_axis1.setEnabled(False)
        self.set_height_btn.setEnabled(False)
        self.set_home_btn.setEnabled(False)
        self.clear_errors_btn.setEnabled(False)
        self.clear_errors_btn.setVisible(False)
        self.status_msg_label.setText("")
        self.update_error_display(0, 0, 0, 0)

    def on_connection_lost(self):
        self.disconnect_odrive()
        self.status_label.setText("Connection Lost!")

    def on_enable_changed(self, state):
        if self.reader_thread is None:
            return

        if state == Qt.Checked:
            # Configure for torque control before entering closed loop
            self.axis.controller.config.control_mode = CONTROL_MODE_TORQUE_CONTROL
            self.axis.controller.config.input_mode = 1  # INPUT_MODE_PASSTHROUGH
            self.reader_thread.request_state(AXIS_STATE_CLOSED_LOOP_CONTROL)
        else:
            self.reader_thread.request_state(AXIS_STATE_IDLE)

    def on_enable_changed_axis1(self, state):
        """Enable/disable axis1 - stub for now"""
        # TODO: Implement axis1 enable/disable
        if state == Qt.Checked:
            self.status_msg_label.setText("Axis 1 enable not yet implemented")
            self.status_msg_label.setStyleSheet("color: #f59e0b;")
            self.enable_checkbox_axis1.setChecked(False)
        pass

    def on_force_changed(self, value):
        """Update the load force in pounds"""
        self.load_force_lbs = value
        if self.reader_thread:
            self.reader_thread.set_load_force(value)

    def on_set_height(self):
        """Set the current height calibration from user input"""
        try:
            height = float(self.height_input.text())
            if 24.0 <= height <= 80.0:
                # Update geometry reference with current motor position
                self.geometry.set_reference(height, self.motor_position)
                self.load_height = height
                self.height_display.setText(f"{height:.1f}")
                self.status_msg_label.setText(f"Height calibrated to {height:.1f} inches")
                self.status_msg_label.setStyleSheet("color: #14b8a6;")
                self.height_input.clear()
            else:
                self.status_msg_label.setText("Height must be 24-80 inches")
                self.status_msg_label.setStyleSheet("color: #ef4444;")
        except ValueError:
            self.status_msg_label.setText("Enter a valid number")
            self.status_msg_label.setStyleSheet("color: #ef4444;")

    def set_home_position(self):
        """Set current height as home (floor) height"""
        self.home_height = self.load_height
        self.home_display.setText(f"{self.home_height:.1f}")

        if self.reader_thread:
            self.reader_thread.set_home_height(self.home_height)

        self.status_msg_label.setText(f"Home set to {self.home_height:.1f} inches")
        self.status_msg_label.setStyleSheet("color: #8b5cf6;")

    def emergency_stop(self):
        self.enable_checkbox.setChecked(False)
        self.enable_checkbox_axis1.setChecked(False)

        if self.reader_thread:
            self.reader_thread.request_state(AXIS_STATE_IDLE)
        # TODO: Also stop axis1 when implemented

    def run_calibration(self):
        if self.reader_thread is None:
            return

        self.enable_checkbox.setChecked(False)
        self.calibrate_btn.setEnabled(False)
        self.calibrate_btn.setText("Calibrating...")
        self.calibrating = True

        self.reader_thread.request_state(AXIS_STATE_FULL_CALIBRATION_SEQUENCE)
        self.status_msg_label.setText("Axis 0 calibrating...")
        self.status_msg_label.setStyleSheet("color: #f59e0b;")

    def run_calibration_axis1(self):
        """Run calibration sequence for axis1"""
        if self.reader_thread is None:
            return

        self.enable_checkbox_axis1.setChecked(False)
        self.calibrate_btn_axis1.setEnabled(False)
        self.calibrate_btn_axis1.setText("Calibrating...")
        self.calibrating_axis1 = True

        self.reader_thread.request_state_axis1(AXIS_STATE_FULL_CALIBRATION_SEQUENCE)
        self.status_msg_label.setText("Axis 1 calibrating...")
        self.status_msg_label.setStyleSheet("color: #f59e0b;")

    def clear_errors(self):
        if self.reader_thread is None:
            return

        self.enable_checkbox.setChecked(False)
        self.reader_thread.request_clear_errors()
        self.status_msg_label.setText("Errors cleared - recalibrate!")
        self.status_msg_label.setStyleSheet("color: #f59e0b;")

    def on_data_received(self, motor_pos, motor_vel, current_iq, voltage, loop_time, current_state, current_state_axis1, load_height):
        # Store motor values
        self.motor_position = motor_pos
        self.motor_velocity = motor_vel
        self.current_iq = current_iq
        self.voltage = voltage
        self.current_state = current_state

        # Store load values
        self.load_height = load_height

        # Update load displays
        self.height_display.setText(f"{load_height:.1f}")
        self.current_display.setText(f"{abs(current_iq):.2f}")
        self.voltage_label.setText(f"{voltage:.1f} V")

        # Update motor stats
        self.motor_pos_label.setText(f"Pos: {motor_pos:.2f} turns")
        self.motor_vel_label.setText(f"Vel: {motor_vel:.1f} t/s")
        mech_adv = self.geometry.get_mechanical_advantage(load_height)
        self.mech_adv_label.setText(f"MA: {mech_adv:.2f}")

        # Check axis0 calibration status
        if self.calibrating and current_state == AXIS_STATE_IDLE:
            self.calibrating = False
            self.calibrate_btn.setText("Calibrate")
            self.calibrate_btn.setEnabled(True)

            if self.has_errors:
                self.status_msg_label.setText("Axis 0 calibration failed!")
                self.status_msg_label.setStyleSheet("color: #ef4444;")
            else:
                self.status_msg_label.setText("Axis 0 calibration complete!")
                self.status_msg_label.setStyleSheet("color: #14b8a6;")

        # Check axis1 calibration status
        if self.calibrating_axis1 and current_state_axis1 == AXIS_STATE_IDLE:
            self.calibrating_axis1 = False
            self.calibrate_btn_axis1.setText("Calibrate")
            self.calibrate_btn_axis1.setEnabled(True)

            # TODO: Check axis1-specific errors when error handling is extended
            self.status_msg_label.setText("Axis 1 calibration complete!")
            self.status_msg_label.setStyleSheet("color: #14b8a6;")

        # Store for plotting
        if self.start_time:
            t = time.time() - self.start_time
            self.time_data.append(t)
            self.height_data.append(load_height)
            self.current_data.append(current_iq)

    def on_errors_received(self, axis_error, motor_error, encoder_error, controller_error):
        self.update_error_display(axis_error, motor_error, encoder_error, controller_error)

    def update_error_display(self, axis_error, motor_error, encoder_error, controller_error):
        has_errors = (axis_error != 0 or motor_error != 0 or
                      encoder_error != 0 or controller_error != 0)

        if has_errors:
            self.has_errors = True
            self.error_panel.setStyleSheet("background-color: #4a1a1a; border-radius: 10px;")
            self.error_status_label.setText("ERRORS DETECTED")
            self.error_status_label.setStyleSheet("color: #ef4444; font-weight: bold;")
            self.clear_errors_btn.setVisible(True)
            self.clear_errors_btn.setEnabled(True)
            self.error_details_panel.setVisible(True)

            # Collect all errors
            all_errors = []
            if axis_error:
                errors = decode_errors(axis_error, AXIS_ERRORS)
                all_errors.extend([f"Axis: {e}" for e in errors])
            if motor_error:
                errors = decode_errors(motor_error, MOTOR_ERRORS)
                all_errors.extend([f"Motor: {e}" for e in errors])
            if encoder_error:
                errors = decode_errors(encoder_error, ENCODER_ERRORS)
                all_errors.extend([f"Encoder: {e}" for e in errors])
            if controller_error:
                errors = decode_errors(controller_error, CONTROLLER_ERRORS)
                all_errors.extend([f"Controller: {e}" for e in errors])

            self.error_detail_label.setText(", ".join(all_errors) if all_errors else "Unknown error")
        else:
            self.has_errors = False
            self.error_panel.setStyleSheet("background-color: #1a4d1a; border-radius: 10px;")
            self.error_status_label.setText("No Errors")
            self.error_status_label.setStyleSheet("color: #4ade80; font-weight: bold;")
            self.clear_errors_btn.setVisible(False)
            self.error_details_panel.setVisible(False)

    def update_plots(self):
        if len(self.time_data) < 2:
            return

        time_list = list(self.time_data)
        current_time = time_list[-1]

        self.height_curve.setData(time_list, list(self.height_data))
        self.current_curve.setData(time_list, list(self.current_data))

        # Auto-range X to show last 10 seconds
        x_range = [max(0, current_time - 10), current_time]
        self.height_plot.setXRange(*x_range, padding=0)
        self.current_plot.setXRange(*x_range, padding=0)

    def closeEvent(self, event):
        self.emergency_stop()
        if self.reader_thread:
            self.reader_thread.stop()
            self.reader_thread.wait()
        event.accept()


def main():
    app = QApplication(sys.argv)
    gui = ExerciseGUI()
    gui.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()