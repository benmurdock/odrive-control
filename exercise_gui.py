#!/usr/bin/env python3
"""
Exercise Machine Simulator GUI

Simulates a weight on a cable using ODrive position control with current limiting.
The motor tries to return to a "home" position but is limited to a maximum current,
creating the feel of lifting and lowering a weight.

Control paradigm:
- Position control mode with input_pos = home position
- current_lim sets the maximum force (simulated weight)
- Motor resists movement up to current_lim, then allows backdriving

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
                          CONTROL_MODE_POSITION_CONTROL)

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
    # Signal: position (turns), velocity (turns/s), current Iq (A), voltage (V), loop_time (ms), current_state (int)
    data_received = pyqtSignal(float, float, float, float, float, int)
    # Signal: axis_error, motor_error, encoder_error, controller_error
    errors_received = pyqtSignal(int, int, int, int)
    connection_lost = pyqtSignal()

    def __init__(self, odrv, axis):
        super().__init__()
        self.odrv = odrv
        self.axis = axis
        self.running = True

        # Position control target (home position)
        self.target_position = 0.0  # turns

        # Current limit (simulated weight)
        self.current_limit = 0.5  # Amps
        self.current_limit_changed = False

        self.requested_state = None
        self.enabled = False
        self.clear_errors_requested = False

    def set_position(self, position):
        """Thread-safe way to update home position"""
        self.target_position = position

    def set_current_limit(self, limit):
        """Thread-safe way to update current limit (weight simulation)"""
        self.current_limit = limit
        self.current_limit_changed = True

    def request_state(self, state):
        """Thread-safe way to request a state change"""
        self.requested_state = state
        if state == AXIS_STATE_CLOSED_LOOP_CONTROL:
            self.enabled = True
        elif state == AXIS_STATE_IDLE:
            self.enabled = False

    def request_clear_errors(self):
        """Thread-safe way to request error clearing"""
        self.clear_errors_requested = True

    def run(self):
        """Fast polling loop for exercise mode"""
        error_check_counter = 0
        gui_update_counter = 0

        while self.running:
            loop_start = time.perf_counter()

            try:
                # Handle pending state change
                if self.requested_state is not None:
                    self.axis.requested_state = self.requested_state
                    self.requested_state = None

                # Handle clear errors
                if self.clear_errors_requested:
                    self.axis.error = 0
                    self.axis.motor.error = 0
                    self.axis.encoder.error = 0
                    self.axis.controller.error = 0
                    self.clear_errors_requested = False

                # Handle current limit change
                if self.current_limit_changed:
                    self.axis.motor.config.current_lim = self.current_limit
                    self.current_limit_changed = False

                # Read current state
                position = self.axis.encoder.pos_estimate
                velocity = self.axis.encoder.vel_estimate
                current_iq = self.axis.motor.current_control.Iq_measured
                voltage = self.odrv.vbus_voltage
                current_state = self.axis.current_state

                # Write position command (always targeting home)
                if current_state == AXIS_STATE_CLOSED_LOOP_CONTROL:
                    self.axis.controller.input_pos = self.target_position

                # Throttle GUI updates to ~50Hz
                gui_update_counter += 1
                if gui_update_counter >= 20:
                    gui_update_counter = 0
                    loop_time = (time.perf_counter() - loop_start) * 1000
                    self.data_received.emit(position, velocity, current_iq, voltage, loop_time, current_state)

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
                print(f"[ERROR] ODrive communication error: {e}")
                self.connection_lost.emit()
                break

            time.sleep(0.001)

    def stop(self):
        self.running = False


class ExerciseGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Exercise Machine Simulator")
        self.setGeometry(100, 100, 900, 700)

        # ODrive connection
        self.odrv = None
        self.axis = None
        self.reader_thread = None

        # Data buffers for plotting
        self.max_points = 1000
        self.time_data = deque(maxlen=self.max_points)
        self.position_data = deque(maxlen=self.max_points)
        self.current_data = deque(maxlen=self.max_points)
        self.start_time = None

        # State
        self.position = 0.0
        self.velocity = 0.0
        self.current_iq = 0.0
        self.voltage = 0.0
        self.home_position = 0.0  # Where the "weight" wants to return to
        self.current_limit = 0.5  # Simulated weight in Amps

        # Error tracking
        self.has_errors = False
        self.calibrating = False
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

        # Header row with enable, calibrate, stop
        header_layout = QHBoxLayout()

        header_label = QLabel("Exercise Control")
        header_label.setObjectName("sectionHeader")
        header_layout.addWidget(header_label)

        self.enable_checkbox = QCheckBox("Enable Motor")
        self.enable_checkbox.setChecked(False)
        self.enable_checkbox.stateChanged.connect(self.on_enable_changed)
        self.enable_checkbox.setEnabled(False)
        header_layout.addWidget(self.enable_checkbox)

        self.calibrate_btn = QPushButton("Calibrate")
        self.calibrate_btn.setStyleSheet("background-color: #2563eb; border: 1px solid #3b82f6;")
        self.calibrate_btn.clicked.connect(self.run_calibration)
        self.calibrate_btn.setEnabled(False)
        header_layout.addWidget(self.calibrate_btn)

        self.status_msg_label = QLabel("")
        self.status_msg_label.setStyleSheet("color: #f59e0b;")
        header_layout.addWidget(self.status_msg_label)

        header_layout.addStretch()

        self.stop_btn = QPushButton("STOP")
        self.stop_btn.setObjectName("stopButton")
        self.stop_btn.clicked.connect(self.emergency_stop)
        header_layout.addWidget(self.stop_btn)

        control_layout.addLayout(header_layout)

        # Separator
        separator1 = QFrame()
        separator1.setFrameShape(QFrame.HLine)
        separator1.setStyleSheet("background-color: #3d3d3d;")
        control_layout.addWidget(separator1)

        # Weight (current limit) setting
        weight_layout = QHBoxLayout()

        weight_label = QLabel("Simulated Weight (Current Limit):")
        weight_label.setStyleSheet("font-size: 12pt;")
        weight_layout.addWidget(weight_label)

        self.weight_input = QDoubleSpinBox()
        self.weight_input.setRange(0.01, 20.0)
        self.weight_input.setValue(0.5)
        self.weight_input.setSingleStep(0.1)
        self.weight_input.setDecimals(2)
        self.weight_input.setSuffix(" A")
        self.weight_input.setFixedWidth(120)
        self.weight_input.valueChanged.connect(self.on_weight_changed)
        weight_layout.addWidget(self.weight_input)

        weight_layout.addStretch()

        # Zero button
        self.zero_btn = QPushButton("Set Zero (Home)")
        self.zero_btn.setObjectName("zeroButton")
        self.zero_btn.clicked.connect(self.set_zero_position)
        self.zero_btn.setEnabled(False)
        weight_layout.addWidget(self.zero_btn)

        control_layout.addLayout(weight_layout)

        # Separator
        separator2 = QFrame()
        separator2.setFrameShape(QFrame.HLine)
        separator2.setStyleSheet("background-color: #3d3d3d;")
        control_layout.addWidget(separator2)

        # Measured values display
        measured_layout = QHBoxLayout()

        # Position display
        pos_container = QVBoxLayout()
        pos_label = QLabel("Position")
        pos_label.setStyleSheet("color: #a0a0a0;")
        pos_label.setAlignment(Qt.AlignCenter)
        pos_container.addWidget(pos_label)

        self.position_display = QLabel("0.00")
        self.position_display.setObjectName("valueDisplay")
        self.position_display.setStyleSheet("color: #f59e0b; font-size: 24pt;")
        self.position_display.setAlignment(Qt.AlignCenter)
        pos_container.addWidget(self.position_display)

        pos_unit = QLabel("turns from home")
        pos_unit.setStyleSheet("color: #a0a0a0;")
        pos_unit.setAlignment(Qt.AlignCenter)
        pos_container.addWidget(pos_unit)

        measured_layout.addLayout(pos_container)

        measured_layout.addStretch()

        # Current display
        current_container = QVBoxLayout()
        current_label = QLabel("Current (Force)")
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

        # Home position display
        home_container = QVBoxLayout()
        home_label = QLabel("Home Position")
        home_label.setStyleSheet("color: #a0a0a0;")
        home_label.setAlignment(Qt.AlignCenter)
        home_container.addWidget(home_label)

        self.home_display = QLabel("0.00")
        self.home_display.setObjectName("valueDisplay")
        self.home_display.setStyleSheet("color: #8b5cf6; font-size: 24pt;")
        self.home_display.setAlignment(Qt.AlignCenter)
        home_container.addWidget(self.home_display)

        home_unit = QLabel("turns (absolute)")
        home_unit.setStyleSheet("color: #a0a0a0;")
        home_unit.setAlignment(Qt.AlignCenter)
        home_container.addWidget(home_unit)

        measured_layout.addLayout(home_container)

        control_layout.addLayout(measured_layout)

        main_layout.addWidget(control_panel)

        # === Plot Section ===
        pg.setConfigOptions(antialias=True)
        pg.setConfigOption('background', '#1e1e1e')
        pg.setConfigOption('foreground', '#e0e0e0')

        self.plot_widget = pg.GraphicsLayoutWidget()
        self.plot_widget.setBackground('#1e1e1e')
        main_layout.addWidget(self.plot_widget)

        # Position plot (relative to home)
        self.position_plot = self.plot_widget.addPlot(
            title="<span style='color: #e0e0e0;'>Position (from home)</span>")
        self.position_plot.setLabel('left', 'Position', units='turns')
        self.position_plot.setLabel('bottom', 'Time', units='s')
        self.position_plot.showGrid(x=True, y=True, alpha=0.3)
        self.position_curve = self.position_plot.plot(pen=pg.mkPen('#f59e0b', width=2))

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

            # Configure for position control with current limiting
            self.axis.requested_state = AXIS_STATE_IDLE
            self.axis.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
            self.axis.controller.config.input_mode = 1  # PASSTHROUGH

            # Set initial current limit
            self.axis.motor.config.current_lim = self.current_limit

            serial = hex(self.odrv.serial_number).upper()
            self.status_label.setText(f"Connected: {serial}")
            self.status_label.setStyleSheet("color: #14b8a6; font-weight: bold;")
            self.connect_btn.setText("Disconnect")
            self.enable_checkbox.setEnabled(True)
            self.calibrate_btn.setEnabled(True)
            self.zero_btn.setEnabled(True)
            self.clear_errors_btn.setEnabled(True)

            # Start reader thread
            self.reader_thread = ExerciseReader(self.odrv, self.axis)
            self.reader_thread.current_limit = self.current_limit
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

        self.status_label.setText("Disconnected")
        self.status_label.setStyleSheet("color: #ef4444; font-weight: bold;")
        self.connect_btn.setText("Connect")
        self.enable_checkbox.setEnabled(False)
        self.enable_checkbox.setChecked(False)
        self.calibrate_btn.setEnabled(False)
        self.zero_btn.setEnabled(False)
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
            self.reader_thread.request_state(AXIS_STATE_CLOSED_LOOP_CONTROL)
        else:
            self.reader_thread.request_state(AXIS_STATE_IDLE)

    def on_weight_changed(self, value):
        """Update the simulated weight (current limit)"""
        self.current_limit = value
        if self.reader_thread:
            self.reader_thread.set_current_limit(value)

    def set_zero_position(self):
        """Set current position as home (zero) position"""
        self.home_position = self.position
        self.home_display.setText(f"{self.home_position:.2f}")

        if self.reader_thread:
            self.reader_thread.set_position(self.home_position)

        self.status_msg_label.setText(f"Home set to {self.home_position:.2f} turns")
        self.status_msg_label.setStyleSheet("color: #8b5cf6;")

    def emergency_stop(self):
        self.enable_checkbox.setChecked(False)

        if self.reader_thread:
            self.reader_thread.request_state(AXIS_STATE_IDLE)

    def run_calibration(self):
        if self.reader_thread is None:
            return

        self.enable_checkbox.setChecked(False)
        self.calibrate_btn.setEnabled(False)
        self.calibrate_btn.setText("Calibrating...")
        self.calibrating = True

        self.reader_thread.request_state(AXIS_STATE_FULL_CALIBRATION_SEQUENCE)
        self.status_msg_label.setText("Calibrating...")
        self.status_msg_label.setStyleSheet("color: #f59e0b;")

    def clear_errors(self):
        if self.reader_thread is None:
            return

        self.enable_checkbox.setChecked(False)
        self.reader_thread.request_clear_errors()
        self.status_msg_label.setText("Errors cleared - recalibrate!")
        self.status_msg_label.setStyleSheet("color: #f59e0b;")

    def on_data_received(self, position, velocity, current_iq, voltage, loop_time, current_state):
        self.position = position
        self.velocity = velocity
        self.current_iq = current_iq
        self.voltage = voltage
        self.current_state = current_state

        # Update displays (position relative to home)
        relative_pos = position - self.home_position
        self.position_display.setText(f"{relative_pos:.2f}")
        self.current_display.setText(f"{abs(current_iq):.2f}")
        self.voltage_label.setText(f"{voltage:.1f} V")

        # Check calibration status
        if self.calibrating and current_state == AXIS_STATE_IDLE:
            self.calibrating = False
            self.calibrate_btn.setText("Calibrate")
            self.calibrate_btn.setEnabled(True)

            if self.has_errors:
                self.status_msg_label.setText("Calibration failed!")
                self.status_msg_label.setStyleSheet("color: #ef4444;")
            else:
                self.status_msg_label.setText("Calibration complete!")
                self.status_msg_label.setStyleSheet("color: #14b8a6;")

        # Store for plotting (position relative to home)
        if self.start_time:
            t = time.time() - self.start_time
            self.time_data.append(t)
            self.position_data.append(relative_pos)
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

        self.position_curve.setData(time_list, list(self.position_data))
        self.current_curve.setData(time_list, list(self.current_data))

        # Auto-range X to show last 10 seconds
        x_range = [max(0, current_time - 10), current_time]
        self.position_plot.setXRange(*x_range, padding=0)
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
