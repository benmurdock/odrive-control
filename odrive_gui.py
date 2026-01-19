#!/usr/bin/env python3
"""
ODrive Motor Controller Test GUI

Real-time monitoring and torque control for ODrive v3.x over USB Native protocol.
Used for testing communication speed and motor performance.

Requirements:
pip install odrive PyQt5 pyqtgraph

Usage:
python odrive_gui.py
"""

import sys
import time
from collections import deque
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout,
                             QHBoxLayout, QLabel, QSlider, QPushButton,
                             QGraphicsProxyWidget, QSpinBox, QStyle,
                             QStyleOptionSlider, QCheckBox, QRadioButton,
                             QButtonGroup, QFrame)
from PyQt5.QtCore import QTimer, Qt, QThread, pyqtSignal
from PyQt5.QtGui import QColor, QPainter, QBrush, QPen, QLinearGradient
import pyqtgraph as pg

import odrive
from odrive.enums import (AXIS_STATE_IDLE, AXIS_STATE_CLOSED_LOOP_CONTROL,
                          AXIS_STATE_FULL_CALIBRATION_SEQUENCE,
                          CONTROL_MODE_VELOCITY_CONTROL, CONTROL_MODE_TORQUE_CONTROL,
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


class CenterZeroSlider(QSlider):
    """Custom slider that fills from center (0) to current position"""
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.max_limit = 100

    def set_max_limit(self, limit):
        self.max_limit = limit
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        opt = QStyleOptionSlider()
        self.initStyleOption(opt)
        groove_rect = self.style().subControlRect(QStyle.CC_Slider, opt, QStyle.SC_SliderGroove, self)
        handle_rect = self.style().subControlRect(QStyle.CC_Slider, opt, QStyle.SC_SliderHandle, self)

        groove_center = groove_rect.center()
        slider_range = self.maximum() - self.minimum()
        if slider_range == 0:
            return

        zero_position = groove_rect.left() + groove_rect.width() * (0 - self.minimum()) / slider_range
        current_position = groove_rect.left() + groove_rect.width() * (self.value() - self.minimum()) / slider_range

        groove_height = 8
        groove_y = groove_center.y() - groove_height // 2

        # Draw groove background
        painter.setBrush(QBrush(QColor(45, 45, 45)))
        painter.setPen(Qt.NoPen)
        painter.drawRoundedRect(groove_rect.left(), groove_y, groove_rect.width(), groove_height, 4, 4)

        # Draw filled section from center to current position
        if self.value() != 0:
            gradient = QLinearGradient()
            if self.value() > 0:
                gradient.setStart(zero_position, 0)
                gradient.setFinalStop(current_position, 0)
                fill_left = int(zero_position)
                fill_width = int(current_position - zero_position)
            else:
                gradient.setStart(current_position, 0)
                gradient.setFinalStop(zero_position, 0)
                fill_left = int(current_position)
                fill_width = int(zero_position - current_position)

            gradient.setColorAt(0, QColor(13, 115, 119))
            gradient.setColorAt(1, QColor(20, 184, 166))
            painter.setBrush(QBrush(gradient))
            painter.drawRoundedRect(fill_left, groove_y, fill_width, groove_height, 4, 4)

        # Draw center line
        painter.setPen(QPen(QColor(100, 100, 100), 2))
        painter.drawLine(int(zero_position), groove_y - 2, int(zero_position), groove_y + groove_height + 2)

        # Draw handle
        handle_gradient = QLinearGradient(handle_rect.topLeft(), handle_rect.bottomRight())
        handle_gradient.setColorAt(0, QColor(20, 184, 166))
        handle_gradient.setColorAt(1, QColor(13, 115, 119))
        painter.setBrush(QBrush(handle_gradient))
        painter.setPen(QPen(QColor(30, 30, 30), 2))
        painter.drawEllipse(handle_rect.center(), 9, 9)


class ODriveReader(QThread):
    """Background thread for fast ODrive communication"""
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

        # Control commands (only one active at a time based on mode)
        self.target_velocity = 0.0  # turns/sec
        self.target_torque = 0.0    # Amps (Iq)
        self.target_position = 0.0  # turns

        # Control mode (torque mode default for exercise machine use case)
        self.control_mode = CONTROL_MODE_TORQUE_CONTROL
        self.requested_mode_change = None  # Pending mode change request

        self.requested_state = None  # Pending state change request
        self.enabled = False  # Track if motor should be in closed loop
        self.clear_errors_requested = False  # Flag to clear errors

    def set_velocity(self, velocity):
        """Thread-safe way to update velocity command"""
        self.target_velocity = velocity

    def set_torque(self, torque):
        """Thread-safe way to update torque command (Iq in Amps)"""
        self.target_torque = torque

    def set_position(self, position):
        """Thread-safe way to update position command (turns)"""
        self.target_position = position

    def request_mode_change(self, mode):
        """Thread-safe way to request a control mode change"""
        print(f"[DEBUG] Requesting mode change: {mode}")
        self.requested_mode_change = mode

    def request_state(self, state):
        """Thread-safe way to request a state change"""
        print(f"[DEBUG] Requesting state: {state}")
        self.requested_state = state
        if state == AXIS_STATE_CLOSED_LOOP_CONTROL:
            self.enabled = True
        elif state == AXIS_STATE_IDLE:
            self.enabled = False
            self.target_velocity = 0.0

    def request_clear_errors(self):
        """Thread-safe way to request error clearing"""
        print("[DEBUG] Requesting clear errors")
        self.clear_errors_requested = True

    def run(self):
        """Fast polling loop - reads state and writes velocity command"""
        print("[DEBUG] ODriveReader thread started")
        error_check_counter = 0
        gui_update_counter = 0
        last_position = 0.0
        last_velocity = 0.0
        last_current = 0.0
        last_voltage = 0.0
        last_state = 0

        while self.running:
            loop_start = time.perf_counter()

            try:
                # Handle pending state change request
                if self.requested_state is not None:
                    print(f"[DEBUG] Applying state change: {self.requested_state}")
                    self.axis.requested_state = self.requested_state
                    self.requested_state = None
                    print("[DEBUG] State change applied")

                # Handle clear errors request
                if self.clear_errors_requested:
                    print("[DEBUG] Clearing errors")
                    self.axis.error = 0
                    self.axis.motor.error = 0
                    self.axis.encoder.error = 0
                    self.axis.controller.error = 0
                    self.clear_errors_requested = False
                    print("[DEBUG] Errors cleared")

                # Handle pending mode change request
                if self.requested_mode_change is not None:
                    print(f"[DEBUG] Applying mode change: {self.requested_mode_change}")
                    self.axis.controller.config.control_mode = self.requested_mode_change
                    self.control_mode = self.requested_mode_change

                    # For position control, also set input_mode to passthrough (1)
                    # This ensures input_pos is used directly without trajectory planning
                    if self.control_mode == CONTROL_MODE_POSITION_CONTROL:
                        self.axis.controller.config.input_mode = 1  # INPUT_MODE_PASSTHROUGH
                        print("[DEBUG] Set input_mode to PASSTHROUGH for position control")
                    elif self.control_mode == CONTROL_MODE_VELOCITY_CONTROL:
                        self.axis.controller.config.input_mode = 1  # INPUT_MODE_PASSTHROUGH
                    elif self.control_mode == CONTROL_MODE_TORQUE_CONTROL:
                        self.axis.controller.config.input_mode = 1  # INPUT_MODE_PASSTHROUGH

                    # Note: Initial values are set by the GUI via set_position/set_torque/set_velocity
                    # immediately after requesting the mode change
                    self.requested_mode_change = None
                    print("[DEBUG] Mode change applied")

                # Read current state
                position = self.axis.encoder.pos_estimate  # turns
                velocity = self.axis.encoder.vel_estimate  # turns/sec
                current_iq = self.axis.motor.current_control.Iq_measured  # amps
                voltage = self.odrv.vbus_voltage  # volts
                current_state = self.axis.current_state

                # Write command based on current control mode (only if in closed loop)
                if current_state == AXIS_STATE_CLOSED_LOOP_CONTROL:
                    # Debug: print mode and command every ~1 second (every 1000 loops)
                    if gui_update_counter == 0:
                        actual_mode = self.axis.controller.config.control_mode
                        actual_input_mode = self.axis.controller.config.input_mode
                        print(f"[THREAD] Mode={self.control_mode}, ODrive_mode={actual_mode}, input_mode={actual_input_mode}, pos={self.target_position:.2f}, encoder_pos={position:.2f}")

                    if self.control_mode == CONTROL_MODE_VELOCITY_CONTROL:
                        self.axis.controller.input_vel = self.target_velocity
                    elif self.control_mode == CONTROL_MODE_TORQUE_CONTROL:
                        self.axis.controller.input_torque = self.target_torque
                    elif self.control_mode == CONTROL_MODE_POSITION_CONTROL:
                        self.axis.controller.input_pos = self.target_position

                # Throttle GUI updates to ~50Hz (every 20 loops at 1ms = 20ms)
                gui_update_counter += 1
                if gui_update_counter >= 20:
                    gui_update_counter = 0
                    loop_time = (time.perf_counter() - loop_start) * 1000  # ms
                    self.data_received.emit(position, velocity, current_iq, voltage, loop_time, current_state)
                    last_position = position
                    last_velocity = velocity
                    last_current = current_iq
                    last_voltage = voltage
                    last_state = current_state

                # Check errors less frequently (every 100 loops = ~100ms)
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

            # Small sleep to prevent CPU spinning - adjust for desired update rate
            # 1ms sleep = ~1000 Hz max theoretical, actual will be lower due to USB latency
            time.sleep(0.001)

        print("[DEBUG] ODriveReader thread stopped")

    def stop(self):
        print("[DEBUG] ODriveReader stop requested")
        self.running = False


class ODriveGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("ODrive Test Interface")
        self.setGeometry(100, 100, 1000, 800)

        # ODrive connection
        self.odrv = None
        self.axis = None  # We'll use axis0 by default
        self.reader_thread = None

        # Data buffers (10 seconds of data at ~100Hz display = 1000 points)
        self.max_points = 1000
        self.time_data = deque(maxlen=self.max_points)
        self.position_data = deque(maxlen=self.max_points)
        self.velocity_data = deque(maxlen=self.max_points)
        self.current_data = deque(maxlen=self.max_points)
        self.start_time = None

        # Current values
        self.position = 0.0
        self.velocity = 0.0
        self.current_iq = 0.0
        self.voltage = 0.0
        self.loop_time_ms = 0.0

        # Control mode (torque default for exercise machine use case)
        self.current_control_mode = CONTROL_MODE_TORQUE_CONTROL

        # Control commands
        self.commanded_velocity = 0.0
        self.commanded_torque = 0.0
        self.commanded_position = 0.0

        # Limits
        self.max_velocity_limit = 10.0  # turns/sec
        self.max_torque_limit = 5.0     # Amps
        self.max_position_limit = 10.0  # turns

        # Error tracking
        self.has_errors = False
        self.calibrating = False
        self.current_state = AXIS_STATE_IDLE

        self.init_ui()

        # Timer for updating plots (slower than data acquisition)
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
            }
            QPushButton#stopButton:hover { background-color: #ef4444; }
            QLabel#sectionHeader {
                font-size: 13pt;
                font-weight: 600;
                color: #14b8a6;
                padding: 8px 0px;
            }
            QLabel#valueDisplay {
                font-size: 15pt;
                font-weight: 600;
                color: #14b8a6;
            }
            QSpinBox, QDoubleSpinBox {
                background-color: #2d2d2d;
                color: #e0e0e0;
                border: 1px solid #3d3d3d;
                border-radius: 6px;
                padding: 6px 10px;
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

        self.loop_time_label = QLabel("Loop: -- ms")
        self.loop_time_label.setStyleSheet("color: #a0a0a0;")
        connection_layout.addWidget(self.loop_time_label)

        main_layout.addWidget(connection_panel)

        # === Error Panel (collapsible) ===
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

        # Error details (hidden by default, shown when errors exist)
        self.error_details_panel = QWidget()
        self.error_details_panel.setStyleSheet("background-color: #3b1a1a; border-radius: 10px;")
        self.error_details_panel.setVisible(False)
        error_details_layout = QVBoxLayout(self.error_details_panel)
        error_details_layout.setContentsMargins(15, 10, 15, 10)
        error_details_layout.setSpacing(5)

        self.axis_error_label = QLabel("")
        self.axis_error_label.setStyleSheet("color: #fca5a5;")
        self.axis_error_label.setWordWrap(True)
        error_details_layout.addWidget(self.axis_error_label)

        self.motor_error_label = QLabel("")
        self.motor_error_label.setStyleSheet("color: #fca5a5;")
        self.motor_error_label.setWordWrap(True)
        error_details_layout.addWidget(self.motor_error_label)

        self.encoder_error_label = QLabel("")
        self.encoder_error_label.setStyleSheet("color: #fca5a5;")
        self.encoder_error_label.setWordWrap(True)
        error_details_layout.addWidget(self.encoder_error_label)

        self.controller_error_label = QLabel("")
        self.controller_error_label.setStyleSheet("color: #fca5a5;")
        self.controller_error_label.setWordWrap(True)
        error_details_layout.addWidget(self.controller_error_label)

        main_layout.addWidget(self.error_details_panel)

        # === Motor Control Panel ===
        control_panel = QWidget()
        control_panel.setStyleSheet("background-color: #252525; border-radius: 10px;")
        control_layout = QVBoxLayout(control_panel)
        control_layout.setContentsMargins(15, 15, 15, 15)
        control_layout.setSpacing(10)

        # Header with enable checkbox and stop button
        control_header = QHBoxLayout()

        header_label = QLabel("Motor Control")
        header_label.setObjectName("sectionHeader")
        control_header.addWidget(header_label)

        self.enable_checkbox = QCheckBox("Enable Motor")
        self.enable_checkbox.setChecked(False)
        self.enable_checkbox.stateChanged.connect(self.on_enable_changed)
        self.enable_checkbox.setEnabled(False)
        control_header.addWidget(self.enable_checkbox)

        self.calibrate_btn = QPushButton("Calibrate")
        self.calibrate_btn.setStyleSheet("background-color: #2563eb; border: 1px solid #3b82f6;")
        self.calibrate_btn.clicked.connect(self.run_calibration)
        self.calibrate_btn.setEnabled(False)
        control_header.addWidget(self.calibrate_btn)

        self.status_msg_label = QLabel("")
        self.status_msg_label.setStyleSheet("color: #f59e0b;")
        control_header.addWidget(self.status_msg_label)

        control_header.addStretch()

        self.stop_btn = QPushButton("STOP")
        self.stop_btn.setObjectName("stopButton")
        self.stop_btn.clicked.connect(self.emergency_stop)
        control_header.addWidget(self.stop_btn)

        control_layout.addLayout(control_header)

        # Create mode button group for radio buttons
        self.mode_button_group = QButtonGroup(self)

        # === Torque Control Row (default) ===
        self.torque_row = QWidget()
        torque_layout = QHBoxLayout(self.torque_row)
        torque_layout.setContentsMargins(5, 5, 5, 5)

        self.torque_radio = QRadioButton()
        self.torque_radio.setChecked(True)  # Torque is default mode
        self.mode_button_group.addButton(self.torque_radio, CONTROL_MODE_TORQUE_CONTROL)
        torque_layout.addWidget(self.torque_radio)

        torque_label = QLabel("Torque")
        torque_label.setFixedWidth(80)
        torque_layout.addWidget(torque_label)

        self.torque_slider = CenterZeroSlider(Qt.Horizontal)
        self.torque_slider.setMinimum(-500)  # -5.0 A (scaled by 100 for 0.01A resolution)
        self.torque_slider.setMaximum(500)   # +5.0 A
        self.torque_slider.setValue(0)
        self.torque_slider.valueChanged.connect(self.on_torque_changed)
        torque_layout.addWidget(self.torque_slider)

        self.torque_value_label = QLabel("0.00")
        self.torque_value_label.setObjectName("valueDisplay")
        self.torque_value_label.setFixedWidth(80)
        self.torque_value_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        torque_layout.addWidget(self.torque_value_label)

        torque_unit = QLabel("A")
        torque_unit.setStyleSheet("color: #a0a0a0;")
        torque_unit.setFixedWidth(50)
        torque_layout.addWidget(torque_unit)

        torque_limit_label = QLabel("Max:")
        torque_limit_label.setStyleSheet("color: #a0a0a0;")
        torque_layout.addWidget(torque_limit_label)

        self.max_torque_spinbox = QSpinBox()
        self.max_torque_spinbox.setMinimum(1)
        self.max_torque_spinbox.setMaximum(20)
        self.max_torque_spinbox.setValue(5)
        self.max_torque_spinbox.setSuffix(" A")
        self.max_torque_spinbox.setFixedWidth(70)
        self.max_torque_spinbox.valueChanged.connect(self.on_max_torque_changed)
        torque_layout.addWidget(self.max_torque_spinbox)

        control_layout.addWidget(self.torque_row)

        # === Position Control Row ===
        self.position_row = QWidget()
        position_layout = QHBoxLayout(self.position_row)
        position_layout.setContentsMargins(5, 5, 5, 5)

        self.position_radio = QRadioButton()
        self.mode_button_group.addButton(self.position_radio, CONTROL_MODE_POSITION_CONTROL)
        position_layout.addWidget(self.position_radio)

        position_label = QLabel("Position")
        position_label.setFixedWidth(80)
        position_layout.addWidget(position_label)

        self.position_slider = CenterZeroSlider(Qt.Horizontal)
        self.position_slider.setMinimum(-100)  # -10.0 turns
        self.position_slider.setMaximum(100)   # +10.0 turns
        self.position_slider.setValue(0)
        self.position_slider.valueChanged.connect(self.on_position_changed)
        position_layout.addWidget(self.position_slider)

        self.position_value_label = QLabel("0.0")
        self.position_value_label.setObjectName("valueDisplay")
        self.position_value_label.setFixedWidth(80)
        self.position_value_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        position_layout.addWidget(self.position_value_label)

        position_unit = QLabel("turns")
        position_unit.setStyleSheet("color: #a0a0a0;")
        position_unit.setFixedWidth(50)
        position_layout.addWidget(position_unit)

        position_limit_label = QLabel("Max:")
        position_limit_label.setStyleSheet("color: #a0a0a0;")
        position_layout.addWidget(position_limit_label)

        self.max_position_spinbox = QSpinBox()
        self.max_position_spinbox.setMinimum(1)
        self.max_position_spinbox.setMaximum(100)
        self.max_position_spinbox.setValue(10)
        self.max_position_spinbox.setSuffix(" t")
        self.max_position_spinbox.setFixedWidth(70)
        self.max_position_spinbox.valueChanged.connect(self.on_max_position_changed)
        position_layout.addWidget(self.max_position_spinbox)

        control_layout.addWidget(self.position_row)

        # === Velocity Control Row ===
        self.velocity_row = QWidget()
        velocity_layout = QHBoxLayout(self.velocity_row)
        velocity_layout.setContentsMargins(5, 5, 5, 5)

        self.velocity_radio = QRadioButton()
        self.mode_button_group.addButton(self.velocity_radio, CONTROL_MODE_VELOCITY_CONTROL)
        velocity_layout.addWidget(self.velocity_radio)

        velocity_label = QLabel("Velocity")
        velocity_label.setFixedWidth(80)
        velocity_layout.addWidget(velocity_label)

        self.velocity_slider = CenterZeroSlider(Qt.Horizontal)
        self.velocity_slider.setMinimum(-100)  # -10.0 turns/s
        self.velocity_slider.setMaximum(100)   # +10.0 turns/s
        self.velocity_slider.setValue(0)
        self.velocity_slider.valueChanged.connect(self.on_velocity_changed)
        velocity_layout.addWidget(self.velocity_slider)

        self.velocity_value_label = QLabel("0.0")
        self.velocity_value_label.setObjectName("valueDisplay")
        self.velocity_value_label.setFixedWidth(80)
        self.velocity_value_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        velocity_layout.addWidget(self.velocity_value_label)

        velocity_unit = QLabel("turns/s")
        velocity_unit.setStyleSheet("color: #a0a0a0;")
        velocity_unit.setFixedWidth(50)
        velocity_layout.addWidget(velocity_unit)

        velocity_limit_label = QLabel("Max:")
        velocity_limit_label.setStyleSheet("color: #a0a0a0;")
        velocity_layout.addWidget(velocity_limit_label)

        self.max_velocity_spinbox = QSpinBox()
        self.max_velocity_spinbox.setMinimum(1)
        self.max_velocity_spinbox.setMaximum(50)
        self.max_velocity_spinbox.setValue(10)
        self.max_velocity_spinbox.setSuffix(" t/s")
        self.max_velocity_spinbox.setFixedWidth(70)
        self.max_velocity_spinbox.valueChanged.connect(self.on_max_velocity_changed)
        velocity_layout.addWidget(self.max_velocity_spinbox)

        control_layout.addWidget(self.velocity_row)

        # Connect mode button group signal
        self.mode_button_group.buttonClicked.connect(self.on_mode_changed)

        # Separator line
        separator = QFrame()
        separator.setFrameShape(QFrame.HLine)
        separator.setStyleSheet("background-color: #3d3d3d;")
        control_layout.addWidget(separator)

        # Measured values display
        measured_layout = QHBoxLayout()
        measured_layout.addWidget(QLabel("Measured:"))

        measured_layout.addWidget(QLabel("Pos:"))
        self.measured_position_label = QLabel("0.00 turns")
        self.measured_position_label.setStyleSheet("color: #f59e0b; font-weight: bold;")
        measured_layout.addWidget(self.measured_position_label)

        measured_layout.addWidget(QLabel("   Vel:"))
        self.measured_velocity_label = QLabel("0.0 turns/s")
        self.measured_velocity_label.setStyleSheet("color: #3b82f6; font-weight: bold;")
        measured_layout.addWidget(self.measured_velocity_label)

        measured_layout.addWidget(QLabel("   Current:"))
        self.measured_current_label = QLabel("0.00 A")
        self.measured_current_label.setStyleSheet("color: #10b981; font-weight: bold;")
        measured_layout.addWidget(self.measured_current_label)

        measured_layout.addStretch()
        control_layout.addLayout(measured_layout)

        main_layout.addWidget(control_panel)

        # Apply initial mode highlighting
        self.update_mode_highlighting()

        # === Plot Section ===
        pg.setConfigOptions(antialias=True)
        pg.setConfigOption('background', '#1e1e1e')
        pg.setConfigOption('foreground', '#e0e0e0')

        self.plot_widget = pg.GraphicsLayoutWidget()
        self.plot_widget.setBackground('#1e1e1e')
        main_layout.addWidget(self.plot_widget)

        # Position plot
        self.position_plot = self.plot_widget.addPlot(
            title="<span style='color: #e0e0e0;'>Position</span>")
        self.position_plot.setLabel('left', 'Position', units='turns')
        self.position_plot.setLabel('bottom', 'Time', units='s')
        self.position_plot.showGrid(x=True, y=True, alpha=0.3)
        self.position_curve = self.position_plot.plot(pen=pg.mkPen('#f59e0b', width=2))

        # Velocity plot
        self.plot_widget.nextRow()
        self.velocity_plot = self.plot_widget.addPlot(
            title="<span style='color: #e0e0e0;'>Velocity</span>")
        self.velocity_plot.setLabel('left', 'Velocity', units='turns/s')
        self.velocity_plot.setLabel('bottom', 'Time', units='s')
        self.velocity_plot.showGrid(x=True, y=True, alpha=0.3)
        self.velocity_curve = self.velocity_plot.plot(pen=pg.mkPen('#3b82f6', width=2))

        # Current plot
        self.plot_widget.nextRow()
        self.current_plot = self.plot_widget.addPlot(
            title="<span style='color: #e0e0e0;'>Motor Current (Iq)</span>")
        self.current_plot.setLabel('left', 'Current', units='A')
        self.current_plot.setLabel('bottom', 'Time', units='s')
        self.current_plot.showGrid(x=True, y=True, alpha=0.3)
        self.current_plot.setYRange(0, 0.5, padding=0)  # Focus on 0-0.5A range
        self.current_curve = self.current_plot.plot(pen=pg.mkPen('#10b981', width=2))

    def toggle_connection(self):
        """Connect or disconnect from ODrive"""
        if self.odrv is not None:
            self.disconnect_odrive()
        else:
            self.connect_odrive()

    def connect_odrive(self):
        """Find and connect to ODrive"""
        self.status_label.setText("Searching...")
        self.status_label.setStyleSheet("color: #f59e0b; font-weight: bold;")
        QApplication.processEvents()

        try:
            self.odrv = odrive.find_any(timeout=10)
            self.axis = self.odrv.axis0

            # Make sure motor is in idle state initially
            self.axis.requested_state = AXIS_STATE_IDLE

            # Set control mode based on current GUI selection
            self.axis.controller.config.control_mode = self.current_control_mode

            serial = hex(self.odrv.serial_number).upper()
            self.status_label.setText(f"Connected: {serial}")
            self.status_label.setStyleSheet("color: #14b8a6; font-weight: bold;")
            self.connect_btn.setText("Disconnect")
            self.enable_checkbox.setEnabled(True)
            self.calibrate_btn.setEnabled(True)
            self.clear_errors_btn.setEnabled(True)

            # Start reader thread with current control mode
            self.reader_thread = ODriveReader(self.odrv, self.axis)
            self.reader_thread.control_mode = self.current_control_mode
            self.reader_thread.data_received.connect(self.on_data_received)
            self.reader_thread.errors_received.connect(self.on_errors_received)
            self.reader_thread.connection_lost.connect(self.on_connection_lost)
            self.reader_thread.start()

            # Start plot updates
            self.start_time = time.time()
            self.plot_timer.start(50)  # 20 Hz plot updates

        except Exception as e:
            self.status_label.setText(f"Error: {str(e)[:30]}")
            self.status_label.setStyleSheet("color: #ef4444; font-weight: bold;")
            self.odrv = None

    def disconnect_odrive(self):
        """Disconnect from ODrive"""
        # Stop motor first
        self.enable_checkbox.setChecked(False)

        # Stop reader thread
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
        self.clear_errors_btn.setEnabled(False)
        self.clear_errors_btn.setVisible(False)
        self.status_msg_label.setText("")
        self.update_error_display(0, 0, 0, 0)  # Clear error display

        # Reset all sliders
        self.torque_slider.setValue(0)
        self.position_slider.setValue(0)
        self.velocity_slider.setValue(0)

        # Reset to default mode (torque)
        self.torque_radio.setChecked(True)
        self.current_control_mode = CONTROL_MODE_TORQUE_CONTROL
        self.update_mode_highlighting()

    def on_connection_lost(self):
        """Handle unexpected disconnection"""
        self.disconnect_odrive()
        self.status_label.setText("Connection Lost!")

    def on_enable_changed(self, state):
        """Enable/disable motor closed loop control"""
        if self.reader_thread is None:
            return

        if state == Qt.Checked:
            # Request closed loop control via background thread
            self.reader_thread.request_state(AXIS_STATE_CLOSED_LOOP_CONTROL)
        else:
            # Request idle via background thread
            self.reader_thread.request_state(AXIS_STATE_IDLE)
            self.torque_slider.setValue(0)
            self.position_slider.setValue(0)
            self.velocity_slider.setValue(0)

    def on_velocity_changed(self, value):
        """Velocity slider changed"""
        # Convert from slider units to turns/sec
        velocity = value / 10.0

        # Enforce limit
        if abs(velocity) > self.max_velocity_limit:
            limited = self.max_velocity_limit if velocity > 0 else -self.max_velocity_limit
            self.velocity_slider.setValue(int(limited * 10))
            return

        self.commanded_velocity = velocity
        self.velocity_value_label.setText(f"{velocity:.1f}")

        # Send to ODrive via reader thread
        if self.reader_thread:
            self.reader_thread.set_velocity(velocity)

    def on_max_velocity_changed(self, value):
        """Update max velocity limit"""
        self.max_velocity_limit = float(value)

        # Update slider range
        slider_max = int(value * 10)
        self.velocity_slider.setMinimum(-slider_max)
        self.velocity_slider.setMaximum(slider_max)

        # Clamp current value if needed
        if abs(self.commanded_velocity) > self.max_velocity_limit:
            limited = self.max_velocity_limit if self.commanded_velocity > 0 else -self.max_velocity_limit
            self.velocity_slider.setValue(int(limited * 10))

    def on_torque_changed(self, value):
        """Torque slider changed"""
        # Convert from slider units to Amps (scaled by 100 for 0.01A resolution)
        torque = value / 100.0

        # Enforce limit
        if abs(torque) > self.max_torque_limit:
            limited = self.max_torque_limit if torque > 0 else -self.max_torque_limit
            self.torque_slider.setValue(int(limited * 100))
            return

        self.commanded_torque = torque
        self.torque_value_label.setText(f"{torque:.2f}")

        # Send to ODrive via reader thread
        if self.reader_thread:
            self.reader_thread.set_torque(torque)

    def on_max_torque_changed(self, value):
        """Update max torque limit"""
        self.max_torque_limit = float(value)

        # Update slider range (scaled by 100 for 0.01A resolution)
        slider_max = int(value * 100)
        self.torque_slider.setMinimum(-slider_max)
        self.torque_slider.setMaximum(slider_max)

        # Clamp current value if needed
        if abs(self.commanded_torque) > self.max_torque_limit:
            limited = self.max_torque_limit if self.commanded_torque > 0 else -self.max_torque_limit
            self.torque_slider.setValue(int(limited * 100))

    def on_position_changed(self, value):
        """Position slider changed"""
        print(f"[DEBUG] on_position_changed called with value={value}")
        # Convert from slider units to turns
        position = value / 10.0
        print(f"[DEBUG] Position in turns: {position}, limit: {self.max_position_limit}")

        # Enforce limit
        if abs(position) > self.max_position_limit:
            limited = self.max_position_limit if position > 0 else -self.max_position_limit
            print(f"[DEBUG] Position over limit, clamping to {limited}")
            self.position_slider.setValue(int(limited * 10))
            return

        self.commanded_position = position
        self.position_value_label.setText(f"{position:.1f}")

        # Send to ODrive via reader thread
        if self.reader_thread:
            print(f"[DEBUG] Sending position {position} to reader thread")
            self.reader_thread.set_position(position)
        else:
            print("[DEBUG] No reader thread available")

    def on_max_position_changed(self, value):
        """Update max position limit"""
        self.max_position_limit = float(value)

        # Update slider range
        slider_max = int(value * 10)
        self.position_slider.setMinimum(-slider_max)
        self.position_slider.setMaximum(slider_max)

        # Clamp current value if needed
        if abs(self.commanded_position) > self.max_position_limit:
            limited = self.max_position_limit if self.commanded_position > 0 else -self.max_position_limit
            self.position_slider.setValue(int(limited * 10))

    def on_mode_changed(self, button):
        """Handle control mode change via radio button"""
        mode = self.mode_button_group.id(button)
        print(f"[GUI] Mode changed to: {mode}")
        self.current_control_mode = mode

        # Update visual highlighting
        self.update_mode_highlighting()

        # Request mode change in background thread
        if self.reader_thread:
            self.reader_thread.request_mode_change(mode)

            # After mode change, sync slider values to current commands
            # This ensures sliders work immediately after switching modes
            if mode == CONTROL_MODE_POSITION_CONTROL and hasattr(self, 'position'):
                # Clamp current position to slider range
                current_pos = max(-self.max_position_limit,
                                 min(self.max_position_limit, self.position))
                self.commanded_position = current_pos
                self.position_slider.setValue(int(current_pos * 10))
                self.position_value_label.setText(f"{current_pos:.1f}")
                # Explicitly send the clamped position to the thread
                self.reader_thread.set_position(current_pos)
            elif mode == CONTROL_MODE_TORQUE_CONTROL:
                # Torque starts at zero
                self.commanded_torque = 0.0
                self.torque_slider.setValue(0)
                self.torque_value_label.setText("0.00")
                self.reader_thread.set_torque(0.0)
            elif mode == CONTROL_MODE_VELOCITY_CONTROL:
                # Velocity starts at zero
                self.commanded_velocity = 0.0
                self.velocity_slider.setValue(0)
                self.velocity_value_label.setText("0.0")
                self.reader_thread.set_velocity(0.0)

    def update_mode_highlighting(self):
        """Update visual highlighting of the active control mode row"""
        # Define styles
        active_style = "background-color: #2d3d2d; border-radius: 5px;"
        inactive_style = "background-color: transparent;"

        # Apply styles based on current mode
        self.torque_row.setStyleSheet(
            active_style if self.current_control_mode == CONTROL_MODE_TORQUE_CONTROL else inactive_style)
        self.position_row.setStyleSheet(
            active_style if self.current_control_mode == CONTROL_MODE_POSITION_CONTROL else inactive_style)
        self.velocity_row.setStyleSheet(
            active_style if self.current_control_mode == CONTROL_MODE_VELOCITY_CONTROL else inactive_style)

    def emergency_stop(self):
        """Emergency stop - disable motor and zero all commands"""
        # Zero all sliders
        self.torque_slider.setValue(0)
        self.position_slider.setValue(0)
        self.velocity_slider.setValue(0)
        self.enable_checkbox.setChecked(False)

        # Request idle via background thread (non-blocking)
        if self.reader_thread:
            self.reader_thread.set_velocity(0)
            self.reader_thread.set_torque(0)
            self.reader_thread.request_state(AXIS_STATE_IDLE)

    def run_calibration(self):
        """Run full calibration sequence on the motor"""
        if self.reader_thread is None:
            return

        # Disable motor control first
        self.enable_checkbox.setChecked(False)
        self.torque_slider.setValue(0)
        self.position_slider.setValue(0)
        self.velocity_slider.setValue(0)
        self.calibrate_btn.setEnabled(False)
        self.calibrate_btn.setText("Calibrating...")
        self.calibrating = True

        # Request calibration via background thread (non-blocking)
        self.reader_thread.request_state(AXIS_STATE_FULL_CALIBRATION_SEQUENCE)
        self.status_msg_label.setText("Calibrating...")
        self.status_msg_label.setStyleSheet("color: #f59e0b;")

    def clear_errors(self):
        """Clear all errors on the ODrive - requires recalibration after"""
        if self.reader_thread is None:
            return

        # Disable motor first
        self.enable_checkbox.setChecked(False)
        self.torque_slider.setValue(0)
        self.position_slider.setValue(0)
        self.velocity_slider.setValue(0)

        # Request clear errors via background thread (non-blocking)
        self.reader_thread.request_clear_errors()
        self.status_msg_label.setText("Errors cleared - recalibrate!")
        self.status_msg_label.setStyleSheet("color: #f59e0b;")

    def on_data_received(self, position, velocity, current_iq, voltage, loop_time, current_state):
        """Called when new data arrives from ODrive"""
        self.position = position
        self.velocity = velocity
        self.current_iq = current_iq
        self.voltage = voltage
        self.loop_time_ms = loop_time
        self.current_state = current_state

        # Update labels
        self.measured_position_label.setText(f"{position:.2f} turns")
        self.measured_velocity_label.setText(f"{velocity:.1f} turns/s")
        self.measured_current_label.setText(f"{current_iq:.2f} A")
        self.voltage_label.setText(f"{voltage:.1f} V")
        self.loop_time_label.setText(f"Loop: {loop_time:.1f} ms")

        # Check if calibration finished (state returned to IDLE)
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

        # Store data for plotting
        if self.start_time:
            t = time.time() - self.start_time
            self.time_data.append(t)
            self.position_data.append(position)
            self.velocity_data.append(velocity)
            self.current_data.append(current_iq)

    def on_errors_received(self, axis_error, motor_error, encoder_error, controller_error):
        """Called when error status is received from ODrive"""
        self.update_error_display(axis_error, motor_error, encoder_error, controller_error)

    def update_error_display(self, axis_error, motor_error, encoder_error, controller_error):
        """Update the error panel based on current errors"""
        has_errors = (axis_error != 0 or motor_error != 0 or
                      encoder_error != 0 or controller_error != 0)

        if has_errors:
            # Show error state
            self.has_errors = True
            self.error_panel.setStyleSheet("background-color: #4a1a1a; border-radius: 10px;")
            self.error_status_label.setText("ERRORS DETECTED")
            self.error_status_label.setStyleSheet("color: #ef4444; font-weight: bold;")
            self.clear_errors_btn.setVisible(True)
            self.clear_errors_btn.setEnabled(True)
            self.error_details_panel.setVisible(True)

            # Decode and display each error type
            if axis_error:
                errors = decode_errors(axis_error, AXIS_ERRORS)
                self.axis_error_label.setText(f"Axis: {', '.join(errors) if errors else f'Unknown (0x{axis_error:04X})'}")
                self.axis_error_label.setVisible(True)
            else:
                self.axis_error_label.setVisible(False)

            if motor_error:
                errors = decode_errors(motor_error, MOTOR_ERRORS)
                self.motor_error_label.setText(f"Motor: {', '.join(errors) if errors else f'Unknown (0x{motor_error:04X})'}")
                self.motor_error_label.setVisible(True)
            else:
                self.motor_error_label.setVisible(False)

            if encoder_error:
                errors = decode_errors(encoder_error, ENCODER_ERRORS)
                self.encoder_error_label.setText(f"Encoder: {', '.join(errors) if errors else f'Unknown (0x{encoder_error:04X})'}")
                self.encoder_error_label.setVisible(True)
            else:
                self.encoder_error_label.setVisible(False)

            if controller_error:
                errors = decode_errors(controller_error, CONTROLLER_ERRORS)
                self.controller_error_label.setText(f"Controller: {', '.join(errors) if errors else f'Unknown (0x{controller_error:04X})'}")
                self.controller_error_label.setVisible(True)
            else:
                self.controller_error_label.setVisible(False)

        else:
            # No errors - show green status
            self.has_errors = False
            self.error_panel.setStyleSheet("background-color: #1a4d1a; border-radius: 10px;")
            self.error_status_label.setText("No Errors")
            self.error_status_label.setStyleSheet("color: #4ade80; font-weight: bold;")
            self.clear_errors_btn.setVisible(False)
            self.error_details_panel.setVisible(False)

    def update_plots(self):
        """Update plot curves (called by timer, slower than data acquisition)"""
        if len(self.time_data) < 2:
            return

        time_list = list(self.time_data)
        current_time = time_list[-1]

        self.position_curve.setData(time_list, list(self.position_data))
        self.velocity_curve.setData(time_list, list(self.velocity_data))
        self.current_curve.setData(time_list, list(self.current_data))

        # Auto-range X to show last 10 seconds
        x_range = [max(0, current_time - 10), current_time]
        self.position_plot.setXRange(*x_range, padding=0)
        self.velocity_plot.setXRange(*x_range, padding=0)
        self.current_plot.setXRange(*x_range, padding=0)

    def closeEvent(self, event):
        """Clean up on close"""
        self.emergency_stop()
        if self.reader_thread:
            self.reader_thread.stop()
            self.reader_thread.wait()
        event.accept()


def main():
    app = QApplication(sys.argv)
    gui = ODriveGUI()
    gui.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
