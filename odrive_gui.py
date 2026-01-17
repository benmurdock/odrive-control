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
                             QStyleOptionSlider, QCheckBox)
from PyQt5.QtCore import QTimer, Qt, QThread, pyqtSignal
from PyQt5.QtGui import QColor, QPainter, QBrush, QPen, QLinearGradient
import pyqtgraph as pg

import odrive
from odrive.enums import (AXIS_STATE_IDLE, AXIS_STATE_CLOSED_LOOP_CONTROL,
                          AXIS_STATE_FULL_CALIBRATION_SEQUENCE, CONTROL_MODE_VELOCITY_CONTROL)

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
        self.target_velocity = 0.0  # turns/sec
        self.requested_state = None  # Pending state change request
        self.enabled = False  # Track if motor should be in closed loop
        self.clear_errors_requested = False  # Flag to clear errors

    def set_velocity(self, velocity):
        """Thread-safe way to update velocity command"""
        self.target_velocity = velocity

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

                # Read current state
                position = self.axis.encoder.pos_estimate  # turns
                velocity = self.axis.encoder.vel_estimate  # turns/sec
                current_iq = self.axis.motor.current_control.Iq_measured  # amps
                voltage = self.odrv.vbus_voltage  # volts
                current_state = self.axis.current_state

                # Write velocity command (only if in closed loop control)
                if current_state == AXIS_STATE_CLOSED_LOOP_CONTROL:
                    self.axis.controller.input_vel = self.target_velocity

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

        # Control
        self.commanded_velocity = 0.0
        self.max_velocity_limit = 10.0  # turns/sec - conservative default

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

        header_label = QLabel("Velocity Control")
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

        # Velocity slider
        velocity_layout = QHBoxLayout()

        velocity_label = QLabel("Velocity")
        velocity_label.setFixedWidth(100)
        velocity_layout.addWidget(velocity_label)

        # Slider uses integer values, we'll scale by 10 for 0.1 turns/s resolution
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
        velocity_layout.addWidget(velocity_unit)

        # Max velocity limit
        limit_label = QLabel("Max:")
        limit_label.setStyleSheet("color: #a0a0a0;")
        velocity_layout.addWidget(limit_label)

        self.max_velocity_spinbox = QSpinBox()
        self.max_velocity_spinbox.setMinimum(1)
        self.max_velocity_spinbox.setMaximum(50)
        self.max_velocity_spinbox.setValue(10)
        self.max_velocity_spinbox.setSuffix(" t/s")
        self.max_velocity_spinbox.setFixedWidth(80)
        self.max_velocity_spinbox.valueChanged.connect(self.on_max_velocity_changed)
        velocity_layout.addWidget(self.max_velocity_spinbox)

        control_layout.addLayout(velocity_layout)

        # Measured velocity display
        measured_layout = QHBoxLayout()
        measured_layout.addWidget(QLabel("Measured:"))
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

            # Set up for velocity control
            self.axis.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL

            serial = hex(self.odrv.serial_number).upper()
            self.status_label.setText(f"Connected: {serial}")
            self.status_label.setStyleSheet("color: #14b8a6; font-weight: bold;")
            self.connect_btn.setText("Disconnect")
            self.enable_checkbox.setEnabled(True)
            self.calibrate_btn.setEnabled(True)
            self.clear_errors_btn.setEnabled(True)

            # Start reader thread
            self.reader_thread = ODriveReader(self.odrv, self.axis)
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

    def emergency_stop(self):
        """Emergency stop - disable motor and zero velocity"""
        self.velocity_slider.setValue(0)
        self.enable_checkbox.setChecked(False)

        # Request idle via background thread (non-blocking)
        if self.reader_thread:
            self.reader_thread.request_state(AXIS_STATE_IDLE)

    def run_calibration(self):
        """Run full calibration sequence on the motor"""
        if self.reader_thread is None:
            return

        # Disable motor control first
        self.enable_checkbox.setChecked(False)
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
