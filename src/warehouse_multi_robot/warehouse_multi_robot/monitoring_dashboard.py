#!/usr/bin/env python3
"""
monitoring_dashboard.py - Integrated PyQt5 Operator Terminal for Multi-Robot Fleet.

This script implements a thread-safe PyQt5 dashboard integrated with ROS2.
It subscribes to status, camera views with XAI overlays, anomalies, and metrics,
rendering them in an Industrial Dark Mode theme under WSL2.
"""

import os
# Force Qt to use X11/XWayland backend so it is fully controllable via xdotool under WSLg
os.environ["QT_QPA_PLATFORM"] = "xcb"

import sys
import json
import time
import signal  # Added for native OS level signal intercept
from typing import Dict, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions  
from rclpy.executors import ExternalShutdownException

from std_msgs.msg import String, Bool
from sensor_msgs.msg import Image

# PyQt5 GUI Imports
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QPushButton, QProgressBar, QTextEdit, QFrame, QGridLayout, QComboBox
)
from PyQt5.QtGui import QImage, QPixmap, QFont, QColor, QPainter, QPen, QPolygonF
from PyQt5.QtCore import QThread, pyqtSignal, QObject, Qt, QTimer, QPointF


class BatteryHistoryGraph(QFrame):
    """Minimalistic real-time battery history line visualizer using QPainter."""
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setStyleSheet("background-color: #15151A; border: 1px solid #3B4252; border-radius: 4px;")
        self.history = {"robot1": [], "robot2": [], "robot3": []}
        self.max_points = 50
        self.setMinimumHeight(110)

    def add_data(self, robot_name: str, value: float):
        """Adds a battery telemetry point and schedules repaint."""
        if robot_name in self.history:
            self.history[robot_name].append(value)
            if len(self.history[robot_name]) > self.max_points:
                self.history[robot_name].pop(0)
            self.update()

    def paintEvent(self, event):
        super().paintEvent(event)
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        w = self.width()
        h = self.height()

        # Draw grid lines
        grid_pen = QPen(QColor("#2E3440"), 1, Qt.DashLine)
        painter.setPen(grid_pen)
        for i in range(1, 4):
            y_grid = int(h * (i / 4.0))
            painter.drawLine(0, y_grid, w, y_grid)

        # Theme matching colors for lines
        colors = {
            "robot1": QColor("#81A1C1"), # Frost Blue
            "robot2": QColor("#A3BE8C"), # Light Green
            "robot3": QColor("#D08770")  # Amber/Orange
        }

        for robot, data in self.history.items():
            if not data:
                continue
            pen = QPen(colors[robot], 2, Qt.SolidLine)
            painter.setPen(pen)

            points = []
            for idx, val in enumerate(data):
                # Map timeline to width, battery level (0-100) to height
                x_pos = (idx / (self.max_points - 1)) * w
                y_pos = h - (val / 100.0) * (h - 12) - 6
                points.append(QPointF(x_pos, y_pos))

            poly = QPolygonF(points)
            painter.drawPolyline(poly)

# ==============================================================================
# 1. THREAD-SAFE ROS2 BRIDGE SIGNALS
# ==============================================================================
class DashboardBridge(QObject):
    """Signals used to safely pass ROS2 message data to the Qt main thread."""
    status_received = pyqtSignal(str, dict)       # robot_name, parsed_json_payload
    camera_received = pyqtSignal(str, QImage)     # robot_name, converted_qimage
    anomaly_received = pyqtSignal(dict)           # parsed_json_anomaly
    shelf_arrived_received = pyqtSignal(dict)     # parsed_json_arrival
    mission_armed_received = pyqtSignal(bool)     # true/false armed state


# ==============================================================================
# 2. DETECTOR AND TELEMETRY SUBSCRIBER NODE
# ==============================================================================
class DashboardROS2Node(Node):
    def __init__(self, bridge: DashboardBridge):
        super().__init__("monitoring_dashboard_node")
        self.bridge = bridge

        qos_status = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE
        )
        qos_transient = QoSProfile(
            depth=10,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE
        )
        qos_claims = QoSProfile(
            depth=100,
            reliability=ReliabilityPolicy.RELIABLE
        )

        self.robots = ["robot1", "robot2", "robot3"]

        # Failure injection publishers (Consolidated JSON layout matching waypoint_sender)
        self.failure_publishers: Dict[str, rclpy.publisher.Publisher] = {}

        # 1. Setup per-robot namespaces dynamic subscriptions
        for r_name in self.robots:
            # Subscribing to consolidated status topic for battery, x, y, status
            self.create_subscription(
                String,
                f"/{r_name}/status",
                lambda msg, name=r_name: self._status_callback(msg, name),
                qos_status
            )
            # Camera frame view subscription
            self.create_subscription(
                Image,
                f"/{r_name}/camera_view",
                lambda msg, name=r_name: self._camera_callback(msg, name),
                10
            )
            # Register failure injector publisher for each robot
            self.failure_publishers[r_name] = self.create_publisher(
                String,
                f"/{r_name}/status",
                qos_status
            )

        # 2. Setup system-wide dynamic topic subscriptions
        self.create_subscription(String, "/anomaly_alert", self._anomaly_callback, 10)
        self.create_subscription(String, "/shelf_arrived", self._shelf_arrived_callback, qos_claims)
        self.create_subscription(Bool, "/mission_armed", self._mission_armed_callback, qos_transient)

        # Publisher to trigger global mission arm dispatch
        self.mission_armed_pub = self.create_publisher(Bool, "/mission_start", qos_transient)

        self.get_logger().info("[Dashboard ROS2] Subscriptions armed. Quiet listening active.")

    def trigger_mission_start(self):
        """Dispatches global arm trigger message to /mission_start."""
        msg = Bool()
        msg.data = True
        self.mission_armed_pub.publish(msg)

    def inject_failure(self, robot_name: str):
        """Publishes FAILED JSON telemetry block to target robot to trigger reallocation."""
        payload = {
            "status": "FAILED",
            "battery": 0.0,
            "x": 0.0,
            "y": 0.0
        }
        msg = String()
        msg.data = json.dumps(payload)
        self.failure_publishers[robot_name].publish(msg)

    # --- SUBSCRIBER CALLBACKS ---
    def _status_callback(self, msg: String, robot_name: str):
        try:
            data = json.loads(msg.data)
            self.bridge.status_received.emit(robot_name, data)
        except json.JSONDecodeError:
            pass

    def _camera_callback(self, msg: Image, robot_name: str):
        """Converts ROS2 Image msg to QImage safely without cv_bridge dependency."""
        try:
            if msg.encoding == "bgr8":
                # Convert BGR raw bytes directly to QImage and execute deep copy to avoid Segfaults
                q_img = QImage(msg.data, msg.width, msg.height, msg.step, QImage.Format_RGB888).rgbSwapped().copy()
                self.bridge.camera_received.emit(robot_name, q_img)
            elif msg.encoding == "mono8":
                q_img = QImage(msg.data, msg.width, msg.height, msg.step, QImage.Format_Grayscale8).copy()
                self.bridge.camera_received.emit(robot_name, q_img)
        except Exception as e:
            self.get_logger().error(f"Image conversion failed for {robot_name}: {e}")

    def _anomaly_callback(self, msg: String):
        try:
            data = json.loads(msg.data)
            self.bridge.anomaly_received.emit(data)
        except json.JSONDecodeError:
            pass

    def _shelf_arrived_callback(self, msg: String):
        try:
            data = json.loads(msg.data)
            self.bridge.shelf_arrived_received.emit(data)
        except json.JSONDecodeError:
            pass

    def _mission_armed_callback(self, msg: Bool):
        self.bridge.mission_armed_received.emit(msg.data)


# ==============================================================================
# 3. ASYNCHRONOUS EXECUTOR SPIN THREAD
# ==============================================================================
class ROS2SpinThread(QThread):
    """Executes ROS2 executor spinning on an isolated system thread with shutdown handling."""
    def __init__(self, node: Node):
        super().__init__()
        self.node = node

    def run(self):
        try:
            rclpy.spin(self.node)
        except (ExternalShutdownException, KeyboardInterrupt):
            pass  # Safely ignore interrupted spin exception upon Ctrl+C exit


# ==============================================================================
# 4. MAIN PYQT5 CONTROL PANEL (THE GUI)
# ==============================================================================
class FleetInspectionDashboard(QMainWindow):
    def __init__(self, ros2_node: DashboardROS2Node, bridge: DashboardBridge):
        super().__init__()
        self.ros2_node = ros2_node
        self.bridge = bridge

        self.setWindowTitle("FLEET INSPECTION TERMINAL - Smart Warehouse Inspection")
        
        # Neutral initial size. Coordinates and sizing are 100% delegated to window_tiler.py to bypass Win32 snapping conflicts
        self.resize(1707, 440)

        self.completed_waypoints_global = set()
        self.total_target_waypoints = 142
        self.proximity_violations = 0
        self.anomaly_latencies = []
        self.start_time = None

        self._setup_style()
        self._build_layout()
        self._connect_signals()

        # Keep a 1Hz GUI timer running to update system timer dynamically
        self.ui_timer = QTimer()
        self.ui_timer.timeout.connect(self._update_chronometer)
        self.ui_timer.start(1000)
        

    def _setup_style(self):
        """Sets up Industrial Dark Mode stylesheet styling with larger, high-contrast fonts."""
        self.setStyleSheet("""
            QMainWindow {
                background-color: #121216;
            }
            QLabel {
                color: #ECEFF4;
                font-family: 'Helvetica Neue', Arial;
                font-size: 12pt; /* Increased from 11pt */
                font-weight: bold;
            }
            QFrame {
                background-color: #1E1E24;
                border: 1px solid #2E3440;
                border-radius: 6px;
            }
            QPushButton {
                background-color: #3B4252;
                color: #ECEFF4;
                border: 1px solid #4C566A;
                border-radius: 4px;
                padding: 6px 12px;
                font-weight: bold;
                font-size: 12pt; /* Increased from 11pt */
            }
            QPushButton:hover {
                background-color: #434C5E;
            }
            QProgressBar {
                border: 1px solid #4C566A;
                border-radius: 4px;
                text-align: center;
                background-color: #2E3440;
                color: #ECEFF4;
                font-weight: bold;
                font-size: 11pt; /* Increased from 10pt */
                min-height: 22px; /* Thicker industrial progress bars */
            }
            QProgressBar::chunk {
                background-color: #2ECC71;
            }
            QTextEdit {
                background-color: #181820;
                color: #A3BE8C;
                font-family: 'Courier New', monospace;
                font-size: 11pt; /* Larger terminal log fonts */
                border: 1px solid #2E3440;
            }
            QComboBox {
                background-color: #2E3440;
                color: #ECEFF4;
                font-size: 11pt;
                font-weight: bold;
                border: 1px solid #4C566A;
                border-radius: 4px;
                padding: 4px;
            }
        """)

    def _build_layout(self):
        """Constructs bottom-spanning structural window layout dynamically."""
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        main_layout = QVBoxLayout(main_widget)
        main_layout.setContentsMargins(10, 10, 10, 10)
        main_layout.setSpacing(8)

        # --- HEADER COMPONENT ---
        header_layout = QHBoxLayout()
        title_label = QLabel("FLEET INSPECTION CONTROL TERMINAL")
        title_label.setFont(QFont("Arial", 14, QFont.Bold))
        title_label.setStyleSheet("color: #88C0D0;")

        self.time_label = QLabel("SYSTEM TIME: 2026-06-22 15:29:45 UTC")
        self.time_label.setFont(QFont("Courier New", 11, QFont.Bold))

        self.mission_state_label = QLabel("MISSION STATE: [ STANDBY ]")
        self.mission_state_label.setFont(QFont("Arial", 11, QFont.Bold))
        self.mission_state_label.setStyleSheet("color: #D08770;")

        header_layout.addWidget(title_label)
        header_layout.addStretch()
        header_layout.addWidget(self.time_label)
        header_layout.addSpacing(25)
        header_layout.addWidget(self.mission_state_label)
        main_layout.addLayout(header_layout)

        # --- BODY PANELS SPLIT ---
        body_layout = QHBoxLayout()
        body_layout.setSpacing(10)

        # [A] ROBOT FLEET STATUS (LEFT PANEL)
        body_layout.addWidget(self._create_left_panel(), stretch=1)

        # [B] ML ANOMALY ALERT (MIDDLE PANEL - STACKED CAMERAS)
        body_layout.addWidget(self._create_middle_panel(), stretch=1)

        # [C] SYSTEM LOGS AND PERFORMANCE METRICS (RIGHT PANEL)
        body_layout.addWidget(self._create_right_panel(), stretch=1)

        main_layout.addLayout(body_layout)

    def _create_left_panel(self) -> QFrame:
        panel = QFrame()
        panel.setFixedWidth(380) # Keep Left panel compact
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(10, 10, 10, 10)
        layout.setSpacing(10)

        label_title = QLabel("[A] ROBOT FLEET STATUS")
        # Visual sync with Panel C: Exact header color, style, and size
        label_title.setStyleSheet("color: #88C0D0; border: none; font-size: 12pt; font-weight: bold;")
        layout.addWidget(label_title)

        self.robot_cards: Dict[str, Dict[str, QLabel]] = {}
        self.robot_progress_bars: Dict[str, QProgressBar] = {}

        for r_name in ["robot1", "robot2", "robot3"]:
            card = QFrame()
            card.setStyleSheet("background-color: #24242C; border: 1px solid #3B4252;")
            card_layout = QVBoxLayout(card)
            card_layout.setContentsMargins(8, 8, 8, 8)
            card_layout.setSpacing(6)

            status_label = QLabel(f">> {r_name.upper()}: [ BOOTSTRAP ]")
            status_label.setFont(QFont("Arial", 12, QFont.Bold)) # Large bold state labels
            card_layout.addWidget(status_label)

            pos_label = QLabel("   Pos: ( Uninitialized )")
            pos_label.setFont(QFont("Courier New", 11, QFont.Bold)) # Large high-contrast coords
            card_layout.addWidget(pos_label)

            prog_bar = QProgressBar()
            prog_bar.setValue(0)
            prog_bar.setFormat("Tasks: %v/%m")
            prog_bar.setMaximum(48 if r_name == "robot1" else 47)
            card_layout.addWidget(prog_bar)

            layout.addWidget(card)

            self.robot_cards[r_name] = {
                "status": status_label,
                "pos": pos_label,
            }
            self.robot_progress_bars[r_name] = prog_bar

        # Action Buttons
        btn_layout = QHBoxLayout()
        self.btn_dispatch = QPushButton("ARM DISPATCH")
        self.btn_dispatch.setStyleSheet("background-color: #BF616A; color: #ECEFF4;")
        self.btn_dispatch.clicked.connect(self._handle_dispatch)

        self.btn_estop = QPushButton("ESTOP SHUTDOWN")
        self.btn_estop.setStyleSheet("background-color: #D84B20; color: #ECEFF4;")
        self.btn_estop.clicked.connect(self._handle_estop)

        btn_layout.addWidget(self.btn_dispatch)
        btn_layout.addWidget(self.btn_estop)
        layout.addLayout(btn_layout)

        # ADD STRETCH: Packs cards tightly at top, killing internal vertical dead space inside Left Panel
        layout.addStretch(1)

        return panel

    def _create_middle_panel(self) -> QFrame:
        panel = QFrame()
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(10, 10, 10, 10)
        layout.setSpacing(10)

        label_title = QLabel("[B] ML ANOMALY CAMERA FEEDS")
        # Visual sync with Panel C: Exact header color, style, and size
        label_title.setStyleSheet("color: #88C0D0; border: none; font-size: 12pt; font-weight: bold;")
        layout.addWidget(label_title)

        self.camera_displays: Dict[str, QLabel] = {}

        for r_name in ["robot1", "robot2", "robot3"]:
            cam_card = QFrame()
            cam_card.setStyleSheet("background-color: #141416; border: 1px solid #3B4252;")
            cam_layout = QHBoxLayout(cam_card)
            cam_layout.setContentsMargins(6, 6, 6, 6)

            name_tag = QLabel(f"{r_name.upper()}")
            name_tag.setFont(QFont("Arial", 12, QFont.Bold))
            name_tag.setFixedWidth(75)

            display = QLabel("No Scanning Feed Active")
            display.setAlignment(Qt.AlignCenter)
            display.setStyleSheet("background-color: #0F0F12; color: #4C566A; font-family: monospace; font-size: 11pt; font-weight: bold;")
            # Camera box height increased significantly from 120 to 140 for large, readable XAI overlays
            display.setFixedHeight(140) 

            cam_layout.addWidget(name_tag)
            cam_layout.addWidget(display)
            layout.addWidget(cam_card)

            self.camera_displays[r_name] = display

        # ADD STRETCH: Packs camera frames tightly, killing internal vertical dead space inside Middle Panel
        layout.addStretch(1)

        return panel

    def _create_right_panel(self) -> QFrame:
        panel = QFrame()
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(8, 8, 8, 8)

        # [C] LIVE SYSTEM LOGS
        label_title = QLabel("[C] LIVE LOGS & SYSTEM PERFORMANCE")
        label_title.setFont(QFont("Arial", 12, QFont.Bold))
        label_title.setStyleSheet("color: #88C0D0; border: none;")
        layout.addWidget(label_title)

        self.log_terminal = QTextEdit()
        self.log_terminal.setReadOnly(True)
        self.log_terminal.append("15:28:00 [SYSTEM] Operator Terminal Bootstrapped. Standing by.")
        layout.addWidget(self.log_terminal, stretch=2)

        # Split lower right into metrics, defect list and live battery graph
        lower_layout = QHBoxLayout()

        # [E] PERFORMANCE METRICS
        metrics_frame = QFrame()
        metrics_layout = QGridLayout(metrics_frame)
        metrics_layout.setContentsMargins(6, 6, 6, 6)

        metrics_title = QLabel("[E] PERFORMANCE METRICS")
        metrics_title.setFont(QFont("Arial", 10, QFont.Bold))
        metrics_title.setStyleSheet("border: none; color: #8FBCBB;")
        metrics_layout.addWidget(metrics_title, 0, 0, 1, 2)

        self.lbl_audit_time = QLabel("Total Audit Time: 0.0s")
        self.lbl_coverage = QLabel("Coverage Loss: 0%")
        self.lbl_latency = QLabel("Anomaly Latency: 0.0ms")
        self.lbl_violations = QLabel("Proximity Violations: 0")

        for lbl in [self.lbl_audit_time, self.lbl_coverage, self.lbl_latency, self.lbl_violations]:
            lbl.setFont(QFont("Arial", 10, QFont.Bold))

        metrics_layout.addWidget(self.lbl_audit_time, 1, 0)
        metrics_layout.addWidget(self.lbl_coverage, 1, 1)
        metrics_layout.addWidget(self.lbl_latency, 2, 0)
        metrics_layout.addWidget(self.lbl_violations, 2, 1)

        # Integrate the missing [G] BATTERY HISTORY GRAPH into right metrics subpanel
        self.battery_graph = BatteryHistoryGraph()
        metrics_layout.addWidget(self.battery_graph, 3, 0, 1, 2)

        lower_layout.addWidget(metrics_frame, stretch=1)

        # [F] DETECTED DEFECTS & INJECTION CONTROL
        defects_frame = QFrame()
        defects_layout = QVBoxLayout(defects_frame)
        defects_layout.setContentsMargins(6, 6, 6, 6)

        defects_title = QLabel("[F] DETECTED ANOMALIES")
        defects_title.setFont(QFont("Arial", 10, QFont.Bold))
        defects_title.setStyleSheet("border: none; color: #BF616A;")
        defects_layout.addWidget(defects_title)

        self.defects_list = QTextEdit()
        self.defects_list.setReadOnly(True)
        self.defects_list.setStyleSheet("background-color: #15151A; color: #EBCB8B; border: none;")
        defects_layout.addWidget(self.defects_list)

        inject_layout = QHBoxLayout()
        self.combo_robots = QComboBox()
        self.combo_robots.addItems(["robot1", "robot2", "robot3"])
        self.combo_robots.setStyleSheet("background-color: #2E3440; color: #ECEFF4; padding: 2px;")

        btn_inject = QPushButton("INJECT FAILURE")
        btn_inject.setStyleSheet("background-color: #BF616A; color: #ECEFF4; font-size: 9pt;")
        btn_inject.clicked.connect(self._handle_inject_failure)

        inject_layout.addWidget(self.combo_robots)
        inject_layout.addWidget(btn_inject)
        defects_layout.addLayout(inject_layout)

        lower_layout.addWidget(defects_frame, stretch=1)
        layout.addLayout(lower_layout, stretch=3)

        return panel

    def _connect_signals(self):
        """Assembles Qt Signal / ROS 2 slot bridges."""
        self.bridge.status_received.connect(self._on_status_received)
        self.bridge.camera_received.connect(self._on_camera_received)
        self.bridge.anomaly_received.connect(self._on_anomaly_received)
        self.bridge.shelf_arrived_received.connect(self._on_shelf_arrived)
        self.bridge.mission_armed_received.connect(self._on_mission_armed)

    # ==============================================================================
    # 5. GRAPHICS UPDATE SLOTS (EVENTS)
    # ==============================================================================
    def _on_status_received(self, robot_name: str, data: dict):
        status = data.get("status", "UNKNOWN")
        battery = data.get("battery", 100.0)
        x = data.get("x", 0.0)
        y = data.get("y", 0.0)

        # Feed the real-time battery history data into custom QPainter graph
        self.battery_graph.add_data(robot_name, battery)

        # Update labels with large crisp fonts
        card_lbls = self.robot_cards[robot_name]
        status_text = f">> {robot_name.upper()}: [ {status} ]  BATTERY: {battery:.1f}%"
        card_lbls["status"].setText(status_text)

        if status == "FAILED":
            card_lbls["status"].setStyleSheet("color: #FF3333; font-weight: bold; font-size: 11pt;")
        elif status == "AMCL_RECOVERY_SPIN":
            card_lbls["status"].setStyleSheet("color: #FF9900; font-weight: bold; font-size: 11pt;")
        elif status == "IDLE":
            card_lbls["status"].setStyleSheet("color: #4C566A; font-weight: bold; font-size: 11pt;")
        else:
            card_lbls["status"].setStyleSheet("color: #00FF66; font-weight: bold; font-size: 11pt;")

        card_lbls["pos"].setText(f"   Pos: ({x:6.2f}, {y:6.2f})")

    def _on_camera_received(self, robot_name: str, q_img: QImage):
        """Drives dynamic high-fidelity image rendering on scan event."""
        pixmap = QPixmap.fromImage(q_img)
        # Scaled smoothly matching the ministacked box footprint size
        scaled_pixmap = pixmap.scaled(self.camera_displays[robot_name].size(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
        self.camera_displays[robot_name].setPixmap(scaled_pixmap)

    def _on_anomaly_received(self, data: dict):
        robot_id = data.get("robot_id", "unknown")
        shelf_id = data.get("shelf_id", "unknown")
        level = data.get("level", 1)
        class_name = data.get("class_name", "damaged")
        confidence = data.get("confidence", 1.0)
        latency = data.get("latency_ms", 0.0)

        self.anomaly_latencies.append(latency)
        avg_latency = sum(self.anomaly_latencies) / len(self.anomaly_latencies)
        self.lbl_latency.setText(f"Anomaly Latency: {avg_latency:.1f}ms")

        if class_name == "damaged":
            alert_text = f"[{robot_id.upper()}] {shelf_id} (L{level}) -> [DAMAGED] {confidence*100:.1f}%"
            self.defects_list.append(alert_text)
            self.log_terminal.append(f"<font color='#FF3333'>[ANOMALY ALERT] {alert_text}</font>")

    def _on_shelf_arrived(self, data: dict):
        robot_id = data.get("robot_id", "unknown")
        shelf_id = data.get("shelf_id", "unknown")
        
        # Guard: Deduplicate completed count
        if shelf_id not in self.completed_waypoints_global:
            self.completed_waypoints_global.add(shelf_id)
            self.log_terminal.append(f"[TASK DONE] {robot_id} successfully inspected: {shelf_id}")
            
            # Progress bar matching
            if robot_id in self.robot_progress_bars:
                val = self.robot_progress_bars[robot_id].value() + 1
                self.robot_progress_bars[robot_id].setValue(val)
                
            # Compute global coverage loss
            completed_count = len(self.completed_waypoints_global)
            coverage_pct = (completed_count / self.total_target_waypoints) * 100.0
            coverage_loss = self.total_target_waypoints - completed_count
            self.lbl_coverage.setText(f"Coverage Loss: {coverage_loss} ({100.0 - coverage_pct:.1f}%)")

    def _on_mission_armed(self, is_armed: bool):
        if is_armed:
            self.mission_state_label.setText("MISSION STATE: [ ACTIVE ]")
            self.mission_state_label.setStyleSheet("color: #00FF66;")
            if self.start_time is None:
                self.start_time = time.time()
                self.log_terminal.append("[SYSTEM] Global Mission ARMED. Timers running.")

    def _update_chronometer(self):
        """Fires 1Hz to compute mission run durations."""
        self.time_label.setText(f"SYSTEM TIME: {time.strftime('%Y-%m-%d %H:%M:%S', time.gmtime())} UTC")
        if self.start_time is not None and self.mission_state_label.text() == "MISSION STATE: [ ACTIVE ]":
            elapsed = time.time() - self.start_time
            self.lbl_audit_time.setText(f"Total Audit Time: {elapsed:.1f}s")

    # ==============================================================================
    # 6. ACTION TRIGGERS (BUTTONS)
    # ==============================================================================
    def _handle_dispatch(self):
        self.log_terminal.append("[CMD SENT] Sending Volatile Trigger to arm all robot agents...")
        self.ros2_node.trigger_mission_start()

    def _handle_estop(self):
        self.log_terminal.append("<font color='#FF3333'>[EMERGENCY] ESTOP SHUTDOWN TRIGGERED! Sending failsafe command...</font>")
        # Inject failure to all robots to stop motion
        for name in ["robot1", "robot2", "robot3"]:
            self.ros2_node.inject_failure(name)

    def _handle_inject_failure(self):
        target_robot = self.combo_robots.currentText()
        self.log_terminal.append(f"<font color='#FF9900'>[FAILURE INJECTED] Simulating hardware crash on {target_robot}...</font>")
        self.ros2_node.inject_failure(target_robot)


# ==============================================================================
# 7. MAIN ENTRYPOINT
# ==============================================================================
def closeEvent(self, event):
        """Cleanly destroys ROS2 contexts when GUI window is closed."""
        try:
            # Force rclpy shutdown to unblock rclpy.spin in the background thread
            rclpy.try_shutdown()
        except Exception:
            pass
        event.accept()

def main(args=None):
    # Instruct Python to let OS terminate the GUI cleanly on Ctrl+C without tracebacks
    signal.signal(signal.SIGINT, signal.SIG_DFL)

    try:
        # Initialize rclpy without registering custom ROS signal handlers
        rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    except Exception:
        pass
        
    app = QApplication(sys.argv)

    bridge = DashboardBridge()
    node = DashboardROS2Node(bridge)
    
    ros_thread = ROS2SpinThread(node)
    ros_thread.start()

    gui = FleetInspectionDashboard(node, bridge)
    gui.show()

    exit_code = app.exec_()
    
    try:
        node.destroy_node()
    except Exception:
        pass
        
    try:
        rclpy.try_shutdown()
    except Exception:
        pass
        
    ros_thread.wait()
    sys.exit(exit_code)


if __name__ == "__main__":
    main()