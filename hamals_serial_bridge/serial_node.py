#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

import serial
import threading
import time

from .protocol import encode_cmd
from .parser import LineParser
from .utils import yaw_to_quat


class SerialBridgeNode(Node):

    def __init__(self):
        super().__init__('hamals_serial_bridge')

        # ==================== PARAMETERS ====================
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('timeout_ms', 50)

        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('odom_pub_hz', 50)

        self.declare_parameter('pose_covariance', [0.0] * 36)
        self.declare_parameter('twist_covariance', [0.0] * 36)

        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('child_frame_id', 'base_link')

        self.declare_parameter('reset_on_startup', True)
        self.declare_parameter('reset_pulse_ms', 100)
        self.declare_parameter('reset_boot_wait_ms', 1500)

        self.declare_parameter('cmd_vel_timeout_ms', 500)
        self.declare_parameter('debug', False)

        # ==================== PARAM READ ====================
        self.port = self.get_parameter('port').value
        self.baudrate = self.get_parameter('baudrate').value
        self.timeout_ms = self.get_parameter('timeout_ms').value

        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.odom_pub_hz = self.get_parameter('odom_pub_hz').value

        self.pose_covariance = self.get_parameter('pose_covariance').value
        self.twist_covariance = self.get_parameter('twist_covariance').value

        self.frame_id = self.get_parameter('frame_id').value
        self.child_frame_id = self.get_parameter('child_frame_id').value

        self.reset_on_startup = self.get_parameter('reset_on_startup').value
        self.reset_pulse_ms = self.get_parameter('reset_pulse_ms').value
        self.reset_boot_wait_ms = self.get_parameter('reset_boot_wait_ms').value

        self.cmd_vel_timeout_ms = self.get_parameter('cmd_vel_timeout_ms').value
        self.debug = self.get_parameter('debug').value

        if len(self.pose_covariance) != 36:
            raise ValueError("pose_covariance must contain 36 elements")

        if len(self.twist_covariance) != 36:
            raise ValueError("twist_covariance must contain 36 elements")

        # ==================== STATE ====================
        self._odom_pub_period = 1.0 / float(self.odom_pub_hz)
        self._last_odom_pub_time = 0.0

        self._last_cmd_time = time.time()
        self._deadman_active = False
        self._running = True

        # Debug
        self._dbg_tx = 0
        self._dbg_odom = 0
        self._dbg_last_cmd = (0.0, 0.0)
        self._dbg_last_odom = (0.0, 0.0, 0.0)

        self._dbg_rx_bytes = 0
        self._dbg_rx_frames = 0
        self._dbg_rx_invalid = 0

        # ==================== SERIAL ====================
        self.ser = None
        self._tx_lock = threading.Lock()

        self._connect_serial()
        self._reset_mcu()

        self.parser = LineParser()

        # ==================== ROS ====================
        self.cmd_sub = self.create_subscription(
            Twist,
            self.cmd_vel_topic,
            self.cmd_vel_callback,
            10
        )

        self.odom_pub = self.create_publisher(
            Odometry,
            self.odom_topic,
            10
        )

        if self.debug:
            self.create_timer(1.0, self._print_debug_panel)

        self._rx_thread = threading.Thread(
            target=self.serial_rx_loop,
            daemon=True
        )
        self._rx_thread.start()

        self.get_logger().info("hamals_serial_bridge started")

    # =====================================================
    # SERIAL SAFE WRITE
    # =====================================================
    def _serial_write(self, data: bytes):
        if not self.ser:
            return
        with self._tx_lock:
            self.ser.write(data)

    # =====================================================
    # SERIAL CONNECT
    # =====================================================
    def _connect_serial(self):
        self.ser = serial.Serial(
            port=self.port,
            baudrate=self.baudrate,
            timeout=self.timeout_ms / 1000.0
        )
        self.get_logger().info(
            f"Serial connected: {self.port} @ {self.baudrate}"
        )

    def _reset_mcu(self):
        if not self.reset_on_startup:
            return

        self.get_logger().info("Resetting MCU via DTR")
        self.ser.dtr = False
        time.sleep(self.reset_pulse_ms / 1000.0)
        self.ser.dtr = True
        time.sleep(self.reset_boot_wait_ms / 1000.0)

    # =====================================================
    # ROS → MCU
    # =====================================================
    def cmd_vel_callback(self, msg: Twist):
        v = msg.linear.x
        w = msg.angular.z

        self._last_cmd_time = time.time()
        self._dbg_tx += 1
        self._dbg_last_cmd = (v, w)

        self._serial_write(encode_cmd(v, w).encode('utf-8'))

    # =====================================================
    # MCU → ROS
    # =====================================================
    def serial_rx_loop(self):
        while rclpy.ok() and self._running:
            try:
                # Deadman
                timeout = (
                    time.time() - self._last_cmd_time
                    > self.cmd_vel_timeout_ms / 1000.0
                )

                if timeout and not self._deadman_active:
                    self._serial_write(
                        encode_cmd(0.0, 0.0).encode('utf-8')
                    )
                    self._deadman_active = True
                elif not timeout:
                    self._deadman_active = False

                raw = self.ser.read(128)
                if not raw:
                    continue

                decoded = raw.decode('utf-8', errors='ignore')
                messages = self.parser.push(decoded)

                self._dbg_rx_bytes = self.parser.bytes_received
                self._dbg_rx_frames = self.parser.valid_frames
                self._dbg_rx_invalid = self.parser.invalid_frames

                for msg in messages:
                    self.handle_serial_message(msg)

            except Exception as e:
                self.get_logger().warn(f"RX error: {e}", throttle_duration_sec=5.0)
                time.sleep(0.1)

    # =====================================================
    # ODOM
    # =====================================================
    def handle_serial_message(self, msg: dict):
        if msg.get('type') != 'odom':
            return

        now = time.time()
        if now - self._last_odom_pub_time < self._odom_pub_period:
            return

        self._last_odom_pub_time = now
        self._publish_odom(msg)

    def _publish_odom(self, msg: dict):
        self._dbg_odom += 1
        self._dbg_last_odom = (msg['x'], msg['y'], msg['yaw'])

        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = self.frame_id
        odom.child_frame_id = self.child_frame_id

        odom.pose.pose.position.x = msg['x']
        odom.pose.pose.position.y = msg['y']

        qx, qy, qz, qw = yaw_to_quat(msg['yaw'])
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        odom.pose.covariance = self.pose_covariance
        odom.twist.covariance = self.twist_covariance

        odom.twist.twist.linear.x = msg['v']
        odom.twist.twist.angular.z = msg['w']

        self.odom_pub.publish(odom)

    # =====================================================
    # DEBUG
    # =====================================================
    def _print_debug_panel(self):
        v, w = self._dbg_last_cmd
        x, y, yaw = self._dbg_last_odom

        panel = f"""
╔══════════════════════════════════════════════════════╗
║        HAMALS SERIAL BRIDGE — DEBUG PANEL            ║
╠══════════════════════════════════════════════════════╣
║ RX bytes          : {self._dbg_rx_bytes:<30}║
║ RX valid frames   : {self._dbg_rx_frames:<30}║
║ RX invalid frames : {self._dbg_rx_invalid:<30}║
║ TX packets        : {self._dbg_tx:<30}║
║ ODOM published    : {self._dbg_odom:<30}║
║ Dead-man active   : {str(self._deadman_active):<30}║
╠══════════════════════════════════════════════════════╣
║ Last cmd_vel      : v={v:.2f}  w={w:.2f}              ║
║ Last odom         : x={x:.2f} y={y:.2f} yaw={yaw:.2f} ║
╚══════════════════════════════════════════════════════╝
"""
        self.get_logger().info(panel)

    # =====================================================
    # SHUTDOWN
    # =====================================================
    def destroy_node(self):
        self._running = False
        if self.ser and self.ser.is_open:
            self.ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SerialBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()