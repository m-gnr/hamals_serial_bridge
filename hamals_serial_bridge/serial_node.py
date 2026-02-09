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


# ======================================================
# SERIAL BRIDGE NODE
# ======================================================
class SerialBridgeNode(Node):
    def __init__(self):
        super().__init__('hamals_serial_bridge')

        # -------------------- PARAMETERS --------------------
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('odom_topic', '/odom_raw')

        self.declare_parameter('timeout_ms', 50)
        self.declare_parameter('odom_pub_hz', 50)

        self.declare_parameter('reset_on_startup', True)
        self.declare_parameter('reset_pulse_ms', 100)
        self.declare_parameter('reset_boot_wait_ms', 1500)

        self.declare_parameter('cmd_vel_timeout_ms', 500)

        # DEBUG FLAGS
        self.declare_parameter('debug_rx', False)
        self.declare_parameter('debug_tx', False)
        self.declare_parameter('debug_odom', False)
        self.declare_parameter('debug_deadman', False)

        # -------------------- PARAM READ --------------------
        self.port = self.get_parameter('port').value
        self.baudrate = self.get_parameter('baudrate').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value

        self.timeout_ms = self.get_parameter('timeout_ms').value
        self.odom_pub_hz = self.get_parameter('odom_pub_hz').value
        if self.odom_pub_hz <= 0:
            raise ValueError("odom_pub_hz must be > 0")

        self.reset_on_startup = self.get_parameter('reset_on_startup').value
        self.reset_pulse_ms = self.get_parameter('reset_pulse_ms').value
        self.reset_boot_wait_ms = self.get_parameter('reset_boot_wait_ms').value

        self.cmd_vel_timeout_ms = self.get_parameter('cmd_vel_timeout_ms').value

        self.debug_rx = self.get_parameter('debug_rx').value
        self.debug_tx = self.get_parameter('debug_tx').value
        self.debug_odom = self.get_parameter('debug_odom').value
        self.debug_deadman = self.get_parameter('debug_deadman').value

        # -------------------- STATE --------------------
        self._odom_pub_period = 1.0 / float(self.odom_pub_hz)
        self._last_odom_pub_time = 0.0

        self._last_cmd_time = time.time()
        self._deadman_active = False
        self._running = True

        # -------------------- SERIAL --------------------
        self.ser = None
        self._connect_serial()
        self._reset_mcu()

        self.parser = LineParser()

        # -------------------- ROS INTERFACES ----------------
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

        # -------------------- PRINT CONFIG SUMMARY ----------------
        self._print_config_summary()

        # -------------------- RX THREAD --------------------
        self._rx_thread = threading.Thread(
            target=self.serial_rx_loop,
            daemon=True
        )
        self._rx_thread.start()

        self.get_logger().info("hamals_serial_bridge started")

    # ======================================================
    # CONFIG SUMMARY
    # ======================================================
    def _print_config_summary(self):
        msg = f"""
    ===== hamals_serial_bridge CONFIG =====
    port               : {self.port}
    baudrate           : {self.baudrate}
    timeout_ms         : {self.timeout_ms}
    cmd_vel_topic      : {self.cmd_vel_topic}
    odom_topic         : {self.odom_topic}
    odom_pub_hz        : {self.odom_pub_hz}
    reset_on_startup   : {self.reset_on_startup}
    cmd_vel_timeout_ms : {self.cmd_vel_timeout_ms}

    ---- debug flags ----
    debug_tx           : {self.debug_tx}
    debug_rx           : {self.debug_rx}
    debug_odom         : {self.debug_odom}
    debug_deadman      : {self.debug_deadman}
    ======================================
    """
        self.get_logger().info(msg)

    # ======================================================
    # DEBUG HELPER
    # ======================================================
    def _debug(self, flag: bool, tag: str, msg: str):
        if flag:
            self.get_logger().info(f"[{tag}] {msg}")

    # ======================================================
    # SERIAL / MCU
    # ======================================================
    def _connect_serial(self):
        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=self.timeout_ms / 1000.0
            )
            self.get_logger().info(
                f"Serial connected: {self.port} @ {self.baudrate}"
            )
        except Exception as e:
            self.get_logger().fatal(f"Serial open failed: {e}")
            raise

    def _reset_mcu(self):
        if not self.reset_on_startup or self.ser is None:
            return

        try:
            self.get_logger().info("Resetting MCU via DTR")
            self.ser.dtr = False
            time.sleep(self.reset_pulse_ms / 1000.0)
            self.ser.dtr = True
            time.sleep(self.reset_boot_wait_ms / 1000.0)
        except Exception as e:
            self.get_logger().warn(f"MCU reset failed: {e}")

    # ======================================================
    # ROS → MCU
    # ======================================================
    def cmd_vel_callback(self, msg: Twist):
        v = msg.linear.x
        w = msg.angular.z
        self._last_cmd_time = time.time()

        self._debug(self.debug_tx, "TX", f"cmd_vel v={v:.3f} w={w:.3f}")

        try:
            self.ser.write(encode_cmd(v, w).encode('utf-8'))
        except Exception as e:
            self.get_logger().error(f"Serial TX error: {e}")

    # ======================================================
    # MCU → ROS
    # ======================================================
    def serial_rx_loop(self):
        while rclpy.ok() and self._running:
            try:
                # -------- Dead-man (edge triggered) --------
                timeout = (
                    time.time() - self._last_cmd_time
                    > self.cmd_vel_timeout_ms / 1000.0
                )

                if timeout and not self._deadman_active:
                    self._debug(
                        self.debug_deadman,
                        "SAFETY",
                        "Dead-man triggered → STOP"
                    )
                    try:
                        self.ser.write(
                            encode_cmd(0.0, 0.0).encode('utf-8')
                        )
                    except:
                        pass
                    self._deadman_active = True
                elif not timeout:
                    self._deadman_active = False

                raw = self.ser.read(128)
                if not raw:
                    continue

                decoded = raw.decode('utf-8', errors='ignore')
                self._debug(self.debug_rx, "RX", decoded.strip())

                messages = self.parser.push(decoded)
                for msg in messages:
                    self._debug(self.debug_rx, "PARSE", str(msg))
                    self.handle_serial_message(msg)

            except Exception as e:
                self.get_logger().warn(f"Serial RX error: {e}")
                time.sleep(0.1)

    # ======================================================
    # ODOM
    # ======================================================
    def handle_serial_message(self, msg: dict):
        if msg.get('type') != 'odom':
            return

        now = time.time()
        if now - self._last_odom_pub_time < self._odom_pub_period:
            return
        self._last_odom_pub_time = now

        self._publish_odom(msg)

    def _publish_odom(self, msg: dict):
        self._debug(
            self.debug_odom,
            "ODOM",
            f"x={msg['x']:.3f} y={msg['y']:.3f} yaw={msg['yaw']:.3f}"
        )

        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        odom.pose.pose.position.x = msg['x']
        odom.pose.pose.position.y = msg['y']

        qx, qy, qz, qw = yaw_to_quat(msg['yaw'])
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        odom.twist.twist.linear.x = msg['v']
        odom.twist.twist.angular.z = msg['w']

        self.odom_pub.publish(odom)

    # ======================================================
    # CLEAN SHUTDOWN
    # ======================================================
    def destroy_node(self):
        self._running = False
        try:
            if self.ser and self.ser.is_open:
                self.get_logger().info("Closing serial connection")
                self.ser.close()
        except Exception as e:
            self.get_logger().warn(f"Serial close error: {e}")

        super().destroy_node()


# ======================================================
# MAIN
# ======================================================
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