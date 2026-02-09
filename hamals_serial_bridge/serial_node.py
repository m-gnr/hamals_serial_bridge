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
        # REQUIRED
        self.declare_parameter('port')
        self.declare_parameter('baudrate')
        self.declare_parameter('cmd_vel_topic')
        self.declare_parameter('odom_topic')

        # OPTIONAL
        self.declare_parameter('timeout_ms', 50)
        self.declare_parameter('odom_pub_hz', 50)

        self.declare_parameter('reset_on_startup', True)
        self.declare_parameter('reset_pulse_ms', 100)
        self.declare_parameter('reset_boot_wait_ms', 1500)

        # -------------------- REQUIRED PARAM CHECK --------------------
        self.port = self._get_required_param('port')
        self.baudrate = self._get_required_param('baudrate')
        self.cmd_vel_topic = self._get_required_param('cmd_vel_topic')
        self.odom_topic = self._get_required_param('odom_topic')

        # -------------------- OPTIONAL PARAMS --------------------
        self.timeout_ms = self.get_parameter('timeout_ms').value
        self.odom_pub_hz = self.get_parameter('odom_pub_hz').value

        self.reset_on_startup   = self.get_parameter('reset_on_startup').value
        self.reset_pulse_ms     = self.get_parameter('reset_pulse_ms').value
        self.reset_boot_wait_ms = self.get_parameter('reset_boot_wait_ms').value

        # odom publish rate control
        self._odom_pub_period = 1.0 / float(self.odom_pub_hz)
        self._last_odom_pub_time = 0.0

        # -------------------- SERIAL SETUP --------------------
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

        # -------------------- RX THREAD ---------------------
        self._rx_thread = threading.Thread(
            target=self.serial_rx_loop,
            daemon=True
        )
        self._rx_thread.start()

        self.get_logger().info("hamals_serial_bridge node started")

    # ======================================================
    # REQUIRED PARAM HELPER
    # ======================================================
    def _get_required_param(self, name: str):
        param = self.get_parameter(name)
        if param.type_ == param.Type.NOT_SET:
            raise RuntimeError(f"Missing required parameter: {name}")
        return param.value

    # ======================================================
    # SERIAL / MCU CONTROL
    # ======================================================
    def _connect_serial(self):
        """Serial port bağlantısını kur"""
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
        """MCU'yu DTR üzerinden resetle"""
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

        line = encode_cmd(v, w)

        try:
            self.ser.write(line.encode('utf-8'))
        except Exception as e:
            self.get_logger().error(f"Serial TX error: {e}")

    # ======================================================
    # MCU → ROS
    # ======================================================
    def serial_rx_loop(self):
        while rclpy.ok():
            try:
                raw = self.ser.read(128)
                if not raw:
                    continue

                decoded = raw.decode('utf-8', errors='ignore')
                messages = self.parser.push(decoded)

                for msg in messages:
                    self.handle_serial_message(msg)

            except Exception as e:
                self.get_logger().warn(f"Serial RX error: {e}")
                time.sleep(0.1)

    def handle_serial_message(self, msg: dict):
        if msg.get('type') != 'odom':
            return

        now = time.time()
        if now - self._last_odom_pub_time < self._odom_pub_period:
            return
        self._last_odom_pub_time = now

        self._publish_odom(msg)

    # ======================================================
    # ODOM PUBLISH
    # ======================================================
    def _publish_odom(self, msg: dict):
        """Odometry mesajını yayınla"""
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