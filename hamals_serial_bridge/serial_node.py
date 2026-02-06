#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

import serial
import threading
import time
import math

from .protocol import encode_cmd
from .parser import LineParser


def yaw_to_quat(yaw: float):
    """Yaw (rad) → quaternion"""
    qz = math.sin(yaw * 0.5)
    qw = math.cos(yaw * 0.5)
    return (0.0, 0.0, qz, qw)


class SerialBridgeNode(Node):
    def __init__(self):
        super().__init__('hamals_serial_bridge')

        # -------------------- PARAMETERS --------------------
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('rate_hz', 50)
        self.declare_parameter('timeout_ms', 100)

        self.port       = self.get_parameter('port').value
        self.baudrate   = self.get_parameter('baudrate').value
        self.rate_hz    = self.get_parameter('rate_hz').value
        self.timeout_ms = self.get_parameter('timeout_ms').value

        # -------------------- SERIAL -----------------------
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

        self.parser = LineParser()

        # -------------------- ROS --------------------------
        self.cmd_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        self.odom_pub = self.create_publisher(
            Odometry,
            '/odom',
            10
        )

        # -------------------- THREAD -----------------------
        self._rx_thread = threading.Thread(
            target=self.serial_rx_loop,
            daemon=True
        )
        self._rx_thread.start()

        self.get_logger().info("hamals_serial_bridge started")

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
                msg = self.parser.feed(decoded)

                if msg:
                    self.handle_serial_message(msg)

            except Exception as e:
                self.get_logger().warn(f"Serial RX error: {e}")
                time.sleep(0.1)

    def handle_serial_message(self, msg: dict):
        if msg['type'] != 'odom':
            return

        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        # Position
        odom.pose.pose.position.x = msg['x']
        odom.pose.pose.position.y = msg['y']

        # Orientation
        qx, qy, qz, qw = yaw_to_quat(msg['yaw'])
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        # Velocity
        odom.twist.twist.linear.x = msg['v']
        odom.twist.twist.angular.z = msg['w']

        self.odom_pub.publish(odom)


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