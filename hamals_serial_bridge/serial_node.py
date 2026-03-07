#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

import serial
import threading
import time
import math

from .protocol import encode_cmd
from .parser import LineParser


class SerialBridgeNode(Node):

    def __init__(self):
        super().__init__('hamals_serial_bridge')

        # ==================== PARAMETERS ====================
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('timeout_ms', 50)

        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('odom_topic', '/odom_raw')
        self.declare_parameter('imu_topic', '/imu/data')

        self.declare_parameter('pose_covariance', [0.0] * 36)
        self.declare_parameter('twist_covariance', [0.0] * 36)

        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('child_frame_id', 'base_footprint')
        self.declare_parameter('imu_frame_id', 'imu_link')

        self.declare_parameter('reset_on_startup', True)
        self.declare_parameter('reset_pulse_ms', 100)
        self.declare_parameter('reset_boot_wait_ms', 1500)

        self.declare_parameter('cmd_vel_timeout_ms', 500)
        self.declare_parameter('cmd_vel_rate_limit_hz', 50)
        self.declare_parameter('debug', False)

        self.declare_parameter('cmd_dedup_enabled', True)
        self.declare_parameter('cmd_dedup_eps_v', 0.01)
        self.declare_parameter('cmd_dedup_eps_w', 0.02)
        self.declare_parameter('cmd_force_resend_ms', 300)

        self.declare_parameter('wheel_radius_m', 0.035)
        self.declare_parameter('track_width_m', 0.18)
        self.declare_parameter('cpr_left', 3959)
        self.declare_parameter('cpr_right', 3963)

        self.declare_parameter('enc_dt_max_s', 0.5)
        self.declare_parameter('enc_dt_min_s', 0.0)

        self.declare_parameter('imu_ang_vel_cov_diag', [9999.0, 9999.0, 0.005])
        self.declare_parameter('imu_lin_acc_cov_diag', [0.01, 9999.0, 9999.0])

        self.declare_parameter('publish_linear_accel', True)

        # ==================== PARAM READ ====================
        self.port = self.get_parameter('port').value
        self.baudrate = self.get_parameter('baudrate').value
        self.timeout_ms = self.get_parameter('timeout_ms').value

        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.imu_topic = self.get_parameter('imu_topic').value

        self.pose_covariance = self.get_parameter('pose_covariance').value
        self.twist_covariance = self.get_parameter('twist_covariance').value

        self.frame_id = self.get_parameter('frame_id').value
        self.child_frame_id = self.get_parameter('child_frame_id').value
        self.imu_frame_id = self.get_parameter('imu_frame_id').value

        self.reset_on_startup = self.get_parameter('reset_on_startup').value
        self.reset_pulse_ms = self.get_parameter('reset_pulse_ms').value
        self.reset_boot_wait_ms = self.get_parameter('reset_boot_wait_ms').value

        self.cmd_vel_timeout_ms = self.get_parameter('cmd_vel_timeout_ms').value
        self.cmd_vel_rate_limit_hz = self.get_parameter('cmd_vel_rate_limit_hz').value
        self.debug = self.get_parameter('debug').value

        self.cmd_dedup_enabled = bool(self.get_parameter('cmd_dedup_enabled').value)
        self.cmd_dedup_eps_v = float(self.get_parameter('cmd_dedup_eps_v').value)
        self.cmd_dedup_eps_w = float(self.get_parameter('cmd_dedup_eps_w').value)
        self.cmd_force_resend_ms = int(self.get_parameter('cmd_force_resend_ms').value)

        self.wheel_radius_m = self.get_parameter('wheel_radius_m').value
        self.track_width_m = self.get_parameter('track_width_m').value
        self.cpr_left = self.get_parameter('cpr_left').value
        self.cpr_right = self.get_parameter('cpr_right').value

        self.enc_dt_max_s = float(self.get_parameter('enc_dt_max_s').value)
        self.enc_dt_min_s = float(self.get_parameter('enc_dt_min_s').value)

        self.imu_ang_vel_cov_diag = self.get_parameter('imu_ang_vel_cov_diag').value
        self.imu_lin_acc_cov_diag = self.get_parameter('imu_lin_acc_cov_diag').value

        self.publish_linear_accel = bool(self.get_parameter('publish_linear_accel').value)

        if len(self.pose_covariance) != 36:
            raise ValueError("pose_covariance must contain 36 elements")
        if len(self.twist_covariance) != 36:
            raise ValueError("twist_covariance must contain 36 elements")
        if len(self.imu_ang_vel_cov_diag) != 3:
            raise ValueError("imu_ang_vel_cov_diag must contain 3 elements")
        if len(self.imu_lin_acc_cov_diag) != 3:
            raise ValueError("imu_lin_acc_cov_diag must contain 3 elements")

        # ==================== STATE ====================
        self._last_enc_time_s = None
        self._last_cmd_time = time.time()
        self._last_cmd_send_time = 0.0
        self._cmd_send_min_interval = 1.0 / float(self.cmd_vel_rate_limit_hz) if self.cmd_vel_rate_limit_hz > 0 else 0.0
        self._deadman_active = False
        self._running = True

        self._last_sent_cmd_v = None
        self._last_sent_cmd_w = None
        self._last_sent_cmd_time = 0.0

        self._dbg_tx = 0
        self._dbg_tx_skipped_dedup = 0
        self._dbg_tx_skipped_ratelimit = 0
        self._dbg_odom = 0
        self._dbg_last_cmd = (0.0, 0.0)
        self._dbg_last_odom = (0.0, 0.0)
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

        self.imu_pub = self.create_publisher(
            Imu,
            self.imu_topic,
            10
        )

        if self.debug:
            self.create_timer(1.0, self._print_debug_panel)

        # Timeout kontrolü için timer (50ms periyot)
        self.create_timer(0.05, self._check_cmd_timeout)

        self._rx_thread = threading.Thread(
            target=self.serial_rx_loop,
            daemon=True
        )
        self._rx_thread.start()

        self._print_startup_config()
        self.get_logger().info("hamals_serial_bridge started")

    # =====================================================
    # STARTUP CONFIG PANEL
    # =====================================================
    def _print_startup_config(self):
        panel = f"""
╔══════════════════════════════════════════════════════╗
║        HAMALS SERIAL BRIDGE — STARTUP CONFIG         ║
╠══════════════════════════════════════════════════════╣
║ Port                : {self.port:<30}║
║ Baudrate            : {self.baudrate:<30}║
║ Timeout (ms)        : {self.timeout_ms:<30}║
╠══════════════════════════════════════════════════════╣
║ cmd_vel_topic       : {self.cmd_vel_topic:<30}║
║ odom_topic          : {self.odom_topic:<30}║
║ imu_topic           : {self.imu_topic:<30}║
╠══════════════════════════════════════════════════════╣
║ frame_id            : {self.frame_id:<30}║
║ child_frame_id      : {self.child_frame_id:<30}║
║ imu_frame_id        : {self.imu_frame_id:<30}║
╠══════════════════════════════════════════════════════╣
║ wheel_radius_m      : {self.wheel_radius_m:<30}║
║ track_width_m       : {self.track_width_m:<30}║
║ cpr_left            : {self.cpr_left:<30}║
║ cpr_right           : {self.cpr_right:<30}║
╠══════════════════════════════════════════════════════╣
║ enc_dt_max_s        : {self.enc_dt_max_s:<30}║
║ enc_dt_min_s        : {self.enc_dt_min_s:<30}║
║ publish_linear_accel: {str(self.publish_linear_accel):<30}║
║ imu_ang_vel_cov_diag: {str(self.imu_ang_vel_cov_diag):<30}║
║ imu_lin_acc_cov_diag: {str(self.imu_lin_acc_cov_diag):<30}║
╠══════════════════════════════════════════════════════╣
║ reset_on_startup    : {str(self.reset_on_startup):<30}║
║ cmd_vel_timeout_ms  : {self.cmd_vel_timeout_ms:<30}║
║ cmd_vel_rate_limit  : {self.cmd_vel_rate_limit_hz:<30}║
║ dedup_enabled       : {str(self.cmd_dedup_enabled):<30}║
║ dedup_eps_v         : {self.cmd_dedup_eps_v:<30}║
║ dedup_eps_w         : {self.cmd_dedup_eps_w:<30}║
║ force_resend_ms     : {self.cmd_force_resend_ms:<30}║
║ debug               : {str(self.debug):<30}║
╚══════════════════════════════════════════════════════╝
"""
        self.get_logger().info(panel)

    # =====================================================
    # SERIAL SAFE WRITE
    # =====================================================
    def _serial_write(self, data: bytes) -> bool:
        """Serial'e yaz. Başarılıysa True, değilse False döner."""
        if not self.ser or not self.ser.is_open:
            return False
        try:
            with self._tx_lock:
                self.ser.write(data)
            return True
        except Exception as e:
            self.get_logger().warn(f"Serial write error: {e}", throttle_duration_sec=5.0)
            return False

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
    # CMD HELPERS
    # =====================================================
    def _is_stop_cmd(self, v: float, w: float) -> bool:
        return abs(v) < 1e-6 and abs(w) < 1e-6

    def _send_cmd(self, v: float, w: float) -> bool:
        """cmd_vel gönder. Sadece başarılı yazımda state günceller."""
        if self._serial_write(encode_cmd(v, w).encode('utf-8')):
            self._dbg_tx += 1
            self._last_sent_cmd_v = v
            self._last_sent_cmd_w = w
            self._last_sent_cmd_time = time.time()
            return True
        return False

    def _maybe_send_cmd(self, v: float, w: float, force: bool = False):
        now = time.time()

        # 1. Force komutu → her zaman gönder
        if force:
            if self._send_cmd(v, w):
                self._last_cmd_send_time = now
            return

        # 2. İlk komut → her zaman gönder
        if self._last_sent_cmd_v is None or self._last_sent_cmd_w is None:
            if self._send_cmd(v, w):
                self._last_cmd_send_time = now
            return

        # 3. Kritik geçişler: stop <-> hareket → her zaman gönder (rate limit'e bakmadan)
        prev_stop = self._is_stop_cmd(self._last_sent_cmd_v, self._last_sent_cmd_w)
        new_stop = self._is_stop_cmd(v, w)
        if prev_stop != new_stop:
            if self._send_cmd(v, w):
                self._last_cmd_send_time = now
            return

        # 4. Dedup kapalıysa normal gönder (rate limit'e tabi)
        if not self.cmd_dedup_enabled:
            # Rate limiting kontrolü
            if self._cmd_send_min_interval > 0:
                time_since_last = now - self._last_cmd_send_time
                if time_since_last < self._cmd_send_min_interval:
                    self._dbg_tx_skipped_ratelimit += 1
                    return
            if self._send_cmd(v, w):
                self._last_cmd_send_time = now
            return

        # 5. Değişim kontrolü
        dv = abs(v - self._last_sent_cmd_v)
        dw = abs(w - self._last_sent_cmd_w)
        changed = (dv > self.cmd_dedup_eps_v) or (dw > self.cmd_dedup_eps_w)

        if changed:
            # Değişim var → rate limit kontrolü yap
            if self._cmd_send_min_interval > 0:
                time_since_last = now - self._last_cmd_send_time
                if time_since_last < self._cmd_send_min_interval:
                    self._dbg_tx_skipped_ratelimit += 1
                    return
            if self._send_cmd(v, w):
                self._last_cmd_send_time = now
        else:
            # Değişim yok → force resend kontrolü
            resend_due = (now - self._last_sent_cmd_time) * 1000.0 >= self.cmd_force_resend_ms
            if resend_due:
                # Force resend → rate limit'e bakmadan gönder (zaten 300ms seyrek)
                if self._send_cmd(v, w):
                    self._last_cmd_send_time = now
            else:
                self._dbg_tx_skipped_dedup += 1

    # =====================================================
    # TIMEOUT CHECKER
    # =====================================================
    def _check_cmd_timeout(self):
        """Periyodik olarak cmd_vel timeout kontrolü yapar."""
        timeout = (
            time.time() - self._last_cmd_time
            > self.cmd_vel_timeout_ms / 1000.0
        )

        if timeout and not self._deadman_active:
            self._maybe_send_cmd(0.0, 0.0, force=True)
            self._deadman_active = True
        elif not timeout:
            self._deadman_active = False

    # =====================================================
    # ROS → MCU
    # =====================================================
    def cmd_vel_callback(self, msg: Twist):
        v = msg.linear.x
        w = msg.angular.z

        self._last_cmd_time = time.time()
        self._dbg_last_cmd = (v, w)

        self._maybe_send_cmd(v, w, force=False)

    # =====================================================
    # MCU → ROS
    # =====================================================
    def serial_rx_loop(self):
        while rclpy.ok() and self._running:
            try:
                raw = self.ser.read_until(b'\n', size=256)
                if not raw:
                    continue

                decoded = raw.decode('utf-8', errors='ignore')
                messages = self.parser.push(decoded)
                for msg in messages:
                    self.handle_serial_message(msg)

                self._dbg_rx_bytes = self.parser.bytes_received
                self._dbg_rx_frames = self.parser.valid_frames
                self._dbg_rx_invalid = self.parser.invalid_frames

                while self.ser.in_waiting > 0:
                    raw2 = self.ser.read_until(b'\n', size=256)
                    if not raw2:
                        break

                    decoded2 = raw2.decode('utf-8', errors='ignore')
                    messages2 = self.parser.push(decoded2)
                    for msg2 in messages2:
                        self.handle_serial_message(msg2)

                    self._dbg_rx_bytes = self.parser.bytes_received
                    self._dbg_rx_frames = self.parser.valid_frames
                    self._dbg_rx_invalid = self.parser.invalid_frames

            except Exception as e:
                self.get_logger().warn(
                    f"RX error: {e}",
                    throttle_duration_sec=5.0
                )
                time.sleep(0.1)

    # =====================================================
    # MESSAGE HANDLING
    # =====================================================
    def handle_serial_message(self, msg: dict):
        t = msg.get('type')
        if t == 'enc':
            self._publish_odom_raw_from_enc(msg)
        elif t == 'imu':
            self._publish_imu(msg)

    # =====================================================
    # ENC → /odom_raw
    # =====================================================
    def _publish_odom_raw_from_enc(self, msg: dict):
        now_s = self.get_clock().now().nanoseconds / 1e9

        if self._last_enc_time_s is None:
            self._last_enc_time_s = now_s
            return

        dt = now_s - self._last_enc_time_s
        self._last_enc_time_s = now_s

        if dt <= self.enc_dt_min_s or dt > self.enc_dt_max_s:
            return

        dl = msg.get('dl', 0)
        dr = msg.get('dr', 0)

        dtheta_l = (2.0 * math.pi * dl) / self.cpr_left
        dtheta_r = (2.0 * math.pi * dr) / self.cpr_right

        omega_l = dtheta_l / dt
        omega_r = dtheta_r / dt

        v_l = omega_l * self.wheel_radius_m
        v_r = omega_r * self.wheel_radius_m

        v = 0.5 * (v_l + v_r)
        w = (v_r - v_l) / self.track_width_m

        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = self.frame_id
        odom.child_frame_id = self.child_frame_id

        odom.pose.pose.position.x = 0.0
        odom.pose.pose.position.y = 0.0
        odom.pose.pose.position.z = 0.0

        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = 0.0
        odom.pose.pose.orientation.w = 1.0

        odom.pose.covariance = self.pose_covariance
        odom.twist.covariance = self.twist_covariance

        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w

        self.odom_pub.publish(odom)

        self._dbg_odom += 1
        self._dbg_last_odom = (v, w)

    # =====================================================
    # IMU → /imu/data
    # =====================================================
    def _publish_imu(self, msg: dict):
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = self.imu_frame_id

        imu_msg.orientation_covariance[0] = -1.0

        imu_msg.angular_velocity.x = 0.0
        imu_msg.angular_velocity.y = 0.0
        imu_msg.angular_velocity.z = msg.get('gz', 0.0)

        imu_msg.angular_velocity_covariance = [
            float(self.imu_ang_vel_cov_diag[0]), 0.0, 0.0,
            0.0, float(self.imu_ang_vel_cov_diag[1]), 0.0,
            0.0, 0.0, float(self.imu_ang_vel_cov_diag[2])
        ]

        if self.publish_linear_accel:
            imu_msg.linear_acceleration.x = msg.get('ax', 0.0)
            imu_msg.linear_acceleration.y = msg.get('ay', 0.0)
            imu_msg.linear_acceleration.z = msg.get('az', 0.0)
        else:
            imu_msg.linear_acceleration.x = 0.0
            imu_msg.linear_acceleration.y = 0.0
            imu_msg.linear_acceleration.z = 0.0

        imu_msg.linear_acceleration_covariance = [
            float(self.imu_lin_acc_cov_diag[0]), 0.0, 0.0,
            0.0, float(self.imu_lin_acc_cov_diag[1]), 0.0,
            0.0, 0.0, float(self.imu_lin_acc_cov_diag[2])
        ]

        self.imu_pub.publish(imu_msg)

    # =====================================================
    # DEBUG PANEL
    # =====================================================
    def _print_debug_panel(self):
        v, w = self._dbg_last_cmd
        odom_v, odom_w = self._dbg_last_odom

        panel = f"""
╔══════════════════════════════════════════════════════╗
║        HAMALS SERIAL BRIDGE — DEBUG PANEL            ║
╠══════════════════════════════════════════════════════╣
║ TX packets        : {self._dbg_tx:<30}║
║ TX dedup skipped  : {self._dbg_tx_skipped_dedup:<30}║
║ TX ratelimit skip : {self._dbg_tx_skipped_ratelimit:<30}║
║ ODOM published    : {self._dbg_odom:<30}║
║ RX bytes          : {self._dbg_rx_bytes:<30}║
║ RX valid frames   : {self._dbg_rx_frames:<30}║
║ RX invalid frames : {self._dbg_rx_invalid:<30}║
║ Dead-man active   : {str(self._deadman_active):<30}║
╠══════════════════════════════════════════════════════╣
║ Last cmd_vel      : v={v:.2f}  w={w:.2f}              ║
║ Last odom_raw twist : v={odom_v:.2f} w={odom_w:.2f}    ║
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