#!/usr/bin/env python

from __future__ import print_function

import math
import threading
import time

import rospy
import serial

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Header
from std_msgs.msg import Float64
import tf


class TeensyBridgeNode(object):
    """
    Bridge between ROS and the Teensy 4.0 firmware.

    Protocol (matches teensy4.0.ino):
      - Commands: ASCII lines "V:vx,vy,vz\n"
          vx, vy in m/s, vz in rad/s, in robot base_link frame.
      - Telemetry lines (at ~20 Hz):
          "E:rb,lb,ru,lu;I:yaw,pitch,roll\n"
        where encoder counts are integers and IMU angles are in degrees.
    """

    def __init__(self):
        # Parameters
        port = rospy.get_param("~port", "/dev/ttyACM0")
        baud = rospy.get_param("~baud", 115200)
        self.odom_frame = rospy.get_param("~odom_frame", "odom")
        self.base_frame = rospy.get_param("~base_frame", "base_link")
        self.odom_topic = rospy.get_param("~odom_topic", "odom")
        self.publish_tf = bool(rospy.get_param("~publish_tf", True))

        # Mecanum + encoder parameters (should mirror URDF + firmware)
        self.wheel_radius = rospy.get_param("~wheel_radius", 0.05)     # m
        self.half_length = rospy.get_param("~half_length", 0.15)       # m
        self.half_width = rospy.get_param("~half_width", 0.1975)       # m
        self.encoder_ticks_per_rev = rospy.get_param("~encoder_ticks_per_rev", 6400.0)
        # Optional odom sign calibration (keep default +1 unless a specific axis is reversed).
        self.odom_vx_sign = float(rospy.get_param("~odom_vx_sign", 1.0))
        self.odom_vy_sign = float(rospy.get_param("~odom_vy_sign", 1.0))
        self.odom_wz_sign = float(rospy.get_param("~odom_wz_sign", 1.0))
        # Optional odom gain calibration for scale tuning per axis.
        self.odom_vx_gain = float(rospy.get_param("~odom_vx_gain", 1.0))
        self.odom_vy_gain = float(rospy.get_param("~odom_vy_gain", 1.0))
        self.odom_wz_gain = float(rospy.get_param("~odom_wz_gain", 1.0))
        # Optional XY frame rotation calibration (degrees) for odometry translation.
        # Positive rotates (vx, vy) counterclockwise in the base frame.
        self.odom_xy_rotation_deg = float(rospy.get_param("~odom_xy_rotation_deg", 0.0))

        # Pose state
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        # Last encoder counts
        self.rb_counts_prev = None
        self.lb_counts_prev = None
        self.ru_counts_prev = None
        self.lu_counts_prev = None

        # Wall time of last *successfully parsed* telemetry line (for inter-message dt)
        self._prev_good_telemetry_time = None

        # Encoder spike guard: see _encoder_spike_limit()
        self._max_encoder_delta_fixed = int(rospy.get_param("~max_encoder_delta_per_cycle", 0))
        self._max_encoder_delta_auto = rospy.get_param("~max_encoder_delta_auto", True)
        self._max_reasonable_wheel_lin_mps = rospy.get_param("~max_reasonable_wheel_lin_mps", 0.15)
        self._max_encoder_delta_margin = rospy.get_param("~max_encoder_delta_margin", 1.5)

        # Jetson-side /cmd_vel watchdog (defense in depth; Teensy CMD_TIMEOUT_MS is still primary on MCU)
        self._cmd_vel_watchdog_ms = float(rospy.get_param("~cmd_vel_watchdog_ms", 900.0))
        self._last_cmd_vel_wall_time = rospy.Time.now()
        self._cmd_vel_watchdog_zero_sent = False

        # Serial open with retries (USB can miss on first boot)
        open_retries = int(rospy.get_param("~serial_open_retries", 5))
        open_retry_delay = float(rospy.get_param("~serial_open_retry_delay_s", 1.0))
        self.ser = None
        last_err = None
        for attempt in range(max(1, open_retries)):
            try:
                rospy.loginfo("Opening serial port %s @ %d (attempt %d/%d)", port, baud, attempt + 1, open_retries)
                self.ser = serial.Serial(port, baudrate=baud, timeout=0.05)
                time.sleep(2.0)  # allow Teensy to reset
                break
            except (serial.SerialException, OSError) as e:
                last_err = e
                rospy.logwarn("Serial open failed: %s", e)
                if attempt + 1 < open_retries:
                    time.sleep(open_retry_delay)
        if self.ser is None:
            rospy.logfatal("Could not open serial after %d attempts: %s", open_retries, last_err)
            raise last_err

        # ROS interfaces
        self.cmd_vel_sub = rospy.Subscriber("cmd_vel", Twist, self.cmd_vel_cb, queue_size=10)
        self.odom_pub = rospy.Publisher(self.odom_topic, Odometry, queue_size=10)
        self.imu_pub = rospy.Publisher("imu/data", Imu, queue_size=10)
        self.telemetry_age_pub = rospy.Publisher("~telemetry_age_sec", Float64, queue_size=1)
        self.tf_broadcaster = tf.TransformBroadcaster()

        self._write_lock = threading.Lock()
        self._stop_event = threading.Event()

        # Serial reader thread
        self.reader_thread = threading.Thread(target=self.serial_reader_loop)
        self.reader_thread.daemon = True
        self.reader_thread.start()

        rospy.Timer(rospy.Duration(0.05), self._cmd_vel_watchdog_tick, oneshot=False)
        rospy.Timer(rospy.Duration(0.2), self._telemetry_age_tick, oneshot=False)

        rospy.loginfo(
            "Encoder spike guard: fixed_limit=%s auto_dt=%s (auto uses max_reasonable_wheel_lin_mps=%.3f)",
            self._max_encoder_delta_fixed,
            self._max_encoder_delta_auto,
            self._max_reasonable_wheel_lin_mps,
        )
        rospy.loginfo(
            "Jetson cmd_vel watchdog: %s ms (0 = disabled). MCU still uses Teensy CMD_TIMEOUT_MS.",
            self._cmd_vel_watchdog_ms,
        )

    def _encoder_spike_limit(self, dt):
        """
        Max |delta counts| per wheel this cycle.
        - If max_encoder_delta_per_cycle > 0: use that fixed ceiling.
        - Else if max_encoder_delta_auto: scale with dt and max reasonable wheel speed
          (avoids rejecting legitimate motion after long telemetry gaps).
        - Else: 0 = spike guard disabled.
        """
        if self._max_encoder_delta_fixed > 0:
            return self._max_encoder_delta_fixed
        if not self._max_encoder_delta_auto:
            return 0
        r = max(float(self.wheel_radius), 1e-6)
        omega_max = float(self._max_reasonable_wheel_lin_mps) / r
        ticks_per_sec = omega_max * float(self.encoder_ticks_per_rev) / (2.0 * math.pi)
        lim = int(ticks_per_sec * float(dt) * float(self._max_encoder_delta_margin)) + 1
        return max(lim, 1)

    def _write_vel_line(self, vx, vy, vz):
        line = "V:{:.3f},{:.3f},{:.3f}\n".format(vx, vy, vz)
        encoded = line.encode("ascii")
        with self._write_lock:
            try:
                self.ser.write(encoded)
            except serial.SerialException as e:
                rospy.logerr("Serial write error: %s", e)

    def _cmd_vel_watchdog_tick(self, _event):
        """If no /cmd_vel for cmd_vel_watchdog_ms, send V:0,0,0 once (MCU watchdog still applies)."""
        if self._cmd_vel_watchdog_ms <= 0.0:
            return
        dt_ms = (rospy.Time.now() - self._last_cmd_vel_wall_time).to_sec() * 1000.0
        if dt_ms > self._cmd_vel_watchdog_ms:
            if not self._cmd_vel_watchdog_zero_sent:
                self._write_vel_line(0.0, 0.0, 0.0)
                self._cmd_vel_watchdog_zero_sent = True
                rospy.logwarn_throttle(
                    5.0,
                    "Jetson /cmd_vel watchdog: sent V:0,0,0 (no cmd_vel for > %.0f ms)",
                    self._cmd_vel_watchdog_ms,
                )
        else:
            self._cmd_vel_watchdog_zero_sent = False

    def _telemetry_age_tick(self, _event):
        """Seconds since last good E:...;I:... line (-1 if none yet). Downstream can treat large age as stale."""
        msg = Float64()
        if self._prev_good_telemetry_time is None:
            msg.data = -1.0
        else:
            msg.data = (rospy.Time.now() - self._prev_good_telemetry_time).to_sec()
        self.telemetry_age_pub.publish(msg)

    def cmd_vel_cb(self, msg):
        """
        Send V:vx,vy,vz command to Teensy.
        """
        self._last_cmd_vel_wall_time = rospy.Time.now()
        self._cmd_vel_watchdog_zero_sent = False
        vx = msg.linear.x
        vy = msg.linear.y
        vz = msg.angular.z
        self._write_vel_line(vx, vy, vz)

    def serial_reader_loop(self):
        """
        Continuously read lines from serial and parse telemetry.
        """
        buf = ""
        while not rospy.is_shutdown() and not self._stop_event.is_set():
            try:
                chunk = self.ser.read(128)
            except serial.SerialException as e:
                rospy.logerr("Serial read error: %s", e)
                time.sleep(0.5)
                continue

            if not chunk:
                continue

            try:
                buf += chunk.decode("ascii", errors="ignore")
            except Exception:
                continue

            while "\n" in buf:
                line, buf = buf.split("\n", 1)
                line = line.strip()
                if not line:
                    continue
                self.handle_telemetry_line(line)

    def handle_telemetry_line(self, line):
        """
        Example line: "E:100,200,150,180;I:10.0,1.0,0.5"
        """
        try:
            e_part, i_part = line.split(";")
        except ValueError:
            rospy.logwarn("Unexpected telemetry format (no ';'): %s", line)
            return

        if not e_part.startswith("E:") or not i_part.startswith("I:"):
            rospy.logwarn("Unexpected telemetry prefix: %s", line)
            return

        # Parse encoders
        try:
            enc_str = e_part[2:]
            rb_str, lb_str, ru_str, lu_str = enc_str.split(",")
            rb = int(rb_str)
            lb = int(lb_str)
            ru = int(ru_str)
            lu = int(lu_str)
        except Exception as e:
            rospy.logwarn("Failed to parse encoder part '%s': %s", e_part, e)
            return

        # Parse IMU (yaw, pitch, roll in deg as sent by firmware)
        try:
            imu_str = i_part[2:]
            yaw_deg_str, pitch_deg_str, roll_deg_str = imu_str.split(",")
            yaw_deg = float(yaw_deg_str)
            pitch_deg = float(pitch_deg_str)
            roll_deg = float(roll_deg_str)
        except Exception as e:
            rospy.logwarn("Failed to parse IMU part '%s': %s", i_part, e)
            return

        # dt = wall time between consecutive *successful* telemetry parses (not intra-callback noise)
        now = rospy.Time.now()
        if self._prev_good_telemetry_time is None:
            dt = 1.0 / 20.0
        else:
            dt = (now - self._prev_good_telemetry_time).to_sec()
        # Sane bounds: ~5 Hz .. ~60 Hz typical; avoid divide-by-near-zero or huge gaps
        dt_min = rospy.get_param("~telemetry_dt_min", 0.012)
        dt_max = rospy.get_param("~telemetry_dt_max", 0.5)
        if dt < dt_min:
            dt = dt_min
        elif dt > dt_max:
            dt = dt_max

        self._prev_good_telemetry_time = now

        self.update_odometry(rb, lb, ru, lu, dt)
        self.publish_imu(yaw_deg, pitch_deg, roll_deg, now)

    def update_odometry(self, rb, lb, ru, lu, dt):
        """
        Integrate wheel encoder counts to odom. We approximate:
          - Convert delta counts → wheel angular velocities
          - Apply mecanum forward kinematics → vx, vy, wz
          - Integrate pose in the odom frame.
        """
        if self.rb_counts_prev is None:
            self.rb_counts_prev = rb
            self.lb_counts_prev = lb
            self.ru_counts_prev = ru
            self.lu_counts_prev = lu
            return

        d_rb = rb - self.rb_counts_prev
        d_lb = lb - self.lb_counts_prev
        d_ru = ru - self.ru_counts_prev
        d_lu = lu - self.lu_counts_prev

        md = self._encoder_spike_limit(dt)
        if md > 0:

            def _spike_guard(name, d):
                if abs(d) > md:
                    rospy.logwarn_throttle(
                        2.0,
                        "Encoder delta spike on %s: %d (>|%d|); treating as 0 this cycle",
                        name,
                        d,
                        md,
                    )
                    return 0
                return d

            d_rb = _spike_guard("rb", d_rb)
            d_lb = _spike_guard("lb", d_lb)
            d_ru = _spike_guard("ru", d_ru)
            d_lu = _spike_guard("lu", d_lu)

        self.rb_counts_prev = rb
        self.lb_counts_prev = lb
        self.ru_counts_prev = ru
        self.lu_counts_prev = lu

        ticks_to_rad = 2.0 * math.pi / float(self.encoder_ticks_per_rev)
        rb_omega = d_rb * ticks_to_rad / dt
        lb_omega = d_lb * ticks_to_rad / dt
        ru_omega = d_ru * ticks_to_rad / dt
        lu_omega = d_lu * ticks_to_rad / dt

        # Map wheels:
        #  LU = front-left, RU = front-right, LB = rear-left, RB = rear-right
        w_fl = lu_omega
        w_fr = ru_omega
        w_rl = lb_omega
        w_rr = rb_omega

        L = self.half_length
        W = self.half_width
        R = self.wheel_radius

        # Standard mecanum forward kinematics
        vx = (w_fl + w_fr + w_rl + w_rr) * (R / 4.0)
        vy = (-w_fl + w_fr + w_rl - w_rr) * (R / 4.0)
        wz = (-w_fl + w_fr - w_rl + w_rr) * (R / (4.0 * (L + W)))

        # Apply optional per-axis sign calibration for odometry frame alignment.
        vx *= self.odom_vx_sign
        vy *= self.odom_vy_sign
        wz *= self.odom_wz_sign

        # Apply optional per-axis gain calibration for odometry scale alignment.
        vx *= self.odom_vx_gain
        vy *= self.odom_vy_gain
        wz *= self.odom_wz_gain

        # Optional XY frame rotation calibration.
        if abs(self.odom_xy_rotation_deg) > 1e-6:
            rot = math.radians(self.odom_xy_rotation_deg)
            c = math.cos(rot)
            s = math.sin(rot)
            vx_rot = c * vx - s * vy
            vy_rot = s * vx + c * vy
            vx = vx_rot
            vy = vy_rot

        # Integrate in odom frame using current yaw
        dx = (vx * math.cos(self.yaw) - vy * math.sin(self.yaw)) * dt
        dy = (vx * math.sin(self.yaw) + vy * math.cos(self.yaw)) * dt
        dyaw = wz * dt

        self.x += dx
        self.y += dy
        self.yaw += dyaw

        quat = tf.transformations.quaternion_from_euler(0.0, 0.0, self.yaw)
        now = rospy.Time.now()

        # Publish odom message
        odom = Odometry()
        odom.header = Header(stamp=now, frame_id=self.odom_frame)
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = quat[0]
        odom.pose.pose.orientation.y = quat[1]
        odom.pose.pose.orientation.z = quat[2]
        odom.pose.pose.orientation.w = quat[3]

        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = wz

        # Placeholder covariances on x, y, yaw and vx, vy, wz (tune after field tests)
        odom.pose.covariance[0] = 0.05
        odom.pose.covariance[7] = 0.05
        odom.pose.covariance[35] = 0.05
        odom.twist.covariance[0] = 0.02
        odom.twist.covariance[7] = 0.02
        odom.twist.covariance[35] = 0.02

        self.odom_pub.publish(odom)

        # Publish TF (optional when another node, e.g. EKF, owns odom->base TF)
        if self.publish_tf:
            self.tf_broadcaster.sendTransform(
                (self.x, self.y, 0.0),
                quat,
                now,
                self.base_frame,
                self.odom_frame,
            )

    def publish_imu(self, yaw_deg, pitch_deg, roll_deg, stamp):
        """
        Publish IMU orientation as sensor_msgs/Imu.
        We use the angles coming from Teensy as already fused.
        """
        for label, v in (
            ("yaw", yaw_deg),
            ("pitch", pitch_deg),
            ("roll", roll_deg),
        ):
            try:
                if not math.isfinite(float(v)):
                    rospy.logwarn_throttle(
                        2.0,
                        "Non-finite IMU angle (%s=%s); skipping Imu publish",
                        label,
                        v,
                    )
                    return
            except (TypeError, ValueError):
                rospy.logwarn_throttle(
                    2.0,
                    "Invalid IMU angle (%s=%s); skipping Imu publish",
                    label,
                    v,
                )
                return

        yaw = math.radians(float(yaw_deg))
        pitch = math.radians(float(pitch_deg))
        roll = math.radians(float(roll_deg))

        quat = tf.transformations.quaternion_from_euler(roll, pitch, yaw)

        msg = Imu()
        msg.header.stamp = stamp
        msg.header.frame_id = "imu_link"
        msg.orientation.x = quat[0]
        msg.orientation.y = quat[1]
        msg.orientation.z = quat[2]
        msg.orientation.w = quat[3]

        # Leave angular_velocity and linear_acceleration unspecified for now
        self.imu_pub.publish(msg)

    def shutdown(self):
        self._stop_event.set()
        rospy.loginfo("Shutting down Teensy bridge...")
        try:
            self._write_vel_line(0.0, 0.0, 0.0)
        except Exception:
            pass
        try:
            self.ser.close()
        except Exception:
            pass


def main():
    rospy.init_node("teensy_bridge")
    node = TeensyBridgeNode()
    rospy.on_shutdown(node.shutdown)
    rospy.spin()


if __name__ == "__main__":
    main()

