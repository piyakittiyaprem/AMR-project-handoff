#!/usr/bin/env python
from __future__ import print_function

import curses
import threading
import time

import rospy
from geometry_msgs.msg import Twist


def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x


class WASDManualTeleop(object):
    def __init__(self):
        # Parameters (damage-safe defaults; tune if needed)
        self.teleop_rate_hz = rospy.get_param("~teleop_rate_hz", 20.0)
        self.teleop_timeout_ms = rospy.get_param("~teleop_timeout_ms", 300)  # deadman per-axis

        self.v_lin_fixed = rospy.get_param("~v_lin_fixed", 0.05)  # m/s
        self.v_yaw_fixed = rospy.get_param("~v_yaw_fixed", 0.15)  # rad/s

        # Safety clamping bounds (logs show the enforced command values)
        self.max_linear_v = rospy.get_param("~max_linear_v", 0.06)  # m/s
        self.max_yaw_rate = rospy.get_param("~max_yaw_rate", 0.20)  # rad/s

        self.log_every_s = rospy.get_param("~log_every_s", 0.5)

        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        # Command state
        self._lock = threading.Lock()
        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0

        now = rospy.Time.now()
        self.last_x_time = now
        self.last_y_time = now
        self.last_w_time = now

        # Curses init (typically main thread; executed in this node thread)
        self._stdscr = None
        self._last_log = rospy.Time.now()

    def set_key_velocity(self, key, now):
        # Fixed-speed mapping in base_link frame
        # W/S -> linear.x (forward/back)
        # A/D -> linear.y (left/right strafe)
        # Q/E -> angular.z (CCW/CW)
        with self._lock:
            if key == ord("w") or key == ord("W"):
                self.vx = +self.v_lin_fixed
                self.last_x_time = now
            elif key == ord("s") or key == ord("S"):
                self.vx = -self.v_lin_fixed
                self.last_x_time = now
            elif key == ord("a") or key == ord("A"):
                self.vy = +self.v_lin_fixed
                self.last_y_time = now
            elif key == ord("d") or key == ord("D"):
                self.vy = -self.v_lin_fixed
                self.last_y_time = now
            elif key == ord("q") or key == ord("Q"):
                self.wz = +self.v_yaw_fixed
                self.last_w_time = now
            elif key == ord("e") or key == ord("E"):
                self.wz = -self.v_yaw_fixed
                self.last_w_time = now
            elif key == ord(" "):
                # Space = immediate full stop
                self.vx = 0.0
                self.vy = 0.0
                self.wz = 0.0
                self.last_x_time = now
                self.last_y_time = now
                self.last_w_time = now

    def apply_deadman(self, now):
        # Deadman per-axis: if that axis hasn't been commanded recently, zero it.
        timeout_s = float(self.teleop_timeout_ms) / 1000.0
        with self._lock:
            if (now - self.last_x_time).to_sec() > timeout_s:
                self.vx = 0.0
            if (now - self.last_y_time).to_sec() > timeout_s:
                self.vy = 0.0
            if (now - self.last_w_time).to_sec() > timeout_s:
                self.wz = 0.0

    def publish_twist(self, now):
        # Clamp to enforced safety bounds before publishing.
        with self._lock:
            vx_c = clamp(self.vx, -self.max_linear_v, self.max_linear_v)
            vy_c = clamp(self.vy, -self.max_linear_v, self.max_linear_v)
            wz_c = clamp(self.wz, -self.max_yaw_rate, self.max_yaw_rate)

            twist = Twist()
            twist.linear.x = vx_c
            twist.linear.y = vy_c
            twist.angular.z = wz_c
            self.pub.publish(twist)

            # Rate-limited debug log to show the commanded values.
            if (now - self._last_log).to_sec() >= self.log_every_s:
                rospy.loginfo(
                    "teleop cmd_vel vx=%.3f vy=%.3f wz=%.3f (clamped to max_linear=%.3f, max_yaw=%.3f)",
                    vx_c, vy_c, wz_c, self.max_linear_v, self.max_yaw_rate
                )
                self._last_log = now

    def run_curses(self, stdscr):
        self._stdscr = stdscr
        curses.noecho()
        curses.cbreak()
        stdscr.nodelay(True)
        stdscr.keypad(True)

        rospy.loginfo("Manual teleop running. WASD/QE for motion, SPACE for stop. Press Ctrl+C to exit.")
        rospy.loginfo("Fixed speeds: v_lin_fixed=%.3f m/s, v_yaw_fixed=%.3f rad/s. Timeout=%d ms.",
                      self.v_lin_fixed, self.v_yaw_fixed, self.teleop_timeout_ms)

        rate = rospy.Rate(self.teleop_rate_hz)
        while not rospy.is_shutdown():
            now = rospy.Time.now()

            # Non-blocking key read
            key = stdscr.getch()
            if key != -1:
                # Debug on keypress (real-time troubleshooting)
                rospy.loginfo("Key pressed: %s", chr(key) if 32 <= key < 127 else str(key))
                self.set_key_velocity(key, now)

            self.apply_deadman(now)
            self.publish_twist(now)
            rate.sleep()

    def run(self):
        # curses wrapper takes care of terminal state restoration
        curses.wrapper(self.run_curses)

    def publish_stop(self):
        """Best-effort zero /cmd_vel on exit (MCU watchdog still applies if this fails)."""
        z = Twist()
        for _ in range(3):
            self.pub.publish(z)
            rospy.sleep(0.05)


def main():
    rospy.init_node("amr_manual_teleop_wasd")
    node = WASDManualTeleop()
    rospy.on_shutdown(node.publish_stop)
    try:
        node.run()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()

