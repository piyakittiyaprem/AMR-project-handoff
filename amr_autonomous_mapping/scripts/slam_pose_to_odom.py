#!/usr/bin/env python
from __future__ import print_function

import math

import rospy
import tf2_ros
from geometry_msgs.msg import Pose, PoseStamped, Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion


def yaw_from_quat(q):
    return euler_from_quaternion([q.x, q.y, q.z, q.w])[2]


def norm_angle(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


class SlamPoseToOdom(object):
    """
    Convert LiDAR SLAM pose (/slam_out_pose) into /odom and inject MPU6050 yaw.
    Position comes from LiDAR SLAM; heading prefers IMU when available.
    """

    def __init__(self):
        self.slam_pose_topic = rospy.get_param("~slam_pose_topic", "/slam_out_pose")
        self.imu_topic = rospy.get_param("~imu_topic", "/imu/data")
        self.odom_topic = rospy.get_param("~odom_topic", "/odom")
        self.odom_frame = rospy.get_param("~odom_frame", "odom")
        self.base_frame = rospy.get_param("~base_frame", "base_footprint")
        self.use_imu_orientation = rospy.get_param("~use_imu_orientation", True)
        self.imu_timeout_s = rospy.get_param("~imu_timeout_s", 0.25)
        self.publish_tf = rospy.get_param("~publish_tf", True)
        # Use "now" timestamps for odom/TF to avoid future extrapolation
        # when slam_out_pose arrives with delayed header stamps.
        self.stamp_now = rospy.get_param("~stamp_now", True)
        self.publish_rate_hz = rospy.get_param("~publish_rate_hz", 30.0)

        self._last_imu = None
        self._latest_pose = None
        self._latest_pose_stamp = None
        self._last_pose_for_twist = None
        self._last_pose_for_twist_stamp = None
        self._vx = 0.0
        self._vy = 0.0
        self._last_pub_t = None
        self._last_pub_yaw = None

        self.pub = rospy.Publisher(self.odom_topic, Odometry, queue_size=20)
        self.tf_pub = tf2_ros.TransformBroadcaster() if self.publish_tf else None

        rospy.Subscriber(self.imu_topic, Imu, self._imu_cb, queue_size=20)
        rospy.Subscriber(self.slam_pose_topic, PoseStamped, self._pose_cb, queue_size=20)
        self._timer = rospy.Timer(rospy.Duration(1.0 / max(1.0, self.publish_rate_hz)), self._publish_cb)

    def _imu_cb(self, msg):
        self._last_imu = msg

    def _imu_fresh(self, stamp):
        if self._last_imu is None:
            return False
        dt = (stamp - self._last_imu.header.stamp).to_sec()
        return abs(dt) <= self.imu_timeout_s

    def _pose_cb(self, msg):
        pose_t = msg.header.stamp if msg.header.stamp != rospy.Time() else rospy.Time.now()
        self._latest_pose = msg.pose
        self._latest_pose_stamp = pose_t

        if self._last_pose_for_twist is not None and self._last_pose_for_twist_stamp is not None:
            dt = (pose_t - self._last_pose_for_twist_stamp).to_sec()
            if dt > 1e-4:
                self._vx = (msg.pose.position.x - self._last_pose_for_twist.position.x) / dt
                self._vy = (msg.pose.position.y - self._last_pose_for_twist.position.y) / dt

        self._last_pose_for_twist = msg.pose
        self._last_pose_for_twist_stamp = pose_t

    def _publish_cb(self, _event):
        pose_available = self._latest_pose is not None
        t = rospy.Time.now() if (self.stamp_now or self._latest_pose_stamp is None) else self._latest_pose_stamp
        pose = self._latest_pose if pose_available else Pose()

        use_imu = self.use_imu_orientation and self._imu_fresh(t)
        if use_imu:
            chosen_q = self._last_imu.orientation
        elif pose_available:
            chosen_q = pose.orientation
        else:
            chosen_q = Quaternion()
            chosen_q.w = 1.0
        chosen_yaw = yaw_from_quat(chosen_q)

        odom = Odometry()
        odom.header.stamp = t
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position = pose.position
        odom.pose.pose.orientation = chosen_q
        odom.twist.twist.linear.x = self._vx if pose_available else 0.0
        odom.twist.twist.linear.y = self._vy if pose_available else 0.0

        if use_imu:
            odom.twist.twist.angular.z = self._last_imu.angular_velocity.z
        elif self._last_pub_t is not None and self._last_pub_yaw is not None:
            dt = (t - self._last_pub_t).to_sec()
            if dt > 1e-4:
                odom.twist.twist.angular.z = norm_angle(chosen_yaw - self._last_pub_yaw) / dt

        self.pub.publish(odom)

        if self.tf_pub is not None:
            tf_msg = TransformStamped()
            tf_msg.header.stamp = odom.header.stamp
            tf_msg.header.frame_id = self.odom_frame
            tf_msg.child_frame_id = self.base_frame
            tf_msg.transform.translation.x = pose.position.x
            tf_msg.transform.translation.y = pose.position.y
            tf_msg.transform.translation.z = pose.position.z
            tf_msg.transform.rotation = chosen_q
            self.tf_pub.sendTransform(tf_msg)

        self._last_pub_t = t
        self._last_pub_yaw = chosen_yaw


def main():
    rospy.init_node("slam_pose_to_odom")
    node = SlamPoseToOdom()
    rospy.loginfo(
        "slam_pose_to_odom: pose=%s imu=%s -> %s (tf=%s)",
        node.slam_pose_topic,
        node.imu_topic,
        node.odom_topic,
        node.publish_tf,
    )
    rospy.spin()


if __name__ == "__main__":
    main()
