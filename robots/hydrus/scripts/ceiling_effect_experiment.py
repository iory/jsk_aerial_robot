#!/usr/bin/env python

from __future__ import print_function

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from aerial_robot_msgs.msg import FlightNav


class CeilingEffectExperimentNode(object):
    def __init__(self):
        rospy.init_node("ceiling_effect_experiment_node")

        self.robot_ns = rospy.get_param("~robot_ns", "/hydrus")

        # ===== 高度制御パラメータ =====
        self.target_z = rospy.get_param("~target_z", 1.0)
        self.z_threshold = rospy.get_param("~z_threshold", 0.03)
        self.vz_threshold = rospy.get_param("~vz_threshold", 0.03)
        self.stable_time = rospy.get_param("~stable_time", 3.0)

        self.kp_z = rospy.get_param("~kp_z", 0.10)
        self.max_vz = rospy.get_param("~max_vz", 0.05)

        # ===== 関節角度パラメータ =====
        # joint2 は固定
        self.q2_const = rospy.get_param("~q2_const", 0.6) # l>1.0

        # joint1, joint3 の開始・終了角度
        self.q1_start = rospy.get_param("~q1_start", 1.57)
        self.q1_goal  = rospy.get_param("~q1_goal", 1.20)

        self.q3_start = rospy.get_param("~q3_start", 1.57)
        self.q3_goal  = rospy.get_param("~q3_goal", 0.50)

        # かなり遅く動かす
        self.motion_duration = rospy.get_param("~motion_duration", 60.0)

        # joint2 を先に固定する時間
        self.joint2_set_time = rospy.get_param("~joint2_set_time", 3.0)

        self.current_z = None
        self.current_vz = None

        self.state = "WAIT_ODOM"
        self.state_start_time = rospy.Time.now()
        self.stable_start_time = None
        self.motion_start_time = None

        self.odom_sub = rospy.Subscriber(
            self.robot_ns + "/uav/baselink/odom",
            Odometry,
            self.odom_callback
        )

        self.nav_pub = rospy.Publisher(
            self.robot_ns + "/uav/nav",
            FlightNav,
            queue_size=1
        )

        self.joint_pub = rospy.Publisher(
            self.robot_ns + "/joints_ctrl",
            JointState,
            queue_size=1
        )

        # 50 Hz
        self.timer = rospy.Timer(rospy.Duration(0.02), self.update)

        rospy.loginfo("ceiling_effect_experiment_node started")
        rospy.loginfo("target_z = %.3f", self.target_z)

    def odom_callback(self, msg):
        self.current_z = msg.pose.pose.position.z
        self.current_vz = msg.twist.twist.linear.z

    def change_state(self, next_state):
        rospy.loginfo("state: %s -> %s", self.state, next_state)
        self.state = next_state
        self.state_start_time = rospy.Time.now()

    def clamp(self, x, min_value, max_value):
        return max(min(x, max_value), min_value)

    def publish_z_velocity_command(self):
        if self.current_z is None:
            return

        error_z = self.target_z - self.current_z
        vz_cmd = self.kp_z * error_z
        vz_cmd = self.clamp(vz_cmd, -self.max_vz, self.max_vz)

        nav_msg = FlightNav()
        nav_msg.control_frame = FlightNav.WORLD_FRAME
        nav_msg.target = FlightNav.COG

        nav_msg.pos_z_nav_mode = FlightNav.VEL_MODE
        nav_msg.target_vel_z = vz_cmd

        self.nav_pub.publish(nav_msg)

    def publish_zero_z_velocity_command(self):
        nav_msg = FlightNav()
        nav_msg.control_frame = FlightNav.WORLD_FRAME
        nav_msg.target = FlightNav.COG

        nav_msg.pos_z_nav_mode = FlightNav.VEL_MODE
        nav_msg.target_vel_z = 0.0

        self.nav_pub.publish(nav_msg)

    def publish_joint_command(self, q1, q2, q3):
        msg = JointState()
        msg.header.stamp = rospy.Time.now()

        msg.name = ["joint1", "joint2", "joint3"]
        msg.position = [q1, q2, q3]
        msg.velocity = [0.0, 0.0, 0.0]
        msg.effort = [0.0, 0.0, 0.0]

        self.joint_pub.publish(msg)

    def is_altitude_stable(self):
        if self.current_z is None or self.current_vz is None:
            return False

        z_error = abs(self.target_z - self.current_z)
        vz_abs = abs(self.current_vz)

        return z_error < self.z_threshold and vz_abs < self.vz_threshold

    def slow_joint_trajectory(self, t):
        T = self.motion_duration

        s = t / T
        s = self.clamp(s, 0.0, 1.0)

        # smooth step
        # 最初と最後の速度を0に近づける
        s = 3.0 * s * s - 2.0 * s * s * s

        q1 = self.q1_start + s * (self.q1_goal - self.q1_start)
        q2 = self.q2_const
        q3 = self.q3_start + s * (self.q3_goal - self.q3_start)

        return q1, q2, q3

    def update(self, event):
        now = rospy.Time.now()
        elapsed = (now - self.state_start_time).to_sec()

        if self.state == "WAIT_ODOM":
            if self.current_z is not None:
                rospy.loginfo("odom received: z = %.3f", self.current_z)
                self.change_state("SET_JOINT2")

        elif self.state == "SET_JOINT2":
            # joint1, joint3 は開始角度，joint2 は固定角度にする
            self.publish_joint_command(
                self.q1_start,
                self.q2_const,
                self.q3_start
            )

            # 高度も同時にゆっくり目標へ近づける
            self.publish_z_velocity_command()

            if elapsed > self.joint2_set_time:
                self.change_state("GO_TARGET_ALTITUDE")

        elif self.state == "GO_TARGET_ALTITUDE":
            # 目標高度へ移動
            self.publish_joint_command(
                self.q1_start,
                self.q2_const,
                self.q3_start
            )

            self.publish_z_velocity_command()

            if self.is_altitude_stable():
                if self.stable_start_time is None:
                    self.stable_start_time = now

                stable_elapsed = (now - self.stable_start_time).to_sec()

                if stable_elapsed > self.stable_time:
                    self.motion_start_time = now
                    self.change_state("SLOW_JOINT_MOTION")
            else:
                self.stable_start_time = None

        elif self.state == "SLOW_JOINT_MOTION":
            # 高度維持しながら joint1, joint3 を非常にゆっくり動かす
            self.publish_z_velocity_command()

            t = (now - self.motion_start_time).to_sec()
            q1, q2, q3 = self.slow_joint_trajectory(t)
            self.publish_joint_command(q1, q2, q3)

            if t > self.motion_duration:
                self.change_state("HOLD")

        elif self.state == "HOLD":
            # 最終姿勢を維持しつつ，高度維持
            self.publish_z_velocity_command()

            self.publish_joint_command(
                self.q1_goal,
                self.q2_const,
                self.q3_goal
            )


if __name__ == "__main__":
    node = CeilingEffectExperimentNode()
    rospy.spin()