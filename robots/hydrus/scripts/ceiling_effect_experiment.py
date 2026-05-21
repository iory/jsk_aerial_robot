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
        self.target_d_R = rospy.get_param(self.robot_ns + "/target_d_R")
        self.ceiling_height = rospy.get_param(self.robot_ns + "/ceiling_height")
        self.ceiling_distance_offset = rospy.get_param(self.robot_ns + "/ceiling_distance_offset")
        self.rotor_radius = rospy.get_param(self.robot_ns + "/rotor_radius")

        self.target_d = self.target_d_R * self.rotor_radius
        self.target_z = (
            self.ceiling_height
            - self.ceiling_distance_offset
            - self.target_d
        )

        self.z_threshold = rospy.get_param("~z_threshold", 0.03)
        self.vz_threshold = rospy.get_param("~vz_threshold", 0.03)
        self.stable_time = rospy.get_param("~stable_time", 3.0)

        # 目標高度へ移動するときに使う速度制御
        self.kp_z = rospy.get_param("~kp_z", 0.10)
        self.max_vz = rospy.get_param("~max_vz", 0.05)

        # ===== 関節角度パラメータ =====
        # q2 は最初に q2_start から q2_const へゆっくり移動
        self.q2_start = rospy.get_param("~q2_start", 1.57)
        self.q2_const = rospy.get_param("~q2_const", 0.6)

        # q1, q3 は q2 固定後，高度到達後に変化
        self.q1_start = rospy.get_param("~q1_start", 1.57)
        self.q1_goal = rospy.get_param("~q1_goal", 1.20)

        self.q3_start = rospy.get_param("~q3_start", 1.57)
        self.q3_goal = rospy.get_param("~q3_goal", 0.50)

        # q2 を目標値まで動かす時間
        self.joint2_set_time = rospy.get_param("~joint2_set_time", 5.0)

        # q1, q3 を動かす時間
        self.motion_duration = rospy.get_param("~motion_duration", 60.0)

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
        rospy.loginfo("target_d_R = %.3f", self.target_d_R)
        rospy.loginfo("rotor_radius = %.4f [m]", self.rotor_radius)
        rospy.loginfo("target_d = %.4f [m]", self.target_d)
        rospy.loginfo("ceiling_height = %.3f [m]", self.ceiling_height)
        rospy.loginfo("ceiling_distance_offset = %.3f [m]", self.ceiling_distance_offset)
        rospy.loginfo("calculated target_z = %.4f [m]", self.target_z)
        rospy.loginfo("q2_start = %.3f", self.q2_start)
        rospy.loginfo("q2_const = %.3f", self.q2_const)
        rospy.loginfo("joint2_set_time = %.3f [s]", self.joint2_set_time)
        rospy.loginfo("motion_duration = %.3f [s]", self.motion_duration)

    def odom_callback(self, msg):
        self.current_z = msg.pose.pose.position.z
        self.current_vz = msg.twist.twist.linear.z

    def change_state(self, next_state):
        rospy.loginfo("state: %s -> %s", self.state, next_state)
        self.state = next_state
        self.state_start_time = rospy.Time.now()

    def clamp(self, x, min_value, max_value):
        return max(min(x, max_value), min_value)

    def smooth_step(self, t, T):
        if T <= 0.0:
            return 1.0

        s = t / T
        s = self.clamp(s, 0.0, 1.0)

        # smooth step
        # s=0,1 で速度が0に近くなる
        return 3.0 * s * s - 2.0 * s * s * s

    def publish_z_velocity_command(self):
        """
        目標高度へ移動するための速度指令。
        GO_TARGET_ALTITUDE で使用する。
        """
        if self.current_z is None:
            return

        error_z = self.target_z - self.current_z
        vz_cmd = self.kp_z * error_z
        vz_cmd = self.clamp(vz_cmd, -self.max_vz, self.max_vz)

        nav_msg = FlightNav()
        nav_msg.header.stamp = rospy.Time.now()
        nav_msg.control_frame = FlightNav.WORLD_FRAME
        nav_msg.target = FlightNav.COG

        nav_msg.pos_z_nav_mode = FlightNav.VEL_MODE
        nav_msg.target_vel_z = vz_cmd

        self.nav_pub.publish(nav_msg)

    def publish_z_position_command(self, target_z):
        """
        指定した高度を維持するための位置指令。
        SET_JOINT2, SLOW_JOINT_MOTION, HOLD で使用する。
        """
        nav_msg = FlightNav()
        nav_msg.header.stamp = rospy.Time.now()
        nav_msg.control_frame = FlightNav.WORLD_FRAME
        nav_msg.target = FlightNav.COG

        nav_msg.pos_z_nav_mode = FlightNav.POS_MODE
        nav_msg.target_pos_z = target_z

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

    def joint2_trajectory(self, t):
        """
        q1, q3 は開始角度で固定し，q2 だけをゆっくり q2_const へ動かす。
        """
        s = self.smooth_step(t, self.joint2_set_time)

        q1 = self.q1_start
        q2 = self.q2_start + s * (self.q2_const - self.q2_start)
        q3 = self.q3_start

        return q1, q2, q3

    def slow_joint_trajectory(self, t):
        """
        q2 は固定し，q1, q3 だけをゆっくり変化させる。
        """
        s = self.smooth_step(t, self.motion_duration)

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
            # 1. q2 だけを先にゆっくり目標値へ移動
            q1, q2, q3 = self.joint2_trajectory(elapsed)
            self.publish_joint_command(q1, q2, q3)

            # この段階では目標高度へ移動せず，現在高度を維持
            if self.current_z is not None:
                self.publish_z_position_command(self.current_z)

            if elapsed > self.joint2_set_time:
                self.change_state("GO_TARGET_ALTITUDE")

        elif self.state == "GO_TARGET_ALTITUDE":
            # 2. q2 が目標値になったあと，姿勢を固定したまま目標高度へ移動
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
            # 3. 目標高度を維持しながら q1, q3 だけをゆっくり変化
            self.publish_z_position_command(self.target_z)

            t = (now - self.motion_start_time).to_sec()
            q1, q2, q3 = self.slow_joint_trajectory(t)
            self.publish_joint_command(q1, q2, q3)

            if t > self.motion_duration:
                self.change_state("HOLD")

        elif self.state == "HOLD":
            # 4. 最終姿勢と目標高度を維持
            self.publish_z_position_command(self.target_z)

            self.publish_joint_command(
                self.q1_goal,
                self.q2_const,
                self.q3_goal
            )


if __name__ == "__main__":
    node = CeilingEffectExperimentNode()
    rospy.spin()