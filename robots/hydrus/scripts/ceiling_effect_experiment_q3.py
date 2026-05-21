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
        if not self.robot_ns.startswith("/"):
            self.robot_ns = "/" + self.robot_ns
        self.robot_ns = self.robot_ns.rstrip("/")

        # ===== 高度制御パラメータ =====
        self.target_d_R = rospy.get_param("~target_d_R", rospy.get_param(self.robot_ns + "/target_d_R", 8.0))
        self.ceiling_height = rospy.get_param(self.robot_ns + "/ceiling_height")
        self.ceiling_distance_offset = rospy.get_param(self.robot_ns + "/ceiling_distance_offset")
        self.rotor_radius = rospy.get_param(self.robot_ns + "/rotor_radius")

        self.target_d = self.target_d_R * self.rotor_radius
        self.target_z = (
            self.ceiling_height
            - self.ceiling_distance_offset
            - self.target_d
        )

        # 高度安定判定
        self.z_threshold = rospy.get_param("~z_threshold", 0.10)
        self.vz_threshold = rospy.get_param("~vz_threshold", 0.03)
        self.stable_time = rospy.get_param("~stable_time", 3.0)

        # 目標高度へ移動するときに使う速度制御
        self.kp_z = rospy.get_param("~kp_z", 0.10)
        self.max_vz = rospy.get_param("~max_vz", 0.05)

        # ===== 関節角度パラメータ =====
        # q1, q2 は固定
        self.q1_const = rospy.get_param("~q1_const", 1.40)
        self.q2_const = rospy.get_param("~q2_const", 0.60)

        # q3 がメイン変形を始める「実験スタート位置」
        self.q3_start_param = rospy.get_param("~q3_start", 1.40)

        # q3 の最終目標
        self.q3_goal = rospy.get_param("~q3_goal", 0.60)

        # 各関節をゆっくり整えるときの速度 [rad/s]
        self.joint_set_speed = rospy.get_param("~joint_set_speed", 0.03)

        # 各準備フェーズに最低でもかける時間 [s]
        self.min_joint_set_time = rospy.get_param("~min_joint_set_time", 15.0)

        # q3 を動かす時間 [s]
        self.motion_duration = rospy.get_param("~motion_duration", 60.0)

        # ===== 現在値 =====
        self.current_z = None
        self.current_vz = None

        self.current_q1 = None
        self.current_q2 = None
        self.current_q3 = None

        # ===== 軌道計算用の開始角度・高度保持 =====
        self.q1_start = None
        self.q2_start = None
        self.q3_start = None

        self.initial_experiment_z = None

        self.joint2_set_time = None
        self.joint1_3_set_time = None
        self.joint1_3_motion_start_time = None

        # ===== 状態管理 =====
        self.state = "WAIT_JOINT_STATE_AND_ODOM"
        self.state_start_time = rospy.Time.now()
        self.stable_start_time = None
        self.motion_start_time = None

        # ===== Subscriber =====
        self.joint_state_sub = rospy.Subscriber(
            self.robot_ns + "/joint_states",
            JointState,
            self.joint_state_callback
        )

        self.odom_sub = rospy.Subscriber(
            self.robot_ns + "/uav/baselink/odom",
            Odometry,
            self.odom_callback
        )

        # ===== Publisher =====
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
        rospy.loginfo("robot_ns = %s", self.robot_ns)
        rospy.loginfo("target_d_R = %.3f", self.target_d_R)
        rospy.loginfo("target_z = %.3f [m]", self.target_z)
        rospy.loginfo("q1 const = %.3f", self.q1_const)
        rospy.loginfo("q2 const = %.3f", self.q2_const)
        rospy.loginfo("q3 start = %.3f, q3 goal = %.3f", self.q3_start_param, self.q3_goal)
        rospy.loginfo("joint_set_speed = %.3f [rad/s]", self.joint_set_speed)
        rospy.loginfo("motion_duration = %.3f [s]", self.motion_duration)

    def joint_state_callback(self, msg):
        name_to_pos = dict(zip(msg.name, msg.position))

        if "joint1" in name_to_pos:
            self.current_q1 = name_to_pos["joint1"]
        if "joint2" in name_to_pos:
            self.current_q2 = name_to_pos["joint2"]
        if "joint3" in name_to_pos:
            self.current_q3 = name_to_pos["joint3"]

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

        return 3.0 * s * s - 2.0 * s * s * s

    def initialize_start_angles(self):
        self.q1_start = self.current_q1
        self.q2_start = self.current_q2
        self.q3_start = self.current_q3

        self.initial_experiment_z = self.current_z

        q2_diff = abs(self.q2_const - self.q2_start)

        if self.joint_set_speed <= 0.0:
            rospy.logwarn("joint_set_speed <= 0. Use default 0.03 rad/s")
            self.joint_set_speed = 0.03

        self.joint2_set_time = max(
            q2_diff / self.joint_set_speed,
            self.min_joint_set_time
        )

        rospy.loginfo("initial joint angles and altitude received")
        rospy.loginfo(
            "Locked initial z for preparation phases = %.3f [m]",
            self.initial_experiment_z
        )
        rospy.loginfo(
            "current joints: q1 = %.3f, q2 = %.3f, q3 = %.3f",
            self.q1_start,
            self.q2_start,
            self.q3_start
        )
        rospy.loginfo(
            "setup target: q1 = %.3f, q2 = %.3f, q3 = %.3f",
            self.q1_const,
            self.q2_const,
            self.q3_start_param
        )
        rospy.loginfo("joint2_set_time = %.3f [s]", self.joint2_set_time)

    def publish_z_velocity_command(self):
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
        msg.velocity = []
        msg.effort = []

        self.joint_pub.publish(msg)

        rospy.loginfo_throttle(
            2.0,
            "[State: %s] joints_ctrl: q1=%.3f, q2=%.3f, q3=%.3f",
            self.state,
            q1,
            q2,
            q3
        )

    def is_altitude_stable(self):
        if self.current_z is None or self.current_vz is None:
            return False

        z_error = abs(self.target_z - self.current_z)
        vz_abs = abs(self.current_vz)

        return z_error < self.z_threshold and vz_abs < self.vz_threshold

    def joint2_trajectory(self, t):
        """
        準備フェーズ1:
        初期高度を維持しながら，q2 のみを q2_const へゆっくり移動。
        q1, q3 は現在角度を維持。
        """
        s = self.smooth_step(t, self.joint2_set_time)

        q1 = self.q1_start
        q2 = self.q2_start + s * (self.q2_const - self.q2_start)
        q3 = self.q3_start

        return q1, q2, q3

    def joint1_3_trajectory(self, t):
        """
        準備フェーズ2:
        初期高度を維持しながら，
        q1 を q1_const へ，
        q3 を q3_start_param へゆっくり揃える。
        q2 は q2_const で固定。
        """
        s = self.smooth_step(t, self.joint1_3_set_time)

        q1 = self.q1_start + s * (self.q1_const - self.q1_start)
        q2 = self.q2_const
        q3 = self.q3_start + s * (self.q3_start_param - self.q3_start)

        return q1, q2, q3

    def slow_joint_trajectory(self, t):
        """
        メイン実験フェーズ:
        天井付近の目標高度を維持しながら，
        q1 = q1_const 固定
        q2 = q2_const 固定
        q3 のみ q3_start_param -> q3_goal へ smooth step で変化
        """
        s = self.smooth_step(t, self.motion_duration)

        q1 = self.q1_const
        q2 = self.q2_const
        q3 = self.q3_start + s * (self.q3_goal - self.q3_start)

        return q1, q2, q3

    def update(self, event):
        now = rospy.Time.now()
        elapsed = (now - self.state_start_time).to_sec()

        if self.state == "WAIT_JOINT_STATE_AND_ODOM":
            if (
                self.current_q1 is not None and
                self.current_q2 is not None and
                self.current_q3 is not None and
                self.current_z is not None
            ):
                self.initialize_start_angles()
                self.change_state("SET_JOINT2")
            else:
                rospy.loginfo_throttle(
                    2.0,
                    "waiting for joint_states and odom: "
                    "q1=%s, q2=%s, q3=%s, z=%s",
                    str(self.current_q1),
                    str(self.current_q2),
                    str(self.current_q3),
                    str(self.current_z)
                )

        elif self.state == "SET_JOINT2":
            # 1. 初期高度を維持しながら，まず q2 のみをゆっくり 0.6 へ移動
            q1, q2, q3 = self.joint2_trajectory(elapsed)
            self.publish_joint_command(q1, q2, q3)

            if self.initial_experiment_z is not None:
                self.publish_z_position_command(self.initial_experiment_z)

            if elapsed > self.joint2_set_time:
                # 次フェーズの開始角度を現在値から取得
                self.q1_start = self.current_q1 if self.current_q1 is not None else q1
                self.q3_start = self.current_q3 if self.current_q3 is not None else q3

                # q1=1.4, q3=1.4 に揃えるための時間
                max_diff = max(
                    abs(self.q1_const - self.q1_start),
                    abs(self.q3_start_param - self.q3_start)
                )

                self.joint1_3_set_time = max(
                    max_diff / self.joint_set_speed,
                    self.min_joint_set_time
                )

                self.joint1_3_motion_start_time = now

                rospy.loginfo(
                    "q2 setup done. Next, moving q1 and q3 to start positions. "
                    "q1: %.3f -> %.3f, q3: %.3f -> %.3f, duration: %.3f [s]",
                    self.q1_start,
                    self.q1_const,
                    self.q3_start,
                    self.q3_start_param,
                    self.joint1_3_set_time
                )

                self.change_state("SET_JOINT1_3")

        elif self.state == "SET_JOINT1_3":
            # 2. 初期高度を維持したまま，q1 を 1.4，q3 を 1.4 に揃える
            t_j13 = (now - self.joint1_3_motion_start_time).to_sec()
            q1, q2, q3 = self.joint1_3_trajectory(t_j13)
            self.publish_joint_command(q1, q2, q3)

            if self.initial_experiment_z is not None:
                self.publish_z_position_command(self.initial_experiment_z)

            if t_j13 > self.joint1_3_set_time:
                # メイン実験の開始姿勢を理想値で明示的に確定
                self.q1_start = self.q1_const
                self.q3_start = self.q3_start_param

                rospy.loginfo(
                    "All joints aligned to start positions: "
                    "q1=%.3f, q2=%.3f, q3=%.3f. Moving to target altitude.",
                    self.q1_start,
                    self.q2_const,
                    self.q3_start
                )

                self.change_state("GO_TARGET_ALTITUDE")

        elif self.state == "GO_TARGET_ALTITUDE":
            # 3. q1=1.4, q2=0.6, q3=1.4 を維持しながら目標高度へ移動
            self.publish_joint_command(
                self.q1_const,
                self.q2_const,
                self.q3_start_param
            )
            self.publish_z_velocity_command()

            if self.is_altitude_stable():
                if self.stable_start_time is None:
                    self.stable_start_time = now

                stable_elapsed = (now - self.stable_start_time).to_sec()

                rospy.loginfo_throttle(
                    1.0,
                    "altitude stable candidate: z=%.3f, target_z=%.3f, vz=%.3f, stable_elapsed=%.2f",
                    self.current_z,
                    self.target_z,
                    self.current_vz,
                    stable_elapsed
                )

                if stable_elapsed > self.stable_time:
                    self.motion_start_time = now

                    rospy.loginfo(
                        "Altitude stabilized. Starting q3-only motion. "
                        "q1=%.3f, q2=%.3f fixed, q3=%.3f -> %.3f",
                        self.q1_const,
                        self.q2_const,
                        self.q3_start_param,
                        self.q3_goal
                    )

                    self.change_state("SLOW_JOINT_MOTION")
            else:
                self.stable_start_time = None

        elif self.state == "SLOW_JOINT_MOTION":
            # 4. 天井付近の目標高度をホールドしながら，q3 のみゆっくり変化
            self.publish_z_position_command(self.target_z)

            t = (now - self.motion_start_time).to_sec()
            q1, q2, q3 = self.slow_joint_trajectory(t)
            self.publish_joint_command(q1, q2, q3)

            if t > self.motion_duration:
                rospy.loginfo(
                    "q3-only motion finished. Holding final pose: "
                    "q1=%.3f, q2=%.3f, q3=%.3f",
                    self.q1_const,
                    self.q2_const,
                    self.q3_goal
                )

                self.change_state("HOLD")

        elif self.state == "HOLD":
            # 5. 最終姿勢と目標高度を維持
            self.publish_z_position_command(self.target_z)
            self.publish_joint_command(
                self.q1_const,
                self.q2_const,
                self.q3_goal
            )


if __name__ == "__main__":
    node = CeilingEffectExperimentNode()
    rospy.spin()