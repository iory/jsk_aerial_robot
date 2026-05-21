#!/usr/bin/env python
from __future__ import print_function

import rospy
from sensor_msgs.msg import JointState


class JointMotionOnlyExperimentNode(object):
    def __init__(self):
        rospy.init_node("joint_motion_only_experiment_node")

        self.robot_ns = rospy.get_param("~robot_ns", "/hydrus")
        if not self.robot_ns.startswith("/"):
            self.robot_ns = "/" + self.robot_ns
        self.robot_ns = self.robot_ns.rstrip("/")

        # ===== 目標関節角度 =====
        self.q2_const = rospy.get_param("~q2_const", 0.6)

        # q1, q3 がメイン変形を始める「実験スタート位置」
        self.q1_start_param = rospy.get_param("~q1_start", 1.40)
        self.q3_start_param = rospy.get_param("~q3_start", 1.40)

        # q1, q3 の最終目標
        self.q1_goal = rospy.get_param("~q1_goal", 0.80)
        self.q3_goal = rospy.get_param("~q3_goal", 0.50)

        # 各関節をゆっくり整えるときの速度 [rad/s]
        self.q2_speed = rospy.get_param("~q2_speed", 0.03)

        # 各準備フェーズに最低でもかける時間 [s]
        self.min_joint2_set_time = rospy.get_param("~min_joint2_set_time", 15.0)

        # q1, q3 を動かす時間
        self.motion_duration = rospy.get_param("~motion_duration", 60.0)

        # 現在角度
        self.current_q1 = None
        self.current_q2 = None
        self.current_q3 = None

        # 軌道計算用の開始・目標角度保持
        self.q1_start = None
        self.q2_start = None
        self.q3_start = None

        self.joint2_set_time = None
        self.joint1_3_set_time = None
        self.joint1_3_motion_start_time = None

        # ===== 状態管理 =====
        self.state = "WAIT_JOINT_STATE"
        self.state_start_time = rospy.Time.now()
        self.motion_start_time = None

        self.joint_state_sub = rospy.Subscriber(
            self.robot_ns + "/joint_states",
            JointState,
            self.joint_state_callback
        )

        self.joint_pub = rospy.Publisher(
            self.robot_ns + "/joints_ctrl",
            JointState,
            queue_size=1
        )

        self.timer = rospy.Timer(rospy.Duration(0.02), self.update)  # 50 Hz

        rospy.loginfo("joint_motion_only_experiment_node started")
        rospy.loginfo("subscribe topic: %s", self.robot_ns + "/joint_states")
        rospy.loginfo("publish topic: %s", self.robot_ns + "/joints_ctrl")
        rospy.loginfo("q2 target = %.3f", self.q2_const)
        rospy.loginfo("q2 speed = %.3f rad/s", self.q2_speed)
        rospy.loginfo("q1 start_param = %.3f, goal = %.3f", self.q1_start_param, self.q1_goal)
        rospy.loginfo("q3 start_param = %.3f, goal = %.3f", self.q3_start_param, self.q3_goal)
        rospy.loginfo("motion_duration = %.3f sec", self.motion_duration)

    def joint_state_callback(self, msg):
        name_to_pos = dict(zip(msg.name, msg.position))

        if "joint1" in name_to_pos:
            self.current_q1 = name_to_pos["joint1"]
        if "joint2" in name_to_pos:
            self.current_q2 = name_to_pos["joint2"]
        if "joint3" in name_to_pos:
            self.current_q3 = name_to_pos["joint3"]

    def clamp(self, x, min_value, max_value):
        return max(min(x, max_value), min_value)

    def smooth_step(self, t, T):
        if T <= 0.0:
            return 1.0
        s = t / T
        s = self.clamp(s, 0.0, 1.0)
        return 3.0 * s * s - 2.0 * s * s * s

    def publish_joint_command(self, q1, q2, q3):
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.name = ["joint1", "joint2", "joint3"]
        msg.position = [q1, q2, q3]

        # velocity / effort は空で送る
        msg.velocity = []
        msg.effort = []

        self.joint_pub.publish(msg)

        rospy.loginfo_throttle(
            2.0,
            "[State: %s] publish joints_ctrl: q1=%.3f, q2=%.3f, q3=%.3f",
            self.state, q1, q2, q3
        )

    def change_state(self, next_state):
        rospy.loginfo("state: %s -> %s", self.state, next_state)
        self.state = next_state
        self.state_start_time = rospy.Time.now()

    def initialize_start_angles(self):
        self.q1_start = self.current_q1
        self.q2_start = self.current_q2
        self.q3_start = self.current_q3

        q2_diff = abs(self.q2_const - self.q2_start)

        if self.q2_speed <= 0.0:
            rospy.logwarn("q2_speed <= 0. Use default 0.03 rad/s")
            self.q2_speed = 0.03

        self.joint2_set_time = max(
            q2_diff / self.q2_speed,
            self.min_joint2_set_time
        )

        rospy.loginfo("initial joint angles received")
        rospy.loginfo(
            "q1_start = %.3f, q2_start = %.3f -> q2_const = %.3f, q3_start = %.3f",
            self.q1_start, self.q2_start, self.q2_const, self.q3_start
        )
        rospy.loginfo("joint2_set_time = %.3f sec", self.joint2_set_time)

    def joint2_trajectory(self, t):
        s = self.smooth_step(t, self.joint2_set_time)
        q1 = self.q1_start
        q2 = self.q2_start + s * (self.q2_const - self.q2_start)
        q3 = self.q3_start
        return q1, q2, q3

    def joint1_3_trajectory(self, t):
        s = self.smooth_step(t, self.joint1_3_set_time)
        q1 = self.q1_start + s * (self.q1_start_param - self.q1_start)
        q2 = self.q2_const
        q3 = self.q3_start + s * (self.q3_start_param - self.q3_start)
        return q1, q2, q3

    def slow_joint_trajectory(self, t):
        s = self.smooth_step(t, self.motion_duration)
        q1 = self.q1_start + s * (self.q1_goal - self.q1_start)
        q2 = self.q2_const
        q3 = self.q3_start + s * (self.q3_goal - self.q3_start)
        return q1, q2, q3

    def update(self, event):
        now = rospy.Time.now()
        elapsed = (now - self.state_start_time).to_sec()

        if self.state == "WAIT_JOINT_STATE":
            if (
                self.current_q1 is not None and
                self.current_q2 is not None and
                self.current_q3 is not None
            ):
                self.initialize_start_angles()
                self.change_state("SET_JOINT2")
            else:
                rospy.loginfo_throttle(
                    2.0,
                    "waiting for joint_states: q1=%s, q2=%s, q3=%s",
                    str(self.current_q1), str(self.current_q2), str(self.current_q3)
                )

        elif self.state == "SET_JOINT2":
            # 1. まず q2 のみを指定速度でゆっくり移動
            q1, q2, q3 = self.joint2_trajectory(elapsed)
            self.publish_joint_command(q1, q2, q3)

            if elapsed > self.joint2_set_time:
                # 移動開始前の状態を取得して次のフェーズの始点とする
                self.q1_start = self.current_q1 if self.current_q1 is not None else q1
                self.q3_start = self.current_q3 if self.current_q3 is not None else q3
                
                max_diff = max(abs(self.q1_start_param - self.q1_start), abs(self.q3_start_param - self.q3_start))
                self.joint1_3_set_time = max(max_diff / self.q2_speed, self.min_joint2_set_time)
                
                self.joint1_3_motion_start_time = now
                rospy.loginfo("q2 setup done. Next, moving q1 and q3 to start positions (Duration: %.1f sec)", self.joint1_3_set_time)
                self.change_state("SET_JOINT1_3")

        elif self.state == "SET_JOINT1_3":
            # 2. q1 と q3 もゆっくり実験開始角度（スタート位置）に揃える
            t_j13 = (now - self.joint1_3_motion_start_time).to_sec()
            q1, q2, q3 = self.joint1_3_trajectory(t_j13)
            self.publish_joint_command(q1, q2, q3)

            if t_j13 > self.joint1_3_set_time:
                # 揃え終わったら、計算上の値ではなく理想のスタート位置（パラメータ値）を明示的に確定させる
                self.q1_start = self.q1_start_param
                self.q3_start = self.q3_start_param
                
                self.motion_start_time = now
                rospy.loginfo(
                    "All joints aligned to start positions: q1=%.3f, q2=%.3f, q3=%.3f. Starting main joint motion.",
                    self.q1_start, self.q2_const, self.q3_start
                )
                self.change_state("SLOW_JOINT_MOTION")

        elif self.state == "SLOW_JOINT_MOTION":
            # 3. 確定した理想的な初期姿勢から、q1, q3 を目標値へゆっくり変化（メイン実験フェーズ）
            t = (now - self.motion_start_time).to_sec()
            q1, q2, q3 = self.slow_joint_trajectory(t)
            self.publish_joint_command(q1, q2, q3)

            if t > self.motion_duration:
                self.change_state("HOLD")

        elif self.state == "HOLD":
            # 4. 最終姿勢を維持
            self.publish_joint_command(self.q1_goal, self.q2_const, self.q3_goal)


if __name__ == "__main__":
    node = JointMotionOnlyExperimentNode()
    rospy.spin()