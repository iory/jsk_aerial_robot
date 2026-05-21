#!/usr/bin/env python
from __future__ import print_function

import rospy
from sensor_msgs.msg import JointState


class CeilingEffectPrepareNode(object):
    def __init__(self):
        rospy.init_node("ceiling_effect_prepare_node")

        self.robot_ns = rospy.get_param("~robot_ns", "/hydrus")
        if not self.robot_ns.startswith("/"):
            self.robot_ns = "/" + self.robot_ns
        self.robot_ns = self.robot_ns.rstrip("/")

        # ===== 関節角度パラメータ =====
        self.q1_const = rospy.get_param("~q1_const", 1.40)
        self.q2_const = rospy.get_param("~q2_const", 0.60)
        self.q3_start_param = rospy.get_param("~q3_start", 1.40)

        self.joint_set_speed = rospy.get_param("~joint_set_speed", 0.03)
        self.min_joint_set_time = rospy.get_param("~min_joint_set_time", 15.0)

        # ===== 現在値 =====
        self.current_q1 = None
        self.current_q2 = None
        self.current_q3 = None

        # ===== 開始値 =====
        self.q1_start = None
        self.q2_start = None
        self.q3_start = None

        self.joint2_set_time = None
        self.joint1_3_set_time = None
        self.joint1_3_motion_start_time = None

        # ===== 状態管理 =====
        self.state = "WAIT_JOINT_STATE"
        self.state_start_time = rospy.Time.now()

        # ===== Subscriber =====
        self.joint_state_sub = rospy.Subscriber(
            self.robot_ns + "/joint_states",
            JointState,
            self.joint_state_callback
        )

        # ===== Publisher =====
        self.joint_pub = rospy.Publisher(
            self.robot_ns + "/joints_ctrl",
            JointState,
            queue_size=1
        )

        self.timer = rospy.Timer(rospy.Duration(0.02), self.update)

        rospy.loginfo("ceiling_effect_prepare_node started")
        rospy.loginfo("robot_ns = %s", self.robot_ns)
        rospy.loginfo("q1 const = %.3f", self.q1_const)
        rospy.loginfo("q2 const = %.3f", self.q2_const)
        rospy.loginfo("q3 start = %.3f", self.q3_start_param)
        rospy.loginfo("joint_set_speed = %.3f [rad/s]", self.joint_set_speed)

    def joint_state_callback(self, msg):
        name_to_pos = dict(zip(msg.name, msg.position))

        if "joint1" in name_to_pos:
            self.current_q1 = name_to_pos["joint1"]
        if "joint2" in name_to_pos:
            self.current_q2 = name_to_pos["joint2"]
        if "joint3" in name_to_pos:
            self.current_q3 = name_to_pos["joint3"]

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

        if self.joint_set_speed <= 0.0:
            rospy.logwarn("joint_set_speed <= 0. Use default 0.03 rad/s")
            self.joint_set_speed = 0.03

        q2_diff = abs(self.q2_const - self.q2_start)

        self.joint2_set_time = max(
            q2_diff / self.joint_set_speed,
            self.min_joint_set_time
        )

        rospy.loginfo("initial joint angles received")
        rospy.loginfo(
            "current joints: q1=%.3f, q2=%.3f, q3=%.3f",
            self.q1_start,
            self.q2_start,
            self.q3_start
        )
        rospy.loginfo(
            "prepare target: q1=%.3f, q2=%.3f, q3=%.3f",
            self.q1_const,
            self.q2_const,
            self.q3_start_param
        )
        rospy.loginfo("joint2_set_time = %.3f [s]", self.joint2_set_time)

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

    def joint2_trajectory(self, t):
        """
        準備フェーズ1:
        q2 のみ q2_const へ移動。
        q1, q3 は初期値のまま。
        """
        s = self.smooth_step(t, self.joint2_set_time)

        q1 = self.q1_start
        q2 = self.q2_start + s * (self.q2_const - self.q2_start)
        q3 = self.q3_start

        return q1, q2, q3

    def joint1_3_trajectory(self, t):
        """
        準備フェーズ2:
        q1, q3 を開始姿勢へ移動。
        q2 は q2_const で固定。
        """
        s = self.smooth_step(t, self.joint1_3_set_time)

        q1 = self.q1_start + s * (self.q1_const - self.q1_start)
        q2 = self.q2_const
        q3 = self.q3_start + s * (self.q3_start_param - self.q3_start)

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
                    str(self.current_q1),
                    str(self.current_q2),
                    str(self.current_q3)
                )

        elif self.state == "SET_JOINT2":
            q1, q2, q3 = self.joint2_trajectory(elapsed)
            self.publish_joint_command(q1, q2, q3)

            if elapsed > self.joint2_set_time:
                self.q1_start = self.current_q1 if self.current_q1 is not None else q1
                self.q3_start = self.current_q3 if self.current_q3 is not None else q3

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
                    "q2 setup done. Next, moving q1 and q3. "
                    "q1: %.3f -> %.3f, q3: %.3f -> %.3f, duration: %.3f [s]",
                    self.q1_start,
                    self.q1_const,
                    self.q3_start,
                    self.q3_start_param,
                    self.joint1_3_set_time
                )

                self.change_state("SET_JOINT1_3")

        elif self.state == "SET_JOINT1_3":
            t_j13 = (now - self.joint1_3_motion_start_time).to_sec()

            q1, q2, q3 = self.joint1_3_trajectory(t_j13)
            self.publish_joint_command(q1, q2, q3)

            if t_j13 > self.joint1_3_set_time:
                rospy.loginfo(
                    "Preparation finished. Holding start pose: "
                    "q1=%.3f, q2=%.3f, q3=%.3f",
                    self.q1_const,
                    self.q2_const,
                    self.q3_start_param
                )

                self.change_state("HOLD_START_POSE")

        elif self.state == "HOLD_START_POSE":
            self.publish_joint_command(
                self.q1_const,
                self.q2_const,
                self.q3_start_param
            )


if __name__ == "__main__":
    node = CeilingEffectPrepareNode()
    rospy.spin()