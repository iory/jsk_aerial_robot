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

        self.q1_goal = rospy.get_param("~q1_goal", 1.20)
        self.q3_goal = rospy.get_param("~q3_goal", 0.50)

        # q2 を動かす速度 [rad/s]
        self.q2_speed = rospy.get_param("~q2_speed", 0.1)

        # q1, q3 を動かす時間
        self.motion_duration = rospy.get_param("~motion_duration", 60.0)

        # 現在角度
        self.current_q1 = None
        self.current_q2 = None
        self.current_q3 = None

        # 実験開始時の角度
        self.q1_start = None
        self.q2_start = None
        self.q3_start = None

        self.joint2_set_time = None

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
        rospy.loginfo("q1 goal = %.3f", self.q1_goal)
        rospy.loginfo("q3 goal = %.3f", self.q3_goal)
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
            rospy.logwarn("q2_speed <= 0. Use default 0.1 rad/s")
            self.q2_speed = 0.1

        self.joint2_set_time = q2_diff / self.q2_speed

        rospy.loginfo("initial joint angles received")
        rospy.loginfo("q1_start = %.3f", self.q1_start)
        rospy.loginfo("q2_start = %.3f -> q2_const = %.3f", self.q2_start, self.q2_const)
        rospy.loginfo("q3_start = %.3f", self.q3_start)
        rospy.loginfo("joint2_set_time = %.3f sec", self.joint2_set_time)

    def joint2_trajectory(self, t):
        """
        q1, q3 は現在角度で固定し、
        q2 だけを q2_start から q2_const へゆっくり動かす。
        速度目安は q2_speed [rad/s]。
        """
        s = self.smooth_step(t, self.joint2_set_time)

        q1 = self.q1_start
        q2 = self.q2_start + s * (self.q2_const - self.q2_start)
        q3 = self.q3_start

        return q1, q2, q3

    def slow_joint_trajectory(self, t):
        """
        q2 は固定し、q1, q3 だけをゆっくり変化させる。
        q1, q3 の開始値も実機の現在値から始める。
        """
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
                    str(self.current_q1),
                    str(self.current_q2),
                    str(self.current_q3)
                )

        elif self.state == "SET_JOINT2":
            q1, q2, q3 = self.joint2_trajectory(elapsed)
            self.publish_joint_command(q1, q2, q3)

            if elapsed > self.joint2_set_time:
                self.motion_start_time = now
                self.change_state("SLOW_JOINT_MOTION")

        elif self.state == "SLOW_JOINT_MOTION":
            t = (now - self.motion_start_time).to_sec()
            q1, q2, q3 = self.slow_joint_trajectory(t)
            self.publish_joint_command(q1, q2, q3)

            if t > self.motion_duration:
                self.change_state("HOLD")

        elif self.state == "HOLD":
            self.publish_joint_command(
                self.q1_goal,
                self.q2_const,
                self.q3_goal
            )


if __name__ == "__main__":
    node = JointMotionOnlyExperimentNode()
    rospy.spin()