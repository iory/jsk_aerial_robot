#!/usr/bin/env python

from __future__ import print_function

import rospy
from sensor_msgs.msg import JointState


class JointOnlyTestNode(object):
    def __init__(self):
        rospy.init_node("joint_only_test_node")

        self.robot_ns = rospy.get_param("~robot_ns", "/hydrus")
        if not self.robot_ns.startswith("/"):
            self.robot_ns = "/" + self.robot_ns
        self.robot_ns = self.robot_ns.rstrip("/")

        self.joint_pub = rospy.Publisher(
            self.robot_ns + "/joints_ctrl",
            JointState,
            queue_size=1
        )

        # 元コードと同じ値
        self.q2_const = rospy.get_param("~q2_const", 0.6)

        self.q1_start = rospy.get_param("~q1_start", 1.57)
        self.q1_goal  = rospy.get_param("~q1_goal", 1.20)

        self.q3_start = rospy.get_param("~q3_start", 1.57)
        self.q3_goal  = rospy.get_param("~q3_goal", 0.50)

        self.motion_duration = rospy.get_param("~motion_duration", 60.0)

        # 元コードの SET_JOINT2 相当
        self.joint2_set_time = rospy.get_param("~joint2_set_time", 3.0)

        self.start_time = rospy.Time.now()

        rospy.loginfo("joint_only_test_node started")
        rospy.loginfo("publish topic: %s", self.robot_ns + "/joints_ctrl")
        rospy.loginfo(
            "SET phase: q1=%.3f, q2=%.3f, q3=%.3f for %.1f sec",
            self.q1_start, self.q2_const, self.q3_start, self.joint2_set_time
        )
        rospy.loginfo(
            "MOTION phase: q1 %.3f -> %.3f, q2 %.3f fixed, q3 %.3f -> %.3f in %.1f sec",
            self.q1_start, self.q1_goal,
            self.q2_const,
            self.q3_start, self.q3_goal,
            self.motion_duration
        )

        self.timer = rospy.Timer(rospy.Duration(0.02), self.update)  # 50 Hz

    def clamp(self, x, min_value, max_value):
        return max(min(x, max_value), min_value)

    def smooth_step(self, s):
        return 3.0 * s * s - 2.0 * s * s * s

    def publish_joint_command(self, q1, q2, q3):
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.name = ["joint1", "joint2", "joint3"]
        msg.position = [q1, q2, q3]
        
        # velocity と effort を空にする（要素を持たせない）
        msg.velocity = []
        msg.effort = []

        self.joint_pub.publish(msg)

        rospy.loginfo_throttle(
            1.0,
            "publish joints_ctrl: q1=%.3f, q2=%.3f, q3=%.3f, subscribers=%d",
            q1, q2, q3, self.joint_pub.get_num_connections()
        )

    def update(self, event):
        now = rospy.Time.now()
        elapsed = (now - self.start_time).to_sec()

        # 最初の3秒：元コードの SET_JOINT2 と同じ
        if elapsed < self.joint2_set_time:
            q1 = self.q1_start
            q2 = self.q2_const
            q3 = self.q3_start
            self.publish_joint_command(q1, q2, q3)
            return

        # その後：元コードの SLOW_JOINT_MOTION と同じ
        t = elapsed - self.joint2_set_time

        s = t / self.motion_duration
        s = self.clamp(s, 0.0, 1.0)
        s = self.smooth_step(s)

        q1 = self.q1_start + s * (self.q1_goal - self.q1_start)
        q2 = self.q2_const
        q3 = self.q3_start + s * (self.q3_goal - self.q3_start)

        self.publish_joint_command(q1, q2, q3)

        if t > self.motion_duration:
            rospy.loginfo_throttle(2.0, "goal reached, holding final joint angles")


if __name__ == "__main__":
    node = JointOnlyTestNode()
    rospy.spin()