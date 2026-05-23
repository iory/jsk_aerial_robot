#!/usr/bin/env python3
import rospy

from std_msgs.msg import Float32, Float64, Float64MultiArray, Float32MultiArray, ColorRGBA
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from jsk_rviz_plugins.msg import OverlayText

from spinal.msg import Pwms
from aerial_robot_msgs.msg import FlightNav


class CeilingEffectVideoOverlay:
    def __init__(self):
        self.d_R = None
        self.l_R_list = None
        self.gain_list = None

        self.pwms = [None, None, None, None]

        # 今回は COG 基準
        # z_actual = z_cog
        # z_target = target_pos_z sent to FlightNav.COG
        # z_error  = z_target - z_actual
        self.z_actual = None
        self.z_target = None

        # ===== ceiling marker parameters =====
        self.frame_id = rospy.get_param("~frame_id", "world")

        self.ceiling_height = rospy.get_param(
            "/hydrus/ceiling_height",
            rospy.get_param("~ceiling_height", 2.73)
        )

        self.ceiling_size_x = rospy.get_param("~ceiling_size_x", 8.0)
        self.ceiling_size_y = rospy.get_param("~ceiling_size_y", 8.0)
        self.ceiling_thickness = rospy.get_param("~ceiling_thickness", 0.05)

        # ===== subscribers =====
        rospy.Subscriber(
            "/hydrus/ceiling_effect/ceiling_distance_ratio",
            Float64,
            self.d_cb
        )

        rospy.Subscriber(
            "/hydrus/ceiling_effect/rotor_distance_ratio",
            Float64MultiArray,
            self.l_cb
        )

        rospy.Subscriber(
            "/hydrus/ceiling_effect/thrust_ratio",
            Float32MultiArray,
            self.gain_cb
        )

        rospy.Subscriber(
            "/hydrus/motor_pwms",
            Pwms,
            self.pwm_cb
        )

        # 変更点:
        # 以前は /hydrus/uav/baselink/odom を読んでいたが、
        # child_frame_id が hydrus/fc なので z_cog ではない。
        # 今回の e_cog = z_target_cog - z_cog を見るため、
        # /hydrus/uav/cog/odom を読む。
        rospy.Subscriber(
            "/hydrus/uav/cog/odom",
            Odometry,
            self.odom_cb
        )

        rospy.Subscriber(
            "/hydrus/uav/nav",
            FlightNav,
            self.nav_cb
        )

        # ===== overlay text =====
        self.pub_text = rospy.Publisher(
            "/ce_viz/text",
            OverlayText,
            queue_size=1
        )

        # ===== PieChart / Plotter用 Float32 topics =====
        self.pub_d_R = rospy.Publisher("/ce_viz/d_R", Float32, queue_size=1)

        # l/R individual topics
        self.pub_l12_R = rospy.Publisher("/ce_viz/l12_R", Float32, queue_size=1)
        self.pub_l13_R = rospy.Publisher("/ce_viz/l13_R", Float32, queue_size=1)
        self.pub_l14_R = rospy.Publisher("/ce_viz/l14_R", Float32, queue_size=1)
        self.pub_l23_R = rospy.Publisher("/ce_viz/l23_R", Float32, queue_size=1)
        self.pub_l24_R = rospy.Publisher("/ce_viz/l24_R", Float32, queue_size=1)
        self.pub_l34_R = rospy.Publisher("/ce_viz/l34_R", Float32, queue_size=1)

        # gain individual topics
        self.pub_gain1 = rospy.Publisher("/ce_viz/gain1", Float32, queue_size=1)
        self.pub_gain2 = rospy.Publisher("/ce_viz/gain2", Float32, queue_size=1)
        self.pub_gain3 = rospy.Publisher("/ce_viz/gain3", Float32, queue_size=1)
        self.pub_gain4 = rospy.Publisher("/ce_viz/gain4", Float32, queue_size=1)

        # PWM individual topics
        self.pub_pwm1 = rospy.Publisher("/ce_viz/pwm1", Float32, queue_size=1)
        self.pub_pwm2 = rospy.Publisher("/ce_viz/pwm2", Float32, queue_size=1)
        self.pub_pwm3 = rospy.Publisher("/ce_viz/pwm3", Float32, queue_size=1)
        self.pub_pwm4 = rospy.Publisher("/ce_viz/pwm4", Float32, queue_size=1)

        # z topics
        self.pub_z_actual = rospy.Publisher("/ce_viz/z_actual", Float32, queue_size=1)
        self.pub_z_target = rospy.Publisher("/ce_viz/z_target", Float32, queue_size=1)
        self.pub_z_error = rospy.Publisher("/ce_viz/z_error", Float32, queue_size=1)

        # ===== ceiling marker publisher =====
        self.pub_ceiling_marker = rospy.Publisher(
            "/ce_viz/ceiling_marker",
            Marker,
            queue_size=1,
            latch=True
        )

        self.timer = rospy.Timer(rospy.Duration(0.05), self.publish)

        rospy.loginfo("ce_video_overlay started")
        rospy.loginfo("ceiling marker frame_id = %s", self.frame_id)
        rospy.loginfo("ceiling_height = %.3f", self.ceiling_height)
        rospy.loginfo("z_actual source = /hydrus/uav/cog/odom")
        rospy.loginfo("z_error definition = z_target - z_cog")

    def d_cb(self, msg):
        self.d_R = msg.data

    def l_cb(self, msg):
        if len(msg.data) > 0:
            self.l_R_list = list(msg.data)

    def gain_cb(self, msg):
        if len(msg.data) > 0:
            self.gain_list = list(msg.data)

    def pwm_cb(self, msg):
        for i in range(min(4, len(msg.motor_value))):
            self.pwms[i] = msg.motor_value[i]

    def odom_cb(self, msg):
        # /hydrus/uav/cog/odom の z
        # つまり z_cog
        self.z_actual = msg.pose.pose.position.z

    def nav_cb(self, msg):
        # POS_MODE のときだけ target_pos_z を使う。
        # VEL_MODE のときは target_pos_z は意味を持たない可能性があるため無視。
        if msg.pos_z_nav_mode == FlightNav.POS_MODE:
            self.z_target = msg.target_pos_z

    def publish_float(self, pub, value):
        if value is not None:
            pub.publish(Float32(float(value)))

    def publish_ceiling_marker(self):
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = rospy.Time.now()

        marker.ns = "ceiling"
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD

        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = self.ceiling_height

        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        marker.scale.x = self.ceiling_size_x
        marker.scale.y = self.ceiling_size_y
        marker.scale.z = self.ceiling_thickness

        marker.color.r = 0.5
        marker.color.g = 0.5
        marker.color.b = 0.5
        marker.color.a = 0.35

        marker.lifetime = rospy.Duration(0.0)

        self.pub_ceiling_marker.publish(marker)

    def publish(self, event):
        # ===== d/R =====
        self.publish_float(self.pub_d_R, self.d_R)

        # ===== l/R 6個を個別publish =====
        # 注意: msg.data の順番が [l12, l13, l14, l23, l24, l34] である前提
        if self.l_R_list is not None and len(self.l_R_list) >= 6:
            self.publish_float(self.pub_l12_R, self.l_R_list[0])
            self.publish_float(self.pub_l13_R, self.l_R_list[1])
            self.publish_float(self.pub_l14_R, self.l_R_list[2])
            self.publish_float(self.pub_l23_R, self.l_R_list[3])
            self.publish_float(self.pub_l24_R, self.l_R_list[4])
            self.publish_float(self.pub_l34_R, self.l_R_list[5])

        # ===== gain 4個を個別publish =====
        if self.gain_list is not None and len(self.gain_list) >= 4:
            self.publish_float(self.pub_gain1, self.gain_list[0])
            self.publish_float(self.pub_gain2, self.gain_list[1])
            self.publish_float(self.pub_gain3, self.gain_list[2])
            self.publish_float(self.pub_gain4, self.gain_list[3])

        # ===== PWM 4個を個別publish =====
        self.publish_float(self.pub_pwm1, self.pwms[0])
        self.publish_float(self.pub_pwm2, self.pwms[1])
        self.publish_float(self.pub_pwm3, self.pwms[2])
        self.publish_float(self.pub_pwm4, self.pwms[3])

        # ===== z =====
        self.publish_float(self.pub_z_actual, self.z_actual)
        self.publish_float(self.pub_z_target, self.z_target)

        z_error = None
        if self.z_actual is not None and self.z_target is not None:
            # e_cog = z_target_cog - z_cog
            z_error = self.z_target - self.z_actual
            self.publish_float(self.pub_z_error, z_error)

        # ===== OverlayText =====
        text = OverlayText()
        text.width = 560
        text.height = 430
        text.left = 20
        text.top = 20
        text.text_size = 18
        text.line_width = 2
        text.font = "DejaVu Sans Mono"

        text.fg_color = ColorRGBA(1.0, 1.0, 1.0, 1.0)
        text.bg_color = ColorRGBA(0.0, 0.0, 0.0, 0.65)

        d_R_str = f"{self.d_R:.3f}" if self.d_R is not None else "--"

        if self.l_R_list is not None and len(self.l_R_list) >= 6:
            l12_str = f"{self.l_R_list[0]:.3f}"
            l13_str = f"{self.l_R_list[1]:.3f}"
            l14_str = f"{self.l_R_list[2]:.3f}"
            l23_str = f"{self.l_R_list[3]:.3f}"
            l24_str = f"{self.l_R_list[4]:.3f}"
            l34_str = f"{self.l_R_list[5]:.3f}"
        else:
            l12_str = l13_str = l14_str = l23_str = l24_str = l34_str = "--"

        if self.gain_list is not None and len(self.gain_list) >= 4:
            gain1_str = f"{self.gain_list[0]:.3f}"
            gain2_str = f"{self.gain_list[1]:.3f}"
            gain3_str = f"{self.gain_list[2]:.3f}"
            gain4_str = f"{self.gain_list[3]:.3f}"
        else:
            gain1_str = gain2_str = gain3_str = gain4_str = "--"

        z_actual_str = f"{self.z_actual:.3f}" if self.z_actual is not None else "--"
        z_target_str = f"{self.z_target:.3f}" if self.z_target is not None else "--"
        z_error_str = f"{z_error:.3f}" if z_error is not None else "--"

        pwm_strs = []
        for pwm in self.pwms:
            pwm_strs.append(str(pwm) if pwm is not None else "--")

        text.text = (
            "Ceiling Effect Compensation\n"
            "---------------------------\n"
            f"d/R        : {d_R_str}\n"
            "\n"
            f"l12/R      : {l12_str}\n"
            f"l13/R      : {l13_str}\n"
            f"l14/R      : {l14_str}\n"
            f"l23/R      : {l23_str}\n"
            f"l24/R      : {l24_str}\n"
            f"l34/R      : {l34_str}\n"
            "\n"
            f"gain1 1/k  : {gain1_str}\n"
            f"gain2 1/k  : {gain2_str}\n"
            f"gain3 1/k  : {gain3_str}\n"
            f"gain4 1/k  : {gain4_str}\n"
            "\n"
            f"z actual   : {z_actual_str} m\n"
            f"z target   : {z_target_str} m\n"
            f"z error    : {z_error_str} m\n"
            f"error def. : target - cog\n"
            "\n"
            f"PWM        : {pwm_strs[0]}, {pwm_strs[1]}, {pwm_strs[2]}, {pwm_strs[3]}"
        )

        self.pub_text.publish(text)

        # ceiling plane in RViz
        self.publish_ceiling_marker()


if __name__ == "__main__":
    rospy.init_node("ce_video_overlay")
    CeilingEffectVideoOverlay()
    rospy.spin()