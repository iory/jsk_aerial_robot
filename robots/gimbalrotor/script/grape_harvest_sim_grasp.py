#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Simulate grasping grape bunches in gazebo (simulation only).

The arm of grape_with_arm has no collision model in gazebo, and friction of the
fingers can not hold a bunch reliably in gazebo classic anyway. This node
stands in for the contact: when the gripper closes with the stem of a bunch
between the finger pads, the bunch is attached to the fingers; when the gripper
opens, the bunch is released and falls with gravity.

Nothing else is simulated here. The gripper still has to close through
arm_controller and the flight still has to bring the stem between the pads, so
the demo program (script/skrobot_grape_harvest_demo.py) is the same as on the
real machine and does not use this node.

Bunch models are the ones whose names start with ``~bunch_prefix``; their origin
is the grasp point on the stem (see script/make_grape_vineyard_world.py).
"""

import numpy as np
import rospy
from gazebo_msgs.msg import LinkStates
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import GetLinkProperties
from gazebo_msgs.srv import SetLinkProperties
from sensor_msgs.msg import JointState
from skrobot.coordinates import Coordinates
from skrobot.coordinates.math import matrix2quaternion
from skrobot.coordinates.math import quaternion2matrix
from skrobot.coordinates.math import wxyz2xyzw
from skrobot.coordinates.math import xyzw2wxyz
from skrobot.models.urdf import RobotModelFromURDF

from gimbalrotor.skrobot_interface import find_gripper_model
from gimbalrotor.skrobot_interface import find_gripper_open_sign
from gimbalrotor.skrobot_interface import gripper_center


def pose_to_coords(pose):
    q = pose.orientation
    return Coordinates(
        pos=[pose.position.x, pose.position.y, pose.position.z],
        rot=quaternion2matrix(xyzw2wxyz(np.array([q.x, q.y, q.z, q.w]))))


def coords_to_pose(coords, pose):
    pose.position.x, pose.position.y, pose.position.z = coords.worldpos()
    q = wxyz2xyzw(matrix2quaternion(coords.worldrot()))
    pose.orientation.x, pose.orientation.y, pose.orientation.z, \
        pose.orientation.w = q


class SimGrasp(object):

    def __init__(self):
        self.robot_ns = rospy.get_param('~robot_ns', 'gimbalrotor')
        self.model_name = rospy.get_param('~model_name', self.robot_ns)
        self.bunch_prefix = rospy.get_param('~bunch_prefix', 'harvest_bunch_')
        self.bunch_link = rospy.get_param('~bunch_link', 'bunch')
        # the gripper counts as closed below close_fraction of the width range
        # and as open above open_fraction
        self.close_fraction = rospy.get_param('~close_fraction', 0.5)
        self.open_fraction = rospy.get_param('~open_fraction', 0.8)
        # half extents [m] of the region around the pad center, in the frame of
        # the first finger link, where a stem is caught
        self.tolerance = np.array(
            rospy.get_param('~grasp_tolerance', [0.03, 0.03, 0.03]))

        urdf = rospy.get_param('/{}/robot_description'.format(self.robot_ns))
        joint_names = rospy.get_param(
            '/{}/arm/arm_controller/joints'.format(self.robot_ns))
        robot = RobotModelFromURDF(urdf=urdf)
        self.gripper = find_gripper_model(urdf, joint_names)
        self.gripper.set_open_sign(find_gripper_open_sign(robot, self.gripper))
        # pad center relative to the midpoint of the finger link origins,
        # which gazebo reports in link_states
        fingers = [getattr(robot, name)
                   for name in self.gripper.finger_link_names]
        midpoint = np.mean([f.worldpos() for f in fingers], axis=0)
        self.pad_offset = fingers[0].worldrot().T.dot(
            gripper_center(robot, self.gripper) - midpoint)
        self.finger_links = ['{}::{}'.format(self.model_name, name)
                             for name in self.gripper.finger_link_names]

        self.drive_angle = None
        self.link_states = None
        self.closed = False
        self.held = None  # name of the held bunch model
        self.held_offset = None  # pose of the bunch in the pad frame

        rospy.wait_for_service('/gazebo/get_link_properties')
        rospy.wait_for_service('/gazebo/set_link_properties')
        self.get_link_properties = rospy.ServiceProxy(
            '/gazebo/get_link_properties', GetLinkProperties)
        self.set_link_properties = rospy.ServiceProxy(
            '/gazebo/set_link_properties', SetLinkProperties)
        self.model_state_pub = rospy.Publisher(
            '/gazebo/set_model_state', ModelState, queue_size=1)
        rospy.Subscriber('/{}/joint_states'.format(self.robot_ns), JointState,
                         self.joint_states_cb, queue_size=1)
        rospy.Subscriber('/gazebo/link_states', LinkStates,
                         self.link_states_cb, queue_size=1)
        rospy.Timer(rospy.Duration(1.0 / rospy.get_param('~rate', 100.0)),
                    self.update)
        rospy.loginfo('[%s] gripper drive joint %s, pad offset %s',
                      rospy.get_name(), self.gripper.drive_joint_name,
                      np.round(self.pad_offset, 4))

    def joint_states_cb(self, msg):
        if self.gripper.drive_joint_name in msg.name:
            self.drive_angle = msg.position[
                msg.name.index(self.gripper.drive_joint_name)]

    def link_states_cb(self, msg):
        self.link_states = msg

    def set_gravity(self, model, enable):
        link = '{}::{}'.format(model, self.bunch_link)
        props = self.get_link_properties(link)
        if not props.success:
            rospy.logerr('[%s] %s', rospy.get_name(), props.status_message)
            return
        result = self.set_link_properties(
            link_name=link, com=props.com, gravity_mode=enable,
            mass=props.mass, ixx=props.ixx, ixy=props.ixy, ixz=props.ixz,
            iyy=props.iyy, iyz=props.iyz, izz=props.izz)
        if not result.success:
            rospy.logerr('[%s] %s', rospy.get_name(), result.status_message)

    def update(self, event):
        states = self.link_states
        angle = self.drive_angle
        if states is None or angle is None:
            return
        index = {name: i for i, name in enumerate(states.name)}
        if any(name not in index for name in self.finger_links):
            return
        fingers = [pose_to_coords(states.pose[index[name]])
                   for name in self.finger_links]
        midpoint = np.mean([f.worldpos() for f in fingers], axis=0)
        pad = Coordinates(
            pos=midpoint + fingers[0].worldrot().dot(self.pad_offset),
            rot=fingers[0].worldrot())

        lower, upper = self.gripper.width_range
        fraction = (self.gripper.width(angle) - lower) / (upper - lower)
        was_closed = self.closed
        if fraction < self.close_fraction:
            self.closed = True
        elif fraction > self.open_fraction:
            self.closed = False

        if self.held is None:
            if self.closed and not was_closed:
                self.try_attach(states, index, pad)
            return
        if not self.closed:
            rospy.loginfo('[%s] release %s', rospy.get_name(), self.held)
            self.set_gravity(self.held, True)
            self.held = None
            self.held_offset = None
            return
        msg = ModelState()
        msg.model_name = self.held
        msg.reference_frame = 'world'
        coords_to_pose(pad.copy_worldcoords().transform(self.held_offset),
                       msg.pose)
        self.model_state_pub.publish(msg)

    def try_attach(self, states, index, pad):
        suffix = '::' + self.bunch_link
        best = None
        nearest = None
        for name, i in index.items():
            if not (name.startswith(self.bunch_prefix)
                    and name.endswith(suffix)):
                continue
            bunch = pose_to_coords(states.pose[i])
            local = pad.inverse_transform_vector(bunch.worldpos())
            distance = np.linalg.norm(local)
            if nearest is None or distance < nearest[0]:
                nearest = (distance, name[:-len(suffix)], local)
            if np.all(np.abs(local) <= self.tolerance) and \
               (best is None or distance < best[0]):
                best = (distance, name[:-len(suffix)], bunch, local)
        if best is None:
            if nearest is None:
                rospy.logwarn('[%s] no bunch model named %s* is found',
                              rospy.get_name(), self.bunch_prefix)
            else:
                rospy.loginfo(
                    '[%s] the gripper closed without a stem between the pads '
                    '(nearest: %s at %s from the pad center, tolerance %s)',
                    rospy.get_name(), nearest[1], np.round(nearest[2], 3),
                    self.tolerance)
            return
        _, model, bunch, local = best
        self.set_gravity(model, False)
        self.held = model
        self.held_offset = pad.copy_worldcoords().inverse_transformation() \
            .transform(bunch)
        rospy.loginfo('[%s] grasp %s (stem at %s from the pad center)',
                      rospy.get_name(), model, np.round(local, 3))


def main():
    rospy.init_node('grape_harvest_sim_grasp')
    SimGrasp()
    rospy.spin()


if __name__ == '__main__':
    main()
