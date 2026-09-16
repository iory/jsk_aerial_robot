#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Harvest grape bunches with grape_with_arm and drop them into a crate.

For each bunch in config/grape_with_arm/GrapeHarvestDemo.yaml, the robot opens
the gripper in front of the bunch, flies forward until the stem is between the
finger pads, closes the gripper, backs off, flies over the crate, lowers the arm
and opens the gripper. Finally it returns above the start position and lands.

The positions in the config are in the field frame: the origin is on the ground
right below the robot before takeoff and x is its heading (see the config). The
gripper targets are converted into CoG targets for the flight controller with
the robot model (forward kinematics and the mass centroid at the arm pose).

The same command runs the demo in gazebo and on the real machine; only the
launch file differs, since the field frame is measured from the robot itself.
In gazebo the robot spawns at the origin of the vineyard world, so the field
frame is the world frame there.

Usage
-----
simulation (gazebo)::

    roslaunch gimbalrotor grape_harvest_sim.launch
    rosrun gimbalrotor skrobot_grape_harvest_demo.py

real machine::

    roslaunch gimbalrotor bringup.launch estimate_mode:=0 airframe:=grape_with_arm
    rosrun gimbalrotor skrobot_grape_harvest_demo.py --dry-run   # print the plan only
    rosrun gimbalrotor skrobot_grape_harvest_demo.py --bunches 0

With ``--targets detect`` the bunches are not read from the config: the robot
looks at what grape_detector sees from where it stands and harvests those,
nearest first (see the ``detection`` section of the config)::

    roslaunch grape_detector grape_detection.launch
    rosrun gimbalrotor skrobot_grape_harvest_demo.py --targets detect

If a step fails, the script stops there and leaves the robot as it is (e.g.
hovering) for the operator.
"""

import argparse
import os
import sys

from jsk_recognition_msgs.msg import BoundingBoxArray
import numpy as np
import rospkg
import rospy
from skrobot.coordinates import CascadedCoords
from skrobot.coordinates import Coordinates
from skrobot.coordinates.math import matrix2ypr
from skrobot.coordinates.math import quaternion2matrix
from skrobot.coordinates.math import xyzw2wxyz
from skrobot.coordinates.math import ypr2matrix
from skrobot.models.urdf import RobotModelFromURDF
import tf2_ros
import yaml

from gimbalrotor.skrobot_interface import GimbalrotorROSRobotInterface
from gimbalrotor.skrobot_interface import gripper_center


class DemoError(RuntimeError):
    pass


def parse_args():
    default_config = os.path.join(
        rospkg.RosPack().get_path('gimbalrotor'), 'config', 'grape_with_arm',
        'GrapeHarvestDemo.yaml')
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('--config', default=default_config,
                        help='layout and motion parameters')
    parser.add_argument('--bunches', type=int, nargs='+', default=None,
                        help='indices of the bunches to harvest '
                        '(default: all, in the order of the config)')
    parser.add_argument('--targets', choices=('config', 'detect'),
                        default='config',
                        help='config (default): the bunches of the config. '
                        'detect: the bunches that grape_detector sees from '
                        'where the robot stands, nearest first')
    parser.add_argument('--field-frame', choices=('start', 'world'),
                        default='start',
                        help='start (default, gazebo and the real machine): '
                        'the field frame is measured from the robot standing '
                        'on the ground before takeoff. world: the field frame '
                        'is the world frame of the odometry, with z = 0 on the '
                        'ground, e.g. with motion capture')
    parser.add_argument('--dry-run', action='store_true',
                        help='print the planned targets and exit without '
                        'moving the robot')
    parser.add_argument('-y', '--yes', action='store_true',
                        help='do not ask for confirmation before takeoff')
    return parser.parse_args(rospy.myargv()[1:])


def lowest_collision_z(robot):
    """Return the lowest point of the collision model in the world frame.

    Parameters
    ----------
    robot : skrobot.model.RobotModel
        Robot model placed at the measured pose.

    Returns
    -------
    float
        Height [m].
    """
    lowest = None
    for link in robot.link_list:
        meshes = link.collision_mesh
        if meshes is None:
            continue
        if not isinstance(meshes, list):
            meshes = [meshes]
        for mesh in meshes:
            z = link.worldcoords().transform_vector(mesh.vertices)[:, 2].min()
            if lowest is None or z < lowest:
                lowest = z
    if lowest is None:
        raise DemoError('the robot model has no collision model to find the '
                        'ground')
    return float(lowest)


class GrapeHarvestDemo(object):
    """Grape harvest mission.

    Parameters
    ----------
    ri : GimbalrotorROSRobotInterface
        Robot interface.
    config : dict
        Contents of ``GrapeHarvestDemo.yaml``.
    """

    def __init__(self, ri, config):
        self.ri = ri
        self.field = config['field']
        self.motion = config['motion']
        self.detection = config['detection']
        self.arm_joint_names = [
            name for name in ri.arm_controller['joint_names']
            if name != ri.gripper.drive_joint_name]
        for key in ('grasp_arm_pose', 'release_arm_pose'):
            unknown = [n for n in self.motion[key]
                       if n not in self.arm_joint_names]
            if unknown:
                raise DemoError('{} has joints {} not in {}'.format(
                    key, unknown, self.arm_joint_names))
        # a separate model to plan with, so that the model of the interface
        # keeps the measured state
        self.plan_robot = RobotModelFromURDF(urdf=rospy.get_param(
            '/{}/robot_description'.format(ri.namespace)))
        self.field_coords = None
        self.yaw = None
        self.home = None
        self.waiting_pose = None
        self.reach_pose = None
        self.ik_move_target = None
        self.ik_rot = None
        self.ik_rotation_mask = None
        self.ik_link_list = None
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

    # ------------------------------------------------------------------
    # frames
    # ------------------------------------------------------------------
    def set_field_frame(self, mode):
        """Define the field frame.

        Parameters
        ----------
        mode : str
            ``'start'``: from the robot standing on the ground before takeoff.
            ``'world'``: the world frame of the odometry, whose z = 0 must be
            the ground (gazebo, motion capture).
        """
        ri = self.ri
        if mode == 'world':
            self.field_coords = Coordinates()
            self.yaw = 0.0
            rospy.loginfo('field frame: the world frame of the odometry')
            return
        if mode != 'start':
            raise DemoError('unknown field frame {}'.format(mode))
        if ri.flight_state != ri.flight.ARM_OFF_STATE:
            raise DemoError(
                'the field frame is defined before takeoff, but the flight '
                'state is {}; land the robot first'.format(ri.flight_state))
        if not ri.update_robot_state(wait_until_update=True):
            raise DemoError('joint states are not received')
        root = ri.robot.root_link.copy_worldcoords()
        yaw = matrix2ypr(root.worldrot())[0]
        origin = root.worldpos().copy()
        origin[2] = lowest_collision_z(ri.robot)
        self.field_coords = Coordinates(pos=origin,
                                        rot=ypr2matrix(yaw, 0.0, 0.0))
        self.yaw = yaw
        rospy.loginfo('field frame: origin %s, yaw %.3f rad (world frame)',
                      np.round(origin, 3), yaw)

    def to_field(self, point_world):
        """Return a point of the world frame in the field frame."""
        return self.field_coords.inverse_transformation().transform_vector(
            np.asarray(point_world, dtype=np.float64))

    # ------------------------------------------------------------------
    # detection
    # ------------------------------------------------------------------
    def detect_bunches(self, observe_time=None, report=True):
        """Return the bunches that the detector sees, in the field frame.

        The boxes of ``detection/topic`` are collected for ``observe_time``,
        and longer if fewer than ``min_observations`` messages have come by
        then (a slow detector), up to ``observe_timeout``. They are
        transformed into the field frame and grouped by distance; a group seen
        in at least ``min_observations`` messages is a bunch. Its grasp point
        is the middle of the top of its box (the gripper pinches the stem just
        above the berries), averaged over the group.

        Returns
        -------
        list of dict
            ``{'name': str, 'stem': [x, y, z]}`` of each bunch, nearest first.
        """
        d = self.detection
        region = d['region']
        if observe_time is None:
            observe_time = d['observe_time']
        if report:
            rospy.loginfo('collecting the boxes of %s for %.1f s',
                          d['topic'], observe_time)
        groups = []  # the points of one bunch, seen in several messages
        start = rospy.get_time()
        end = start + observe_time
        timeout = start + max(observe_time, d['observe_timeout'])
        messages = 0
        while not rospy.is_shutdown():
            now = rospy.get_time()
            if now >= timeout or (
                    now >= end and messages >= d['min_observations']):
                break
            try:
                msg = rospy.wait_for_message(
                    d['topic'], BoundingBoxArray,
                    timeout=max(0.1, timeout - now))
            except rospy.ROSException:
                break
            messages += 1
            for box in msg.boxes:
                point = self.box_grasp_point(box, msg.header.frame_id)
                if point is None:
                    continue
                if not all(region[axis][0] <= point[i] <= region[axis][1]
                           for i, axis in enumerate(('x', 'y', 'z'))):
                    continue
                for group in groups:
                    if np.linalg.norm(np.mean(group, axis=0) - point) \
                       <= d['cluster_radius']:
                        group.append(point)
                        break
                else:
                    groups.append([point])
        if messages == 0:
            raise DemoError(
                'no message on {}; is grape_detection.launch running?'.format(
                    d['topic']))
        bunches = []
        for group in groups:
            if len(group) < d['min_observations']:
                continue
            bunches.append(
                {'stem': [float(v) for v in np.mean(group, axis=0)],
                 'seen': len(group),
                 'spread': float(np.linalg.norm(np.std(group, axis=0)))})
        bunches.sort(key=lambda b: np.linalg.norm(b['stem']))
        bunches = bunches[:d['max_targets']]
        if not bunches:
            raise DemoError(
                'no bunch was seen {} times in {} messages inside {}'.format(
                    d['min_observations'], messages, region))
        for i, bunch in enumerate(bunches):
            bunch['name'] = 'detected_{}'.format(i)
            if report:
                rospy.loginfo('bunch %d: stem %s (seen %d of %d messages, '
                              'spread %.3f m)', i, np.round(bunch['stem'], 3),
                              bunch['seen'], messages, bunch['spread'])
        return bunches

    def box_grasp_point(self, box, frame_id):
        """Return the grasp point of a bounding box in the field frame.

        Parameters
        ----------
        box : jsk_recognition_msgs.msg.BoundingBox
            Box around the berries of a bunch.
        frame_id : str
            Frame of the array, used when the box carries none.

        Returns
        -------
        numpy.ndarray or None
            Point on the stem; None if the box is too big to be a bunch or if
            its frame can not be transformed.
        """
        d = self.detection
        size = np.array([box.dimensions.x, box.dimensions.y,
                         box.dimensions.z])
        if size.max() > d['max_box_size'] or size.min() <= 0.0:
            return None
        frame = box.header.frame_id or frame_id
        try:
            transform = self.tf_buffer.lookup_transform(
                d['world_frame'], frame, box.header.stamp,
                rospy.Duration(0.5))
        except tf2_ros.TransformException as e:
            rospy.logwarn_throttle(
                5.0, 'no transform %s -> %s: %s', frame, d['world_frame'], e)
            return None
        t = transform.transform.translation
        q = transform.transform.rotation
        world_to_frame = Coordinates(
            pos=[t.x, t.y, t.z],
            rot=quaternion2matrix(xyzw2wxyz(np.array([q.x, q.y, q.z, q.w]))))
        p = box.pose.position
        r = box.pose.orientation
        frame_to_box = Coordinates(
            pos=[p.x, p.y, p.z],
            rot=quaternion2matrix(xyzw2wxyz(np.array([r.x, r.y, r.z, r.w]))))
        box_coords = world_to_frame.copy_worldcoords().transform(frame_to_box)
        # the stem hangs over the middle of the top of the box
        corners = np.array([[sx, sy, sz] for sx in (-0.5, 0.5)
                            for sy in (-0.5, 0.5) for sz in (-0.5, 0.5)])
        top = max(box_coords.transform_vector(corner * size)[2]
                  for corner in corners)
        center = box_coords.worldpos()
        return self.to_field(
            [center[0], center[1], top + d['stem_offset']])

    def to_world(self, point):
        return self.field_coords.transform_vector(
            np.asarray(point, dtype=np.float64))

    def heading(self):
        return self.field_coords.worldrot()[:, 0]

    def arm_pose(self, key):
        pose = {name: 0.0 for name in self.arm_joint_names}
        pose.update(self.motion[key])
        return pose

    def cog_target(self, tcp_world, arm_pose, place_pose=None):
        """CoG position that puts the gripper center at ``tcp_world``.

        The body is assumed level with the yaw of the field frame, which is
        how the robot hovers.

        Parameters
        ----------
        tcp_world : numpy.ndarray
            Target of the gripper center in the world frame [m].
        arm_pose : dict
            Arm joint angles [rad] while the robot flies there; the CoG
            depends on them.
        place_pose : dict or None
            Arm joint angles whose gripper is placed at ``tcp_world``; the arm
            may still be in ``arm_pose`` when the body gets there, and moves
            to this pose afterwards. ``arm_pose`` if None.

        Returns
        -------
        numpy.ndarray
            CoG target in the world frame [m].
        """
        if place_pose is None:
            place_pose = arm_pose
        yaw_rot = ypr2matrix(self.yaw, 0.0, 0.0)
        root = np.asarray(tcp_world, dtype=np.float64) \
            - yaw_rot.dot(self.body_tcp(place_pose))
        return root + yaw_rot.dot(self.body_centroid(arm_pose))

    def body_tcp(self, arm_pose):
        """Gripper center of an arm pose, in the frame of the root link."""
        robot = self.pose_plan_robot(arm_pose)
        return gripper_center(robot, self.ri.gripper)

    def body_centroid(self, arm_pose):
        """Mass centroid of an arm pose, in the frame of the root link."""
        robot = self.pose_plan_robot(arm_pose)
        return robot.centroid()

    def pose_plan_robot(self, arm_pose):
        """The planning model at an arm pose, with its root at the origin."""
        robot = self.plan_robot
        robot.angle_vector(np.zeros(len(robot.angle_vector())))
        self.set_arm(robot, arm_pose)
        robot.newcoords(Coordinates())
        return robot

    def measured_tcp(self, place_pose=None):
        """Gripper center in the world frame, from the measured state.

        Parameters
        ----------
        place_pose : dict or None
            If given, the gripper center this arm pose would have at the
            measured pose of the body, which is what a flight to a
            ``place_pose`` target is judged by while the arm is elsewhere.
            The measured arm if None.
        """
        if not self.ri.update_robot_state(wait_until_update=True):
            raise DemoError('joint states are not received')
        if place_pose is None:
            return gripper_center(self.ri.robot, self.ri.gripper)
        root = self.ri.robot.root_link.copy_worldcoords()
        return root.transform_vector(self.body_tcp(place_pose))

    # ------------------------------------------------------------------
    # arm
    # ------------------------------------------------------------------
    def set_arm(self, robot, arm_pose):
        """Set the arm joints of a model, leaving the gripper as it is."""
        for name, angle in arm_pose.items():
            getattr(robot, name).joint_angle(angle)

    def setup_arm(self):
        """Prepare the inverse kinematics and the poses of the arm.

        The gripper center is the target of the inverse kinematics, with the
        orientation of the stretched arm. Only the rotation about the axis of
        the gripper frame that is vertical in the body is constrained: the
        gripper keeps pointing along the body, so the stem slides into the
        fingers the same way wherever the arm reaches, and the pitch joint is
        free to reach up or down.
        """
        m = self.motion
        robot = self.pose_plan_robot({})
        gripper = self.ri.gripper
        base = getattr(robot, gripper.finger_link_names[0]).parent_link
        stretched = gripper_center(robot, gripper)
        self.ik_move_target = CascadedCoords(
            parent=base, name='gripper_center',
            pos=base.worldcoords().inverse_transform_vector(stretched))
        self.ik_rot = base.worldrot().copy()
        vertical = int(np.argmax(np.abs(self.ik_rot[2, :])))
        self.ik_rotation_mask = 'xyz'[vertical]
        self.ik_link_list = [getattr(robot, name).child_link
                             for name in self.arm_joint_names]

        unknown = [n for n in m['waiting_arm_pose']
                   if n not in self.arm_joint_names]
        if unknown:
            raise DemoError('waiting_arm_pose has joints {} not in {}'.format(
                unknown, self.arm_joint_names))
        self.waiting_pose = {name: 0.0 for name in self.arm_joint_names}
        self.waiting_pose.update(m['waiting_arm_pose'])
        # the gripper on the stem: a little short of the stretched arm, so
        # that the arm can still reach forward as well as aside
        self.reach_pose = self.arm_ik(
            stretched - np.array([m['arm_reach_margin'], 0.0, 0.0]),
            seed=self.waiting_pose)
        rospy.loginfo(
            'arm: waits at %s (gripper %s), holds the stem at %s (gripper %s) '
            'in the body frame', self.round_pose(self.waiting_pose),
            np.round(self.body_tcp(self.waiting_pose), 3),
            self.round_pose(self.reach_pose),
            np.round(self.body_tcp(self.reach_pose), 3))

    @staticmethod
    def round_pose(arm_pose):
        return {name: round(angle, 3) for name, angle in arm_pose.items()}

    def arm_ik(self, target, seed):
        """Return the arm angles that put the gripper center at a point.

        Parameters
        ----------
        target : array_like
            ``(3,)`` target of the gripper center in the frame of the root
            link, so that no odometry enters the solution [m].
        seed : dict
            Arm angles the inverse kinematics starts from.

        Returns
        -------
        dict
            ``{joint name: angle [rad]}`` of the arm joints.
        """
        robot = self.pose_plan_robot(seed)
        result = robot.inverse_kinematics(
            Coordinates(pos=np.asarray(target, dtype=np.float64),
                        rot=self.ik_rot),
            link_list=self.ik_link_list, move_target=self.ik_move_target,
            rotation_mask=self.ik_rotation_mask, stop=100, thre=0.001)
        if result is False:
            raise DemoError(
                'the arm can not reach {} in the body frame'.format(
                    np.round(target, 3)))
        pose = {name: float(getattr(robot, name).joint_angle())
                for name in self.arm_joint_names}
        limit = self.motion['max_arm_angle']
        if max(abs(a) for a in pose.values()) > limit:
            raise DemoError(
                'the arm would fold to {} to reach {}, further than '
                'max_arm_angle {}'.format(self.round_pose(pose),
                                          np.round(target, 3), limit))
        return pose

    def stem_in_body(self, label, stem_world):
        """Return a stem in the frame of the root link, from the measured pose.

        Where the body actually is does not matter for the arm, only where
        the stem is relative to it.

        Parameters
        ----------
        label : str
            Name of the motion for the log.
        stem_world : numpy.ndarray
            Stem in the world frame [m].

        Returns
        -------
        numpy.ndarray
            ``(3,)`` stem in the root frame [m].
        """
        if not self.ri.update_robot_state(wait_until_update=True):
            raise DemoError('joint states are not received')
        root = self.ri.robot.root_link.copy_worldcoords()
        target = root.inverse_transform_vector(
            np.asarray(stem_world, dtype=np.float64))
        correction = np.linalg.norm(target - self.body_tcp(self.reach_pose))
        rospy.loginfo('[%s] the arm reaches %s in the body frame, %.3f m from '
                      'where the body was placed for', label,
                      np.round(target, 3), correction)
        if correction > self.motion['max_arm_correction']:
            raise DemoError(
                '[{}] the stem is {:.3f} m off where the body was placed for, '
                'more than max_arm_correction'.format(label, correction))
        return target

    def reach_arm_pose(self, label, stem_world):
        """Return the arm angles that put the gripper on a stem."""
        return self.arm_ik(self.stem_in_body(label, stem_world),
                           seed=self.reach_pose)

    def approach_arm_path(self, label, stem_world):
        """Return the arm poses that bring the gripper straight onto a stem.

        Going straight from the folded arm to the stem, the fingers sweep
        across it from the side. The first pose puts the open gripper
        ``pre_reach_distance`` behind the stem, in line with it; the rest
        move it forward along the body to the stem, so the stem only ever
        enters the gripper from the front.

        Parameters
        ----------
        label : str
            Name of the motion for the log.
        stem_world : numpy.ndarray
            Stem in the world frame [m].

        Returns
        -------
        list of dict
            Arm poses, the one behind the stem first and the grasp last.
        """
        m = self.motion
        target = self.stem_in_body(label, stem_world)
        back = np.array([m['pre_reach_distance'], 0.0, 0.0])
        poses = []
        seed = self.reach_pose
        for s in np.linspace(0.0, 1.0, m['reach_steps'] + 1):
            seed = self.arm_ik(target - (1.0 - s) * back, seed=seed)
            poses.append(seed)
        return poses

    def hold_body(self, label, arm_pose):
        """Keep the body where it is while the arm moves to a pose.

        The flight controller holds the CoG, so a reaching arm would pull the
        body back by as much as the CoG moves; the CoG target is moved with
        the arm instead.
        """
        if not self.ri.update_robot_state(wait_until_update=True):
            raise DemoError('joint states are not received')
        root = self.ri.robot.root_link.copy_worldcoords()
        cog = root.worldpos() + ypr2matrix(self.yaw, 0.0, 0.0).dot(
            self.body_centroid(arm_pose))
        m = self.motion
        self.ri.move_to(cog, yaw=self.yaw, wait=False,
                        pos_thresh=m['fine_pos_thresh'],
                        vel_thresh=m['fine_vel_thresh'],
                        yaw_thresh=m['fine_yaw_thresh'])
        rospy.logdebug('[%s] hold the body: CoG -> %s', label,
                       np.round(cog, 3))

    def reach_with_arm(self, label, stem_world):
        """Put the gripper on the stem with the arm, and keep it there.

        Swinging the arm out turns the body (in gazebo the yaw swings by
        0.04 rad and settles in about 3 s), and the servos do not land
        exactly on the angles they are given, so the gripper is watched until
        it is steady, measured from the odometry and the joint angles. A
        steady error is taken out by sending the arm again.

        Parameters
        ----------
        label : str
            Name of the motion for the log.
        stem_world : numpy.ndarray
            Stem in the world frame [m].
        """
        m = self.motion
        stem_world = np.asarray(stem_world, dtype=np.float64)
        tolerance = m['gripper_tolerance']
        aim = stem_world.copy()
        path = self.approach_arm_path(label, aim)
        self.hold_body(label, path[0])
        self.move_arm(label + ' behind the stem', path[0])
        self.hold_body(label, path[-1])
        self.move_arm_path(label + ' in', path[1:], m['reach_time'])
        tries = 1
        for _ in range(m['max_settle_windows']):
            errors = self.gripper_errors(stem_world, m['settle_time'])
            mean = errors.mean(axis=0)
            worst = np.linalg.norm(errors, axis=1).max()
            spread = np.linalg.norm(errors - mean, axis=1).max()
            rospy.loginfo('[%s] gripper to stem over %.1f s: mean %s, max '
                          '%.3f m, spread %.3f m', label, m['settle_time'],
                          np.round(mean, 3), worst, spread)
            if worst <= tolerance:
                return
            # while the body still swings the mean is a transient; wait
            if spread > 0.5 * tolerance:
                continue
            if tries >= m['max_arm_tries']:
                break
            aim = aim - mean
            pose = self.reach_arm_pose(label, aim)
            self.hold_body(label, pose)
            self.move_arm(label + ' reach', pose)
            tries += 1
        raise DemoError(
            '[{}] the arm did not hold the gripper within {} m of the stem '
            '({} arm motions)'.format(label, tolerance, tries))

    def refine_stem(self, label, stem_field):
        """Measure a bunch again from close by, and return its stem.

        Parameters
        ----------
        label : str
            Name of the motion for the log.
        stem_field : array_like
            Expected stem in the field frame [m].

        Returns
        -------
        numpy.ndarray
            Stem in the field frame: the detection nearest to the expected
            one, or the expected one when nothing is seen near it.
        """
        expected = np.asarray(stem_field, dtype=np.float64)
        if self.motion['refine_time'] <= 0.0:
            return expected
        try:
            bunches = self.detect_bunches(
                observe_time=self.motion['refine_time'], report=False)
        except DemoError as e:
            rospy.logwarn('[%s] no detection to refine the stem: %s', label, e)
            return expected
        near = [b for b in bunches
                if np.linalg.norm(np.array(b['stem']) - expected)
                <= self.motion['refine_radius']]
        if not near:
            rospy.logwarn('[%s] no bunch within %.2f m of %s was detected '
                          'from here; keep the target of the plan', label,
                          self.motion['refine_radius'], np.round(expected, 3))
            return expected
        best = min(near, key=lambda b: np.linalg.norm(
            np.array(b['stem']) - expected))
        stem = np.array(best['stem'])
        rospy.loginfo('[%s] the bunch is at %s, %.3f m from the planned stem',
                      label, np.round(stem, 3),
                      np.linalg.norm(stem - expected))
        return stem

    # ------------------------------------------------------------------
    # motions
    # ------------------------------------------------------------------
    def fly_tcp(self, label, tcp_world, arm_pose, precise=False,
                place_pose=None):
        """Fly so that the gripper center reaches ``tcp_world``.

        Parameters
        ----------
        label : str
            Name of the motion for the log.
        tcp_world : numpy.ndarray
            Target of the gripper center in the world frame [m].
        arm_pose : dict
            Arm joint angles [rad] during the motion.
        precise : bool
            If True, the thresholds for a stem or the crate are used, and the
            motion ends only when the gripper error, measured with the
            odometry and the joint angles, stays within ``gripper_tolerance``
            for ``settle_time``. A steady error (e.g. a yaw error or a sagging
            arm) is corrected by moving the CoG target.
        place_pose : dict or None
            Arm pose whose gripper is placed at ``tcp_world`` (see
            ``cog_target``); ``arm_pose`` if None.
        """
        m = self.motion
        if precise:
            pos_thresh = m['fine_pos_thresh']
            vel_thresh = m['fine_vel_thresh']
            yaw_thresh = m['fine_yaw_thresh']
        else:
            pos_thresh = m['coarse_pos_thresh']
            vel_thresh = m['coarse_vel_thresh']
            yaw_thresh = m['coarse_yaw_thresh']
        cog = self.cog_target(tcp_world, arm_pose, place_pose)
        rospy.loginfo('[%s] gripper -> %s (CoG -> %s)', label,
                      np.round(tcp_world, 3), np.round(cog, 3))
        if not self.ri.move_to(cog, yaw=self.yaw, pos_thresh=pos_thresh,
                               vel_thresh=vel_thresh, yaw_thresh=yaw_thresh,
                               timeout=m['flight_timeout']):
            raise DemoError('[{}] the flight did not converge'.format(label))
        if not precise:
            error = self.measured_tcp(place_pose) - tcp_world
            rospy.loginfo('[%s] gripper error %s (%.3f m)', label,
                          np.round(error, 3), np.linalg.norm(error))
            return

        # the convergence check passes at a turning point of an overshoot,
        # so watch the gripper for a while before grasping or releasing
        tolerance = m['gripper_tolerance']
        corrections = 0
        for _ in range(m['max_settle_windows']):
            errors = self.gripper_errors(tcp_world, m['settle_time'],
                                         place_pose)
            mean = errors.mean(axis=0)
            worst = np.linalg.norm(errors, axis=1).max()
            spread = np.linalg.norm(errors - mean, axis=1).max()
            rospy.loginfo('[%s] gripper error over %.1f s: mean %s, max '
                          '%.3f m, spread %.3f m', label, m['settle_time'],
                          np.round(mean, 3), worst, spread)
            if worst <= tolerance:
                return
            # correct only a steady offset; while the robot still moves, the
            # mean is a transient and correcting it chases the motion
            if spread <= 0.5 * tolerance \
               and np.linalg.norm(mean) > 0.5 * tolerance \
               and corrections < m['max_corrections']:
                cog = cog - mean
                corrections += 1
                rospy.loginfo('[%s] correct the CoG target to %s', label,
                              np.round(cog, 3))
                if not self.ri.move_to(cog, yaw=self.yaw,
                                       pos_thresh=pos_thresh,
                                       vel_thresh=vel_thresh,
                                       yaw_thresh=yaw_thresh,
                                       timeout=m['flight_timeout']):
                    raise DemoError(
                        '[{}] the flight did not converge'.format(label))
        raise DemoError(
            '[{}] the gripper did not settle within {} m ({} windows of {} '
            's, {} corrections)'.format(label, tolerance,
                                        m['max_settle_windows'],
                                        m['settle_time'], corrections))

    def gripper_errors(self, tcp_world, duration, place_pose=None):
        """Sample the gripper error for ``duration`` [s] at 10 Hz."""
        errors = []
        end = rospy.get_time() + duration
        while not rospy.is_shutdown():
            errors.append(self.measured_tcp(place_pose) - tcp_world)
            if rospy.get_time() >= end:
                break
            rospy.sleep(0.1)
        return np.array(errors)

    def move_arm(self, label, arm_pose):
        ri = self.ri
        if not ri.update_robot_state(wait_until_update=True):
            raise DemoError('joint states are not received')
        # joints not in arm_pose, i.e. the gripper, keep the measured angle
        for name, angle in arm_pose.items():
            getattr(ri.robot, name).joint_angle(angle)
        rospy.loginfo('[%s] arm -> %s', label,
                      {name: round(angle, 3)
                       for name, angle in arm_pose.items()})
        ri.angle_vector(ri.robot.angle_vector(), self.motion['arm_time'])
        ri.wait_interpolation()
        result = ri.controller_table['arm_controller'][0].get_result()
        if result is None:
            raise DemoError('[{}] no result of the arm motion'.format(label))
        if result.error_code != 0:
            # a joint out of the goal tolerance of arm_controller; the precise
            # flight motions correct the gripper position with the measured
            # joint angles, and stop the demo if they can not
            rospy.logwarn('[%s] arm motion: %s', label, result.error_string)

    def move_arm_path(self, label, arm_poses, duration):
        """Move the arm through poses, as one trajectory of ``duration`` [s]."""
        ri = self.ri
        if not ri.update_robot_state(wait_until_update=True):
            raise DemoError('joint states are not received')
        avs = []
        for pose in arm_poses:
            for name, angle in pose.items():
                getattr(ri.robot, name).joint_angle(angle)
            avs.append(ri.robot.angle_vector().copy())
        rospy.loginfo('[%s] arm through %d poses to %s in %.1f s', label,
                      len(avs), self.round_pose(arm_poses[-1]), duration)
        ri.angle_vector_sequence(avs, [duration / len(avs)] * len(avs))
        ri.wait_interpolation()
        result = ri.controller_table['arm_controller'][0].get_result()
        if result is None:
            raise DemoError('[{}] no result of the arm motion'.format(label))
        if result.error_code != 0:
            rospy.logwarn('[%s] arm motion: %s', label, result.error_string)

    def measured_gripper_width(self):
        if not self.ri.update_robot_state(wait_until_update=True):
            raise DemoError('joint states are not received')
        return self.ri.gripper_width()

    # arm_controller moves the gripper together with the arm joints, so the
    # result of a gripper motion also fails when an arm joint is out of its
    # goal tolerance; the gripper is checked with the measured width instead
    def open_gripper(self, label):
        rospy.loginfo('[%s] open the gripper', label)
        self.ri.stop_grasp(time=self.motion['gripper_time'])
        width = self.measured_gripper_width()
        target = self.ri.gripper.width_range[1]
        rospy.loginfo('[%s] gripper width %.4f m', label, width)
        if target - width > self.motion['gripper_width_tolerance']:
            raise DemoError('[{}] the gripper did not open: width {:.4f} m, '
                            'target {:.4f} m'.format(label, width, target))

    def close_gripper(self, label):
        rospy.loginfo('[%s] close the gripper', label)
        self.ri.start_grasp(time=self.motion['gripper_time'])
        rospy.sleep(self.motion['hold_time'])
        # on the real machine a stem stops the fingers before fully closed
        rospy.loginfo('[%s] gripper width %.4f m (fully closed: %.4f m)',
                      label, self.measured_gripper_width(),
                      self.ri.gripper.width_range[0])

    # ------------------------------------------------------------------
    # plan
    # ------------------------------------------------------------------
    def bunch_waypoints(self, bunch):
        """Gripper waypoints in the world frame for one bunch.

        The waypoints near the bunch are where the gripper of ``reach_pose``
        would be: the robot flies with the arm folded and the arm reaches out
        only at the stem, so the flight never has to put the gripper on it.
        """
        m = self.motion
        stem = self.to_world(bunch['stem'])
        heading = self.heading()
        up = np.array([0.0, 0.0, 1.0])
        crate = self.field['crates'][self.field['drop_crate']]
        crate_bottom = self.to_world(crate['position'])
        release = crate_bottom + up * m['release_height']
        return {
            'pregrasp': stem - m['approach_distance'] * heading,
            'grasp': stem,
            'retreat': stem - m['retreat_distance'] * heading
            + m['lift_height'] * up,
            'above_crate': release + m['crate_clearance'] * up,
            'release': release,
        }

    def print_plan(self, bunches):
        release = self.arm_pose('release_arm_pose')
        for index, bunch in bunches:
            waypoints = self.bunch_waypoints(bunch)
            rospy.loginfo('bunch %d (%s): stem %s', index, bunch['name'],
                          np.round(self.to_world(bunch['stem']), 3))
            for name, tcp in waypoints.items():
                if name in ('above_crate', 'release'):
                    cog = self.cog_target(tcp, release)
                else:
                    cog = self.cog_target(tcp, self.waiting_pose,
                                          self.reach_pose)
                rospy.loginfo('  %-12s gripper %s CoG %s', name,
                              np.round(tcp, 3), np.round(cog, 3))

    def harvest(self, index, bunch):
        m = self.motion
        label = 'bunch {} {}'.format(index, bunch['name'])
        waypoints = self.bunch_waypoints(bunch)
        release_pose = self.arm_pose('release_arm_pose')
        folded = dict(arm_pose=self.waiting_pose, place_pose=self.reach_pose)

        self.move_arm(label, self.waiting_pose)
        self.open_gripper(label)
        self.fly_tcp(label + ' pregrasp', waypoints['pregrasp'], **folded)
        # line up with the bunch, so that the arm reaches straight out
        self.fly_tcp(label + ' grasp', waypoints['grasp'], precise=True,
                     **folded)
        # from here the body stays where it is and the arm does the rest
        stem = self.refine_stem(label, bunch['stem'])
        self.reach_with_arm(label, self.to_world(stem))
        self.close_gripper(label)
        # pull the bunch straight out before folding the arm aside
        pull = self.approach_arm_path(label, self.measured_tcp())
        self.hold_body(label, pull[0])
        self.move_arm_path(label + ' out', pull[-2::-1], m['reach_time'])
        self.hold_body(label, self.waiting_pose)
        self.move_arm(label + ' fold', self.waiting_pose)
        self.fly_tcp(label + ' retreat', waypoints['retreat'], **folded)
        self.fly_tcp(label + ' above crate', waypoints['above_crate'],
                     self.waiting_pose, place_pose=release_pose)
        self.hold_body(label, release_pose)
        self.move_arm(label, release_pose)
        self.fly_tcp(label + ' release', waypoints['release'], release_pose,
                     precise=True)
        self.open_gripper(label)
        rospy.sleep(m['hold_time'])
        self.fly_tcp(label + ' above crate', waypoints['above_crate'],
                     release_pose)
        self.hold_body(label, self.waiting_pose)
        self.move_arm(label, self.waiting_pose)

    def run(self, bunches):
        ri = self.ri
        m = self.motion
        if not ri.start():
            raise DemoError('failed to start motors')
        if not ri.takeoff():
            raise DemoError('failed to take off')
        self.home = ri.cog_position()
        rospy.loginfo('hovering at %s', np.round(self.home, 3))
        for index, bunch in bunches:
            self.harvest(index, bunch)
        rospy.loginfo('return home')
        if not ri.move_to(self.home, yaw=self.yaw,
                          pos_thresh=m['coarse_pos_thresh'],
                          vel_thresh=m['coarse_vel_thresh'],
                          yaw_thresh=m['coarse_yaw_thresh'],
                          timeout=m['flight_timeout']):
            raise DemoError('failed to return to {}'.format(
                np.round(self.home, 3)))
        if not ri.land():
            raise DemoError('failed to land')


def main():
    args = parse_args()
    rospy.init_node('skrobot_grape_harvest_demo', disable_signals=True)
    with open(args.config) as f:
        config = yaml.safe_load(f)
    ri = GimbalrotorROSRobotInterface()
    try:
        demo = GrapeHarvestDemo(ri, config)
        demo.set_field_frame(args.field_frame)
        demo.setup_arm()
        if args.targets == 'detect':
            all_bunches = demo.detect_bunches()
        else:
            all_bunches = config['field']['bunches']
        indices = args.bunches if args.bunches is not None \
            else list(range(len(all_bunches)))
        for index in indices:
            if not 0 <= index < len(all_bunches):
                raise DemoError('bunch index {} is out of [0, {})'.format(
                    index, len(all_bunches)))
        bunches = [(index, all_bunches[index]) for index in indices]
        demo.print_plan(bunches)
        if args.dry_run:
            return
        if not args.yes:
            answer = input('start motors and take off? [y/N] ')
            if answer.strip().lower() != 'y':
                rospy.loginfo('canceled')
                return
        demo.run(bunches)
    except DemoError as e:
        rospy.logerr('%s. stop the demo.', e)
        sys.exit(1)
    rospy.loginfo('finished')


if __name__ == '__main__':
    main()
