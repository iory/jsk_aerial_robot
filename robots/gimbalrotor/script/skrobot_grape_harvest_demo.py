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
    def detect_bunches(self):
        """Return the bunches that the detector sees, in the field frame.

        The boxes of ``detection/topic`` are collected for ``observe_time``,
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
        rospy.loginfo('collecting the boxes of %s for %.1f s',
                      d['topic'], d['observe_time'])
        groups = []  # the points of one bunch, seen in several messages
        end = rospy.get_time() + d['observe_time']
        messages = 0
        while rospy.get_time() < end and not rospy.is_shutdown():
            try:
                msg = rospy.wait_for_message(
                    d['topic'], BoundingBoxArray,
                    timeout=max(0.1, end - rospy.get_time()))
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
            rospy.loginfo('bunch %d: stem %s (seen %d of %d messages, spread '
                          '%.3f m)', i, np.round(bunch['stem'], 3),
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

    def cog_target(self, tcp_world, arm_pose):
        """CoG position that brings the gripper center to ``tcp_world``.

        The body is assumed level with the yaw of the field frame, which is
        how the robot hovers.

        Parameters
        ----------
        tcp_world : numpy.ndarray
            Target of the gripper center in the world frame [m].
        arm_pose : dict
            Arm joint angles [rad].

        Returns
        -------
        numpy.ndarray
            CoG target in the world frame [m].
        """
        robot = self.plan_robot
        robot.angle_vector(np.zeros(len(robot.angle_vector())))
        for name, angle in arm_pose.items():
            getattr(robot, name).joint_angle(angle)
        robot.newcoords(Coordinates(rot=ypr2matrix(self.yaw, 0.0, 0.0)))
        return tcp_world - gripper_center(robot, self.ri.gripper) \
            + robot.centroid()

    def measured_tcp(self):
        """Gripper center from the measured pose and joint angles."""
        if not self.ri.update_robot_state(wait_until_update=True):
            raise DemoError('joint states are not received')
        return gripper_center(self.ri.robot, self.ri.gripper)

    # ------------------------------------------------------------------
    # motions
    # ------------------------------------------------------------------
    def fly_tcp(self, label, tcp_world, arm_pose, precise=False):
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
        cog = self.cog_target(tcp_world, arm_pose)
        rospy.loginfo('[%s] gripper -> %s (CoG -> %s)', label,
                      np.round(tcp_world, 3), np.round(cog, 3))
        if not self.ri.move_to(cog, yaw=self.yaw, pos_thresh=pos_thresh,
                               vel_thresh=vel_thresh, yaw_thresh=yaw_thresh,
                               timeout=m['flight_timeout']):
            raise DemoError('[{}] the flight did not converge'.format(label))
        if not precise:
            error = self.measured_tcp() - tcp_world
            rospy.loginfo('[%s] gripper error %s (%.3f m)', label,
                          np.round(error, 3), np.linalg.norm(error))
            return

        # the convergence check passes at a turning point of an overshoot,
        # so watch the gripper for a while before grasping or releasing
        tolerance = m['gripper_tolerance']
        corrections = 0
        for _ in range(m['max_settle_windows']):
            errors = self.gripper_errors(tcp_world, m['settle_time'])
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

    def gripper_errors(self, tcp_world, duration):
        """Sample the gripper error for ``duration`` [s] at 10 Hz."""
        errors = []
        end = rospy.get_time() + duration
        while not rospy.is_shutdown():
            errors.append(self.measured_tcp() - tcp_world)
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
        rospy.loginfo('[%s] arm -> %s', label, arm_pose)
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
        """Gripper waypoints in the world frame for one bunch."""
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
        grasp = self.arm_pose('grasp_arm_pose')
        release = self.arm_pose('release_arm_pose')
        for index, bunch in bunches:
            waypoints = self.bunch_waypoints(bunch)
            rospy.loginfo('bunch %d (%s):', index, bunch['name'])
            for name, tcp in waypoints.items():
                pose = release if name == 'release' else grasp
                rospy.loginfo('  %-12s gripper %s CoG %s', name,
                              np.round(tcp, 3),
                              np.round(self.cog_target(tcp, pose), 3))

    def harvest(self, index, bunch):
        m = self.motion
        label = 'bunch {} {}'.format(index, bunch['name'])
        waypoints = self.bunch_waypoints(bunch)
        grasp_pose = self.arm_pose('grasp_arm_pose')
        release_pose = self.arm_pose('release_arm_pose')

        self.move_arm(label, grasp_pose)
        self.open_gripper(label)
        self.fly_tcp(label + ' pregrasp', waypoints['pregrasp'], grasp_pose)
        # line up in front of the bunch, so that the approach is straight
        self.fly_tcp(label + ' pregrasp', waypoints['pregrasp'], grasp_pose,
                     precise=True)
        self.fly_tcp(label + ' grasp', waypoints['grasp'], grasp_pose,
                     precise=True)
        self.close_gripper(label)
        self.fly_tcp(label + ' retreat', waypoints['retreat'], grasp_pose)
        self.fly_tcp(label + ' above crate', waypoints['above_crate'],
                     grasp_pose)
        self.move_arm(label, release_pose)
        self.fly_tcp(label + ' release', waypoints['release'], release_pose,
                     precise=True)
        self.open_gripper(label)
        rospy.sleep(m['hold_time'])
        self.fly_tcp(label + ' above crate', waypoints['above_crate'],
                     release_pose)
        self.move_arm(label, grasp_pose)

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
