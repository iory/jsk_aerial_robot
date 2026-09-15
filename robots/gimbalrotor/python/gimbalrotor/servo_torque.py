#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Servo torque on/off for the joints of the arm controller of gimbalrotor.

Depends only on ROS messages (no scikit-robot), so that it runs from the
system python, e.g. in ``script/arm_torque_server.py``.
"""

import threading

import actionlib
import control_msgs.msg
import rospy
from sensor_msgs.msg import JointState
from spinal.msg import ServoTorqueCmd
from spinal.msg import ServoTorqueStates
import trajectory_msgs.msg


def load_servo_ids(namespace):
    """Return ``{joint name: servo id}`` of every servo in servo_bridge.

    Parameters
    ----------
    namespace : str
        Robot namespace.

    Returns
    -------
    dict
        Servo id (the index of spinal) of each servo joint.
    """
    param = '/{}/servo_controller'.format(namespace)
    if not rospy.has_param(param):
        raise RuntimeError('rosparam {} is not found'.format(param))
    servo_ids = {}
    for group in rospy.get_param(param).values():
        if not isinstance(group, dict):
            continue
        for key, servo in group.items():
            if key.startswith('controller') and isinstance(servo, dict) \
               and 'name' in servo and 'id' in servo:
                servo_ids[servo['name']] = int(servo['id'])
    return servo_ids


class ArmServoTorque(object):
    """Switch the servo torque of the joints of ``arm_controller``.

    spinal turns a servo torque back on with any position command, so
    ``arm_hardware_interface`` leaves joints turned off through
    ``servo/torque_enable`` out of its commands. Turning a joint on first moves
    the target of ``arm_controller`` to the measured positions, so that a
    joint moved by hand while it was off does not jump back.

    Parameters
    ----------
    namespace : str
        Robot namespace.
    arm_controller_ns : str
        Namespace of the arm controller relative to ``namespace``.
    """

    def __init__(self, namespace='gimbalrotor',
                 arm_controller_ns='arm/arm_controller'):
        self.namespace = namespace
        joints_param = '/{}/{}/joints'.format(namespace, arm_controller_ns)
        if not rospy.has_param(joints_param):
            raise RuntimeError('rosparam {} is not found'.format(joints_param))
        self.joint_names = list(rospy.get_param(joints_param))
        self.servo_ids = load_servo_ids(namespace)
        missing = [n for n in self.joint_names if n not in self.servo_ids]
        if missing:
            raise RuntimeError(
                'servo ids of {} are not found in /{}/servo_controller'.format(
                    missing, namespace))
        self._lock = threading.Lock()
        self._torque_states = None
        self._positions = {}
        self._torque_pub = rospy.Publisher(
            '/{}/servo/torque_enable'.format(namespace), ServoTorqueCmd,
            queue_size=10)
        self._torque_sub = rospy.Subscriber(
            '/{}/servo/torque_states'.format(namespace), ServoTorqueStates,
            self._torque_states_cb, queue_size=1)
        self._joint_sub = rospy.Subscriber(
            '/{}/joint_states'.format(namespace), JointState,
            self._joint_states_cb, queue_size=1)
        self._action = actionlib.SimpleActionClient(
            '/{}/{}/follow_joint_trajectory'.format(
                namespace, arm_controller_ns),
            control_msgs.msg.FollowJointTrajectoryAction)

    def _torque_states_cb(self, msg):
        self._torque_states = list(msg.torque_enable)

    def _joint_states_cb(self, msg):
        positions = dict(self._positions)
        positions.update(zip(msg.name, msg.position))
        self._positions = positions

    def _check_joint_names(self, joint_names):
        if joint_names is None:
            return list(self.joint_names)
        if isinstance(joint_names, str):
            raise TypeError(
                'joint_names must be a list of joint names, but given a str '
                '{!r}'.format(joint_names))
        joint_names = list(joint_names)
        unknown = [n for n in joint_names if n not in self.joint_names]
        if unknown:
            raise ValueError(
                '{} are not joints of arm_controller {}'.format(
                    unknown, self.joint_names))
        return joint_names

    @staticmethod
    def _check_available():
        if rospy.get_param('/use_sim_time', False):
            raise RuntimeError(
                'servo torque can not be switched in simulation: '
                'the gazebo joints have no servo to turn off')

    def states(self):
        """Return whether the torque of each arm joint is on.

        Returns
        -------
        dict
            ``{joint name: bool}`` from ``<namespace>/servo/torque_states``.
        """
        states = self._torque_states
        if states is None:
            raise RuntimeError(
                '/{}/servo/torque_states is not received'.format(
                    self.namespace))
        return {name: bool(states[self.servo_ids[name]])
                for name in self.joint_names
                if self.servo_ids[name] < len(states)}

    def off(self, joint_names=None, timeout=2.0):
        """Turn off the servo torque of arm joints.

        The joints go limp: an arm joint holding against gravity falls.

        Parameters
        ----------
        joint_names : list of str or None
            Joints of ``arm_controller``. If None, all of them.
        timeout : float
            Time to wait for ``servo/torque_states`` to report off [s].

        Returns
        -------
        bool
            True if every joint is reported off within ``timeout``.
        """
        self._check_available()
        joint_names = self._check_joint_names(joint_names)
        with self._lock:
            return self._switch(joint_names, False, timeout)

    def on(self, joint_names=None, timeout=2.0):
        """Turn on the servo torque of arm joints, holding their current pose.

        Parameters
        ----------
        joint_names : list of str or None
            Joints of ``arm_controller``. If None, all of them.
        timeout : float
            Time to wait for ``servo/torque_states`` to report on [s].

        Returns
        -------
        bool
            True if every joint is reported on within ``timeout``.
        """
        self._check_available()
        joint_names = self._check_joint_names(joint_names)
        with self._lock:
            self._hold_current_positions()
            return self._switch(joint_names, True, timeout)

    def _switch(self, joint_names, enable, timeout):
        msg = ServoTorqueCmd()
        msg.index = [self.servo_ids[name] for name in joint_names]
        msg.torque_enable = [1 if enable else 0] * len(joint_names)
        start = rospy.get_time()
        last_sent = None
        while not rospy.is_shutdown():
            now = rospy.get_time()
            states = self._torque_states
            if states is not None and all(
                    self.servo_ids[name] < len(states)
                    and bool(states[self.servo_ids[name]]) == enable
                    for name in joint_names):
                return True
            if now - start > timeout:
                rospy.logwarn(
                    '[%s] torque %s of %s is not confirmed by '
                    '/%s/servo/torque_states in %.1f s', rospy.get_name(),
                    'on' if enable else 'off', joint_names, self.namespace,
                    timeout)
                return False
            # spinal may miss a command; repeat it until the state follows
            if last_sent is None or now - last_sent > 0.5:
                self._torque_pub.publish(msg)
                last_sent = now
            rospy.sleep(0.02)
        return False

    def _hold_current_positions(self, duration=0.1, timeout=5.0):
        """Replace the arm_controller target by the measured positions."""
        start = rospy.get_time()
        while not rospy.is_shutdown() and not all(
                n in self._positions for n in self.joint_names):
            if rospy.get_time() - start > timeout:
                missing = [n for n in self.joint_names
                           if n not in self._positions]
                raise RuntimeError(
                    'joint states of {} are not received from '
                    '/{}/joint_states'.format(missing, self.namespace))
            rospy.sleep(0.02)
        if not self._action.wait_for_server(rospy.Duration(timeout)):
            raise RuntimeError('arm_controller action server is not available')
        positions = self._positions
        goal = control_msgs.msg.FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = list(self.joint_names)
        goal.trajectory.points = [trajectory_msgs.msg.JointTrajectoryPoint(
            positions=[positions[n] for n in self.joint_names],
            time_from_start=rospy.Duration(duration))]
        self._action.send_goal(goal)
        # the result is not checked: a joint that is still off may drift out
        # of the goal tolerance, and the controller holds its target anyway
        self._action.wait_for_result(rospy.Duration(duration + 2.0))
