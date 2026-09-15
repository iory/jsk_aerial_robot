#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Take off, fly to a target in the world frame, move the arm, return and land.

Usage
-----
rosrun gimbalrotor skrobot_go_and_return.py --target 1.0 0.0 1.2
rosrun gimbalrotor skrobot_go_and_return.py --target 1.0 0.0 1.2 \
    --arm-angles 0.0 -0.5 -0.5 0.4 0.0

The world frame is defined by the state estimator; with LIO, its origin is the
pose where the estimator started. If a step fails, the script stops there and
leaves the robot as it is (e.g. hovering) for the operator.
"""

import argparse
import sys

import numpy as np
import rospy

from gimbalrotor.skrobot_interface import GimbalrotorROSRobotInterface


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('--target', type=float, nargs=3, required=True,
                        metavar=('X', 'Y', 'Z'),
                        help='target CoG position in the world frame [m]')
    parser.add_argument('--yaw', type=float, default=None,
                        help='target yaw [rad] (default: keep)')
    parser.add_argument('--arm-angles', type=float, nargs='+', default=None,
                        help='arm joint angles [rad] at the target, in the '
                        'order of arm_controller/joints')
    parser.add_argument('--arm-time', type=float, default=3.0,
                        help='duration of the arm motion [s]')
    parser.add_argument('--hold-time', type=float, default=2.0,
                        help='hold time at the target [s]')
    parser.add_argument('--timeout', type=float, default=30.0,
                        help='timeout of each flight motion [s]')
    parser.add_argument('-y', '--yes', action='store_true',
                        help='do not ask for confirmation before takeoff')
    return parser.parse_args(rospy.myargv()[1:])


def fail(message):
    rospy.logerr(message + ' stop the script.')
    sys.exit(1)


def move_arm(ri, joint_names, angles, duration):
    robot = ri.robot
    ri.update_robot_state(wait_until_update=True)
    for name, angle in zip(joint_names, angles):
        getattr(robot, name).joint_angle(angle)
    ri.angle_vector(robot.angle_vector(), duration)
    ri.wait_interpolation()
    result = ri.controller_table['arm_controller'][0].get_result()
    if result is None or result.error_code != 0:
        fail('arm motion failed: {}'.format(
            None if result is None else result.error_string))


def main():
    args = parse_args()
    rospy.init_node('skrobot_go_and_return', disable_signals=True)
    ri = GimbalrotorROSRobotInterface()
    arm_joint_names = ri.arm_controller['joint_names']
    if args.arm_angles is not None \
       and len(args.arm_angles) != len(arm_joint_names):
        fail('--arm-angles needs {} values for {}.'.format(
            len(arm_joint_names), arm_joint_names))

    rospy.loginfo('current CoG position: %s, target: %s',
                  ri.cog_position(), args.target)
    if not args.yes:
        answer = input('start motors and take off? [y/N] ')
        if answer.strip().lower() != 'y':
            rospy.loginfo('canceled')
            return

    if not ri.start():
        fail('failed to start motors.')
    if not ri.takeoff():
        fail('failed to take off.')
    home = ri.cog_position()
    home_yaw = ri.yaw()
    rospy.loginfo('hovering at %s', home)

    if not ri.move_to(args.target, yaw=args.yaw, timeout=args.timeout):
        fail('failed to reach the target.')
    rospy.sleep(args.hold_time)

    if args.arm_angles is not None:
        ri.update_robot_state(wait_until_update=True)
        initial_arm_angles = [getattr(ri.robot, name).joint_angle()
                              for name in arm_joint_names]
        move_arm(ri, arm_joint_names, args.arm_angles, args.arm_time)
        rospy.sleep(args.hold_time)
        move_arm(ri, arm_joint_names, initial_arm_angles, args.arm_time)

    if not ri.move_to(home, yaw=home_yaw, timeout=args.timeout):
        fail('failed to return to {}.'.format(np.round(home, 3)))
    if not ri.land():
        fail('failed to land.')
    rospy.loginfo('finished')


if __name__ == '__main__':
    main()
