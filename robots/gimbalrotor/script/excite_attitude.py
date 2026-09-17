#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Shake the roll or pitch target of a hovering gimbalrotor by a small multisine, for identification.

The flight data of a hover alone cannot tell the attitude plant from the controller, because every
command reacts to the attitude. A known excitation added to the target, which does not depend on the
attitude, can: the response of the attitude, the rotor speeds and the gimbals to it gives the frequency
response of the plant (see script/analyze_excitation.py).

The target goes through uav/nav (roll_nav_mode / pitch_nav_mode = POS_MODE), which the navigator takes only
in HOVER. The excitation is a sum of sines between --fmin and --fmax with Schroeder phases (a low crest
factor), faded in and out over 2 s, so the target starts and ends at 0.

Safety:
- it starts only in HOVER and stops as soon as the robot leaves HOVER
- it sends the target 0 and stops if |roll| or |pitch| exceeds --max-tilt, or on Ctrl-C
- the navigator resets the roll / pitch target to 0 when the motors stop, so a target left by a landing
  during the excitation does not stay for the next flight

usage::

    rosrun gimbalrotor excite_attitude.py --axis pitch --amplitude 3 --duration 60

Record the flight (bringup.launch rosbag:=true): uav/nav carries the excitation.
"""

import argparse

from aerial_robot_msgs.msg import FlightNav
from nav_msgs.msg import Odometry
import numpy as np
import rospy
from std_msgs.msg import UInt8
from tf.transformations import euler_from_quaternion

HOVER_STATE = 5
FADE = 2.0  # [s]


def multisine(t, freqs, amplitude):
    """Sum of sines with Schroeder phases, scaled so that its peak over one period is ``amplitude``.

    Parameters
    ----------
    t : numpy.ndarray
        Times [s].
    freqs : numpy.ndarray
        Frequencies [Hz].
    amplitude : float
        Peak value.

    Returns
    -------
    numpy.ndarray
        The signal at ``t``.
    """
    n = len(freqs)
    phases = -np.pi * np.arange(1, n + 1) * np.arange(n) / n
    x = np.sin(2 * np.pi * np.outer(t, freqs) + phases).sum(1)
    return x


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('--robot-ns', default='gimbalrotor')
    parser.add_argument('--axis', choices=('roll', 'pitch'), default='pitch')
    parser.add_argument('--amplitude', type=float, default=3.0, help='[deg] peak of the target')
    parser.add_argument('--duration', type=float, default=60.0, help='[s]')
    parser.add_argument('--fmin', type=float, default=0.3, help='[Hz]')
    parser.add_argument('--fmax', type=float, default=3.0, help='[Hz]')
    parser.add_argument('--period', type=float, default=10.0,
                        help='[s] period of the multisine; its frequencies are multiples of 1 / period')
    parser.add_argument('--rate', type=float, default=50.0, help='[Hz]')
    parser.add_argument('--max-tilt', type=float, default=15.0, help='[deg] stop beyond this roll / pitch')
    args = parser.parse_args(rospy.myargv()[1:])
    if not 0 < args.amplitude <= 10.0:
        parser.error('--amplitude must be in (0, 10] deg')

    rospy.init_node('excite_attitude')
    ns = '/' + args.robot_ns
    status = {'state': None, 'rpy': None}
    rospy.Subscriber(ns + '/flight_state', UInt8, lambda m: status.__setitem__('state', m.data))

    def odom_cb(msg):
        o = msg.pose.pose.orientation
        status['rpy'] = euler_from_quaternion([o.x, o.y, o.z, o.w])

    rospy.Subscriber(ns + '/uav/cog/odom', Odometry, odom_cb)
    pub = rospy.Publisher(ns + '/uav/nav', FlightNav, queue_size=10)

    freqs = np.arange(1, int(args.fmax * args.period) + 1) / args.period
    freqs = freqs[freqs >= args.fmin]
    # scale to the peak over one period
    probe = multisine(np.arange(0, args.period, 0.001), freqs, 1.0)
    scale = np.radians(args.amplitude) / np.abs(probe).max()
    rospy.loginfo('[excite attitude] %s, %d sines %.2f-%.2f Hz, peak %.1f deg, %.0f s',
                  args.axis, len(freqs), freqs[0], freqs[-1], args.amplitude, args.duration)

    def send(value):
        msg = FlightNav()
        msg.header.stamp = rospy.Time.now()
        msg.control_frame = FlightNav.WORLD_FRAME
        msg.target = FlightNav.COG
        msg.pos_xy_nav_mode = FlightNav.NO_NAVIGATION
        msg.yaw_nav_mode = FlightNav.NO_NAVIGATION
        msg.pos_z_nav_mode = FlightNav.NO_NAVIGATION
        if args.axis == 'roll':
            msg.roll_nav_mode = FlightNav.POS_MODE
            msg.target_roll = value
        else:
            msg.pitch_nav_mode = FlightNav.POS_MODE
            msg.target_pitch = value
        pub.publish(msg)

    deadline = rospy.get_time() + 5.0
    while not rospy.is_shutdown() and (status['state'] is None or status['rpy'] is None):
        if rospy.get_time() > deadline:
            rospy.logerr('[excite attitude] no flight_state or uav/cog/odom')
            return
        rospy.sleep(0.1)
    if status['state'] != HOVER_STATE:
        rospy.logerr('[excite attitude] the robot is not in HOVER (flight_state %s): not started', status['state'])
        return

    rate = rospy.Rate(args.rate)
    start = rospy.get_time()
    reason = 'done'
    try:
        while not rospy.is_shutdown():
            t = rospy.get_time() - start
            if t > args.duration:
                break
            if status['state'] != HOVER_STATE:
                reason = 'the robot left HOVER (flight_state %s)' % status['state']
                break
            roll, pitch = status['rpy'][0], status['rpy'][1]
            if max(abs(roll), abs(pitch)) > np.radians(args.max_tilt):
                reason = 'tilt %.1f / %.1f deg beyond %.1f deg' % (np.degrees(roll), np.degrees(pitch), args.max_tilt)
                break
            fade = min(1.0, t / FADE, (args.duration - t) / FADE)
            send(fade * scale * multisine(np.array([t]), freqs, 1.0)[0])
            rate.sleep()
    finally:
        # the navigator takes this only in HOVER; out of HOVER its reset at the motor stop clears the target
        for _ in range(5):
            send(0.0)
            rospy.sleep(0.02)
        rospy.loginfo('[excite attitude] stopped: %s; target %s back to 0', reason, args.axis)


if __name__ == '__main__':
    main()
