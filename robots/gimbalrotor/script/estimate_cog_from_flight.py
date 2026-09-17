#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Estimate the real centroid of a gimbalrotor from the motor outputs of a hover.

In a steady hover the rotors hold the robot up and cancel the moment of its weight about the root
frame: with the force of rotor i, F_i = f_i n_i at p_i, and its drag torque m_f_rate sigma_i f_i n_i
(the wrench allocation of aerial_robot_model), the moment of the rotors is

    M = sum_i (p_i x F_i + m_f_rate sigma_i F_i)

and, since the thrust carries the weight (sum_i F_i = -m g on average), M = r x sum_i F_i, where r is
the centroid in the root frame. That gives the x and y of r from the thrust alone, without the mass or
the attitude; the z of r is taken from the robot model (a hover barely observes it).

The thrust f_i comes from motor_pwms of spinal (the duty x 2000, after the attitude PID of the flight
controller) through the inverse of the polynomial of MotorInfo.yaml at the recorded voltage, as spinal
converts it; the directions n_i and the positions p_i come from the robot model at the recorded gimbal
and arm angles. A duty at max_pwm is saturated and its sample is dropped.

usage::

    rosrun gimbalrotor estimate_cog_from_flight.py flight_0.bag [flight_1.bag ...] \\
        --robot-description robot.urdf [--min-height 0.1]

The samples are taken while the flight state is TAKEOFF or HOVER and the robot is --min-height above
where it stood, and averaged; --window restricts them to a time range of the bags.

Checked in gazebo (grape_with_arm hovering with its arm stretched): the centroid from the thrust was
within 0.6 mm of the model (x 0.1158 against 0.1164 m). The thrust there adds up to half the weight:
spinal of the simulation converts with another voltage than the first reference assumed by
--voltage 25.2, which scales all the thrust alike and so leaves the centroid unchanged. On the real
machine spinal converts with the voltage it publishes as battery_voltage_status, which is used here.
"""

import argparse

import numpy as np
import rosbag
import rospkg
import yaml
from skrobot.coordinates import Coordinates
from skrobot.models.urdf import RobotModelFromURDF

TAKEOFF_STATE = 3
HOVER_STATE = 5


def load_motor_info(path):
    info = yaml.safe_load(open(path))['motor_info']
    refs = []
    for i in range(1, info['vel_ref_num'] + 1):
        ref = info['ref%d' % i]
        refs.append(dict(voltage=ref['voltage'], max_thrust=ref['max_thrust'],
                         poly=[ref['polynominal%d' % k] for k in range(5)]))
    return info, refs


def pwm_to_thrust(duty, voltage, refs):
    """Invert pwm = poly(v_factor f / 10) / 100 of spinal (POLYNOMINAL_MODE).

    Parameters
    ----------
    duty : float
        Duty of the motor, 0.5 .. 1.0.
    voltage : float
        Voltage of the robot [V].
    refs : list of dict
        The references of MotorInfo.yaml.

    Returns
    -------
    float
        Thrust [N].
    """
    ref = min(refs, key=lambda r: abs(voltage - r['voltage']))
    v_factor = ref['voltage'] / voltage * np.sqrt(ref['voltage'] / voltage)
    percent = duty * 100.0
    # the polynomial is monotonic over the thrust range: bisection on the scaled thrust
    lo, hi = 0.0, ref['max_thrust'] * 0.1 * 1.5
    poly = ref['poly']
    for _ in range(60):
        mid = (lo + hi) / 2.0
        value = sum(c * mid ** k for k, c in enumerate(poly))
        if value < percent:
            lo = mid
        else:
            hi = mid
    return (lo + hi) / 2.0 * 10.0 / v_factor


def rotor_directions(urdf_text):
    """sigma_i of the rotors: the z of the axis of the continuous joint rotor<i>."""
    import xml.etree.ElementTree as ET
    root = ET.fromstring(urdf_text)
    sigma = {}
    for joint in root.iter('joint'):
        name = joint.get('name')
        if name.startswith('rotor') and name[5:].isdigit() and joint.get('type') == 'continuous':
            sigma[int(name[5:])] = float(joint.find('axis').get('xyz').split()[2])
    m_f_rate = float(root.find('m_f_rate').get('value'))
    return sigma, m_f_rate


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('bags', nargs='+')
    parser.add_argument('--robot-description', required=True,
                        help='URDF of the robot as flown (e.g. rosparam get /<ns>/robot_description)')
    parser.add_argument('--motor-info', default=None,
                        help='MotorInfo.yaml (default: config/MotorInfo.yaml of gimbalrotor)')
    parser.add_argument('--robot-ns', default='gimbalrotor')
    parser.add_argument('--min-height', type=float, default=0.1,
                        help='[m] above the ground height before the takeoff')
    parser.add_argument('--voltage', type=float, default=None,
                        help='[V] instead of battery_voltage_status, e.g. 25.2 in gazebo, where spinal '
                        'converts with the first reference of MotorInfo.yaml')
    parser.add_argument('--window', type=float, nargs=2, default=None,
                        help='[s from the start of the first bag] only these samples')
    args = parser.parse_args()

    motor_info_path = args.motor_info or (
        rospkg.RosPack().get_path('gimbalrotor') + '/config/MotorInfo.yaml')
    info, refs = load_motor_info(motor_info_path)
    max_duty = info['max_pwm']
    urdf_text = open(args.robot_description).read()
    sigma, m_f_rate = rotor_directions(urdf_text)
    robot = RobotModelFromURDF(urdf_file=args.robot_description)
    rotor_num = len(sigma)

    ns = '/' + args.robot_ns
    topics = [ns + t for t in ('/motor_pwms', '/battery_voltage_status', '/joint_states',
                               '/flight_state', '/uav/cog/odom')]
    voltage = args.voltage
    state = z = joints = None
    ground = None
    start = None
    rows = []
    skipped = dict(state=0, height=0, saturated=0, missing=0)
    for bag_path in args.bags:
        with rosbag.Bag(bag_path) as bag:
            for topic, msg, t in bag.read_messages(topics=topics):
                ts = t.to_sec()
                if start is None:
                    start = ts
                if topic.endswith('battery_voltage_status'):
                    if args.voltage is None:
                        voltage = msg.data
                elif topic.endswith('joint_states'):
                    joints = dict(zip(msg.name, msg.position))
                elif topic.endswith('flight_state'):
                    state = msg.data
                elif topic.endswith('cog/odom'):
                    z = msg.pose.pose.position.z
                    if state in (0, 1, 2):
                        ground = z
                elif topic.endswith('motor_pwms'):
                    if args.window and not args.window[0] <= ts - start <= args.window[1]:
                        continue
                    if None in (voltage, joints, z, ground):
                        skipped['missing'] += 1
                        continue
                    if state not in (TAKEOFF_STATE, HOVER_STATE):
                        skipped['state'] += 1
                        continue
                    if z - ground < args.min_height:
                        skipped['height'] += 1
                        continue
                    duty = np.array(msg.motor_value[:rotor_num], dtype=float) / 2000.0
                    if np.any(duty >= max_duty - 1e-3):
                        skipped['saturated'] += 1
                        continue
                    thrust = [pwm_to_thrust(d, voltage, refs) for d in duty]
                    rows.append((ts - start, voltage, dict(joints), thrust))
    print('samples used: %d, skipped: %s' % (len(rows), skipped))
    if not rows:
        raise SystemExit('no hover sample; check --min-height, --window and the topics of the bags')

    forces = []
    moments = []
    r_z = []
    for ts, volt, joint_angles, thrust in rows:
        robot.angle_vector(np.zeros(len(robot.angle_vector())))
        for name, angle in joint_angles.items():
            if hasattr(robot, name):
                getattr(robot, name).joint_angle(angle)
        robot.newcoords(Coordinates())
        force = np.zeros(3)
        moment = np.zeros(3)
        for i in range(1, rotor_num + 1):
            frame = getattr(robot, 'thrust%d' % i).copy_worldcoords()
            n = frame.worldrot()[:, 2]
            f = thrust[i - 1] * n
            force += f
            moment += np.cross(frame.worldpos(), f) + m_f_rate * sigma[i] * f
        forces.append(force)
        moments.append(moment)
        r_z.append(robot.centroid()[2])
    force = np.mean(forces, axis=0)
    moment = np.mean(moments, axis=0)
    rz = float(np.mean(r_z))
    # M = r x F  ->  M_x = r_y F_z - r_z F_y,  M_y = r_z F_x - r_x F_z
    r_x = (rz * force[0] - moment[1]) / force[2]
    r_y = (moment[0] + rz * force[1]) / force[2]
    model = []
    for ts, volt, joint_angles, thrust in rows[::max(1, len(rows) // 50)]:
        robot.angle_vector(np.zeros(len(robot.angle_vector())))
        for name, angle in joint_angles.items():
            if hasattr(robot, name):
                getattr(robot, name).joint_angle(angle)
        robot.newcoords(Coordinates())
        model.append(robot.centroid())
    model = np.mean(model, axis=0)
    thrusts = np.array([row[3] for row in rows])
    print('time %.1f .. %.1f s, voltage %.2f .. %.2f V' % (
        rows[0][0], rows[-1][0], min(r[1] for r in rows), max(r[1] for r in rows)))
    print('mean thrust per rotor [N]: %s, total %.2f N (%.3f kg)' % (
        np.round(thrusts.mean(0), 2), thrusts.sum(1).mean(), force[2] / 9.80665))
    print('centroid from the thrust (root frame): x %+.4f y %+.4f m (z %.4f from the model)' % (r_x, r_y, rz))
    print('centroid of the robot model:           x %+.4f y %+.4f m' % (model[0], model[1]))
    print('model - thrust: x %+.4f y %+.4f m' % (model[0] - r_x, model[1] - r_y))


if __name__ == '__main__':
    main()
