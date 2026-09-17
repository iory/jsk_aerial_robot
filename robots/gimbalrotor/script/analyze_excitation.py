#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Frequency response of the attitude of a gimbalrotor from a flight excited by excite_attitude.py.

With the excitation r (the roll / pitch target of uav/nav), which does not depend on the flight, the
plant from the rotor torque tau (esc_telem rpm and joint_states gimbal angles) to the attitude theta is

    P(f) = S_r,theta(f) / S_r,tau(f)

at the frequencies of the multisine, unbiased by the feedback of the controller. It is fitted by
exp(-j w delay) / (I (j w)^2), and the actuation from the command of the controller
(debug/pose/pid: an angular acceleration u, which the allocation turns into I_model u) to the torque by

    A(f) = S_r,tau(f) / S_r,(I_model u)(f),  fitted by g exp(-j w delay_a).

The loop C(f) A(f) P(f) of the PID of the controller (C = I_model (Kp + Ki / s + Kd s)) gives the crossover
and the phase margin, now and for other gains.

usage::

    rosrun gimbalrotor analyze_excitation.py flight.bag --axis pitch --mass 3.9 \\
        --cog 0.0071 0.0034 -0.0172 --model-inertia 0.109 --gains 15 1 10
"""

import argparse

import numpy as np
import rosbag

NS = '/gimbalrotor'
FS = 100.0
L = 0.22309
ROTOR_XY = np.array([[-L, -L], [L, -L], [L, L], [-L, L]])
GIMBAL_YAW = np.array([-2.3562, -0.7854, 0.7854, 2.3562])
H = 0.0585


def read(path, axis):
    idx = 0 if axis == 'roll' else 1
    d = {k: [] for k in ('r', 'rpm', 'gst', 'gyro', 'state', 'u')}
    with rosbag.Bag(path) as bag:
        for topic, msg, t in bag.read_messages(topics=[NS + s for s in (
                '/uav/nav', '/esc_telem', '/joint_states', '/imu', '/flight_state', '/debug/pose/pid')]):
            ts = t.to_sec()
            if topic.endswith('uav/nav'):
                mode = msg.roll_nav_mode if axis == 'roll' else msg.pitch_nav_mode
                if mode == 2:
                    d['r'].append((ts, msg.target_roll if axis == 'roll' else msg.target_pitch))
            elif topic.endswith('esc_telem'):
                d['rpm'].append([ts] + [getattr(msg, 'esc_telemetry_%d' % i).rpm for i in range(1, 5)])
            elif topic.endswith('joint_states'):
                pos = dict(zip(msg.name, msg.position))
                d['gst'].append([ts] + [pos.get('gimbal%d' % i, np.nan) for i in range(1, 5)])
            elif topic.endswith('pose/pid'):
                v = getattr(msg, axis).total
                if len(v):
                    d['u'].append((ts, v[0]))
            elif topic.endswith('/imu'):
                # the angular velocity of the axis (spinal gyro)
                d['gyro'].append((ts, msg.gyro[idx]))
            else:
                d['state'].append((ts, msg.data))
    return {k: np.array(v, float) for k, v in d.items()}


def rotor_torque(grid, rpm, gst, mass, cog, axis):
    R = np.column_stack([np.interp(grid, rpm[:, 0], rpm[:, 1 + i]) for i in range(4)])
    G = np.column_stack([np.interp(grid, gst[:, 0], gst[:, 1 + i]) for i in range(4)])
    k = mass * 9.80665 / (R ** 2).sum(1).mean()
    tau = np.zeros((len(grid), 3))
    for i in range(4):
        radial = np.array([np.cos(GIMBAL_YAW[i]), np.sin(GIMBAL_YAW[i]), 0.0])
        n = np.outer(np.cos(G[:, i]), [0, 0, 1]) + np.outer(np.sin(G[:, i]), np.cross(radial, [0, 0, 1]))
        F = n * (k * R[:, i] ** 2)[:, None]
        p = np.array([ROTOR_XY[i, 0], ROTOR_XY[i, 1], H]) - cog
        tau += np.cross(p, F)
    return tau[:, 0 if axis == 'roll' else 1]


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('bag')
    parser.add_argument('--axis', choices=('roll', 'pitch'), default='pitch')
    parser.add_argument('--mass', type=float, required=True)
    parser.add_argument('--cog', type=float, nargs=3, required=True)
    parser.add_argument('--model-inertia', type=float, required=True, help='[kg m2] of the axis in the robot model')
    parser.add_argument('--gains', type=float, nargs=3, default=(15.0, 1.0, 10.0), help='Kp Ki Kd of the axis')
    parser.add_argument('--period', type=float, default=10.0, help='period of the multisine [s]')
    args = parser.parse_args()

    d = read(args.bag, args.axis)
    if len(d['r']) == 0:
        raise SystemExit('no excitation of %s in uav/nav' % args.axis)
    r = d['r']
    t0 = r[0, 0]
    n_periods = int((r[-1, 0] - t0) / args.period)
    # skip the first period (fade in) and use whole periods
    t_from = t0 + args.period
    t_to = t0 + n_periods * args.period
    if t_to - t_from < 2 * args.period:
        raise SystemExit('the excitation needs three periods or more')
    n_used = int(round((t_to - t_from) / args.period))
    per = int(round(args.period * FS))
    grid = t_from + np.arange(n_used * per) / FS
    rs = np.interp(grid, r[:, 0], r[:, 1])
    w = np.interp(grid, d['gyro'][:, 0], d['gyro'][:, 1])
    tau = rotor_torque(grid, d['rpm'], d['gst'], args.mass, np.array(args.cog), args.axis)
    u = args.model_inertia * np.interp(grid, d['u'][:, 0], d['u'][:, 1])
    freqs = np.fft.rfftfreq(per, 1 / FS)
    # one spectrum per period: the multisine repeats exactly, the disturbance does not
    Rk = np.array([np.fft.rfft(rs[k * per:(k + 1) * per]) for k in range(n_used)])
    Wk = np.array([np.fft.rfft(w[k * per:(k + 1) * per]) for k in range(n_used)])
    Tk = np.array([np.fft.rfft(tau[k * per:(k + 1) * per]) for k in range(n_used)])
    Uk = np.array([np.fft.rfft(u[k * per:(k + 1) * per]) for k in range(n_used)])
    Rm = Rk.mean(0)
    lines = [i for i in range(1, len(freqs)) if abs(Rm[i]) > 0.1 * np.abs(Rm[1:]).max()]
    print('%s: %d periods of %.0f s, %d lines %.2f-%.2f Hz' % (
        args.axis, n_used, args.period, len(lines), freqs[lines[0]], freqs[lines[-1]]))
    om = 2 * np.pi * freqs[lines]
    # rate / r and torque / r, averaged over the periods (the disturbance averages out), and their spread
    GW = (Wk[:, lines] / Rk[:, lines])
    GT = (Tk[:, lines] / Rk[:, lines])
    P_rate = GW.mean(0) / GT.mean(0)
    P = P_rate / (1j * om)
    # relative standard error of P from the spread over the periods
    rel = np.sqrt((np.abs(GW - GW.mean(0)) ** 2).mean(0) / np.abs(GW.mean(0)) ** 2
                  + (np.abs(GT - GT.mean(0)) ** 2).mean(0) / np.abs(GT.mean(0)) ** 2) / np.sqrt(max(n_used - 1, 1))
    weight = 1.0 / np.maximum(rel, 0.05) ** 2
    # fit P = exp(-j w L) / (I (j w)^2) on the log of P (magnitude and phase alike)
    best = None
    for delay in np.arange(0.0, 0.4, 0.005):
        base = np.exp(-1j * om * delay) / ((1j * om) ** 2)
        # log P = log base - log I: the complex residual of log(P / base) should be the real -log I
        res = np.log(P / base)
        res = np.real(res) + 1j * np.angle(np.exp(1j * np.imag(res)))
        log_inv_i = np.sum(weight * np.real(res)) / np.sum(weight)
        err = np.sum(weight * np.abs(res - log_inv_i) ** 2) / np.sum(weight)
        if best is None or err < best[0]:
            best = (err, delay, np.exp(-log_inv_i))
    err, delay, inertia = best
    print('plant: I %.3f kg m2 (model %.3f), delay %.3f s, rms error of log P %.2f' % (
        inertia, args.model_inertia, delay, np.sqrt(err)))
    print('  f[Hz]  |P| measured  |P| fit   phase measured  fit [deg]  rel. error')
    for f, p, e in zip(freqs[lines], P, rel):
        m = np.exp(-1j * 2 * np.pi * f * delay) / (inertia * (1j * 2 * np.pi * f) ** 2)
        print('  %5.2f  %10.3f %9.3f   %8.0f %8.0f   %6.2f' % (
            f, abs(p), abs(m), np.degrees(np.angle(p)), np.degrees(np.angle(m)), e))

    # actuation: torque / commanded torque = g exp(-j w delay_a)
    GU = (Uk[:, lines] / Rk[:, lines]).mean(0)
    A = GT.mean(0) / GU
    best = None
    for delay_a in np.arange(0.0, 0.4, 0.005):
        res = np.log(A / np.exp(-1j * om * delay_a))
        res = np.real(res) + 1j * np.angle(np.exp(1j * np.imag(res)))
        log_g = np.sum(weight * np.real(res)) / np.sum(weight)
        e = np.sum(weight * np.abs(res - log_g) ** 2) / np.sum(weight)
        if best is None or e < best[0]:
            best = (e, delay_a, np.exp(log_g))
    err_a, delay_a, gain_a = best
    print('actuation: torque / commanded torque %.2f, delay %.3f s, rms error of log A %.2f' % (
        gain_a, delay_a, np.sqrt(err_a)))

    def margins(kp, ki, kd):
        wgrid = np.logspace(-1, 2, 4000)
        s = 1j * wgrid
        C = args.model_inertia * (kp + ki / s + kd * s)
        Lw = C * gain_a * np.exp(-s * (delay + delay_a)) / (inertia * s ** 2)
        mag = np.abs(Lw)
        cross = np.where(np.diff(np.sign(mag - 1)))[0]
        if len(cross) == 0:
            return None
        wc = wgrid[cross[-1]]
        pm = 180 + np.degrees(np.angle(Lw[cross[-1]]))
        pm = (pm + 180) % 360 - 180
        return wc / (2 * np.pi), pm

    kp, ki, kd = args.gains
    m = margins(kp, ki, kd)
    print('loop with Kp %.1f Ki %.1f Kd %.1f: crossover %.2f Hz, phase margin %.0f deg' % (kp, ki, kd, m[0], m[1]))
    print('other gains (crossover Hz / phase margin deg):')
    for kd_ in (4, 6, 8, 10, 12, 15, 20):
        row = []
        for kp_ in (5, 10, 15, 20, 30):
            mm = margins(kp_, ki, kd_)
            row.append('%4.2f/%3.0f' % mm if mm else '   -    ')
        print('  Kd %4.1f: %s  (Kp 5 10 15 20 30)' % (kd_, '  '.join(row)))


if __name__ == '__main__':
    main()
