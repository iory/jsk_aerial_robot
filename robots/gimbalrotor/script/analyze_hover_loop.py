#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Attitude loop of a gimbalrotor from a plain hover (no excitation), where it can be done without bias.

Two paths of the loop have no feedback inside them, so a hover alone gives them:
- actuation: the torque commanded by the controller (I_model u, u from debug/pose/pid) to the rotor
  torque (esc_telem rpm and joint_states gimbal angles): tau(t) = g I_model u(t - delay_a)
- plant: the rotor torque to the rate (spinal gyro): I dw/dt = tau(t - delay_p) + d, which a disturbance
  d correlated with tau through the loop could bias; the fit quality (r2) tells how much d matters.
With them the loop C A P of the PID of the controller gives the crossover and the phase margin for the
current and other gains. For a clean plant use excite_attitude.py and analyze_excitation.py.

Caveats found on the hovers of 2026-09-17: the rotor torque also carries the torque of the forward / side
forces of the gimbals (the rotors are above the centroid), which the position control commands and which
follows the attitude, so the actuation delay comes out longer than the path of the thrusts alone (0.27 s
against 0.07 s for the pitch); and on a synthetic hover the plant inertia came out 1.4 times too large.

usage::

    rosrun gimbalrotor analyze_hover_loop.py flight.bag [flight.bag ...] --axis pitch --mass 3.9 \\
        --cog 0.0071 0.0034 -0.0172 --model-inertia 0.109 --gains 15 1 10
"""

import argparse

import numpy as np
import rosbag
from scipy import signal

from analyze_excitation import rotor_torque

NS = '/gimbalrotor'
FS = 100.0


def read(path, axis):
    idx = 0 if axis == 'roll' else 1
    d = {k: [] for k in ('rpm', 'gst', 'gyro', 'state', 'u')}
    with rosbag.Bag(path) as bag:
        for topic, msg, t in bag.read_messages(topics=[NS + s for s in (
                '/esc_telem', '/joint_states', '/imu', '/flight_state', '/debug/pose/pid')]):
            ts = t.to_sec()
            if topic.endswith('esc_telem'):
                d['rpm'].append([ts] + [getattr(msg, 'esc_telemetry_%d' % i).rpm for i in range(1, 5)])
            elif topic.endswith('joint_states'):
                pos = dict(zip(msg.name, msg.position))
                d['gst'].append([ts] + [pos.get('gimbal%d' % i, np.nan) for i in range(1, 5)])
            elif topic.endswith('/imu'):
                d['gyro'].append((ts, msg.gyro[idx]))
            elif topic.endswith('pose/pid'):
                v = getattr(msg, axis).total
                if len(v):
                    d['u'].append((ts, v[0]))
            else:
                d['state'].append((ts, msg.data))
    return {k: np.array(v, float) for k, v in d.items()}


def fit_lag(x, y, lags):
    """y(t) = g x(t - lag) + c: the lag of the best r2, its gain and r2."""
    best = None
    for lag in lags:
        n = int(round(lag * FS))
        xx, yy = (x[:len(x) - n], y[n:]) if n > 0 else (x, y)
        A = np.column_stack([xx, np.ones(len(xx))])
        coef, *_ = np.linalg.lstsq(A, yy, rcond=None)
        r2 = 1 - (yy - A.dot(coef)).var() / yy.var()
        if best is None or r2 > best[2]:
            best = (lag, coef[0], r2)
    return best


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('bags', nargs='+')
    parser.add_argument('--axis', choices=('roll', 'pitch'), default='pitch')
    parser.add_argument('--mass', type=float, required=True)
    parser.add_argument('--cog', type=float, nargs=3, required=True)
    parser.add_argument('--model-inertia', type=float, required=True)
    parser.add_argument('--gains', type=float, nargs=3, default=(15.0, 1.0, 10.0))
    args = parser.parse_args()

    bp = signal.butter(2, [0.2 / (0.5 * FS), 8.0 / (0.5 * FS)], btype='band')
    lp = signal.butter(2, 8.0 / (0.5 * FS))
    taus, cmds, wdots = [], [], []
    for path in args.bags:
        d = read(path, args.axis)
        st = d['state']
        hover = st[st[:, 1] == 5, 0]
        if len(hover) == 0 or len(d['u']) == 0:
            print('%s: no hover with debug/pose/pid' % path)
            continue
        grid = np.arange(hover.min() + 1.0, hover.max(), 1 / FS)
        tau = rotor_torque(grid, d['rpm'], d['gst'], args.mass, np.array(args.cog), args.axis)
        u = args.model_inertia * np.interp(grid, d['u'][:, 0], d['u'][:, 1])
        w = signal.filtfilt(*lp, np.interp(grid, d['gyro'][:, 0], d['gyro'][:, 1]))
        print('%s: hover %.1f s' % (path, grid[-1] - grid[0]))
        taus.append(signal.filtfilt(*bp, tau))
        cmds.append(signal.filtfilt(*bp, u))
        wdots.append(signal.filtfilt(*bp, np.gradient(w, 1 / FS)))
    if not taus:
        raise SystemExit('no data')
    # fit each flight separately and pool by concatenating the lagged pairs
    lags = np.arange(0.0, 0.41, 0.01)

    def pooled(xs, ys):
        best = None
        for lag in lags:
            n = int(round(lag * FS))
            xx = np.concatenate([x[:len(x) - n] if n else x for x in xs])
            yy = np.concatenate([y[n:] for y in ys])
            A = np.column_stack([xx, np.ones(len(xx))])
            coef, *_ = np.linalg.lstsq(A, yy, rcond=None)
            r2 = 1 - (yy - A.dot(coef)).var() / yy.var()
            if best is None or r2 > best[2]:
                best = (lag, coef[0], r2)
        return best

    delay_a, gain_a, r2_a = pooled(cmds, taus)
    delay_p, inv_i, r2_p = pooled(taus, wdots)
    inertia = 1 / inv_i
    print('actuation: torque = %.2f x commanded torque, %.2f s later (r2 %.2f)' % (gain_a, delay_a, r2_a))
    print('plant: I %.3f kg m2 (model %.3f), %.2f s later (r2 %.2f)' % (inertia, args.model_inertia, delay_p, r2_p))
    delay = delay_a + delay_p

    def margins(kp, ki, kd):
        wgrid = np.logspace(-1, 2, 4000)
        s = 1j * wgrid
        Lw = args.model_inertia * (kp + ki / s + kd * s) * gain_a * np.exp(-s * delay) / (inertia * s ** 2)
        mag = np.abs(Lw)
        cross = np.where(np.diff(np.sign(mag - 1)))[0]
        if len(cross) == 0:
            return None
        pm = (180 + np.degrees(np.angle(Lw[cross[-1]])) + 180) % 360 - 180
        return wgrid[cross[-1]] / (2 * np.pi), pm

    kp, ki, kd = args.gains
    m = margins(kp, ki, kd)
    print('loop with Kp %.1f Ki %.1f Kd %.1f (delay %.2f s): crossover %.2f Hz, phase margin %.0f deg' % (
        kp, ki, kd, delay, m[0], m[1]))
    print('other gains (crossover Hz / phase margin deg):')
    for kd_ in (4, 6, 8, 10, 12, 15, 20):
        row = []
        for kp_ in (5, 10, 15, 20, 30):
            mm = margins(kp_, ki, kd_)
            row.append('%4.2f/%3.0f' % mm if mm else '   -    ')
        print('  Kd %4.1f: %s  (Kp 5 10 15 20 30)' % (kd_, '  '.join(row)))


if __name__ == '__main__':
    main()
