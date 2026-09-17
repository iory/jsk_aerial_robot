#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Thin a room map of fast_lio for script/room_localization.py and for git.

fast_lio writes x y z intensity normal_xyz curvature per point, 146 MB for a room; the match only uses
xyz and thins both clouds to ``~voxel`` (0.1 m) itself. This keeps xyz, one point (the mean) per voxel.

usage::

    rosrun gimbalrotor thin_room_map.py scans.pcd maps/<room>.pcd 0.05
"""
import sys
import numpy as np


def read_pcd(path):
    with open(path, 'rb') as f:
        header = {}
        while True:
            line = f.readline().decode('ascii').strip()
            key, _, value = line.partition(' ')
            header[key] = value.split()
            if key == 'DATA':
                break
        if header['DATA'][0] != 'binary':
            raise ValueError('only binary PCD is read')
        fields = header['FIELDS']
        if header['TYPE'] != ['F'] * len(fields) or header['SIZE'] != ['4'] * len(fields):
            raise ValueError('only float32 fields are read')
        data = np.frombuffer(f.read(), dtype=np.float32).reshape(-1, len(fields))
    return data[:, [fields.index(a) for a in 'xyz']]


def write_pcd(path, xyz):
    xyz = np.ascontiguousarray(xyz, dtype=np.float32)
    header = ('# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\n'
              'TYPE F F F\nCOUNT 1 1 1\nWIDTH {0}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {0}\n'
              'DATA binary\n').format(len(xyz))
    with open(path, 'wb') as f:
        f.write(header.encode('ascii'))
        f.write(xyz.tobytes())


def main():
    src, dst, voxel = sys.argv[1], sys.argv[2], float(sys.argv[3])
    xyz = read_pcd(src).astype(np.float64)
    xyz = xyz[np.isfinite(xyz).all(axis=1)]
    keys = np.floor(xyz / voxel).astype(np.int64)
    _, inverse, counts = np.unique(keys, axis=0, return_inverse=True, return_counts=True)
    inverse = inverse.reshape(-1)
    sums = np.zeros((len(counts), 3))
    np.add.at(sums, inverse, xyz)
    thinned = sums / counts[:, None]
    write_pcd(dst, thinned)
    print('%d points -> %d points (voxel %.3f m)' % (len(xyz), len(thinned), voxel))


if __name__ == '__main__':
    main()
