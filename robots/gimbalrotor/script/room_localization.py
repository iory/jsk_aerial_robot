#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Put the lidar odometry of a run into the frame of a saved map, so that the coordinates repeat.

fast_lio starts its world (``~odom_frame``, ``camera_init``) at the pose where it was switched on, so the
same place has different coordinates on every run. The room does not move: this node matches the cloud of
the current run against a cloud recorded once (``~map``) and publishes the transform between them as
``~map_frame -> ~odom_frame``. Targets can then be given in the map frame on every run, and the
``camera_init`` of fast_lio stays where it is.

How the match works. A room has two wall directions at a right angle, so the angle that makes the
histograms of the point coordinates sharpest is the direction of its walls; the rotation between the run
and the map is the difference of the two angles. With the run turned into the map frame, the shift along
each axis is the one that correlates the two histograms best, refined to less than a bin by the parabola
through the peak. It needs no initial guess and no iteration.

A rectangular room repeats every 180 deg (and every 90 deg when it is nearly square), so all four turns are
tried and the one whose histograms correlate best wins, ``~yaw_hint`` breaking a tie: place the robot in
roughly the heading it had when the map was recorded.

Parameters
----------
~map : str
    PCD of the room, e.g. the scans.pcd of fast_lio (binary or ascii, xyz first).
~cloud : str
    Cloud of the current run in ``~odom_frame`` (default: /cloud_registered of fast_lio).
~map_frame : str, ~odom_frame : str
    Frames of the transform (default: map and camera_init).
~duration : float
    Cloud is collected for this long before the match [s] (default: 5.0).
~yaw_hint : float
    Expected rotation from ``~odom_frame`` to ``~map_frame`` [rad] (default: 0.0).
~voxel : float
    Both clouds are thinned to one point per voxel before the match [m] (default: 0.1).
~bin : float
    Bin of the histograms [m] (default: 0.05).
~angle_step : float
    Step of the wall direction search [deg] (default: 0.05).

Services
--------
~relocalize : std_srvs/Empty
    Collect the cloud again and republish the transform.
"""

import threading

import numpy as np
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import PointCloud2
from std_srvs.srv import Empty
from std_srvs.srv import EmptyResponse


def load_pcd_xyz(path):
    """Read the xyz of a PCD file (binary or ascii) whose first three fields are x, y, z."""
    with open(path, 'rb') as f:
        fields = None
        while True:
            line = f.readline().decode('ascii', 'replace')
            if line.startswith('FIELDS'):
                fields = line.split()[1:]
            if line.startswith('DATA'):
                binary = line.split()[1].strip() == 'binary'
                break
        if binary:
            data = np.frombuffer(f.read(), dtype=np.float32).reshape(-1, len(fields))
            return data[:, :3].astype(np.float64)
    rows = []
    with open(path) as f:
        header = True
        for line in f:
            if header:
                header = not line.startswith('DATA')
                continue
            values = line.split()
            rows.append([float(values[0]), float(values[1]), float(values[2])])
    return np.array(rows)


def cloud_to_xyz(msg):
    """Return the xyz of a PointCloud2 as (n, 3)."""
    offsets = {f.name: f.offset for f in msg.fields}
    if not {'x', 'y', 'z'} <= set(offsets):
        raise ValueError('the cloud has no xyz fields')
    raw = np.frombuffer(msg.data, dtype=np.uint8)
    raw = raw[:(len(raw) // msg.point_step) * msg.point_step].reshape(-1, msg.point_step)
    xyz = np.empty((len(raw), 3), dtype=np.float32)
    for i, axis in enumerate('xyz'):
        xyz[:, i] = raw[:, offsets[axis]:offsets[axis] + 4].copy().view(np.float32).reshape(-1)
    return xyz[np.isfinite(xyz).all(axis=1)].astype(np.float64)


def voxel_downsample(xyz, voxel):
    _, index = np.unique(np.floor(xyz / voxel).astype(np.int64), axis=0, return_index=True)
    return xyz[index]


def rotation(yaw):
    """Rotation into a frame turned by yaw: row vectors are rotated with p.dot(rotation(yaw).T)."""
    c, s = np.cos(yaw), np.sin(yaw)
    return np.array([[c, s], [-s, c]])


def histogram(values, low, high, bin_size):
    edges = np.arange(low, high + bin_size, bin_size)
    hist, _ = np.histogram(values, bins=edges)
    return np.convolve(hist.astype(np.float64), np.ones(3) / 3.0, mode='same')


def sharpness(values, bin_size):
    """How concentrated the points are along one axis: walls across it make tall peaks."""
    hist = histogram(values, values.min() - bin_size, values.max() + bin_size, bin_size)
    return float((hist ** 2).sum()) / max(float(len(values)) ** 2, 1e-9)


def wall_direction(xy, bin_size, angle_step):
    """The direction of the walls, within 0..90 deg: the angle whose histograms are sharpest."""
    best = None
    for angle in np.arange(0.0, 90.0, angle_step):
        yaw = np.radians(angle)
        local = xy.dot(rotation(yaw).T)
        score = sharpness(local[:, 0], bin_size) + sharpness(local[:, 1], bin_size)
        if best is None or score > best[0]:
            best = (score, yaw)
    return best[1]


def best_shift(run_values, map_values, bin_size, margin=6.0):
    """The shift that lines the run histogram up with the map histogram, and how well they match."""
    low = min(map_values.min(), run_values.min()) - margin
    high = max(map_values.max(), run_values.max()) + margin
    run_hist = histogram(run_values, low, high, bin_size)
    map_hist = histogram(map_values, low, high, bin_size)
    correlation = np.correlate(map_hist, run_hist, mode='full')
    peak = int(correlation.argmax())
    lag = float(peak - (len(run_hist) - 1))
    if 0 < peak < len(correlation) - 1:
        left, middle, right = correlation[peak - 1], correlation[peak], correlation[peak + 1]
        denominator = left - 2 * middle + right
        if denominator != 0:
            lag += 0.5 * float((left - right) / denominator)  # sub bin peak of the parabola
    score = float(correlation[peak] / (np.linalg.norm(map_hist) * np.linalg.norm(run_hist) + 1e-9))
    return lag * bin_size, score


def align(map_xy, run_xy, bin_size, angle_step, yaw_hint):
    """Return (translation, yaw, score) that put the run cloud onto the map cloud."""
    map_yaw = wall_direction(map_xy, bin_size, angle_step)
    run_yaw = wall_direction(run_xy, bin_size, angle_step)
    best = None
    for extra in (0.0, np.pi / 2, np.pi, -np.pi / 2):
        yaw = float(np.arctan2(np.sin(map_yaw - run_yaw - extra), np.cos(map_yaw - run_yaw - extra)))
        rotated = run_xy.dot(rotation(yaw))
        shift_x, score_x = best_shift(rotated[:, 0], map_xy[:, 0], bin_size)
        shift_y, score_y = best_shift(rotated[:, 1], map_xy[:, 1], bin_size)
        hint = abs(float(np.arctan2(np.sin(yaw - yaw_hint), np.cos(yaw - yaw_hint))))
        score = score_x + score_y - 0.05 * hint
        if best is None or score > best[0]:
            best = (score, yaw, np.array([shift_x, shift_y]))
    score, yaw, shift = best
    return shift, yaw, score


class RoomLocalization(object):
    def __init__(self):
        self.map_frame = rospy.get_param('~map_frame', 'map')
        self.odom_frame = rospy.get_param('~odom_frame', 'camera_init')
        self.duration = rospy.get_param('~duration', 5.0)
        self.yaw_hint = rospy.get_param('~yaw_hint', 0.0)
        self.voxel = rospy.get_param('~voxel', 0.1)
        self.bin = rospy.get_param('~bin', 0.05)
        self.angle_step = rospy.get_param('~angle_step', 0.05)
        self.map_xy = voxel_downsample(load_pcd_xyz(rospy.get_param('~map')), self.voxel)[:, :2]
        rospy.loginfo('[room localization] map: %d points after a %.2f m voxel', len(self.map_xy), self.voxel)

        self.lock = threading.Lock()
        self.clouds = []
        self.collecting = False
        self.broadcaster = tf2_ros.StaticTransformBroadcaster()
        self.sub = rospy.Subscriber(rospy.get_param('~cloud', '/cloud_registered'), PointCloud2,
                                    self.cloud_callback, queue_size=5)
        rospy.Service('~relocalize', Empty, self.relocalize)

    def cloud_callback(self, msg):
        with self.lock:
            if self.collecting:
                self.clouds.append(cloud_to_xyz(msg))

    def collect(self):
        with self.lock:
            self.clouds = []
            self.collecting = True
        rospy.loginfo('[room localization] collecting %s for %.1f s', self.sub.resolved_name, self.duration)
        rospy.sleep(self.duration)
        with self.lock:
            self.collecting = False
            clouds, self.clouds = self.clouds, []
        if not clouds:
            rospy.logerr('[room localization] no cloud on %s', self.sub.resolved_name)
            return None
        return np.concatenate(clouds)

    def localize(self):
        xyz = self.collect()
        if xyz is None:
            return False
        run_xy = voxel_downsample(xyz, self.voxel)[:, :2]
        if len(run_xy) < 500:
            rospy.logerr('[room localization] only %d points in the run cloud', len(run_xy))
            return False
        start = rospy.Time.now()
        shift, yaw, score = align(self.map_xy, run_xy, self.bin, self.angle_step, self.yaw_hint)
        rospy.loginfo('[room localization] %d run points, match score %.3f, took %.1f s',
                      len(run_xy), score, (rospy.Time.now() - start).to_sec())
        rospy.loginfo('[room localization] %s -> %s: translation (%.3f, %.3f) m, rotation %.2f deg',
                      self.map_frame, self.odom_frame, shift[0], shift[1], np.degrees(yaw))

        transform = TransformStamped()
        transform.header.stamp = rospy.Time.now()
        transform.header.frame_id = self.map_frame
        transform.child_frame_id = self.odom_frame
        transform.transform.translation.x = float(shift[0])
        transform.transform.translation.y = float(shift[1])
        transform.transform.translation.z = 0.0
        transform.transform.rotation.z = float(np.sin(yaw / 2))
        transform.transform.rotation.w = float(np.cos(yaw / 2))
        self.broadcaster.sendTransform(transform)
        return True

    def relocalize(self, _):
        self.localize()
        return EmptyResponse()


def main():
    rospy.init_node('room_localization')
    node = RoomLocalization()
    if not node.localize():
        rospy.logwarn('[room localization] no transform published; call ~relocalize to try again')
    rospy.spin()


if __name__ == '__main__':
    main()
