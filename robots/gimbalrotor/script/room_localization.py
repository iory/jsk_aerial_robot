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
tried and the one whose cloud then lies closest to the map wins, after each has been walked to its own best
fit: the walls are symmetric but what stands inside the room is not. ``~yaw_hint`` only breaks a tie, and helps in a room that really is symmetric.

When the match is wrong, or the room is too symmetric to decide, tell it where the robot is: the map is
published on ``~map_cloud`` for rviz, and "2D Pose Estimate" of rviz (geometry_msgs/PoseWithCovarianceStamped
on ``~initial_pose``) starts a new match that keeps the turn closest to that pose and the shift within
``~prior_window`` of it, like the initial pose of a 2D localization.

Parameters
----------
~map : str
    PCD of the room, e.g. the scans.pcd of fast_lio (binary or ascii, xyz first).
~cloud : str
    Cloud of the current run in ``~odom_frame`` (default: /cloud_registered of fast_lio).
~map_frame : str, ~odom_frame : str
    Frames of the transform (default: map and camera_init).
~world_frame : str
    Frame of the state estimation. When ``~robot_odom`` and ``~lio_odom`` are both received, the
    transform ``~map_frame -> ~world_frame`` is published as well, which is what a flight target given
    in map coordinates is converted with (default: world; empty to skip it).
~robot_ns : str
    Namespace of the robot, which names its frames (default: the namespace of the node, or gimbalrotor).
~robot_odom : str
    Odometry of the state estimation, in ``~world_frame`` (default: /<robot_ns>/uav/baselink/odom).
~lio_odom : str
    Odometry of the lidar odometry, in ``~odom_frame`` (default: /Odometry of fast_lio).
~lidar_frame : str
    Frame of the robot model the lidar odometry describes, i.e. the body frame of fast_lio
    (default: <robot_ns>/lidar_imu).
~base_frame : str
    Frame of the robot model that is level when the robot stands on the ground (default: <robot_ns>/root).
    fast_lio starts its frame at the pose of the lidar, so a lidar mounted upside down or tilted gives an
    upside down or tilted ``camera_init``, and so a map recorded in it. The roll and pitch of the lidar in
    this frame are taken off both clouds before the match, the map frame is level, and the published
    ``map -> camera_init`` carries that tilt. This assumes the robot stands level when fast_lio starts.
~mount_roll, ~mount_pitch : float
    Roll and pitch of the lidar in ``~base_frame`` [deg], used instead of the robot model when set.
~floor_at_zero : bool
    Put the floor of the map (its lowest horizontal plane) at z = 0 of the map frame (default: true).

The height is matched too: the height histogram of the run, after the plan view match, is lined up with
that of the map (the floor and the ceiling are its peaks), since the lidar need not be at the same height
at the start of the run as at the start of the recording.
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
~max_points : int
    At most this many points of each cloud are used for the wall direction search, which is the
    expensive part (default: 40000).
~overlap_cell : float
    Cell of the plan view distance map the four turns are compared on, and refined against [m]
    (default: 0.1).
~map_cloud : str
    Topic the map is published on, latched, for rviz (default: ~map_cloud; empty to skip it).
~initial_pose : str
    Pose of the robot in the map frame, e.g. from "2D Pose Estimate" of rviz, which starts a new match
    with that pose as the prior (default: /initialpose).
~prior_window : float
    How far the shift may be from the pose given that way [m] (default: 3.0).
~tie : float
    Fits whose mean distance to the map is within this of the best one are taken as fitting equally
    well, and then the given pose or ``~yaw_hint`` decides [m] (default: 0.004).

Services
--------
~relocalize : std_srvs/Empty
    Collect the cloud again and republish the transform.
"""

import threading

import numpy as np
import rospy
import tf.transformations as tft
import tf2_ros
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField
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


def wall_direction(xy, bin_size, angle_step, max_points=40000):
    """The direction of the walls, within 0..90 deg: the angle whose histograms are sharpest."""
    if len(xy) > max_points:
        xy = xy[::int(np.ceil(len(xy) / float(max_points)))]
    best = None
    for angle in np.arange(0.0, 90.0, angle_step):
        yaw = np.radians(angle)
        local = xy.dot(rotation(yaw).T)
        score = sharpness(local[:, 0], bin_size) + sharpness(local[:, 1], bin_size)
        if best is None or score > best[0]:
            best = (score, yaw)
    return best[1]


def best_shift(run_values, map_values, bin_size, margin=6.0, prior=None, window=None):
    """The shift that lines the run histogram up with the map histogram, and how well they match.

    With a prior only the shifts within window of it are considered, so that a pose given by hand wins
    over a wrong but sharper match elsewhere in the room.
    """
    low = min(map_values.min(), run_values.min()) - margin
    high = max(map_values.max(), run_values.max()) + margin
    run_hist = histogram(run_values, low, high, bin_size)
    map_hist = histogram(map_values, low, high, bin_size)
    correlation = np.correlate(map_hist, run_hist, mode='full')
    if prior is not None and window is not None:
        lags = (np.arange(len(correlation)) - (len(run_hist) - 1)) * bin_size
        allowed = np.abs(lags - prior) <= window
        if allowed.any():
            correlation = np.where(allowed, correlation, -np.inf)
    peak = int(correlation.argmax())
    lag = float(peak - (len(run_hist) - 1))
    if 0 < peak < len(correlation) - 1:
        left, middle, right = correlation[peak - 1], correlation[peak], correlation[peak + 1]
        denominator = left - 2 * middle + right
        # the neighbours are not finite at the edge of the window of a prior
        if np.isfinite([left, middle, right]).all() and denominator != 0:
            lag += 0.5 * float((left - right) / denominator)  # sub bin peak of the parabola
    score = float(correlation[peak] / (np.linalg.norm(map_hist) * np.linalg.norm(run_hist) + 1e-9))
    if not np.isfinite(lag):
        raise ValueError('the shift did not converge')
    return lag * bin_size, score


class DistanceMap(object):
    """Distance from any point of the plan view to the nearest point of the map.

    A chamfer transform over the occupied cells, which is enough to tell a match that sits on the map
    from one that is merely inside it, and to refine it: the mean distance of the run to the map is the
    cost both of choosing between the four turns and of the refinement.
    """

    def __init__(self, xy, cell=0.1, truncate=1.0, margin=2.0):
        self.cell = cell
        self.truncate = truncate
        self.origin = xy.min(0) - margin
        size = np.ceil((xy.max(0) + margin - self.origin) / cell).astype(int) + 1
        grid = np.full(size[::-1], np.inf)
        index = np.floor((xy - self.origin) / cell).astype(int)
        grid[index[:, 1], index[:, 0]] = 0.0
        self.grid = self._chamfer(grid, cell)

    @staticmethod
    def _chamfer(grid, cell):
        """Two pass chamfer distance, with the usual 1 and sqrt(2) steps."""
        straight, diagonal = cell, cell * np.sqrt(2.0)
        rows, cols = grid.shape
        for i in range(rows):
            row = grid[i]
            if i > 0:
                np.minimum(row, grid[i - 1] + straight, out=row)
                np.minimum(row[1:], grid[i - 1][:-1] + diagonal, out=row[1:])
                np.minimum(row[:-1], grid[i - 1][1:] + diagonal, out=row[:-1])
            for j in range(1, cols):  # left to right inside the row
                if row[j - 1] + straight < row[j]:
                    row[j] = row[j - 1] + straight
            for j in range(cols - 2, -1, -1):
                if row[j + 1] + straight < row[j]:
                    row[j] = row[j + 1] + straight
        for i in range(rows - 2, -1, -1):
            row = grid[i]
            np.minimum(row, grid[i + 1] + straight, out=row)
            np.minimum(row[1:], grid[i + 1][:-1] + diagonal, out=row[1:])
            np.minimum(row[:-1], grid[i + 1][1:] + diagonal, out=row[:-1])
            for j in range(cols - 2, -1, -1):
                if row[j + 1] + straight < row[j]:
                    row[j] = row[j + 1] + straight
            for j in range(1, cols):
                if row[j - 1] + straight < row[j]:
                    row[j] = row[j - 1] + straight
        return grid

    def cost(self, xy):
        """Mean distance of these points to the map, truncated so that outliers do not dominate."""
        index = np.floor((xy - self.origin) / self.cell).astype(int)
        inside = ((index[:, 0] >= 0) & (index[:, 0] < self.grid.shape[1]) &
                  (index[:, 1] >= 0) & (index[:, 1] < self.grid.shape[0]))
        distance = np.full(len(xy), self.truncate)
        if inside.any():
            distance[inside] = np.minimum(self.grid[index[inside, 1], index[inside, 0]], self.truncate)
        return float(distance.mean())


def refine(distance_map, run_xy, shift, yaw, passes=3):
    """Walk the transform to the smallest mean distance, over a shrinking step."""
    step_xy, step_yaw = 0.2, np.radians(1.0)
    best = distance_map.cost(run_xy.dot(rotation(yaw)) + shift)
    for _ in range(passes):
        improved = True
        while improved:
            improved = False
            for delta in ([step_xy, 0.0], [-step_xy, 0.0], [0.0, step_xy], [0.0, -step_xy]):
                candidate = shift + np.array(delta)
                cost = distance_map.cost(run_xy.dot(rotation(yaw)) + candidate)
                if cost < best:
                    best, shift, improved = cost, candidate, True
            for delta in (step_yaw, -step_yaw):
                candidate = yaw + delta
                cost = distance_map.cost(run_xy.dot(rotation(candidate)) + shift)
                if cost < best:
                    best, yaw, improved = cost, candidate, True
        step_xy, step_yaw = step_xy / 4.0, step_yaw / 4.0
    return shift, yaw, best


def angle_difference(a, b):
    return abs(float(np.arctan2(np.sin(a - b), np.cos(a - b))))


def align(map_xy, run_xy, bin_size, angle_step, yaw_hint=0.0, max_points=40000, overlap_cell=0.1,
          prior=None, prior_window=3.0, tie=0.004, map_yaw=None, distance_map=None):
    """Return (translation, yaw, score) that put the run cloud onto the map cloud.

    The wall directions give the rotation up to a multiple of 90 deg and the histograms give the shift;
    each of the four turns is then walked to its best fit on the distance map of the map, and the one
    that lies closest to the map wins.

    A room seen from one spot often fits in more than one pose equally well (its walls repeat every
    180 deg). Those ties are broken by the hint: the heading ``yaw_hint``, or with a ``prior``
    ((x, y), yaw) the pose closest to it. With a prior the shift is also searched within
    ``prior_window`` of it, so that a pose given by hand can pick a fit the whole room search would
    not, but a prior far from any fit still ends at the best fit of the room.

    ``map_yaw`` and ``distance_map`` depend only on the map and can be passed in to save their cost.
    """
    if map_yaw is None:
        map_yaw = wall_direction(map_xy, bin_size, angle_step, max_points)
    if distance_map is None:
        distance_map = DistanceMap(map_xy, overlap_cell)
    run_yaw = wall_direction(run_xy, bin_size, angle_step, max_points)

    searches = [(extra, None) for extra in (0.0, np.pi / 2, np.pi, -np.pi / 2)]
    if prior is not None:
        yaw_hint = prior[1]
        nearest = min((extra for extra, _ in searches),
                      key=lambda extra: angle_difference(map_yaw - run_yaw - extra, prior[1]))
        searches.append((nearest, prior[0]))

    candidates = []
    for extra, around in searches:
        yaw = float(np.arctan2(np.sin(map_yaw - run_yaw - extra), np.cos(map_yaw - run_yaw - extra)))
        rotated = run_xy.dot(rotation(yaw))
        window = None if around is None else prior_window
        shift_x, _ = best_shift(rotated[:, 0], map_xy[:, 0], bin_size,
                                prior=None if around is None else around[0], window=window)
        shift_y, _ = best_shift(rotated[:, 1], map_xy[:, 1], bin_size,
                                prior=None if around is None else around[1], window=window)
        shift, yaw, cost = refine(distance_map, run_xy, np.array([shift_x, shift_y]), yaw)
        rospy.loginfo('[room localization] turn %6.1f deg%s: shift (%5.2f, %5.2f), mean distance %.3f m',
                      np.degrees(yaw), '' if around is None else ' near the given pose',
                      shift[0], shift[1], cost)
        candidates.append((cost, yaw, shift))

    lowest = min(cost for cost, _, _ in candidates)
    close = [c for c in candidates if c[0] <= lowest + max(0.2 * lowest, tie)]

    def distance_to_hint(candidate):
        _, yaw, shift = candidate
        score = angle_difference(yaw, yaw_hint) / np.radians(45.0)
        if prior is not None:
            score += float(np.linalg.norm(shift - prior[0])) / prior_window
        return score

    cost, yaw, shift = min(close, key=distance_to_hint)
    return shift, yaw, cost


def matrix_of_pose(position, orientation):
    """4x4 matrix of a geometry_msgs pose or transform."""
    matrix = tft.quaternion_matrix([orientation.x, orientation.y, orientation.z, orientation.w])
    matrix[:3, 3] = [position.x, position.y, position.z]
    return matrix


def planar_of(matrix):
    """The (x, y, heading) of a 4x4 transform, which is what a match in the plan view can give.

    The heading is taken from the first column, so a sensor mounted upside down still gives the
    heading of its x axis on the ground.
    """
    return matrix[:2, 3].copy(), float(np.arctan2(matrix[1, 0], matrix[0, 0]))


def level_rotation(roll, pitch):
    """Rotation that takes the frame of a lidar with this roll and pitch to a level frame."""
    return tft.euler_matrix(roll, pitch, 0.0, 'sxyz')


def matrix_of_planar(shift, yaw, height=0.0):
    matrix = tft.rotation_matrix(yaw, (0, 0, 1))
    matrix[0, 3] = shift[0]
    matrix[1, 3] = shift[1]
    matrix[2, 3] = height
    return matrix


def floor_height(z, bin_size=0.02, strength=0.3):
    """Height of the floor: the lowest of the horizontal planes of a level cloud.

    A horizontal plane is a peak of the height histogram; the lowest peak of at least ``strength`` of the
    highest one is taken, so that the ceiling or a table top, which can be bigger, is not.
    """
    edges = np.arange(z.min() - bin_size, z.max() + 2 * bin_size, bin_size)
    hist, _ = np.histogram(z, bins=edges)
    hist = np.convolve(hist.astype(np.float64), np.ones(5) / 5.0, mode='same')
    peaks = [i for i in range(1, len(hist) - 1)
             if hist[i] >= hist[i - 1] and hist[i] >= hist[i + 1] and hist[i] >= strength * hist.max()]
    return float(edges[peaks[0]] + bin_size / 2.0)


class RoomLocalization(object):
    def __init__(self):
        self.map_frame = rospy.get_param('~map_frame', 'map')
        self.odom_frame = rospy.get_param('~odom_frame', 'camera_init')
        self.duration = rospy.get_param('~duration', 5.0)
        self.yaw_hint = rospy.get_param('~yaw_hint', 0.0)
        self.voxel = rospy.get_param('~voxel', 0.1)
        self.bin = rospy.get_param('~bin', 0.05)
        self.angle_step = rospy.get_param('~angle_step', 0.05)
        self.max_points = rospy.get_param('~max_points', 40000)
        self.overlap_cell = rospy.get_param('~overlap_cell', 0.1)
        self.tie = rospy.get_param('~tie', 0.004)
        self.prior_window = rospy.get_param('~prior_window', 3.0)

        self.world_frame = rospy.get_param('~world_frame', 'world')
        # tf frames have no leading slash; the node may run in the namespace of the robot or outside it
        robot_ns = rospy.get_param('~robot_ns', rospy.get_namespace().strip('/') or 'gimbalrotor')
        self.lidar_frame = rospy.get_param('~lidar_frame', robot_ns + '/lidar_imu')
        self.base_frame = rospy.get_param('~base_frame', robot_ns + '/root')

        self.lock = threading.Lock()
        self.clouds = []
        self.robot_odom = None
        self.lio_odom = None
        self.broadcaster = tf2_ros.StaticTransformBroadcaster()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.level = self.mount_level()
        raw = load_pcd_xyz(rospy.get_param('~map'))
        self.map_xyz = voxel_downsample(self.to_level(raw), self.voxel)
        self.map_floor = floor_height(self.map_xyz[:, 2])
        if rospy.get_param('~floor_at_zero', True):
            self.map_xyz[:, 2] -= self.map_floor
            rospy.loginfo('[room localization] map floor at %.2f m of the recording, moved to z = 0',
                          self.map_floor)
        self.height = 0.0  # of the last match, for a pose given before the next one
        self.map_xy = self.map_xyz[:, :2]
        rospy.loginfo('[room localization] map: %d points after a %.2f m voxel, height %.2f .. %.2f m',
                      len(self.map_xy), self.voxel, self.map_xyz[:, 2].min(), self.map_xyz[:, 2].max())
        # what depends only on the map is computed once
        self.map_yaw = wall_direction(self.map_xy, self.bin, self.angle_step, self.max_points)
        self.distance_map = DistanceMap(self.map_xy, self.overlap_cell)
        rospy.loginfo('[room localization] map walls at %.2f deg', np.degrees(self.map_yaw))
        self.sub = rospy.Subscriber(rospy.get_param('~cloud', '/cloud_registered'), PointCloud2,
                                    self.cloud_callback, queue_size=5)
        if self.world_frame:
            rospy.Subscriber(rospy.get_param('~robot_odom', '/' + robot_ns + '/uav/baselink/odom'), Odometry,
                             self.robot_odom_callback, queue_size=1)
            rospy.Subscriber(rospy.get_param('~lio_odom', '/Odometry'), Odometry,
                             self.lio_odom_callback, queue_size=1)
        rospy.Subscriber(rospy.get_param('~initial_pose', '/initialpose'), PoseWithCovarianceStamped,
                         self.initial_pose_callback, queue_size=1)
        map_cloud_topic = rospy.get_param('~map_cloud', '~map_cloud')
        self.map_pub = None
        if map_cloud_topic:
            self.map_pub = rospy.Publisher(map_cloud_topic, PointCloud2, queue_size=1, latch=True)
            self.map_pub.publish(self.map_cloud_msg())
        rospy.Service('~relocalize', Empty, self.relocalize)

    def mount_level(self):
        """The 4x4 rotation that takes camera_init (the lidar at start) to a level frame."""
        if rospy.has_param('~mount_roll') or rospy.has_param('~mount_pitch'):
            roll = np.radians(rospy.get_param('~mount_roll', 0.0))
            pitch = np.radians(rospy.get_param('~mount_pitch', 0.0))
            source = 'parameters'
        else:
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.base_frame, self.lidar_frame, rospy.Time(0), rospy.Duration(10.0)).transform
            except tf2_ros.TransformException as e:
                rospy.logerr('[room localization] no transform %s -> %s, so the clouds are taken as level; '
                             'start the robot model or set ~mount_roll and ~mount_pitch (%s)',
                             self.base_frame, self.lidar_frame, e)
                return np.eye(4)
            rotation_ = transform.rotation
            roll, pitch, _ = tft.euler_from_quaternion(
                [rotation_.x, rotation_.y, rotation_.z, rotation_.w], 'sxyz')
            source = '%s -> %s' % (self.base_frame, self.lidar_frame)
        rospy.loginfo('[room localization] lidar mount from %s: roll %.1f deg, pitch %.1f deg',
                      source, np.degrees(roll), np.degrees(pitch))
        return level_rotation(roll, pitch)

    def to_level(self, xyz):
        """Points of camera_init in the level frame."""
        return xyz.dot(self.level[:3, :3].T)

    def map_cloud_msg(self):
        """The map as a PointCloud2 in the map frame, so that rviz can show where to put the robot."""
        points = self.map_xyz.astype(np.float32)
        msg = PointCloud2()
        msg.header.frame_id = self.map_frame
        msg.header.stamp = rospy.Time.now()
        msg.height = 1
        msg.width = len(points)
        msg.fields = [PointField('x', 0, PointField.FLOAT32, 1),
                      PointField('y', 4, PointField.FLOAT32, 1),
                      PointField('z', 8, PointField.FLOAT32, 1)]
        msg.point_step = 12
        msg.row_step = 12 * len(points)
        msg.is_dense = True
        msg.data = points.tobytes()
        return msg

    def initial_pose_callback(self, msg):
        """Match again with the pose a person gave, e.g. with "2D Pose Estimate" of rviz."""
        if msg.header.frame_id.strip('/') != self.map_frame.strip('/'):
            rospy.logwarn('[room localization] the initial pose is in %s, not in %s; set the fixed frame '
                          'of rviz to %s', msg.header.frame_id, self.map_frame, self.map_frame)
            return
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        yaw = tft.euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])[2]
        rospy.loginfo('[room localization] pose given: (%.2f, %.2f) m, %.1f deg in %s',
                      position.x, position.y, np.degrees(yaw), self.map_frame)
        prior = self.prior_from_pose((np.array([position.x, position.y]), float(yaw)))
        # put the robot where it was pointed at right away, then let the match move it to where it fits
        self.publish(matrix_of_planar(prior[0], prior[1], self.height).dot(self.level), 'the given pose')
        self.localize(prior=prior)

    def robot_odom_callback(self, msg):
        self.robot_odom = msg

    def lio_odom_callback(self, msg):
        self.lio_odom = msg

    def world_transform(self, map_to_odom):
        """The map frame in the world frame of the state estimation, or None.

        The lidar odometry and the state estimation describe the same robot in their own frames, so
        comparing the two poses of the moment gives the transform between the frames:
        map -> world = (map -> odom) (odom -> lidar) (baselink -> lidar)^-1 (world -> baselink)^-1.
        """
        robot_odom, lio_odom = self.robot_odom, self.lio_odom
        if robot_odom is None or lio_odom is None:
            rospy.logwarn('[room localization] no %s or %s yet, publishing only %s -> %s',
                          'robot odometry', 'lidar odometry', self.map_frame, self.odom_frame)
            return None
        baselink_to_lidar = self.robot_to_lidar()
        if baselink_to_lidar is None:
            return None
        world_to_baselink = matrix_of_pose(robot_odom.pose.pose.position, robot_odom.pose.pose.orientation)
        odom_to_lidar = matrix_of_pose(lio_odom.pose.pose.position, lio_odom.pose.pose.orientation)
        world_to_lidar = world_to_baselink.dot(baselink_to_lidar)
        return map_to_odom.dot(odom_to_lidar).dot(np.linalg.inv(world_to_lidar))

    def cloud_callback(self, msg):
        """Keep the clouds of the last ~duration, so that a match can start at once."""
        now = rospy.get_time()
        xyz = cloud_to_xyz(msg)
        with self.lock:
            self.clouds.append((now, xyz))
            self.clouds = [(stamp, cloud) for stamp, cloud in self.clouds if now - stamp <= self.duration]

    def collect(self):
        """The clouds of the last ~duration, waiting only for what is missing."""
        start = rospy.get_time()
        while not rospy.is_shutdown():
            with self.lock:
                clouds = list(self.clouds)
            span = clouds[-1][0] - clouds[0][0] if clouds else 0.0
            if span >= 0.8 * self.duration:
                break
            if rospy.get_time() - start > self.duration + 5.0:
                break
            rospy.sleep(0.2)
        if not clouds:
            rospy.logerr('[room localization] no cloud on %s', self.sub.resolved_name)
            return None
        return np.concatenate([cloud for _, cloud in clouds])

    def prior_from_pose(self, map_to_robot):
        """Turn a pose of the robot in the map frame into a prior of the map -> odom transform.

        The robot is where the person said and the lidar odometry says where it is in its own frame, so
        the two give the transform between the frames: map -> odom = (map -> robot) (odom -> robot)^-1.
        The pose is the pose of the robot, so the odometry of the lidar is taken back to the frame of
        the robot first, which matters when the lidar is not mounted level.
        """
        lio_odom = self.lio_odom
        if lio_odom is None:
            rospy.logwarn('[room localization] no lidar odometry yet, using the pose as the prior directly')
            return map_to_robot
        odom_to_robot = matrix_of_pose(lio_odom.pose.pose.position, lio_odom.pose.pose.orientation)
        robot_to_lidar = self.robot_to_lidar(self.base_frame)
        if robot_to_lidar is not None:
            odom_to_robot = odom_to_robot.dot(np.linalg.inv(robot_to_lidar))
        # in the level frame the robot stands level, so its x axis is its heading
        shift, yaw = planar_of(self.level.dot(odom_to_robot))
        map_to_odom = matrix_of_planar(map_to_robot[0], map_to_robot[1]).dot(
            np.linalg.inv(matrix_of_planar(shift, yaw)))
        return planar_of(map_to_odom)

    def robot_to_lidar(self, frame=None):
        """The lidar in a frame of the robot model (default: that of the robot odometry), or None."""
        if frame is None:
            if self.robot_odom is None:
                return None
            frame = self.robot_odom.child_frame_id
        try:
            transform = self.tf_buffer.lookup_transform(
                frame, self.lidar_frame, rospy.Time(0), rospy.Duration(2.0)).transform
        except tf2_ros.TransformException as e:
            rospy.logwarn('[room localization] no transform %s -> %s (%s)', frame, self.lidar_frame, e)
            return None
        return matrix_of_pose(transform.translation, transform.rotation)

    def localize(self, prior=None):
        xyz = self.collect()
        if xyz is None:
            return False
        run_xyz = voxel_downsample(self.to_level(xyz), self.voxel)
        run_xy = run_xyz[:, :2]
        if len(run_xy) < 500:
            rospy.logerr('[room localization] only %d points in the run cloud', len(run_xy))
            return False
        start = rospy.Time.now()
        shift, yaw, score = align(self.map_xy, run_xy, self.bin, self.angle_step, self.yaw_hint,
                                  self.max_points, self.overlap_cell, prior, self.prior_window,
                                  self.tie, self.map_yaw, self.distance_map)
        rospy.loginfo('[room localization] %d run points, mean distance to the map %.3f m, took %.1f s',
                      len(run_xy), score, (rospy.Time.now() - start).to_sec())
        # the match is between level frames; camera_init keeps the tilt of the lidar
        # the height: line the horizontal planes of the run up with those of the map
        self.height, _ = best_shift(run_xyz[:, 2], self.map_xyz[:, 2], 0.02, margin=3.0)
        rospy.loginfo('[room localization] height of camera_init in the map: %.3f m', self.height)
        self.publish(matrix_of_planar(shift, yaw, self.height).dot(self.level), 'the match')
        return True

    def publish(self, map_to_odom, source):
        """Publish map -> odom, and map -> world when the robot odometry is there."""
        shift, yaw = planar_of(map_to_odom)
        rospy.loginfo('[room localization] %s -> %s from %s: translation (%.3f, %.3f) m, rotation %.2f deg',
                      self.map_frame, self.odom_frame, source, shift[0], shift[1], np.degrees(yaw))
        transforms = [self.transform_msg(self.odom_frame, map_to_odom)]
        if self.world_frame:
            map_to_world = self.world_transform(map_to_odom)
            if map_to_world is not None:
                translation = map_to_world[:3, 3]
                world_yaw = np.arctan2(map_to_world[1, 0], map_to_world[0, 0])
                rospy.loginfo('[room localization] %s -> %s: translation (%.3f, %.3f, %.3f) m, '
                              'rotation %.2f deg', self.map_frame, self.world_frame, translation[0],
                              translation[1], translation[2], np.degrees(world_yaw))
                transforms.append(self.transform_msg(self.world_frame, map_to_world))
        self.broadcaster.sendTransform(transforms)

    def transform_msg(self, child_frame, matrix):
        quaternion = tft.quaternion_from_matrix(matrix)
        transform = TransformStamped()
        transform.header.stamp = rospy.Time.now()
        transform.header.frame_id = self.map_frame
        transform.child_frame_id = child_frame
        transform.transform.translation.x = float(matrix[0, 3])
        transform.transform.translation.y = float(matrix[1, 3])
        transform.transform.translation.z = float(matrix[2, 3])
        transform.transform.rotation.x = float(quaternion[0])
        transform.transform.rotation.y = float(quaternion[1])
        transform.transform.rotation.z = float(quaternion[2])
        transform.transform.rotation.w = float(quaternion[3])
        return transform

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
