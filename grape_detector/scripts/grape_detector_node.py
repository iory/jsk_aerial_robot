#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Detect grape bunches in a color image with an open vocabulary YOLO.

The detector is open vocabulary: the classes are the prompts of
``~classes`` (e.g. "a bunch of grapes"), so nothing has to be trained.
``~backend`` picks how it runs:

* ``ultralytics``: YOLO-World or YOLOE with torch, on a gpu or the cpu.
* ``adla``: a YOLOE converted to .adla by scripts/export_adla_model.py, on the
  NPU of a Khadas VIM4.
* ``auto`` (default): ``adla`` on a machine with that NPU, ``ultralytics``
  otherwise, so that the same launch runs in gazebo and on the robot.

``~model`` is the model of the backend; if it is empty, ``~ultralytics_model``
or ``~adla_model`` of the backend is used, the latter relative to models/ of
this package.

Published for each image:

* ``~output/image``           the image with the detections drawn on it
* ``~output/rects``           jsk_recognition_msgs/RectArray, the boxes in the image
* ``~output/class``           jsk_recognition_msgs/ClassificationResult, their prompts and scores
* ``~output/cluster_indices`` jsk_recognition_msgs/ClusterPointIndices, the points of each bunch

The indices are the pixels of a box whose depth is close to the front of that box, so that the
canopy behind the bunch is left out; they index an organized point cloud of the color image
(depth_image_proc/point_cloud_xyzrgb of the aligned depth). launch/grape_detection.launch runs that
node and jsk_pcl_ros/ClusterPointIndicesDecomposer, which turns the indices into a
jsk_recognition_msgs/BoundingBoxArray in the frame of the camera.
"""

import os

import cv2
from grape_detector.detectors import create_detector
from grape_detector.detectors import resolve_backend
import numpy as np
import rospkg
import rospy
from jsk_recognition_msgs.msg import ClassificationResult
from jsk_recognition_msgs.msg import ClusterPointIndices
from jsk_recognition_msgs.msg import Rect
from jsk_recognition_msgs.msg import RectArray
import message_filters
from pcl_msgs.msg import PointIndices
from sensor_msgs.msg import Image

DEPTH_ENCODINGS = {'32FC1': (np.float32, 1.0), '16UC1': (np.uint16, 0.001)}


def image_to_numpy(msg):
    """Return the image of a sensor_msgs/Image as an (H, W, 3) BGR array."""
    if msg.encoding not in ('rgb8', 'bgr8'):
        raise ValueError('unsupported image encoding {}'.format(msg.encoding))
    image = np.frombuffer(msg.data, dtype=np.uint8).reshape(
        msg.height, msg.width, 3)
    if msg.encoding == 'rgb8':
        image = image[:, :, ::-1]
    return np.ascontiguousarray(image)


def numpy_to_image(image, header, encoding='bgr8'):
    """Return a sensor_msgs/Image of an (H, W, 3) BGR array.

    Parameters
    ----------
    image : numpy.ndarray
        ``(H, W, 3)`` BGR image.
    header : std_msgs.msg.Header
        Header of the message.
    encoding : str
        ``'bgr8'`` or ``'rgb8'``; the channels are swapped for ``'rgb8'``.
    """
    if encoding not in ('rgb8', 'bgr8'):
        raise ValueError('unsupported image encoding {}'.format(encoding))
    if encoding == 'rgb8':
        image = image[:, :, ::-1]
    msg = Image()
    msg.header = header
    msg.height, msg.width = image.shape[:2]
    msg.encoding = encoding
    msg.is_bigendian = 0
    msg.step = 3 * msg.width
    msg.data = np.ascontiguousarray(image, dtype=np.uint8).tobytes()
    return msg


def depth_to_numpy(msg):
    """Return the depth of a sensor_msgs/Image in meters, with 0 where invalid."""
    if msg.encoding not in DEPTH_ENCODINGS:
        raise ValueError('unsupported depth encoding {}'.format(msg.encoding))
    dtype, scale = DEPTH_ENCODINGS[msg.encoding]
    depth = np.frombuffer(msg.data, dtype=dtype).reshape(
        msg.height, msg.width).astype(np.float32) * scale
    return np.nan_to_num(depth, nan=0.0, posinf=0.0, neginf=0.0)


def draw_detections(image, detections, classes):
    """Return a copy of a BGR image with the detections drawn on it."""
    canvas = image.copy()
    thickness = max(1, int(round(sum(image.shape[:2]) / 600.0)))
    for box, score, label in zip(detections.boxes, detections.scores,
                                 detections.labels):
        x1, y1, x2, y2 = (int(round(v)) for v in box)
        cv2.rectangle(canvas, (x1, y1), (x2, y2), (0, 255, 0), thickness)
        text = '{} {:.2f}'.format(classes[label], score)
        (width, height), baseline = cv2.getTextSize(
            text, cv2.FONT_HERSHEY_SIMPLEX, 0.25 * thickness, thickness)
        top = max(y1 - height - baseline, 0)
        cv2.rectangle(canvas, (x1, top), (x1 + width, top + height + baseline),
                      (0, 255, 0), -1)
        cv2.putText(canvas, text, (x1, top + height), cv2.FONT_HERSHEY_SIMPLEX,
                    0.25 * thickness, (0, 0, 0), thickness, cv2.LINE_AA)
    return canvas


class GrapeDetector(object):

    def __init__(self):
        classes = rospy.get_param('~classes', ['a bunch of grapes'])
        self.conf = rospy.get_param('~conf', 0.05)
        self.iou = rospy.get_param('~iou', 0.5)
        self.max_detections = rospy.get_param('~max_detections', 20)
        # the prompts are synonyms: one bunch is one box whichever it matches
        self.agnostic_nms = rospy.get_param('~agnostic_nms', True)
        # a bunch hangs in front of the canopy: keep the points within
        # depth_margin behind the front of the box
        self.depth_quantile = rospy.get_param('~depth_quantile', 0.2)
        self.depth_margin = rospy.get_param('~depth_margin', 0.15)
        self.min_depth = rospy.get_param('~min_depth', 0.2)
        self.max_depth = rospy.get_param('~max_depth', 10.0)
        self.min_points = rospy.get_param('~min_points', 30)
        self.min_rate = rospy.get_param('~min_interval', 0.0)
        self.last_stamp = None

        requested = rospy.get_param('~backend', 'auto')
        backend = resolve_backend(requested)
        model_path = self.model_path(backend)
        rospy.loginfo('[%s] backend %s (%s)', rospy.get_name(), backend,
                      requested)
        self.detector = create_detector(
            backend, model_path, classes, rospy.get_param('~device', 'auto'))
        self.classes = self.detector.classes
        rospy.loginfo('[%s] %s with %s on %s, classes %s', rospy.get_name(),
                      model_path, backend, self.detector.device, self.classes)

        self.pub_image = rospy.Publisher('~output/image', Image, queue_size=1)
        self.pub_rects = rospy.Publisher(
            '~output/rects', RectArray, queue_size=1)
        self.pub_class = rospy.Publisher(
            '~output/class', ClassificationResult, queue_size=1)
        self.pub_indices = rospy.Publisher(
            '~output/cluster_indices', ClusterPointIndices, queue_size=1)

        queue_size = rospy.get_param('~queue_size', 5)
        slop = rospy.get_param('~slop', 0.2)
        image_sub = message_filters.Subscriber('~input/image', Image)
        depth_sub = message_filters.Subscriber('~input/depth', Image)
        sync = message_filters.ApproximateTimeSynchronizer(
            [image_sub, depth_sub], queue_size, slop)
        sync.registerCallback(self.callback)

    @staticmethod
    def model_path(backend):
        """Return ``~model``, or the default model of a backend."""
        model_path = rospy.get_param('~model', '')
        if model_path:
            return model_path
        if backend == 'adla':
            model_path = rospy.get_param('~adla_model')
            if not os.path.isabs(model_path):
                model_path = os.path.join(
                    rospkg.RosPack().get_path('grape_detector'), 'models',
                    model_path)
            return model_path
        return rospy.get_param('~ultralytics_model')

    def callback(self, image_msg, depth_msg):
        if self.min_rate > 0.0 and self.last_stamp is not None \
           and (image_msg.header.stamp - self.last_stamp).to_sec() \
           < self.min_rate:
            return
        self.last_stamp = image_msg.header.stamp
        image = image_to_numpy(image_msg)
        depth = depth_to_numpy(depth_msg)
        if depth.shape[:2] != image.shape[:2]:
            rospy.logwarn_throttle(
                10.0, '[%s] the depth image %s is not aligned to the color '
                'image %s; use aligned_depth_to_color', rospy.get_name(),
                depth.shape[:2], image.shape[:2])
            return

        detections = self.detector.detect(
            image, self.conf, self.iou, self.max_detections,
            self.agnostic_nms)

        rects = RectArray(header=image_msg.header)
        classification = ClassificationResult(
            header=image_msg.header, target_names=self.classes)
        cluster_indices = ClusterPointIndices(header=depth_msg.header)
        height, width = image.shape[:2]
        for box, score, label in zip(detections.boxes, detections.scores,
                                     detections.labels):
            x1, y1, x2, y2 = (int(round(v)) for v in box)
            x1, x2 = max(0, x1), min(width, x2)
            y1, y2 = max(0, y1), min(height, y2)
            if x2 <= x1 or y2 <= y1:
                continue
            rects.rects.append(
                Rect(x=x1, y=y1, width=x2 - x1, height=y2 - y1))
            classification.labels.append(int(label))
            classification.label_names.append(self.classes[label])
            classification.label_proba.append(float(score))
            indices = self.box_indices(depth, x1, y1, x2, y2, width)
            cluster_indices.cluster_indices.append(
                PointIndices(header=depth_msg.header, indices=indices))

        self.pub_rects.publish(rects)
        self.pub_class.publish(classification)
        self.pub_indices.publish(cluster_indices)
        if self.pub_image.get_num_connections() > 0:
            self.pub_image.publish(numpy_to_image(
                draw_detections(image, detections, self.classes),
                image_msg.header, image_msg.encoding))

    def box_indices(self, depth, x1, y1, x2, y2, width):
        """Return the indices of the points of a bunch inside a box.

        The points of the box whose depth is within ``depth_margin`` behind
        the front of the box; the canopy behind the bunch is farther away.

        Parameters
        ----------
        depth : numpy.ndarray
            Depth image [m], 0 where invalid.
        x1, y1, x2, y2 : int
            The box in the image.
        width : int
            Width of the image, to index the organized point cloud.

        Returns
        -------
        list of int
            Indices of the points, empty if the box has too few of them.
        """
        patch = depth[y1:y2, x1:x2]
        valid = (patch >= self.min_depth) & (patch <= self.max_depth)
        if valid.sum() < self.min_points:
            return []
        front = np.quantile(patch[valid], self.depth_quantile)
        near = valid & (patch <= front + self.depth_margin) \
            & (patch >= front - self.depth_margin)
        if near.sum() < self.min_points:
            return []
        rows, columns = np.nonzero(near)
        return ((rows + y1) * width + (columns + x1)).astype(np.int32).tolist()


def main():
    rospy.init_node('grape_detector')
    GrapeDetector()
    rospy.spin()


if __name__ == '__main__':
    main()
