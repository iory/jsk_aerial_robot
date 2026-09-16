#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Feed a recorded video to grape_detection.launch and write what it detects.

Each frame is published as the color image of a camera, with a flat depth image
and a pinhole camera info, and the next frame is sent only after the detector
has answered, so every frame is detected whatever the speed of the detector.
The detection images are written into a video, and the boxes of each frame into
a json file.

The depth is flat, so the 3d boxes only show that the pipeline runs. Use a
namespace that no real camera publishes in, and a master of its own when a
robot is running::

    export ROS_MASTER_URI=http://localhost:11399
    roscore -p 11399 &
    roslaunch grape_detector grape_detection.launch backend:=adla \\
        camera_ns:=grape_replay/camera min_interval:=0.0 &
    rosrun grape_detector replay_video.py grape.mp4 --focal-length <fx> \\
        --output detected.mp4 --result detected.json

``<fx>`` is the focal length [px] in the camera_info of the camera that
recorded the video.
"""

import argparse
import json
import time

import cv2
import numpy as np
import rospy
from jsk_recognition_msgs.msg import ClassificationResult
from jsk_recognition_msgs.msg import RectArray
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image


def image_message(frame, stamp, frame_id):
    """Return a BGR frame as an rgb8 sensor_msgs/Image."""
    height, width = frame.shape[:2]
    msg = Image(height=height, width=width, encoding='rgb8', step=3 * width,
                data=np.ascontiguousarray(frame[:, :, ::-1]).tobytes())
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    return msg


def camera_info(header, width, height, focal_length):
    """Return the info of a pinhole camera centered in the image."""
    cx = width / 2.0
    cy = height / 2.0
    return CameraInfo(
        header=header, width=width, height=height,
        distortion_model='plumb_bob', D=[0.0] * 5,
        K=[focal_length, 0.0, cx, 0.0, focal_length, cy, 0.0, 0.0, 1.0],
        R=[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
        P=[focal_length, 0.0, cx, 0.0, 0.0, focal_length, cy, 0.0,
           0.0, 0.0, 1.0, 0.0])


class Replay(object):
    """Publish frames and wait for the answer of the detector.

    Parameters
    ----------
    camera_ns : str
        Namespace of the camera topics the detector reads.
    detector_ns : str
        Namespace of the outputs of grape_detector_node.py.
    """

    def __init__(self, camera_ns, detector_ns):
        self.pub_image = rospy.Publisher(
            camera_ns + '/color/image_raw', Image, queue_size=1)
        self.pub_info = rospy.Publisher(
            camera_ns + '/color/camera_info', CameraInfo, queue_size=1)
        self.pub_depth = rospy.Publisher(
            camera_ns + '/aligned_depth_to_color/image_raw', Image,
            queue_size=1)
        self.latest = {}
        for key, topic, cls in (
                ('image', detector_ns + '/output/image', Image),
                ('rects', detector_ns + '/output/rects', RectArray),
                ('class', detector_ns + '/output/class',
                 ClassificationResult)):
            rospy.Subscriber(topic, cls, self.store, key)

    def store(self, msg, key):
        self.latest[key] = msg

    def wait_for_detector(self, timeout):
        """Wait until the detector subscribes to the camera."""
        deadline = time.time() + timeout
        while self.pub_image.get_num_connections() == 0 \
                or self.pub_depth.get_num_connections() == 0:
            if rospy.is_shutdown() or time.time() > deadline:
                raise RuntimeError(
                    'nothing subscribes to {} within {} s'.format(
                        self.pub_image.name, timeout))
            time.sleep(0.1)

    def answered(self, stamp):
        """The detection of the frame of ``stamp``, or None."""
        image = self.latest.get('image')
        rects = self.latest.get('rects')
        classes = self.latest.get('class')
        if image is None or rects is None or classes is None:
            return None
        if not (image.header.stamp == stamp and rects.header.stamp == stamp
                and classes.header.stamp == stamp):
            return None
        return image, rects, classes

    def detect(self, image, info, depth, timeout, resend):
        """Publish a frame until the detector answers it.

        The synchronizers of the detector need the image and the depth of
        the same stamp, and may drop the first ones, so the frame is sent
        again every ``resend`` seconds.

        Returns
        -------
        tuple or None
            ``(image, rects, classes)`` of the detector, None on timeout.
        """
        stamp = image.header.stamp
        deadline = time.time() + timeout
        while not rospy.is_shutdown() and time.time() < deadline:
            self.pub_info.publish(info)
            self.pub_depth.publish(depth)
            self.pub_image.publish(image)
            resend_at = time.time() + resend
            while time.time() < resend_at:
                result = self.answered(stamp)
                if result is not None:
                    return result
                time.sleep(0.005)
        return None


def main():
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('video', help='the video to feed')
    parser.add_argument('--output', required=True,
                        help='video of the detection images to write')
    parser.add_argument('--result', default=None,
                        help='json of the boxes of each frame to write')
    parser.add_argument('--camera-ns', default='/grape_replay/camera',
                        help='camera_ns of grape_detection.launch, with a '
                        'leading slash; not the one of a real camera')
    parser.add_argument('--detector-ns', default='/grape_detector')
    parser.add_argument('--frame-id', default='camera_color_optical_frame')
    parser.add_argument('--depth', type=float, default=1.0,
                        help='[m] of the flat depth image')
    parser.add_argument('--focal-length', type=float, required=True,
                        help='[px] of the camera info, e.g. the fx of the '
                        'camera_info of the camera that recorded the video')
    parser.add_argument('--max-frames', type=int, default=None,
                        help='stop after this many frames')
    parser.add_argument('--timeout', type=float, default=10.0,
                        help='[s] to wait for the detection of a frame')
    parser.add_argument('--resend', type=float, default=0.5,
                        help='[s] between two sends of a frame')
    args = parser.parse_args(rospy.myargv()[1:])

    rospy.init_node('grape_replay_video')
    capture = cv2.VideoCapture(args.video)
    if not capture.isOpened():
        raise RuntimeError('can not open {}'.format(args.video))
    fps = capture.get(cv2.CAP_PROP_FPS)
    total = int(capture.get(cv2.CAP_PROP_FRAME_COUNT))
    width = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
    writer = cv2.VideoWriter(args.output, cv2.VideoWriter_fourcc(*'mp4v'),
                             fps, (width, height))
    depth_mm = np.full((height, width), int(round(args.depth * 1000.0)),
                       dtype=np.uint16).tobytes()

    replay = Replay(args.camera_ns, args.detector_ns)
    replay.wait_for_detector(args.timeout)
    frames = []
    start = time.time()
    # stop with ctrl-c: what is done so far is still written
    while not rospy.is_shutdown():
        if args.max_frames is not None and len(frames) >= args.max_frames:
            break
        ok, frame = capture.read()
        if not ok:
            break
        index = len(frames)
        # the frames are stamped by their time in the video
        stamp = rospy.Time.from_sec(1.0 + index / fps)
        image = image_message(frame, stamp, args.frame_id)
        depth = Image(height=height, width=width, encoding='16UC1',
                      step=2 * width, data=depth_mm)
        depth.header = image.header
        info = camera_info(image.header, width, height, args.focal_length)
        sent = time.time()
        result = replay.detect(image, info, depth, args.timeout, args.resend)
        if result is None and rospy.is_shutdown():
            break
        if result is None:
            rospy.logwarn('frame %d: no detection within %.1f s', index,
                          args.timeout)
            writer.write(frame)
            frames.append({'frame': index, 'boxes': None})
            continue
        detected, rects, classes = result
        drawn = np.frombuffer(detected.data, np.uint8).reshape(
            height, width, 3)
        if detected.encoding == 'rgb8':
            drawn = drawn[:, :, ::-1]
        writer.write(np.ascontiguousarray(drawn))
        frames.append({
            'frame': index,
            'seconds': round(time.time() - sent, 3),
            'boxes': [[r.x, r.y, r.width, r.height] for r in rects.rects],
            'labels': list(classes.label_names),
            'scores': [round(p, 3) for p in classes.label_proba]})
        if len(frames) % 100 == 0:
            rospy.loginfo('%d / %d frames, %.1f frames/s', len(frames), total,
                          len(frames) / (time.time() - start))
    writer.release()
    if args.result is not None:
        with open(args.result, 'w') as f:
            json.dump(frames, f)
    answered = [f for f in frames if f['boxes'] is not None]
    # print: rospy may be shut down already
    print('{} / {} frames, {} answered, {} with a detection, {:.2f} frames/s; '
          'wrote {}'.format(len(frames), total, len(answered),
                            sum(1 for f in answered if f['boxes']),
                            len(frames) / max(time.time() - start, 1e-9),
                            args.output), flush=True)


if __name__ == '__main__':
    main()
