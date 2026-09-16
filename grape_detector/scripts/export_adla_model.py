#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Convert a YOLOE to an .adla model for the NPU of a Khadas VIM4.

Run on a development PC with docker, not on the VIM4:

1. The prompts of the config are set in the model and it is exported to ONNX.
2. The ONNX is cut at the last convolutions of the head: the NPU toolkit can
   not convert the box decoding, which grape_detector does in numpy instead.
3. The images of ``--calibration`` are letterboxed for the quantization.
4. adla_convert of the VIM4 NPU SDK converts the model in the docker image of
   Khadas.
5. The .adla is written with a yaml file next to it, which tells
   grape_detector the prompts and the layout of the model.

The SDK is https://github.com/khadas/vim4_npu_sdk (git lfs). Its branch has to
match the NPU driver of the VIM4: ``npu-ddk-1.7.5.5`` for a kernel older than
the 241129 image, the default branch otherwise (see its README). Check the
driver with ``dmesg | grep -i adla`` on the VIM4.

Only a YOLOE without attention runs on the NPU of the 1.7.5.5 driver: the
yoloe-v8 models do, yoloe-11 and yoloe-26, which have attention, do not.

Example::

    rosrun grape_detector export_adla_model.py \\
        --model yoloe-v8l-seg.pt --sdk ~/vim4_npu_sdk \\
        --calibration ~/grape_images ~/grape.mp4 \\
        --output ~/grape_models/yoloe-v8l_640_int8.adla
"""

import argparse
import glob
import os
import shutil
import subprocess
import tempfile

import cv2
import numpy as np
import yaml

from grape_detector import yolo_head
from grape_detector.detectors import sidecar_path

IMAGE_EXTENSIONS = ('.jpg', '.jpeg', '.png', '.bmp')
# operations the NPU of the 1.7.5.5 driver fails on at runtime
UNSUPPORTED_OPS = ('MatMul', 'Softmax')
SDK_WORKDIR = '/home/khadas/npu'
CALIBRATION_WORKDIR = '/calibration'


def default_config():
    import rospkg
    return os.path.join(rospkg.RosPack().get_path('grape_detector'),
                        'config', 'GrapeDetector.yaml')


def export_onnx(model_path, classes, size, workdir):
    """Export a YOLOE with the prompts set, and return the ONNX path."""
    from ultralytics import YOLO
    model = YOLO(model_path)
    model.set_classes(list(classes))
    exported = model.export(format='onnx', imgsz=size, simplify=True)
    path = os.path.join(workdir, 'full.onnx')
    shutil.move(exported, path)
    return path


def cut_head(full_path, workdir):
    """Cut the ONNX at the last convolutions of the head, and return its path."""
    import onnx
    from onnx.utils import extract_model
    names = yolo_head.raw_output_names(onnx.load(full_path))
    path = os.path.join(workdir, 'raw.onnx')
    extract_model(full_path, path, ['images'], names)
    model = onnx.load(path)
    ops = sorted({node.op_type for node in model.graph.node}
                 & set(UNSUPPORTED_OPS))
    if ops:
        print('WARNING: the model has {}, which the NPU of the 1.7.5.5 '
              'driver fails to run; use a yoloe-v8 model with that driver'
              .format(ops), flush=True)
    return path


def check_cut(full_path, raw_path, image, size, classes):
    """Compare the decoded outputs of the cut model with the full model.

    The full model of a YOLOE-v8 ends with ``(1, 4 + classes + masks, N)``
    of center, size and scores; an end-to-end model has another output and is
    not compared.

    Returns
    -------
    int
        Channels of the box outputs of the cut model.
    """
    import onnxruntime
    rgb, _, _ = yolo_head.letterbox(image, size)
    blob = rgb.transpose(2, 0, 1)[None].astype(np.float32) / 255.0
    full = onnxruntime.InferenceSession(
        full_path, providers=['CPUExecutionProvider']).run(
            None, {'images': blob})[0][0]
    raw = onnxruntime.InferenceSession(
        raw_path, providers=['CPUExecutionProvider']).run(
            None, {'images': blob})
    if raw[1].shape[1] != len(classes):
        raise RuntimeError('the cut model has {} classes, not {}'.format(
            raw[1].shape[1], len(classes)))
    box_channels = raw[0].shape[1]
    boxes, scores = yolo_head.decode([r[0] for r in raw], size)
    if full.ndim != 2 or full.shape[1] != len(boxes):
        print('the full model ends with {}; the cut is not compared'.format(
            full.shape), flush=True)
        return box_channels
    center = full[:2].T
    half = full[2:4].T / 2.0
    full_boxes = np.concatenate([center - half, center + half], axis=1)
    full_scores = full[4:4 + len(classes)].T
    box_error = float(np.abs(full_boxes - boxes).max())
    score_error = float(np.abs(full_scores - scores).max())
    print('the cut model decodes to the full model within {:.2g} px and '
          '{:.2g} of the scores'.format(box_error, score_error), flush=True)
    if box_error > 0.01 or score_error > 1e-4:
        raise RuntimeError('the decoding of the cut model is wrong')
    return box_channels


def calibration_images(sources, count):
    """Return ``count`` images spread over image files, directories and videos."""
    images = []
    videos = []
    for source in sources:
        if os.path.isdir(source):
            images.extend(sorted(
                path for path in glob.glob(os.path.join(source, '*'))
                if path.lower().endswith(IMAGE_EXTENSIONS)))
        elif source.lower().endswith(IMAGE_EXTENSIONS):
            images.append(source)
        else:
            videos.append(source)
    frames = [cv2.imread(path) for path in images]
    if videos:
        per_video = max(1, (count - len(frames)) // len(videos))
        for video in videos:
            frames.extend(video_frames(video, per_video))
    frames = [frame for frame in frames if frame is not None]
    if not frames:
        raise RuntimeError('no image in {}'.format(sources))
    picked = np.linspace(0, len(frames) - 1, min(count, len(frames)))
    return [frames[int(round(i))] for i in picked]


def video_frames(path, count):
    capture = cv2.VideoCapture(path)
    total = int(capture.get(cv2.CAP_PROP_FRAME_COUNT))
    if total <= 0:
        raise RuntimeError('can not read the video {}'.format(path))
    frames = []
    for index in np.linspace(0, total - 1, min(count, total)):
        capture.set(cv2.CAP_PROP_POS_FRAMES, int(index))
        ok, frame = capture.read()
        if ok:
            frames.append(frame)
    capture.release()
    return frames


def write_calibration(images, size, directory):
    """Write letterboxed images and the list of them in the container."""
    os.makedirs(directory)
    lines = []
    for i, image in enumerate(images):
        rgb, _, _ = yolo_head.letterbox(image, size)
        name = 'calib_{:04d}.jpg'.format(i)
        cv2.imwrite(os.path.join(directory, name), rgb[:, :, ::-1])
        lines.append('{}/images/{}'.format(CALIBRATION_WORKDIR, name))
    with open(os.path.join(directory, '..', 'dataset.txt'), 'w') as f:
        f.write('\n'.join(lines) + '\n')


def convert(args, raw_path, workdir):
    """Run adla_convert in docker and return the .adla path."""
    toolkit = os.path.join(args.sdk, 'adla-toolkit-binary', 'bin',
                           'adla_convert')
    if not os.path.exists(toolkit):
        raise RuntimeError(
            '{} is missing; clone the sdk with git lfs'.format(toolkit))
    command = [
        'docker', 'run', '--rm',
        '--user', '{}:{}'.format(os.getuid(), os.getgid()),
        '-e', 'HOME=/tmp',
        '-v', '{}:{}'.format(os.path.abspath(args.sdk), SDK_WORKDIR),
        '-v', '{}:{}'.format(workdir, CALIBRATION_WORKDIR),
        '-w', CALIBRATION_WORKDIR,
        args.docker_image,
        SDK_WORKDIR + '/adla-toolkit-binary/bin/adla_convert',
        '--model-type', 'onnx',
        '--model', CALIBRATION_WORKDIR + '/' + os.path.basename(raw_path),
        '--inputs', 'images',
        '--input-shapes', '3,{0},{0}'.format(args.imgsz),
        '--dtypes', 'float32',
        '--quantize-dtype', args.quantize,
        # the model takes uint8 rgb and divides it by 255 itself
        '--channel-mean-value', '0,0,0,255',
        '--source-file', CALIBRATION_WORKDIR + '/dataset.txt',
        '--batch-size', '1',
        '--target-platform', args.target_platform,
        '--outdir', CALIBRATION_WORKDIR + '/adla',
    ]
    print(' '.join(command), flush=True)
    subprocess.check_call(command)
    outputs = glob.glob(os.path.join(workdir, 'adla', '*.adla'))
    if len(outputs) != 1:
        raise RuntimeError('adla_convert wrote {}'.format(outputs))
    return outputs[0]


def main():
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--model', required=True,
                        help='YOLOE weights, e.g. yoloe-v8l-seg.pt')
    parser.add_argument('--sdk', required=True,
                        help='checkout of khadas/vim4_npu_sdk')
    parser.add_argument('--calibration', nargs='+', required=True,
                        help='images, directories of images or videos of '
                        'the scene, for the quantization')
    parser.add_argument('--output', required=True, help='the .adla to write')
    parser.add_argument('--config', default=None,
                        help='config with the prompts in "classes" '
                        '(default: config/GrapeDetector.yaml)')
    parser.add_argument('--imgsz', type=int, default=640)
    parser.add_argument('--num-calibration', type=int, default=50)
    parser.add_argument('--quantize', default='int8',
                        choices=('int8', 'int16'))
    parser.add_argument('--docker-image', default='numbqq/npu-vim4')
    parser.add_argument('--target-platform', default='PRODUCT_PID0XA003',
                        help='the platform of the VIM4 (A311D2)')
    parser.add_argument('--keep-workdir', action='store_true')
    args = parser.parse_args()

    if not args.output.endswith('.adla'):
        parser.error('--output has to end with .adla')
    config = args.config if args.config is not None else default_config()
    with open(config) as f:
        classes = list(yaml.safe_load(f)['classes'])

    workdir = tempfile.mkdtemp(prefix='grape_adla_')
    try:
        full_path = export_onnx(args.model, classes, args.imgsz, workdir)
        raw_path = cut_head(full_path, workdir)
        images = calibration_images(args.calibration, args.num_calibration)
        box_channels = check_cut(full_path, raw_path, images[0], args.imgsz,
                                 classes)
        write_calibration(images, args.imgsz,
                          os.path.join(workdir, 'images'))
        adla_path = convert(args, raw_path, workdir)

        output = os.path.abspath(args.output)
        if not os.path.isdir(os.path.dirname(output)):
            os.makedirs(os.path.dirname(output))
        shutil.copyfile(adla_path, output)
        meta = dict(classes=classes, imgsz=args.imgsz,
                    box_channels=box_channels,
                    source=os.path.basename(args.model),
                    quantize=args.quantize,
                    target_platform=args.target_platform)
        with open(sidecar_path(output), 'w') as f:
            yaml.safe_dump(meta, f, default_flow_style=False,
                           allow_unicode=True)
        print('wrote {} and {}'.format(output, sidecar_path(output)),
              flush=True)
    finally:
        if args.keep_workdir:
            print('the work files are in {}'.format(workdir), flush=True)
        else:
            shutil.rmtree(workdir)


if __name__ == '__main__':
    main()
