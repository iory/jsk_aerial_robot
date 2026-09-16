# -*- coding: utf-8 -*-
"""Open vocabulary detectors of grape bunches.

Each detector takes a BGR image and returns the boxes, the scores and the
classes of what it found; the classes are text prompts, so nothing is trained.

* ``UltralyticsDetector`` runs YOLO-World or YOLOE with torch, on a gpu or the
  cpu, and sets the prompts when it starts.
* ``AdlaDetector`` runs a YOLOE converted to .adla on the NPU of a Khadas VIM4.
  The prompts are fixed in the model when it is converted
  (scripts/export_adla_model.py), and written next to it in a yaml file.
"""

import os

import numpy as np
import yaml

from grape_detector import yolo_head


class Detections(object):
    """Detections in an image.

    Attributes
    ----------
    boxes : numpy.ndarray
        ``(N, 4)`` x1, y1, x2, y2 in the image.
    scores : numpy.ndarray
        ``(N,)``.
    labels : numpy.ndarray
        ``(N,)`` index of the class of each box.
    """

    def __init__(self, boxes, scores, labels):
        self.boxes = np.asarray(boxes, dtype=np.float32).reshape(-1, 4)
        self.scores = np.asarray(scores, dtype=np.float32).reshape(-1)
        self.labels = np.asarray(labels, dtype=np.int64).reshape(-1)


class UltralyticsDetector(object):
    """YOLO-World or YOLOE of ultralytics.

    Parameters
    ----------
    model_path : str
        Weights, e.g. ``yolov8x-worldv2.pt`` or ``yoloe-v8l-seg.pt``; a
        name alone is downloaded by ultralytics.
    classes : list of str
        Text prompts.
    device : str
        ``auto`` (the gpu if torch sees one), ``cpu`` or ``cuda:N``.
    """

    def __init__(self, model_path, classes, device='auto'):
        import torch
        from ultralytics import YOLO
        if device == 'auto':
            # the venv has a cpu-only torch on a machine built without a gpu
            device = 'cuda:0' if torch.cuda.is_available() else 'cpu'
        self.classes = list(classes)
        # a yoloe or a -world model by its file name; both take the prompts
        self.model = YOLO(model_path)
        self.model.set_classes(list(self.classes))
        self.model.to(device)
        self.device = device

    def detect(self, image, conf, iou, max_detections, agnostic):
        """Detect in a BGR image; see ``yolo_head.select`` for the arguments.

        Returns
        -------
        Detections
        """
        result = self.model.predict(
            source=image, conf=conf, iou=iou, max_det=max_detections,
            agnostic_nms=agnostic, device=self.device, verbose=False)[0]
        boxes = result.boxes
        return Detections(boxes.xyxy.cpu().numpy(), boxes.conf.cpu().numpy(),
                          boxes.cls.cpu().numpy())


def sidecar_path(model_path):
    """The yaml file that describes a converted model."""
    return os.path.splitext(model_path)[0] + '.yaml'


class AdlaDetector(object):
    """A YOLOE converted to .adla, on the NPU of a Khadas VIM4.

    Parameters
    ----------
    model_path : str
        The .adla file. ``<model>.yaml`` next to it has ``classes`` (the
        prompts baked into the model), ``imgsz`` and ``box_channels`` (4, or
        ``4 * reg_max``).
    classes : list of str or None
        The prompts the node is configured with; they have to be the ones
        of the model, which can not change them.
    """

    def __init__(self, model_path, classes=None):
        from grape_detector.adla import AdlaModel
        for path in (model_path, sidecar_path(model_path)):
            if not os.path.exists(path):
                raise IOError(
                    '{} is missing; the models of the release are downloaded '
                    'when grape_detector is built on a VIM4 (or with '
                    '-DGRAPE_DETECTOR_DOWNLOAD_ADLA_MODELS=ON), others are '
                    'made by scripts/export_adla_model.py'.format(path))
        with open(sidecar_path(model_path)) as f:
            meta = yaml.safe_load(f)
        self.classes = list(meta['classes'])
        if classes is not None and list(classes) != self.classes:
            raise ValueError(
                'the prompts {} are not the ones {} was converted with ({}); '
                'convert the model again with scripts/export_adla_model.py'
                .format(list(classes), model_path, self.classes))
        self.size = int(meta['imgsz'])
        self.box_channels = int(meta['box_channels'])
        if self.box_channels == len(self.classes):
            raise ValueError(
                'the box and the class outputs of {} have the same number of '
                'channels ({}) and can not be told apart'.format(
                    model_path, self.box_channels))
        self.model = AdlaModel(model_path)
        self.device = 'npu'

    def raw_outputs(self, outputs):
        """Arrange the flat outputs of the NPU as the six outputs of the head.

        The outputs are matched by their sizes, which are unique.
        """
        raw = []
        for stride in yolo_head.STRIDES:
            cells = self.size // stride
            for channels in (self.box_channels, len(self.classes)):
                match = [o for o in outputs if o.size == channels * cells ** 2]
                if len(match) != 1:
                    raise RuntimeError(
                        'the model has {} outputs of {} x {} x {}; is it made '
                        'by export_adla_model.py for {} classes?'.format(
                            len(match), channels, cells, cells,
                            len(self.classes)))
                raw.append(match[0].reshape(channels, cells, cells))
        return raw

    def detect(self, image, conf, iou, max_detections, agnostic):
        """Detect in a BGR image; see ``yolo_head.select`` for the arguments.

        Returns
        -------
        Detections
        """
        rgb, scale, offset = yolo_head.letterbox(image, self.size)
        outputs = self.model.run(rgb)
        boxes, scores = yolo_head.decode(
            self.raw_outputs(outputs), self.size, conf)
        boxes, scores, labels = yolo_head.select(
            boxes, scores, conf, iou, max_detections, agnostic)
        boxes = (boxes - np.tile(offset, 2)) / scale
        height, width = image.shape[:2]
        boxes[:, 0::2] = boxes[:, 0::2].clip(0, width)
        boxes[:, 1::2] = boxes[:, 1::2].clip(0, height)
        return Detections(boxes, scores, labels)


BACKENDS = ('ultralytics', 'adla')


def resolve_backend(backend):
    """Return the backend to run with.

    Parameters
    ----------
    backend : str
        ``auto``, ``ultralytics`` or ``adla``. ``auto`` is ``adla`` on a
        machine with the NPU of a Khadas VIM4 and its runtime, and
        ``ultralytics`` otherwise.

    Returns
    -------
    str
        ``ultralytics`` or ``adla``.
    """
    if backend == 'auto':
        from grape_detector import adla
        return 'adla' if adla.available() else 'ultralytics'
    if backend not in BACKENDS:
        raise ValueError('unknown backend {}, choose auto or one of {}'.format(
            backend, BACKENDS))
    return backend


def create_detector(backend, model_path, classes, device):
    """Return the detector of a backend.

    Parameters
    ----------
    backend : str
        ``ultralytics`` or ``adla``, see ``resolve_backend``.
    model_path : str
        Weights of the model.
    classes : list of str
        Text prompts.
    device : str
        Device of the ultralytics backend; the adla backend runs on the NPU.
    """
    if backend == 'ultralytics':
        return UltralyticsDetector(model_path, classes, device)
    if backend == 'adla':
        return AdlaDetector(model_path, classes)
    raise ValueError('unknown backend {}, choose one of {}'.format(
        backend, BACKENDS))
