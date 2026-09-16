# -*- coding: utf-8 -*-
"""Pre and post processing of a YOLO whose head is cut before the box decoding.

An NPU compiler may not convert the decoding at the end of a YOLO head (the
anchors and the strides are constants it does not handle), so the model is cut
at the last convolution of each level of the head, and what the head would do
after it is done here in numpy.
"""

import cv2
import numpy as np

STRIDES = (8, 16, 32)
LETTERBOX_COLOR = 114


def raw_output_names(model):
    """Return the last convolutions of a YOLO head in an ONNX graph.

    Parameters
    ----------
    model : onnx.ModelProto
        A YOLOv8-style model exported by ultralytics.

    Returns
    -------
    list of str
        ``[box0, cls0, box1, cls1, box2, cls2]``, the box and the class
        outputs of each level, from the finest level to the coarsest.
    """
    produced = set()
    for node in model.graph.node:
        produced.update(node.output)
    heads = sorted({name.split('/')[1] for name in produced
                    if name.startswith('/model.')
                    and '/cv2.0/cv2.0.2/Conv_output_0' in name})
    if len(heads) != 1:
        raise RuntimeError(
            'expected one detection head, found {}'.format(heads))
    names = []
    for level in range(len(STRIDES)):
        for branch in ('cv2', 'cv3'):
            names.append('/{0}/{1}.{2}/{1}.{2}.2/Conv_output_0'.format(
                heads[0], branch, level))
    missing = [name for name in names if name not in produced]
    if missing:
        raise RuntimeError('not in the graph: {}'.format(missing))
    return names


def letterbox(image, size):
    """Fit an image into a square the way ultralytics does.

    Parameters
    ----------
    image : numpy.ndarray
        ``(H, W, 3)`` BGR image.
    size : int
        Side of the input of the model.

    Returns
    -------
    rgb : numpy.ndarray
        ``(size, size, 3)`` uint8 RGB image, the image scaled and centered.
    scale : float
        Scale from the image to ``rgb``.
    offset : numpy.ndarray
        ``(x, y)`` of the image in ``rgb``.
    """
    height, width = image.shape[:2]
    scale = min(float(size) / height, float(size) / width)
    new_height = int(round(height * scale))
    new_width = int(round(width * scale))
    top = (size - new_height) // 2
    left = (size - new_width) // 2
    canvas = np.full((size, size, 3), LETTERBOX_COLOR, dtype=np.uint8)
    canvas[top:top + new_height, left:left + new_width] = cv2.resize(
        image, (new_width, new_height), interpolation=cv2.INTER_LINEAR)
    rgb = np.ascontiguousarray(canvas[:, :, ::-1])
    return rgb, scale, np.array([left, top], dtype=np.float32)


def sigmoid(x):
    return 1.0 / (1.0 + np.exp(-x))


def decode(raw, size, conf=0.0):
    """Decode the outputs of a cut YOLO head.

    Parameters
    ----------
    raw : list of numpy.ndarray
        ``[box0, cls0, box1, cls1, box2, cls2]``, each ``(C, H, W)``. The
        box outputs have 4 channels, or ``4 * reg_max`` of a distribution
        focal loss head.
    size : int
        Side of the input of the model.
    conf : float
        Only the anchors whose best score reaches this are decoded; the
        boxes of a distribution focal loss head are the most of the work.

    Returns
    -------
    boxes : numpy.ndarray
        ``(N, 4)`` x1, y1, x2, y2 in the input of the model.
    scores : numpy.ndarray
        ``(N, classes)`` score of each class.
    """
    boxes = []
    scores = []
    for level, stride in enumerate(STRIDES):
        cls = raw[2 * level + 1]
        score = sigmoid(cls.reshape(cls.shape[0], -1)).T
        keep = np.nonzero(score.max(axis=1) >= conf)[0]
        box = raw[2 * level].reshape(raw[2 * level].shape[0], -1)[:, keep]
        if box.shape[0] != 4:
            # the expectation of the distribution over reg_max bins of each side
            reg_max = box.shape[0] // 4
            logits = box.reshape(4, reg_max, -1)
            e = np.exp(logits - logits.max(axis=1, keepdims=True))
            probability = e / e.sum(axis=1, keepdims=True)
            box = (probability * np.arange(reg_max).reshape(1, reg_max, 1)
                   ).sum(axis=1)
        cells = size // stride
        anchor_x = keep % cells + 0.5
        anchor_y = keep // cells + 0.5
        boxes.append(np.stack([anchor_x - box[0], anchor_y - box[1],
                               anchor_x + box[2], anchor_y + box[3]],
                              axis=1) * stride)
        scores.append(score[keep])
    return np.concatenate(boxes), np.concatenate(scores)


def nms(boxes, scores, iou):
    """Return the indices of the boxes kept by non maximum suppression.

    Parameters
    ----------
    boxes : numpy.ndarray
        ``(N, 4)`` x1, y1, x2, y2.
    scores : numpy.ndarray
        ``(N,)``.
    iou : float
        A box overlapping a better one by more than this is removed.

    Returns
    -------
    numpy.ndarray
        Indices of the kept boxes, best first.
    """
    # stable: equal scores (common in a quantized model) keep the order of the anchors
    order = np.argsort(-scores, kind='stable')
    areas = np.prod(np.maximum(boxes[:, 2:] - boxes[:, :2], 0.0), axis=1)
    keep = []
    while order.size > 0:
        best = order[0]
        keep.append(best)
        rest = order[1:]
        top_left = np.maximum(boxes[best, :2], boxes[rest, :2])
        bottom_right = np.minimum(boxes[best, 2:], boxes[rest, 2:])
        inter = np.prod(np.maximum(bottom_right - top_left, 0.0), axis=1)
        overlap = inter / (areas[best] + areas[rest] - inter + 1e-9)
        order = rest[overlap <= iou]
    return np.array(keep, dtype=np.int64)


def select(boxes, scores, conf, iou, max_detections, agnostic):
    """Pick the detections from the decoded outputs.

    Parameters
    ----------
    boxes : numpy.ndarray
        ``(N, 4)`` x1, y1, x2, y2.
    scores : numpy.ndarray
        ``(N, classes)``.
    conf : float
        Minimum score of a detection.
    iou : float
        Threshold of the non maximum suppression.
    max_detections : int
        Most detections returned.
    agnostic : bool
        Suppress overlapping boxes of different classes as well.

    Returns
    -------
    boxes : numpy.ndarray
        ``(M, 4)``.
    scores : numpy.ndarray
        ``(M,)``.
    labels : numpy.ndarray
        ``(M,)`` class of each detection.
    """
    labels = scores.argmax(axis=1)
    best = scores[np.arange(len(labels)), labels]
    candidates = np.nonzero(best >= conf)[0]
    boxes = boxes[candidates]
    best = best[candidates]
    labels = labels[candidates]
    if len(candidates) == 0:
        return boxes, best, labels
    if agnostic:
        keep = nms(boxes, best, iou)
    else:
        # shift each class away from the others, as ultralytics does
        offset = labels[:, None].astype(boxes.dtype) * (boxes.max() + 1.0)
        keep = nms(boxes + offset, best, iou)
    keep = keep[:max_detections]
    return boxes[keep], best[keep], labels[keep]
