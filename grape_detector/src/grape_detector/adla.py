# -*- coding: utf-8 -*-
"""Run an .adla model on the NPU of an Amlogic board (Khadas VIM4).

A thin ctypes binding of the aml_nnsdk (libnnsdk.so of the board), so that no
python package of the vendor is needed. The structures follow nn_sdk.h of
https://github.com/khadas/vim4_npu_applications (include/nn_sdk.h).
"""

import ctypes
import ctypes.util

import numpy as np

MAX_NAME_LENGTH = 64
OUTPUT_MAX_NUM = 32
INPUT_CHANNEL = 3
ADDRESS_MAX_NUM = 16

# enums of nn_sdk.h
ADLA_LOADABLE = 7            # amlnn_model_type
NN_ADLA_FILE = 4             # amlnn_nbg_type
RGB24_RAW_DATA = 0           # amlnn_input_type
AML_OUTDATA_FLOAT32 = 0      # aml_output_format_t
AML_OUTPUT_ORDER_NCHW = 2    # aml_output_order_t


class AssignUserAddress(ctypes.Structure):
    _fields_ = [('inAddr_size', ctypes.c_uint),
                ('outAddr_size', ctypes.c_uint),
                ('io_type', ctypes.c_int),
                ('inAddr', ctypes.c_void_p * ADDRESS_MAX_NUM),
                ('outAddr', ctypes.c_void_p * ADDRESS_MAX_NUM)]


class ForwardCtrl(ctypes.Structure):
    _fields_ = [('enCoreId', ctypes.c_int),
                ('invoke_id', ctypes.c_int64),
                ('timeout_ms', ctypes.c_int32)]


class CompilerArgs(ctypes.Structure):
    _fields_ = [('batch_multiplier', ctypes.c_int32),
                ('compiler_only', ctypes.c_int32)]


class Config(ctypes.Structure):
    """aml_config"""
    _fields_ = [('typeSize', ctypes.c_int),
                ('length', ctypes.c_int),
                ('path', ctypes.c_char_p),
                ('pdata', ctypes.c_char_p),
                ('modelType', ctypes.c_int),
                ('nbgType', ctypes.c_int),
                ('inOut', AssignUserAddress),
                ('forward_ctrl', ForwardCtrl),
                ('compiler_args', CompilerArgs)]


class InputInfo(ctypes.Structure):
    _fields_ = [('valid', ctypes.c_int),
                ('int16_type', ctypes.c_int),
                ('preprocess_debug', ctypes.c_int),
                ('mean', ctypes.c_float * INPUT_CHANNEL),
                ('scale', ctypes.c_float),
                ('input_format', ctypes.c_int)]


class Input(ctypes.Structure):
    """nn_input"""
    _fields_ = [('typeSize', ctypes.c_int),
                ('input_index', ctypes.c_int),
                ('size', ctypes.c_int),
                ('input', ctypes.POINTER(ctypes.c_ubyte)),
                ('input_type', ctypes.c_int),
                ('info', InputInfo)]


class OutputConfig(ctypes.Structure):
    """aml_output_config_t"""
    _fields_ = [('typeSize', ctypes.c_int),
                ('mdType', ctypes.c_int),
                ('perfMode', ctypes.c_int),
                ('format', ctypes.c_int),
                ('order', ctypes.c_int)]


class OutputBuffer(ctypes.Structure):
    """outBuf_t"""
    _fields_ = [('size', ctypes.c_uint),
                ('name', ctypes.c_char * MAX_NAME_LENGTH),
                ('buf', ctypes.POINTER(ctypes.c_ubyte)),
                ('param', ctypes.c_void_p),
                ('out_format', ctypes.c_int)]


class Output(ctypes.Structure):
    """nn_output"""
    _fields_ = [('typeSize', ctypes.c_int),
                ('num', ctypes.c_uint),
                ('out', OutputBuffer * OUTPUT_MAX_NUM)]


def load_library(name='nnsdk'):
    """Return libnnsdk with the prototypes of the functions used here."""
    path = ctypes.util.find_library(name)
    if path is None:
        raise OSError(
            'lib{}.so is not found; it comes with the libadla package of '
            'the VIM4 image'.format(name))
    lib = ctypes.CDLL(path)
    lib.aml_module_create.argtypes = [ctypes.POINTER(Config)]
    lib.aml_module_create.restype = ctypes.c_void_p
    lib.aml_module_input_set.argtypes = [ctypes.c_void_p,
                                         ctypes.POINTER(Input)]
    lib.aml_module_input_set.restype = ctypes.c_int
    lib.aml_module_output_get.argtypes = [ctypes.c_void_p, OutputConfig]
    lib.aml_module_output_get.restype = ctypes.POINTER(Output)
    lib.aml_module_destroy.argtypes = [ctypes.c_void_p]
    lib.aml_module_destroy.restype = ctypes.c_int
    return lib


class AdlaModel(object):
    """An .adla model loaded on the NPU.

    Parameters
    ----------
    path : str
        The .adla file made by adla_convert of the board's toolkit.
    """

    def __init__(self, path):
        self.context = None
        self.lib = load_library()
        # kept alive as long as the model: the sdk may keep the pointer
        self._path = path.encode()
        config = Config()
        config.typeSize = ctypes.sizeof(Config)
        config.path = self._path
        config.modelType = ADLA_LOADABLE
        config.nbgType = NN_ADLA_FILE
        self.context = self.lib.aml_module_create(ctypes.byref(config))
        if not self.context:
            raise RuntimeError('aml_module_create failed for {}'.format(path))

    def run(self, rgb):
        """Run the model on an image.

        Parameters
        ----------
        rgb : numpy.ndarray
            ``(H, W, 3)`` uint8 RGB image of the size of the input of the
            model; the normalization is inside the model.

        Returns
        -------
        list of numpy.ndarray
            Each output as a flat float32 array, in the order of the model.
            The layout of each is NCHW.
        """
        if self.context is None:
            raise RuntimeError('the model is released')
        rgb = np.ascontiguousarray(rgb, dtype=np.uint8)
        data = Input()
        data.typeSize = ctypes.sizeof(Input)
        data.input_index = 0
        data.size = rgb.size
        data.input = rgb.ctypes.data_as(ctypes.POINTER(ctypes.c_ubyte))
        data.input_type = RGB24_RAW_DATA
        if self.lib.aml_module_input_set(self.context,
                                         ctypes.byref(data)) != 0:
            raise RuntimeError('aml_module_input_set failed')
        config = OutputConfig()
        config.typeSize = ctypes.sizeof(OutputConfig)
        config.format = AML_OUTDATA_FLOAT32
        config.order = AML_OUTPUT_ORDER_NCHW
        output = self.lib.aml_module_output_get(self.context, config)
        if not output:
            raise RuntimeError('aml_module_output_get failed')
        outputs = []
        for i in range(output.contents.num):
            buffer = output.contents.out[i]
            count = buffer.size // np.dtype(np.float32).itemsize
            # the buffers belong to the sdk and are reused by the next run
            outputs.append(np.ctypeslib.as_array(
                ctypes.cast(buffer.buf, ctypes.POINTER(ctypes.c_float)),
                shape=(count,)).copy())
        return outputs

    def release(self):
        if self.context is not None:
            self.lib.aml_module_destroy(self.context)
            self.context = None

    def __del__(self):
        self.release()
