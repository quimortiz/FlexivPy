from enum import auto
from typing import TYPE_CHECKING, Optional
from dataclasses import dataclass

import cyclonedds.idl as idl
import cyclonedds.idl.annotations as annotate
import cyclonedds.idl.types as types
from cyclonedds.idl.types import int64, float32, array, int32, byte, sequence


@dataclass
@annotate.final
@annotate.autoid("sequential")
class FlexivCmd(idl.IdlStruct):
    tau_ff: array[types.float64, 7]
    q: array[types.float64, 7]
    dq: array[types.float64, 7]
    kp: array[types.float64, 7]
    kv: array[types.float64, 7]
    timestamp: str
    mode: int32


@dataclass
@annotate.final
@annotate.autoid("sequential")
class FlexivState(idl.IdlStruct):
    q: array[types.float64, 7]
    dq: array[types.float64, 7]
    tau: array[types.float64, 7]
    timestamp: str


# Define an IDL structure for the image data
@dataclass
@annotate.final
@annotate.autoid("sequential")
class FlexivImage(idl.IdlStruct):
    # data: bytes
    data: sequence[byte]
    timestamp: str


