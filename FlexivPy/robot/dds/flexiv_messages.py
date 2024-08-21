from enum import auto
from typing import TYPE_CHECKING, Optional
from dataclasses import dataclass
from dataclasses import dataclass, field

import cyclonedds.idl as idl
import cyclonedds.idl.annotations as annotate
import cyclonedds.idl.types as types
from cyclonedds.idl.types import int64, float32, array, int32, byte, sequence


@dataclass
@annotate.final
@annotate.autoid("sequential")
class FlexivCmd(idl.IdlStruct):
    tau_ff: array[types.float64, 7] = field(default_factory=lambda: 7*[.0])
    q: array[types.float64, 7]= field(default_factory=lambda: 7*[.0])
    dq: array[types.float64, 7]= field(default_factory=lambda: 7*[.0])
    kp: array[types.float64, 7]= field(default_factory=lambda: 7*[.0])
    kv: array[types.float64, 7]= field(default_factory=lambda: 7*[.0])
    mode: int32 = 2
    g_cmd: str = "" # "close", "open"
    timestamp: str = ""
    special_cmd: str = "" # "go-home", 
    tau_ff_with_gravity: bool = False

@dataclass
@annotate.final
@annotate.autoid("sequential")
class FlexivState(idl.IdlStruct):
    q: array[types.float64, 7]
    dq: array[types.float64, 7]
    tau: array[types.float64, 7]
    ft_sensor: array[types.float64, 6] = field(default_factory=lambda: [.0, .0, .0, .0, .0, .0])
    timestamp: str = ""
    g_state: str = ""
    g_moving: bool = False
    g_force: types.float64 = -1
    g_width: types.float64 = -1
    state: str = "" # going-home, home, user, waiting





# Define an IDL structure for the image data
@dataclass
@annotate.final
@annotate.autoid("sequential")
class EnvImage(idl.IdlStruct):
    # data: bytes
    data: sequence[byte]
    timestamp: str

@dataclass
@annotate.final
@annotate.autoid("sequential")
class EnvState(idl.IdlStruct):
    names: sequence[str]
    poses: sequence[array[types.float64, 7]]
    timestamp: str



