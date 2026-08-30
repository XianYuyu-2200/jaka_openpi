"""ctypes adapter for the external JAKA C++ backend."""

from __future__ import annotations

import ctypes
from dataclasses import dataclass
import pathlib
import time


class JakaSdkError(RuntimeError):
    pass


@dataclass(frozen=True)
class ArmObservation:
    joint_position: tuple[float, ...]
    monotonic_ns: int


class JakaArmBackend:
    """Read state/FK by default; motion requires two explicit opt-ins."""

    def __init__(self, library: pathlib.Path, robot_ip: str, *, allow_motion: bool = False) -> None:
        self.allow_motion = allow_motion
        self._lib = ctypes.CDLL(str(library))
        self._configure_api()
        self._handle = self._lib.jaka_arm_create(robot_ip.encode(), int(allow_motion))
        if not self._handle:
            raise JakaSdkError("failed to create JAKA backend")
        self._connected = False
        self._armed = False

    def _configure_api(self) -> None:
        double6 = ctypes.POINTER(ctypes.c_double)
        self._lib.jaka_arm_create.argtypes = [ctypes.c_char_p, ctypes.c_int]
        self._lib.jaka_arm_create.restype = ctypes.c_void_p
        self._lib.jaka_arm_destroy.argtypes = [ctypes.c_void_p]
        for name in (
            "jaka_arm_connect",
            "jaka_arm_disconnect",
            "jaka_arm_arm_motion",
            "jaka_arm_disarm_motion",
            "jaka_arm_stop",
        ):
            function = getattr(self._lib, name)
            function.argtypes = [ctypes.c_void_p]
            function.restype = ctypes.c_int
        self._lib.jaka_arm_read_state.argtypes = [ctypes.c_void_p, double6]
        self._lib.jaka_arm_forward_kinematics.argtypes = [ctypes.c_void_p, double6, double6]
        self._lib.jaka_arm_send_servo_j.argtypes = [ctypes.c_void_p, double6]
        self._lib.jaka_o6_read_state.argtypes = [ctypes.c_void_p, double6]
        self._lib.jaka_o6_read_state.restype = ctypes.c_int

    @staticmethod
    def _check(result: int, operation: str) -> None:
        if result != 0:
            raise JakaSdkError(f"{operation} failed with JAKA SDK code {result}")

    def connect(self) -> None:
        self._check(self._lib.jaka_arm_connect(self._handle), "connect")
        self._connected = True

    def read(self) -> ArmObservation:
        values = (ctypes.c_double * 6)()
        self._check(self._lib.jaka_arm_read_state(self._handle, values), "read_state")
        return ArmObservation(tuple(values), time.monotonic_ns())

    def forward_kinematics(self, joint_position: tuple[float, ...]) -> tuple[float, ...]:
        if len(joint_position) != 6:
            raise ValueError("joint_position must contain 6 values")
        joints = (ctypes.c_double * 6)(*joint_position)
        pose = (ctypes.c_double * 6)()
        self._check(self._lib.jaka_arm_forward_kinematics(self._handle, joints, pose), "kine_forward")
        return tuple(pose)

    def arm_motion(self, confirmation: str) -> None:
        if self._armed:
            return
        if not self.allow_motion or confirmation != "ENABLE_POLICY_CONTROL":
            raise PermissionError("motion requires allow_motion=True and ENABLE_POLICY_CONTROL")
        self._check(self._lib.jaka_arm_arm_motion(self._handle), "arm_motion")
        self._armed = True

    def send_joint_position(self, target: tuple[float, ...]) -> None:
        if not self._armed:
            raise PermissionError("motion backend is not armed")
        values = (ctypes.c_double * 6)(*target)
        self._check(self._lib.jaka_arm_send_servo_j(self._handle, values), "servo_j")

    def stop(self) -> None:
        if self._handle:
            self._lib.jaka_arm_stop(self._handle)
        self._armed = False

    def read_o6_state(self) -> tuple[float, ...]:
        values = (ctypes.c_double * 6)()
        self._check(self._lib.jaka_o6_read_state(self._handle, values), "read_o6_state")
        return tuple(values)

    def close(self) -> None:
        if not self._handle:
            return
        self.stop()
        if self._connected:
            self._lib.jaka_arm_disconnect(self._handle)
        self._lib.jaka_arm_destroy(self._handle)
        self._handle = None
        self._connected = False

    def __enter__(self) -> JakaArmBackend:
        self.connect()
        return self

    def __exit__(self, *_: object) -> None:
        self.close()
