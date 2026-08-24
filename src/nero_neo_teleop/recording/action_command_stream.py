"""Timestamped local command stream shared by teleop and data recording."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import json
from pathlib import Path
import socket
import threading
import time

import numpy as np


PROTOCOL_VERSION = 1
ARM_ACTION_DOF = 8


@dataclass(frozen=True)
class ArmCommand:
    hand: str
    vector: np.ndarray
    monotonic_ns: int
    sequence: int
    control_state: str


class ArmCommandPublisher:
    """Best-effort nonblocking publisher; robot control never waits on recording."""

    def __init__(self, path: str, hand: str) -> None:
        if hand not in {"left", "right"}:
            raise ValueError("hand must be left or right")
        self.path = path
        self.hand = hand
        self._socket = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM) if path else None
        if self._socket is not None:
            self._socket.setblocking(False)
        self.sent = 0
        self.dropped = 0

    def publish(
        self,
        *,
        joint_target_rad: np.ndarray,
        gripper_target_normalized: float,
        monotonic_ns: int,
        sequence: int,
        control_state: str,
    ) -> None:
        if self._socket is None:
            return
        joints = np.asarray(joint_target_rad, dtype=np.float64)
        if joints.shape != (7,) or not np.isfinite(joints).all():
            raise ValueError("joint target must be a finite seven-vector")
        gripper = float(gripper_target_normalized)
        if not np.isfinite(gripper) or not 0.0 <= gripper <= 1.0:
            raise ValueError("normalized gripper target must be in [0, 1]")
        payload = json.dumps(
            {
                "version": PROTOCOL_VERSION,
                "hand": self.hand,
                "monotonic_ns": int(monotonic_ns),
                "sequence": int(sequence),
                "joint_target_rad": joints.tolist(),
                "gripper_target_normalized": gripper,
                "control_state": str(control_state),
            },
            separators=(",", ":"),
        ).encode("utf-8")
        try:
            self._socket.sendto(payload, self.path)
            self.sent += 1
        except OSError:
            # Recording is observational. A missing/full socket must never
            # interrupt the robot control loop.
            self.dropped += 1

    def close(self) -> None:
        if self._socket is not None:
            self._socket.close()
            self._socket = None


class ArmCommandReceiver:
    """Buffer timestamped Unix datagrams and select the command nearest a frame."""

    def __init__(self, path: str | Path, hand: str, *, capacity: int = 512) -> None:
        if hand not in {"left", "right"}:
            raise ValueError("hand must be left or right")
        if capacity < 4:
            raise ValueError("capacity must be at least four")
        self.path = Path(path)
        self.hand = hand
        self._commands: deque[ArmCommand] = deque(maxlen=capacity)
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._socket: socket.socket | None = None
        self._error: BaseException | None = None
        self.invalid_packets = 0

    def start(self) -> None:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.path.unlink(missing_ok=True)
        self._socket = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
        self._socket.settimeout(0.2)
        self._socket.bind(str(self.path))
        self._stop.clear()
        self._error = None
        self._thread = threading.Thread(target=self._receive, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
        if self._socket is not None:
            self._socket.close()
        self._thread = None
        self._socket = None
        self.path.unlink(missing_ok=True)

    def clear(self) -> None:
        with self._lock:
            self._commands.clear()

    def wait_ready(self, *, after_ns: int, timeout_sec: float = 10.0) -> ArmCommand:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            if self._error is not None:
                raise RuntimeError(f"{self.hand} command receiver failed: {self._error}")
            with self._lock:
                command = self._commands[-1] if self._commands else None
            if command is not None and command.monotonic_ns >= after_ns:
                return command
            time.sleep(0.01)
        raise TimeoutError(f"No fresh {self.hand} controller command within {timeout_sec:.1f}s")

    def nearest(self, target_ns: int, *, max_offset_sec: float = 0.075) -> ArmCommand:
        if self._error is not None:
            raise RuntimeError(f"{self.hand} command receiver failed: {self._error}")
        with self._lock:
            commands = tuple(self._commands)
        if not commands:
            raise RuntimeError(f"No {self.hand} controller commands are buffered")
        command = min(commands, key=lambda item: abs(item.monotonic_ns - target_ns))
        offset_sec = abs(command.monotonic_ns - target_ns) / 1e9
        if offset_sec > max_offset_sec:
            raise RuntimeError(
                f"Nearest {self.hand} controller command is {offset_sec * 1000:.1f}ms "
                f"from the recording frame"
            )
        return command

    def _receive(self) -> None:
        assert self._socket is not None
        try:
            while not self._stop.is_set():
                try:
                    payload = self._socket.recv(4096)
                except TimeoutError:
                    continue
                try:
                    packet = json.loads(payload)
                    command = self._decode(packet)
                except (KeyError, TypeError, ValueError, json.JSONDecodeError):
                    self.invalid_packets += 1
                    continue
                with self._lock:
                    if not self._commands or command.sequence > self._commands[-1].sequence:
                        self._commands.append(command)
        except BaseException as exc:
            if not self._stop.is_set():
                self._error = exc

    def _decode(self, packet: dict) -> ArmCommand:
        if int(packet["version"]) != PROTOCOL_VERSION or packet["hand"] != self.hand:
            raise ValueError("command packet protocol or hand mismatch")
        joints = np.asarray(packet["joint_target_rad"], dtype=np.float32)
        gripper = float(packet["gripper_target_normalized"])
        if joints.shape != (7,) or not np.isfinite(joints).all():
            raise ValueError("invalid command joints")
        if not np.isfinite(gripper) or not 0.0 <= gripper <= 1.0:
            raise ValueError("invalid command gripper")
        vector = np.asarray([*joints, gripper], dtype=np.float32)
        return ArmCommand(
            hand=self.hand,
            vector=vector,
            monotonic_ns=int(packet["monotonic_ns"]),
            sequence=int(packet["sequence"]),
            control_state=str(packet["control_state"]),
        )
