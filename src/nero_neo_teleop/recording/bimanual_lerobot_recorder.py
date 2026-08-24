"""Record passive dual-NERO demonstrations in LeRobot v3 format."""

from __future__ import annotations

import argparse
from collections import deque
from dataclasses import dataclass
import json
import os
from pathlib import Path
import select
import signal
import socket
import struct
import subprocess
import sys
import threading
import time
from typing import Any, Protocol

import numpy as np


from nero_vla.camera_reader import CameraFrame, SyntheticCameraReader, V4L2CameraReader
from nero_vla.dual_can import require_can_role
from nero_neo_teleop.recording.action_command_stream import ArmCommandReceiver
ARM_DOF = 8
STATE_DOF = 2 * ARM_DOF
GRIPPER_OPEN_M = 0.09
ARM_NAMES = [*[f"joint_{index}.pos" for index in range(1, 8)], "gripper.pos"]
STATE_NAMES = [*[f"left_{name}" for name in ARM_NAMES], *[f"right_{name}" for name in ARM_NAMES]]


class StateSource(Protocol):
    def start(self) -> None: ...
    def stop(self) -> None: ...
    def wait_ready(self, timeout_sec: float = 5.0) -> "ArmState": ...
    def snapshot(self, max_age_sec: float = 0.2) -> "ArmState": ...


@dataclass(frozen=True)
class ArmState:
    vector: np.ndarray
    monotonic_ns: int
    unix_ns: int


@dataclass(frozen=True)
class BimanualSample:
    scheduled_monotonic_ns: int
    left: ArmState
    right: ArmState
    state_vector: np.ndarray
    world: CameraFrame
    left_wrist: CameraFrame
    right_wrist: CameraFrame


@dataclass(frozen=True)
class BimanualAction:
    vector: np.ndarray
    left_monotonic_ns: int
    right_monotonic_ns: int
    source: str
    left_sequence: int | None = None
    right_sequence: int | None = None
    left_control_state: str | None = None
    right_control_state: str | None = None


class NeroCanStateSource:
    """Passively decode NERO feedback without creating a second SDK controller."""

    CAN_FRAME = struct.Struct("=IB3x8s")
    JOINT_FRAME_MAP = {
        0x2A5: (0, 1),
        0x2A6: (2, 3),
        0x2A7: (4, 5),
        0x2A9: (6,),
    }

    def __init__(self, can_port: str, name: str) -> None:
        self.can_port = can_port
        self.name = name
        self._bus: socket.socket | None = None
        self._thread: threading.Thread | None = None
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._joints = np.full(7, np.nan, dtype=np.float64)
        self._joint_stamps = np.zeros(7, dtype=np.int64)
        self._gripper_m = float("nan")
        self._gripper_stamp = 0
        self._error: BaseException | None = None

    def start(self) -> None:
        require_can_role(self.can_port, "follower", recovery_timeout_sec=3.0)
        self._stop.clear()
        self._error = None
        self._bus = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        self._bus.settimeout(0.2)
        self._bus.bind((self.can_port,))
        self._thread = threading.Thread(target=self._receive, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
        if self._bus is not None:
            self._bus.close()
        self._thread = None
        self._bus = None

    def wait_ready(self, timeout_sec: float = 5.0) -> ArmState:
        deadline = time.monotonic() + timeout_sec
        last_error: Exception | None = None
        while time.monotonic() < deadline:
            try:
                return self.snapshot(max_age_sec=timeout_sec)
            except RuntimeError as exc:
                last_error = exc
                time.sleep(0.02)
        raise TimeoutError(f"No complete {self.name} CAN state on {self.can_port}: {last_error}")

    def snapshot(self, max_age_sec: float = 0.2) -> ArmState:
        if self._bus is None:
            raise RuntimeError(f"{self.name} state source is not started")
        if self._error is not None:
            raise RuntimeError(f"{self.name} CAN listener failed: {self._error}")
        with self._lock:
            joints = self._joints.copy()
            joint_stamps = self._joint_stamps.copy()
            gripper_m = self._gripper_m
            gripper_stamp = self._gripper_stamp
        if not np.isfinite(joints).all() or not np.isfinite(gripper_m):
            raise RuntimeError(f"Incomplete {self.name} joint/gripper feedback")
        stamps = np.r_[joint_stamps, gripper_stamp]
        if int(np.max(stamps) - np.min(stamps)) > 100_000_000:
            raise RuntimeError(f"Incoherent {self.name} CAN feedback snapshot")
        values = np.asarray(
            [*joints, np.clip(gripper_m / GRIPPER_OPEN_M, 0.0, 1.0)], dtype=np.float32
        )
        if values.shape != (ARM_DOF,) or not np.isfinite(values).all():
            raise RuntimeError(f"Invalid {self.name} state: {values}")
        feedback_ns = int(np.min(stamps))
        age_sec = (time.monotonic_ns() - feedback_ns) / 1e9
        if age_sec > max_age_sec:
            raise RuntimeError(f"{self.name} CAN state is stale: {age_sec:.3f}s")
        return ArmState(values, feedback_ns, time.time_ns())

    def _receive(self) -> None:
        assert self._bus is not None
        try:
            while not self._stop.is_set():
                try:
                    frame = self._bus.recv(self.CAN_FRAME.size)
                except TimeoutError:
                    continue
                if len(frame) != self.CAN_FRAME.size:
                    continue
                can_id, length, payload = self.CAN_FRAME.unpack(frame)
                frame_id = can_id & 0x7FF
                if length < 8:
                    continue
                received_ns = time.monotonic_ns()
                if frame_id in self.JOINT_FRAME_MAP:
                    indices = self.JOINT_FRAME_MAP[frame_id]
                    raw = [int.from_bytes(payload[offset:offset + 4], "big", signed=True) for offset in (0, 4)]
                    with self._lock:
                        for value, index in zip(raw, indices):
                            self._joints[index] = np.deg2rad(value * 1e-3)
                            self._joint_stamps[index] = received_ns
                elif frame_id == 0x2A8:
                    mode = payload[7]
                    if mode != 0:
                        continue
                    width_raw = int.from_bytes(payload[0:4], "big", signed=True)
                    with self._lock:
                        self._gripper_m = width_raw * 1e-6
                        self._gripper_stamp = received_ns
        except BaseException as exc:
            if not self._stop.is_set():
                self._error = exc


class SyntheticStateSource:
    def __init__(self, phase: float) -> None:
        self.phase = phase
        self.started = time.monotonic()

    def start(self) -> None:
        self.started = time.monotonic()

    def stop(self) -> None:
        return None

    def wait_ready(self, timeout_sec: float = 5.0) -> ArmState:
        return self.snapshot()

    def snapshot(self, max_age_sec: float = 0.2) -> ArmState:
        elapsed = time.monotonic() - self.started
        vector = np.zeros(ARM_DOF, dtype=np.float32)
        vector[:7] = 0.1 * np.sin(elapsed + self.phase + np.arange(7) * 0.1)
        vector[7] = 0.5
        return ArmState(vector, time.monotonic_ns(), time.time_ns())


def dataset_features(height: int, width: int, image_dtype: str) -> dict[str, dict[str, Any]]:
    image_feature = {
        "dtype": image_dtype,
        "shape": (height, width, 3),
        "names": ["height", "width", "channels"],
        "info": {"is_depth_map": False},
    }
    return {
        "observation.state": {"dtype": "float32", "shape": (STATE_DOF,), "names": STATE_NAMES},
        "action": {"dtype": "float32", "shape": (STATE_DOF,), "names": STATE_NAMES},
        "observation.images.world": dict(image_feature),
        "observation.images.left_wrist": dict(image_feature),
        "observation.images.right_wrist": dict(image_feature),
    }


def feedback_monotonic_ns(timestamp: float) -> int:
    """Convert an SDK receive timestamp from either supported clock domain."""
    now_monotonic = time.monotonic()
    now_unix = time.time()
    if abs(timestamp - now_monotonic) <= 10.0:
        return round(timestamp * 1e9)
    if abs(timestamp - now_unix) <= 10.0:
        return round((now_monotonic + timestamp - now_unix) * 1e9)
    raise RuntimeError(f"CAN feedback timestamp is outside known clock domains: {timestamp}")


def resolved_camera_device(device: str) -> str:
    path = Path(device)
    if not path.exists():
        raise FileNotFoundError(f"Camera device does not exist: {device}")
    return str(path.resolve())


def take_sample(
    scheduled_ns: int,
    left: StateSource,
    right: StateSource,
    world: Any,
    left_wrist: Any,
    right_wrist: Any,
) -> BimanualSample:
    left_state = left.snapshot(max_age_sec=0.2)
    right_state = right.snapshot(max_age_sec=0.2)
    vector = np.concatenate((left_state.vector, right_state.vector)).astype(np.float32)
    if vector.shape != (STATE_DOF,) or not np.isfinite(vector).all():
        raise RuntimeError(f"Invalid bimanual state vector: {vector}")
    return BimanualSample(
        scheduled_ns,
        left_state,
        right_state,
        vector,
        world.latest(max_age_sec=0.2),
        left_wrist.latest(max_age_sec=0.2),
        right_wrist.latest(max_age_sec=0.2),
    )


def max_joint_speed_deg_s(previous: BimanualSample, current: BimanualSample) -> float:
    elapsed = (current.scheduled_monotonic_ns - previous.scheduled_monotonic_ns) / 1e9
    if elapsed <= 0:
        return 0.0
    left_delta = current.state_vector[:7] - previous.state_vector[:7]
    right_delta = current.state_vector[8:15] - previous.state_vector[8:15]
    return float(np.max(np.abs(np.rad2deg(np.concatenate((left_delta, right_delta))))) / elapsed)


class DualReleaseAutoStop:
    """Detect synchronized gripper release followed by stationary arms."""

    def __init__(
        self,
        *,
        closed_threshold: float,
        open_threshold: float,
        synchronization_sec: float,
        stationary_sec: float,
        stationary_speed_deg_s: float,
    ) -> None:
        if not 0.0 <= closed_threshold < open_threshold <= 1.0:
            raise ValueError("gripper thresholds must satisfy 0 <= closed < open <= 1")
        if min(synchronization_sec, stationary_sec, stationary_speed_deg_s) <= 0.0:
            raise ValueError("auto-stop timing and speed values must be positive")
        self.closed_threshold = float(closed_threshold)
        self.open_threshold = float(open_threshold)
        self.synchronization_ns = round(synchronization_sec * 1e9)
        self.stationary_ns = round(stationary_sec * 1e9)
        self.stationary_speed_deg_s = float(stationary_speed_deg_s)
        self._closed_seen = [False, False]
        self._open_state = [False, False]
        self._release_ns: list[int | None] = [None, None]
        self._paired = False
        self._stationary_since_ns: int | None = None

    @property
    def paired(self) -> bool:
        return self._paired

    def update(
        self,
        previous: BimanualSample,
        current: BimanualSample,
    ) -> tuple[bool, str, float]:
        values = (float(current.left.vector[7]), float(current.right.vector[7]))
        now_ns = current.scheduled_monotonic_ns
        for index, value in enumerate(values):
            if value <= self.closed_threshold:
                self._closed_seen[index] = True
                self._open_state[index] = False
                if self._paired:
                    self._cancel_pair()
            elif value >= self.open_threshold and not self._open_state[index]:
                self._open_state[index] = True
                if self._closed_seen[index]:
                    self._release_ns[index] = now_ns

        left_release, right_release = self._release_ns
        if not self._paired and left_release is not None and right_release is not None:
            if abs(left_release - right_release) <= self.synchronization_ns:
                self._paired = True
                self._stationary_since_ns = None

        speed = max_joint_speed_deg_s(previous, current)
        if not self._paired:
            return False, "waiting_release", speed
        if speed <= self.stationary_speed_deg_s:
            if self._stationary_since_ns is None:
                self._stationary_since_ns = now_ns
            if now_ns - self._stationary_since_ns >= self.stationary_ns:
                return True, "dual_release_stationary", speed
            return False, "release_waiting_stationary", speed
        self._stationary_since_ns = None
        return False, "release_arms_moving", speed

    def _cancel_pair(self) -> None:
        self._paired = False
        self._release_ns = [None, None]
        self._stationary_since_ns = None


class SingleReleaseAutoStop:
    """Detect one gripper closing, reopening, then both arms becoming stationary."""

    def __init__(
        self,
        *,
        side: str,
        closed_threshold: float,
        open_threshold: float,
        stationary_sec: float,
        stationary_speed_deg_s: float,
    ) -> None:
        if side not in {"left", "right"}:
            raise ValueError("side must be left or right")
        if not 0.0 <= closed_threshold < open_threshold <= 1.0:
            raise ValueError("gripper thresholds must satisfy 0 <= closed < open <= 1")
        if min(stationary_sec, stationary_speed_deg_s) <= 0.0:
            raise ValueError("auto-stop timing and speed values must be positive")
        self.side = side
        self.closed_threshold = float(closed_threshold)
        self.open_threshold = float(open_threshold)
        self.stationary_ns = round(stationary_sec * 1e9)
        self.stationary_speed_deg_s = float(stationary_speed_deg_s)
        self._closed_seen = False
        self._released = False
        self._stationary_since_ns: int | None = None

    def update(
        self,
        previous: BimanualSample,
        current: BimanualSample,
    ) -> tuple[bool, str, float]:
        arm = current.left if self.side == "left" else current.right
        value = float(arm.vector[7])
        now_ns = current.scheduled_monotonic_ns
        if value <= self.closed_threshold:
            self._closed_seen = True
            self._released = False
            self._stationary_since_ns = None
        elif self._closed_seen and value >= self.open_threshold:
            self._released = True

        speed = max_joint_speed_deg_s(previous, current)
        if not self._released:
            return False, f"waiting_{self.side}_release", speed
        if speed <= self.stationary_speed_deg_s:
            if self._stationary_since_ns is None:
                self._stationary_since_ns = now_ns
            if now_ns - self._stationary_since_ns >= self.stationary_ns:
                return True, f"{self.side}_release_stationary", speed
            return False, f"{self.side}_release_waiting_stationary", speed
        self._stationary_since_ns = None
        return False, f"{self.side}_release_arms_moving", speed


class InactivityAutoStop:
    """Stop after both arms remain stationary for a continuous interval."""

    def __init__(self, *, stationary_sec: float, stationary_speed_deg_s: float) -> None:
        if min(stationary_sec, stationary_speed_deg_s) <= 0.0:
            raise ValueError("inactivity auto-stop timing and speed must be positive")
        self.stationary_ns = round(stationary_sec * 1e9)
        self.stationary_speed_deg_s = float(stationary_speed_deg_s)
        self._stationary_since_ns: int | None = None

    def update(
        self,
        previous: BimanualSample,
        current: BimanualSample,
    ) -> tuple[bool, str, float]:
        speed = max_joint_speed_deg_s(previous, current)
        now_ns = current.scheduled_monotonic_ns
        if speed > self.stationary_speed_deg_s:
            self._stationary_since_ns = None
            return False, "arms_moving", speed
        if self._stationary_since_ns is None:
            self._stationary_since_ns = now_ns
        if now_ns - self._stationary_since_ns >= self.stationary_ns:
            return True, "both_arms_inactive", speed
        return False, "waiting_inactivity", speed


def feedback_action(sample: BimanualSample) -> BimanualAction:
    return BimanualAction(
        vector=sample.state_vector.copy(),
        left_monotonic_ns=sample.left.monotonic_ns,
        right_monotonic_ns=sample.right.monotonic_ns,
        source="next_feedback",
    )


def controller_action(
    observation: BimanualSample,
    left_source: ArmCommandReceiver,
    right_source: ArmCommandReceiver,
) -> BimanualAction:
    left = left_source.nearest(observation.scheduled_monotonic_ns)
    right = right_source.nearest(observation.scheduled_monotonic_ns)
    vector = np.concatenate((left.vector, right.vector)).astype(np.float32)
    if vector.shape != (STATE_DOF,) or not np.isfinite(vector).all():
        raise RuntimeError(f"Invalid bimanual controller action: {vector}")
    return BimanualAction(
        vector=vector,
        left_monotonic_ns=left.monotonic_ns,
        right_monotonic_ns=right.monotonic_ns,
        source="controller_command",
        left_sequence=left.sequence,
        right_sequence=right.sequence,
        left_control_state=left.control_state,
        right_control_state=right.control_state,
    )


def timing_record(index: int, observation: BimanualSample, action: BimanualAction) -> dict[str, Any]:
    tick = observation.scheduled_monotonic_ns
    return {
        "frame_index": index,
        "scheduled_monotonic_ns": tick,
        "left_state_monotonic_ns": observation.left.monotonic_ns,
        "right_state_monotonic_ns": observation.right.monotonic_ns,
        "world_monotonic_ns": observation.world.monotonic_ns,
        "left_wrist_monotonic_ns": observation.left_wrist.monotonic_ns,
        "right_wrist_monotonic_ns": observation.right_wrist.monotonic_ns,
        "left_state_age_ms": (tick - observation.left.monotonic_ns) / 1e6,
        "right_state_age_ms": (tick - observation.right.monotonic_ns) / 1e6,
        "world_age_ms": (tick - observation.world.monotonic_ns) / 1e6,
        "left_wrist_age_ms": (tick - observation.left_wrist.monotonic_ns) / 1e6,
        "right_wrist_age_ms": (tick - observation.right_wrist.monotonic_ns) / 1e6,
        "action_source": action.source,
        "left_action_offset_ms": (
            action.left_monotonic_ns - observation.scheduled_monotonic_ns
        ) / 1e6,
        "right_action_offset_ms": (
            action.right_monotonic_ns - observation.scheduled_monotonic_ns
        ) / 1e6,
        "left_action_sequence": action.left_sequence,
        "right_action_sequence": action.right_sequence,
        "left_control_state": action.left_control_state,
        "right_control_state": action.right_control_state,
        "world_sequence": observation.world.sequence,
        "left_wrist_sequence": observation.left_wrist.sequence,
        "right_wrist_sequence": observation.right_wrist.sequence,
    }


def add_frame(dataset: Any, observation: BimanualSample, action: BimanualAction, task: str) -> None:
    dataset.add_frame(
        {
            "observation.state": observation.state_vector,
            "observation.images.world": observation.world.image_rgb,
            "observation.images.left_wrist": observation.left_wrist.image_rgb,
            "observation.images.right_wrist": observation.right_wrist.image_rgb,
            "action": action.vector,
            "task": task,
        }
    )


def wait_until(deadline_ns: int) -> None:
    remaining = deadline_ns - time.monotonic_ns()
    if remaining > 0:
        time.sleep(remaining / 1e9)


def finish_requested() -> bool:
    if not sys.stdin.isatty():
        return False
    readable, _, _ = select.select([sys.stdin], [], [], 0)
    if not readable:
        return False
    sys.stdin.readline()
    return True


def choose_after_episode(auto_save: bool) -> str:
    if auto_save:
        return "save"
    while True:
        answer = input("Save episode [s], discard [d], or quit [q]? ").strip().lower()
        if answer in {"s", "save"}:
            return "save"
        if answer in {"d", "discard"}:
            return "discard"
        if answer in {"q", "quit"}:
            return "quit"


def write_diagnostics(root: Path, episode_index: int, rows: list[dict[str, Any]]) -> Path:
    path = root / "diagnostics" / f"episode_{episode_index:06d}.jsonl"
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as stream:
        for row in rows:
            stream.write(json.dumps(row, separators=(",", ":")) + "\n")
    return path


def validate_existing_dataset(root: Path) -> int:
    """Reject incomplete or corrupt LeRobot roots before resume touches them."""
    import pyarrow.parquet as pq

    tasks = root / "meta/tasks.parquet"
    episode_files = sorted((root / "meta/episodes").rglob("*.parquet"))
    data_files = sorted((root / "data").rglob("*.parquet"))
    if not tasks.is_file() or not episode_files:
        raise RuntimeError(f"Dataset is incomplete and cannot be resumed: {root}")
    paths = [tasks, *episode_files, *data_files]
    for path in paths:
        pq.ParquetFile(path)
    return len(paths)


def start_managed_controller(command: str, startup_sec: float) -> subprocess.Popen[str]:
    print(f"starting managed dual-arm controller: {command}", flush=True)
    process = subprocess.Popen(
        command,
        shell=True,
        executable="/bin/bash",
        start_new_session=True,
        text=True,
    )
    time.sleep(startup_sec)
    status = process.poll()
    if status is not None:
        raise RuntimeError(f"Managed controller exited during startup with status {status}")
    return process


def stop_managed_controller(
    process: subprocess.Popen[str],
    timeout_sec: float = 10.0,
    *,
    allow_existing_failure: bool = False,
) -> None:
    if process.poll() is not None:
        if process.returncode != 0:
            if allow_existing_failure:
                print(
                    f"managed controller had already exited with status {process.returncode}",
                    flush=True,
                )
                return
            raise RuntimeError(f"Managed controller exited with status {process.returncode}")
        return
    print("stopping managed CPV controller before automatic Home", flush=True)
    os.killpg(process.pid, signal.SIGINT)
    try:
        process.wait(timeout=timeout_sec)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGTERM)
        try:
            process.wait(timeout=3.0)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=3.0)
    if process.returncode not in (0, 130, -signal.SIGINT):
        raise RuntimeError(f"Managed controller stopped with status {process.returncode}")
    print("managed CPV controller stopped cleanly", flush=True)


def require_managed_controller_alive(process: subprocess.Popen[str] | None) -> None:
    if process is not None and process.poll() is not None:
        raise RuntimeError(f"Managed controller exited unexpectedly with status {process.returncode}")


def run_home_command(command: str, timeout_sec: float) -> None:
    print(f"automatic dual-arm Home: {command}", flush=True)
    result = subprocess.run(
        command,
        shell=True,
        executable="/bin/bash",
        timeout=timeout_sec,
        check=False,
    )
    if result.returncode != 0:
        raise RuntimeError(f"Automatic dual-arm Home failed with status {result.returncode}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--left-can", default="can_left")
    parser.add_argument("--right-can", default="can_right")
    parser.add_argument(
        "--world-camera",
        default="/dev/v4l/by-path/pci-0000:07:00.4-usb-0:1.2:1.0-video-index0",
    )
    parser.add_argument(
        "--left-wrist-camera",
        default="/dev/v4l/by-path/pci-0000:07:00.3-usb-0:2.3:1.0-video-index0",
    )
    parser.add_argument(
        "--right-wrist-camera",
        default="/dev/v4l/by-path/pci-0000:07:00.4-usb-0:1.3:1.0-video-index0",
    )
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument(
        "--episode-seconds",
        type=float,
        default=0.0,
        help="maximum episode duration; 0 disables the time limit",
    )
    parser.add_argument(
        "--episodes",
        "--successful-episodes",
        dest="episodes",
        type=int,
        default=3,
        help="number of saved successful episodes; discarded attempts are replaced",
    )
    parser.add_argument("--task", required=True)
    parser.add_argument("--repo-id", default="local/nero_bimanual_demo")
    parser.add_argument("--root", type=Path, required=True)
    parser.add_argument("--image-storage", choices=("video", "image"), default="video")
    parser.add_argument("--resume", action="store_true")
    parser.add_argument("--auto-start", action="store_true")
    parser.add_argument("--auto-save", action="store_true")
    parser.add_argument("--synthetic-inputs", action="store_true")
    parser.add_argument(
        "--action-source",
        choices=("next_feedback", "controller_command"),
        default="next_feedback",
        help="training action label source; controller_command records executed CPV/gripper targets",
    )
    parser.add_argument(
        "--action-socket-dir",
        type=Path,
        default=None,
        help="directory containing left.sock and right.sock command streams",
    )
    parser.add_argument("--no-motion-start-detection", action="store_false", dest="motion_start")
    parser.set_defaults(motion_start=True)
    parser.add_argument("--motion-start-threshold-deg-s", type=float, default=2.0)
    parser.add_argument("--motion-start-consecutive", type=int, default=3)
    parser.add_argument("--motion-preroll-seconds", type=float, default=0.2)
    parser.add_argument("--min-episode-seconds", type=float, default=1.0)
    parser.add_argument(
        "--no-dual-release-auto-stop",
        action="store_false",
        dest="dual_release_auto_stop",
    )
    parser.set_defaults(dual_release_auto_stop=True)
    parser.add_argument(
        "--release-auto-stop-mode",
        choices=("dual", "left", "right", "idle", "off"),
        default="dual",
        help="event used to end an episode after the arms become stationary",
    )
    parser.add_argument("--release-closed-threshold", type=float, default=0.45)
    parser.add_argument("--release-open-threshold", type=float, default=0.75)
    parser.add_argument("--release-sync-seconds", type=float, default=0.5)
    parser.add_argument("--release-stationary-seconds", type=float, default=0.5)
    parser.add_argument("--release-stationary-speed-deg-s", type=float, default=0.5)
    parser.add_argument(
        "--managed-controller-command",
        default="",
        help="start this controller for each attempt and stop it before automatic Home",
    )
    parser.add_argument("--managed-controller-startup-sec", type=float, default=2.0)
    parser.add_argument(
        "--return-home-command",
        default="",
        help="command executed after the managed controller has stopped",
    )
    parser.add_argument("--return-delay-seconds", type=float, default=1.0)
    parser.add_argument("--return-timeout-sec", type=float, default=60.0)
    args = parser.parse_args()
    if args.left_can == args.right_can:
        parser.error("left-can and right-can must be different")
    if args.fps <= 0 or args.episode_seconds < 0 or args.episodes <= 0:
        parser.error("fps and episodes must be positive; episode-seconds must be nonnegative")
    if args.width <= 0 or args.height <= 0:
        parser.error("camera dimensions must be positive")
    if args.motion_start_threshold_deg_s <= 0 or args.motion_start_consecutive <= 0:
        parser.error("motion-start threshold and consecutive count must be positive")
    if args.motion_preroll_seconds < 0:
        parser.error("motion-preroll-seconds must be non-negative")
    if args.min_episode_seconds <= 0:
        parser.error("min-episode-seconds must be positive")
    if args.episode_seconds > 0 and args.min_episode_seconds > args.episode_seconds:
        parser.error("min-episode-seconds must not exceed a finite episode-seconds")
    if not 0.0 <= args.release_closed_threshold < args.release_open_threshold <= 1.0:
        parser.error("release thresholds must satisfy 0 <= closed < open <= 1")
    if min(
        args.release_sync_seconds,
        args.release_stationary_seconds,
        args.release_stationary_speed_deg_s,
    ) <= 0.0:
        parser.error("release auto-stop timing and speed values must be positive")
    if args.managed_controller_startup_sec < 0 or args.return_delay_seconds < 0:
        parser.error("controller startup and return delay must be non-negative")
    if args.return_timeout_sec <= 0:
        parser.error("return-timeout-sec must be positive")
    if bool(args.managed_controller_command) != bool(args.return_home_command):
        parser.error(
            "managed-controller-command and return-home-command must be configured together"
        )
    if args.action_source == "controller_command" and args.action_socket_dir is None:
        parser.error("controller_command action source requires --action-socket-dir")
    if args.action_source == "controller_command" and args.synthetic_inputs:
        parser.error("controller_command action source is unavailable with synthetic inputs")
    if not args.dual_release_auto_stop:
        args.release_auto_stop_mode = "off"
    root_exists = args.root.exists()
    root_has_data = root_exists and any(args.root.iterdir())
    if root_exists and not args.resume:
        parser.error(f"new dataset root must not exist: {args.root}")
    if args.resume and not root_has_data:
        parser.error(f"cannot resume missing or empty dataset: {args.root}")
    if args.resume:
        checked = validate_existing_dataset(args.root)
        print(f"dataset parquet health check passed: {checked} files", flush=True)
    return args


def main() -> None:
    args = parse_args()
    from lerobot.configs.video import RGBEncoderConfig
    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    use_video = args.image_storage == "video"
    encoder = RGBEncoderConfig(vcodec="h264", crf=23, preset="ultrafast", g=args.fps)
    dataset_options = {
        "root": args.root,
        "image_writer_threads": 0 if use_video else 4,
        "rgb_encoder": encoder,
        "streaming_encoding": use_video,
        "encoder_queue_maxsize": 180,
        "encoder_threads": 6,
    }
    def open_dataset() -> Any:
        if args.resume:
            result = LeRobotDataset.resume(args.repo_id, **dataset_options)
            if result.fps != args.fps:
                result.finalize()
                raise RuntimeError(f"Existing dataset is {result.fps} FPS, requested {args.fps}")
            expected = dataset_features(args.height, args.width, args.image_storage)
            for key, feature in expected.items():
                actual = result.features.get(key)
                if actual is None or tuple(actual["shape"]) != tuple(feature["shape"]):
                    result.finalize()
                    raise RuntimeError(
                        f"Existing dataset feature mismatch for {key}: {actual} != {feature}"
                    )
            print(
                f"resuming dataset at {result.num_episodes}/{args.episodes} successful episodes",
                flush=True,
            )
            return result
        return LeRobotDataset.create(
            args.repo_id,
            args.fps,
            robot_type="nero_bimanual",
            features=dataset_features(args.height, args.width, args.image_storage),
            use_videos=use_video,
            **dataset_options,
        )

    if args.synthetic_inputs:
        left: StateSource = SyntheticStateSource(0.0)
        right: StateSource = SyntheticStateSource(0.5)
        world = SyntheticCameraReader("world", args.width, args.height, args.fps)
        left_wrist = SyntheticCameraReader("left_wrist", args.width, args.height, args.fps)
        right_wrist = SyntheticCameraReader("right_wrist", args.width, args.height, args.fps)
    else:
        left = NeroCanStateSource(args.left_can, "left")
        right = NeroCanStateSource(args.right_can, "right")
        world_device = resolved_camera_device(args.world_camera)
        left_wrist_device = resolved_camera_device(args.left_wrist_camera)
        right_wrist_device = resolved_camera_device(args.right_wrist_camera)
        print(
            "camera_devices "
            f"world={args.world_camera}->{world_device} "
            f"left_wrist={args.left_wrist_camera}->{left_wrist_device} "
            f"right_wrist={args.right_wrist_camera}->{right_wrist_device}",
            flush=True,
        )
        world = V4L2CameraReader(
            world_device, width=args.width, height=args.height, fps=args.fps, name="world"
        )
        left_wrist = V4L2CameraReader(
            left_wrist_device,
            width=args.width,
            height=args.height,
            fps=args.fps,
            name="left_wrist",
        )
        right_wrist = V4L2CameraReader(
            right_wrist_device,
            width=args.width,
            height=args.height,
            fps=args.fps,
            name="right_wrist",
        )

    sources = (world, left_wrist, right_wrist, left, right)
    command_receivers: tuple[ArmCommandReceiver, ArmCommandReceiver] | None = None
    if args.action_source == "controller_command":
        assert args.action_socket_dir is not None
        command_receivers = (
            ArmCommandReceiver(args.action_socket_dir / "left.sock", "left"),
            ArmCommandReceiver(args.action_socket_dir / "right.sock", "right"),
        )
    dataset = None
    try:
        if command_receivers is not None:
            for receiver in command_receivers:
                receiver.start()
        for source in sources:
            source.start()
        for source in sources:
            source.wait_ready()
        print(
            f"inputs ready: left={args.left_can} right={args.right_can} "
            f"cameras=3 {args.width}x{args.height}@{args.fps}fps",
            flush=True,
        )
        dataset = open_dataset()
        recording_config_path = args.root / "recording_config.json"
        recording_config = {
            "schema_version": 1,
            "action_source": args.action_source,
            "action_alignment": (
                "nearest_timestamped_executed_command"
                if args.action_source == "controller_command"
                else "next_feedback_sample"
            ),
            "fps": args.fps,
            "task": args.task,
        }
        if recording_config_path.exists():
            existing_config = json.loads(recording_config_path.read_text(encoding="utf-8"))
            if existing_config != recording_config:
                raise RuntimeError(
                    f"Recording configuration mismatch: {existing_config} != {recording_config}"
                )
        elif args.resume:
            raise RuntimeError(
                "Existing dataset has no recording_config.json; refusing to infer its action labels"
            )
        else:
            recording_config_path.write_text(
                json.dumps(recording_config, indent=2, sort_keys=True) + "\n",
                encoding="utf-8",
            )
        print(
            f"action_labels={args.action_source} alignment={recording_config['action_alignment']}",
            flush=True,
        )
        if use_video:
            print("video_encoding=h264/ultrafast streaming=true streams=3 threads=6", flush=True)

        attempts = 0
        while dataset.num_episodes < args.episodes:
            attempts += 1
            controller: subprocess.Popen[str] | None = None
            if not args.auto_start:
                input(
                    f"Press Enter to PREPARE successful episode {dataset.num_episodes + 1}/"
                    f"{args.episodes} (attempt {attempts}); recording starts only after motion: "
                )
            command_started_ns = time.monotonic_ns()
            if command_receivers is not None:
                for receiver in command_receivers:
                    receiver.clear()
            if args.managed_controller_command:
                controller = start_managed_controller(
                    args.managed_controller_command,
                    args.managed_controller_startup_sec,
                )
            period_ns = round(1e9 / args.fps)
            frame_limit = (
                None if args.episode_seconds == 0 else round(args.episode_seconds * args.fps)
            )
            next_tick = time.monotonic_ns()
            previous: BimanualSample | None = None
            diagnostics: list[dict[str, Any]] = []

            def make_action(
                observation: BimanualSample, next_feedback: BimanualSample
            ) -> BimanualAction:
                if command_receivers is None:
                    return feedback_action(next_feedback)
                return controller_action(
                    observation, command_receivers[0], command_receivers[1]
                )

            try:
                if command_receivers is not None:
                    for receiver in command_receivers:
                        receiver.wait_ready(after_ns=command_started_ns, timeout_sec=15.0)
                    print("timestamped left/right controller command streams are ready", flush=True)
                if args.motion_start:
                    preroll_frames = round(args.motion_preroll_seconds * args.fps)
                    armed: deque[BimanualSample] = deque(
                        maxlen=preroll_frames + args.motion_start_consecutive + 1
                    )
                    streak = 0
                    print(
                        f"armed; waiting for either arm >="
                        f"{args.motion_start_threshold_deg_s:.1f}deg/s",
                        flush=True,
                    )
                    while True:
                        require_managed_controller_alive(controller)
                        wait_until(next_tick)
                        current = take_sample(next_tick, left, right, world, left_wrist, right_wrist)
                        armed.append(current)
                        if previous is not None:
                            speed = max_joint_speed_deg_s(previous, current)
                            streak = streak + 1 if speed >= args.motion_start_threshold_deg_s else 0
                            if streak >= args.motion_start_consecutive:
                                print(f"motion detected at {speed:.2f}deg/s; recording", flush=True)
                                next_tick += period_ns
                                break
                        previous = current
                        next_tick += period_ns
                    buffered = list(armed)
                    for observation, next_feedback in zip(buffered[:-1], buffered[1:]):
                        action = make_action(observation, next_feedback)
                        add_frame(dataset, observation, action, args.task)
                        diagnostics.append(timing_record(len(diagnostics), observation, action))
                    previous = buffered[-1]

                duration_text = (
                    "unlimited" if frame_limit is None else f"{args.episode_seconds:.1f}s"
                )
                auto_stop: DualReleaseAutoStop | SingleReleaseAutoStop | InactivityAutoStop | None
                if args.release_auto_stop_mode == "dual":
                    auto_stop = DualReleaseAutoStop(
                        closed_threshold=args.release_closed_threshold,
                        open_threshold=args.release_open_threshold,
                        synchronization_sec=args.release_sync_seconds,
                        stationary_sec=args.release_stationary_seconds,
                        stationary_speed_deg_s=args.release_stationary_speed_deg_s,
                    )
                elif args.release_auto_stop_mode in {"left", "right"}:
                    auto_stop = SingleReleaseAutoStop(
                        side=args.release_auto_stop_mode,
                        closed_threshold=args.release_closed_threshold,
                        open_threshold=args.release_open_threshold,
                        stationary_sec=args.release_stationary_seconds,
                        stationary_speed_deg_s=args.release_stationary_speed_deg_s,
                    )
                elif args.release_auto_stop_mode == "idle":
                    auto_stop = InactivityAutoStop(
                        stationary_sec=args.release_stationary_seconds,
                        stationary_speed_deg_s=args.release_stationary_speed_deg_s,
                    )
                else:
                    auto_stop = None
                auto_stop_text = {
                    "dual": "dual release",
                    "left": "left release",
                    "right": "right release",
                    "idle": "both arms inactive",
                    "off": "manual Enter only",
                }[args.release_auto_stop_mode]
                print(
                    f"recording task={args.task!r} max={duration_text}; press Enter to finish; "
                    f"auto-stop={auto_stop_text} + "
                    f"{args.release_stationary_seconds:.1f}s stationary",
                    flush=True,
                )
                while frame_limit is None or len(diagnostics) < frame_limit:
                    require_managed_controller_alive(controller)
                    wait_until(next_tick)
                    current = take_sample(next_tick, left, right, world, left_wrist, right_wrist)
                    if previous is not None:
                        action = make_action(previous, current)
                        add_frame(dataset, previous, action, args.task)
                        diagnostics.append(timing_record(len(diagnostics), previous, action))
                        if auto_stop is not None:
                            should_stop, stop_state, arm_speed = auto_stop.update(previous, current)
                            diagnostics[-1].update(
                                auto_stop_state=stop_state,
                                auto_stop_arm_speed_deg_s=arm_speed,
                                left_gripper_normalized=float(current.left.vector[7]),
                                right_gripper_normalized=float(current.right.vector[7]),
                            )
                            if should_stop:
                                print(
                                    f"episode auto-stopped: {auto_stop_text}; both arms remained "
                                    f"stationary for {args.release_stationary_seconds:.1f}s",
                                    flush=True,
                                )
                                previous = current
                                next_tick += period_ns
                                break
                    previous = current
                    next_tick += period_ns
                    if diagnostics and finish_requested():
                        print(f"episode stopped at {len(diagnostics) / args.fps:.2f}s", flush=True)
                        break
            finally:
                if controller is not None:
                    stop_managed_controller(
                        controller,
                        allow_existing_failure=sys.exc_info()[0] is not None,
                    )

            if args.return_home_command:
                if args.return_delay_seconds:
                    print(
                        f"waiting {args.return_delay_seconds:.1f}s before automatic Home",
                        flush=True,
                    )
                    time.sleep(args.return_delay_seconds)
                run_home_command(args.return_home_command, args.return_timeout_sec)

            duration_sec = len(diagnostics) / args.fps
            if duration_sec < args.min_episode_seconds:
                dataset.clear_episode_buffer()
                print(
                    f"episode rejected automatically: {duration_sec:.2f}s is below "
                    f"the {args.min_episode_seconds:.2f}s minimum; replacement required",
                    flush=True,
                )
                continue
            decision = choose_after_episode(args.auto_save)
            if decision == "save":
                episode_index = dataset.num_episodes
                dataset.save_episode()
                path = write_diagnostics(args.root, episode_index, diagnostics)
                print(
                    f"saved episode={episode_index} frames={len(diagnostics)} "
                    f"success={dataset.num_episodes}/{args.episodes} attempts={attempts} "
                    f"diagnostics={path}",
                    flush=True,
                )
            else:
                dataset.clear_episode_buffer()
                if decision == "quit":
                    print("pending episode discarded; quitting", flush=True)
                    break
                print("episode discarded; replacement attempt required", flush=True)
        if dataset.num_episodes == args.episodes:
            print(f"target reached: {dataset.num_episodes} successful episodes", flush=True)
        print(
            f"camera_rates_hz world={world.measured_hz:.1f} "
            f"left_wrist={left_wrist.measured_hz:.1f} right_wrist={right_wrist.measured_hz:.1f}",
            flush=True,
        )
    except KeyboardInterrupt:
        if dataset is not None and dataset.has_pending_frames():
            dataset.clear_episode_buffer()
        print("interrupted; pending episode discarded", flush=True)
    except BaseException:
        if dataset is not None and dataset.has_pending_frames():
            dataset.clear_episode_buffer()
        print("ERROR: current attempt discarded before shutdown", flush=True)
        raise
    finally:
        if command_receivers is not None:
            for receiver in reversed(command_receivers):
                receiver.stop()
        for source in reversed(sources):
            source.stop()
        if dataset is not None:
            dataset.finalize()


if __name__ == "__main__":
    main()
