#!/usr/bin/env python3
from __future__ import annotations

import json
import socket
import struct
import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import Any

import numpy as np


# Keep the original wire identifiers so already-installed Neo 3 APKs remain
# compatible while the repository and Python APIs use PICO naming.
INPUT_SCHEMA = "nero.quest.input.v1"
SYNC_SCHEMA = "nero.quest.sync.v1"
BINARY_INPUT_MAGIC = b"NQ01"
BINARY_INPUT_HEADER_SIZE = 28
BINARY_INPUT_STATE_SIZE = 72
BINARY_INPUT_PACKET_SIZE = (
    BINARY_INPUT_HEADER_SIZE + 3 * BINARY_INPUT_STATE_SIZE
)


@dataclass(frozen=True)
class SyncEstimate:
    rtt_ms: float
    quest_minus_host_ms: float


@dataclass(frozen=True)
class InputSample:
    packet: dict[str, Any]
    peer: tuple[str, int]
    receive_unix_ns: int
    receive_monotonic_ns: int
    age_ms: float | None


def _decode_binary_state(payload: bytes, offset: int) -> tuple[dict[str, Any], int]:
    if offset + BINARY_INPUT_STATE_SIZE > len(payload):
        raise ValueError("truncated PICO binary controller state")
    flags = payload[offset]
    values = struct.unpack_from("<17f", payload, offset + 4)
    state = {
        "tracked": bool(flags & 0x01),
        "stickClick": bool(flags & 0x02),
        "primary": bool(flags & 0x04),
        "secondary": bool(flags & 0x08),
    }
    names = (
        "px", "py", "pz", "qx", "qy", "qz", "qw",
        "vx", "vy", "vz", "avx", "avy", "avz",
        "trigger", "grip", "stickX", "stickY",
    )
    state.update(zip(names, values))
    return state, offset + BINARY_INPUT_STATE_SIZE


def decode_binary_input_packet(payload: bytes) -> dict[str, Any]:
    """Decode the allocation-free PICO packet emitted by Unity.

    Layout is little-endian: magic[4], sequence[int64], unix_ns[int64],
    monotonic_sec[double], then three controller states. Each state contains
    four flag bytes followed by sixteen float32 values.
    """
    if len(payload) != BINARY_INPUT_PACKET_SIZE:
        raise ValueError(
            f"invalid PICO binary packet size {len(payload)}, "
            f"expected {BINARY_INPUT_PACKET_SIZE}"
        )
    magic, sequence, unix_time_ns, monotonic_time_sec = struct.unpack_from(
        "<4sqqd", payload, 0
    )
    if magic != BINARY_INPUT_MAGIC:
        raise ValueError(f"invalid PICO binary packet magic {magic!r}")
    offset = BINARY_INPUT_HEADER_SIZE
    head, offset = _decode_binary_state(payload, offset)
    left, offset = _decode_binary_state(payload, offset)
    right, offset = _decode_binary_state(payload, offset)
    return {
        "schema": INPUT_SCHEMA,
        "sequence": sequence,
        "unixTimeMs": unix_time_ns // 1_000_000,
        "unixTimeNs": unix_time_ns,
        "monotonicTimeSec": monotonic_time_sec,
        "head": head,
        "left": left,
        "right": right,
    }


class PicoUdpStream:
    def __init__(
        self,
        *,
        bind: str = "0.0.0.0",
        port: int = 50150,
        timeout_sec: float = 2.0,
        sync_interval_sec: float = 0.5,
    ) -> None:
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket.bind((bind, port))
        self.socket.settimeout(timeout_sec)
        self.timeout_sec = timeout_sec
        self.sync_interval_sec = sync_interval_sec

        self.receive_times: deque[float] = deque()
        self.last_sequence: int | None = None
        self.received = 0
        self.missing = 0
        self.sync_sequence = 0
        self.last_sync_send = 0.0
        self.pending_sync: dict[int, tuple[int, int]] = {}
        self.sync_samples: deque[SyncEstimate] = deque(maxlen=32)

    def close(self) -> None:
        self.socket.close()

    @property
    def rx_hz(self) -> int:
        return len(self.receive_times)

    @property
    def loss_percent(self) -> float:
        total = self.received + self.missing
        return 100.0 * self.missing / total if total else 0.0

    @property
    def sync_estimate(self) -> SyncEstimate | None:
        if not self.sync_samples:
            return None
        return min(self.sync_samples, key=lambda sample: sample.rtt_ms)

    def receive(self) -> InputSample:
        while True:
            payload, peer = self.socket.recvfrom(65535)
            receive_monotonic_ns = time.monotonic_ns()
            receive_unix_ns = time.time_ns()
            if payload.startswith(BINARY_INPUT_MAGIC):
                packet = decode_binary_input_packet(payload)
            else:
                packet = json.loads(payload)
            schema = packet.get("schema")

            if schema == SYNC_SCHEMA and packet.get("kind") == "pong":
                self._handle_pong(packet, receive_unix_ns, receive_monotonic_ns)
                continue
            if schema != INPUT_SCHEMA:
                continue

            self._record_input(int(packet["sequence"]), receive_monotonic_ns)
            self._maybe_send_sync(peer)
            estimate = self.sync_estimate
            age_ms = None
            if estimate is not None:
                quest_send_unix_ns = int(
                    packet.get("unixTimeNs", int(packet["unixTimeMs"]) * 1_000_000)
                )
                offset_ns = estimate.quest_minus_host_ms * 1_000_000.0
                age_ms = (receive_unix_ns - quest_send_unix_ns + offset_ns) / 1_000_000.0
            return InputSample(
                packet=packet,
                peer=peer,
                receive_unix_ns=receive_unix_ns,
                receive_monotonic_ns=receive_monotonic_ns,
                age_ms=age_ms,
            )

    def _record_input(self, sequence: int, receive_monotonic_ns: int) -> None:
        if self.last_sequence is not None:
            if sequence <= self.last_sequence:
                self.received = 0
                self.missing = 0
            elif sequence > self.last_sequence + 1:
                self.missing += sequence - self.last_sequence - 1
        self.last_sequence = sequence
        self.received += 1

        now = receive_monotonic_ns / 1_000_000_000.0
        self.receive_times.append(now)
        while self.receive_times and now - self.receive_times[0] > 1.0:
            self.receive_times.popleft()

    def _maybe_send_sync(self, peer: tuple[str, int]) -> None:
        now = time.monotonic()
        if now - self.last_sync_send < self.sync_interval_sec:
            return
        self.last_sync_send = now
        sequence = self.sync_sequence
        self.sync_sequence += 1
        host_send_unix_ns = time.time_ns()
        host_send_monotonic_ns = time.monotonic_ns()
        packet = {
            "schema": SYNC_SCHEMA,
            "kind": "ping",
            "sequence": sequence,
            "hostSendUnixNs": host_send_unix_ns,
            "hostSendMonotonicNs": host_send_monotonic_ns,
        }
        self.pending_sync[sequence] = (host_send_unix_ns, host_send_monotonic_ns)
        self.socket.sendto(json.dumps(packet, separators=(",", ":")).encode(), peer)

        oldest_allowed = sequence - 16
        self.pending_sync = {
            key: value for key, value in self.pending_sync.items() if key >= oldest_allowed
        }

    def _handle_pong(
        self,
        packet: dict[str, Any],
        host_receive_unix_ns: int,
        host_receive_monotonic_ns: int,
    ) -> None:
        sequence = int(packet["sequence"])
        pending = self.pending_sync.pop(sequence, None)
        if pending is None:
            return
        host_send_unix_ns, host_send_monotonic_ns = pending
        quest_receive_unix_ns = int(packet["questReceiveUnixNs"])
        quest_receive_monotonic_ns = int(packet["questReceiveMonotonicNs"])
        quest_send_unix_ns = int(packet["questSendUnixNs"])
        quest_send_monotonic_ns = int(packet["questSendMonotonicNs"])

        quest_processing_ns = max(0, quest_send_monotonic_ns - quest_receive_monotonic_ns)
        rtt_ns = max(
            0,
            host_receive_monotonic_ns - host_send_monotonic_ns - quest_processing_ns,
        )
        offset_ns = (
            (quest_receive_unix_ns - host_send_unix_ns)
            + (quest_send_unix_ns - host_receive_unix_ns)
        ) / 2.0
        self.sync_samples.append(
            SyncEstimate(
                rtt_ms=rtt_ns / 1_000_000.0,
                quest_minus_host_ms=offset_ns / 1_000_000.0,
            )
        )


@dataclass(frozen=True)
class LatestControllerSample:
    sample: InputSample | None
    received_monotonic: float | None
    error: str | None


class PicoInputMonitor:
    """Receive PICO controller packets outside the fixed-rate robot loop."""

    def __init__(self, bind: str, port: int) -> None:
        self.stream = PicoUdpStream(bind=bind, port=port, timeout_sec=0.2)
        self._lock = threading.Lock()
        self._sample: InputSample | None = None
        self._received_monotonic: float | None = None
        self._error: str | None = None
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name="pico-udp", daemon=True)

    def start(self) -> None:
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        self.stream.close()
        self._thread.join(timeout=1.0)

    def latest(self) -> LatestControllerSample:
        with self._lock:
            return LatestControllerSample(self._sample, self._received_monotonic, self._error)

    def wait_ready(self, hand: str, timeout_sec: float = 10.0) -> InputSample:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            latest = self.latest()
            if latest.sample is not None and bool(latest.sample.packet[hand]["tracked"]):
                return latest.sample
            time.sleep(0.02)
        raise RuntimeError(f"No tracked PICO controller {hand} packet within {timeout_sec:.1f}s")

    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                sample = self.stream.receive()
            except socket.timeout:
                continue
            except OSError as exc:
                if not self._stop.is_set():
                    with self._lock:
                        self._error = f"{type(exc).__name__}: {exc}"
                break
            except Exception as exc:
                with self._lock:
                    self._error = f"{type(exc).__name__}: {exc}"
                continue
            with self._lock:
                self._sample = sample
                self._received_monotonic = time.monotonic()
                self._error = None


def apply_deadzone(value: float, deadzone: float) -> float:
    value = float(np.clip(value, -1.0, 1.0))
    magnitude = abs(value)
    if magnitude <= deadzone:
        return 0.0
    return float(np.sign(value) * (magnitude - deadzone) / (1.0 - deadzone))


def joystick_rotation_step(
    stick_x: float,
    stick_y: float,
    *,
    rate_rad_s: float,
    dt: float,
) -> np.ndarray:
    return rate_rad_s * dt * np.asarray([stick_x, 0.0, stick_y], dtype=np.float64)
