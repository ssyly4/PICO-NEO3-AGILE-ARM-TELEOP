#!/usr/bin/env python3
"""Record single right-arm NERO demonstrations in LeRobot v3 format."""

from __future__ import annotations

import argparse
from collections import deque
from dataclasses import dataclass
import json
from pathlib import Path
import sys
import time
from typing import Any

import numpy as np

from nero_neo_teleop.recording.single_arm_helpers import (
    CommandInactivityAutoStop,
    controller_motion_started,
)
from nero_vla.camera_reader import CameraFrame, SyntheticCameraReader, V4L2CameraReader
from nero_neo_teleop.recording.action_command_stream import ArmCommandReceiver
from nero_neo_teleop.recording.bimanual_lerobot_recorder import (
    NeroCanStateSource,
    choose_after_episode,
    finish_requested,
    require_managed_controller_alive,
    run_home_command,
    start_managed_controller,
    stop_managed_controller,
    validate_existing_dataset,
    wait_until,
    write_diagnostics,
)

ARM_DOF = 8
STATE_NAMES = [*[f"joint_{index}.pos" for index in range(1, 8)], "gripper.pos"]
ACTION_NAMES = [
    *[f"next_feedback.joint_{index}.pos" for index in range(1, 8)],
    "next_feedback.gripper_opening",
]


@dataclass(frozen=True)
class RightSample:
    scheduled_monotonic_ns: int
    state: Any
    state_vector: np.ndarray
    world: CameraFrame
    right_wrist: CameraFrame


@dataclass(frozen=True)
class RightAction:
    vector: np.ndarray
    monotonic_ns: int
    source: str
    sequence: int | None = None
    control_state: str | None = None


def dataset_features(height: int, width: int, image_dtype: str) -> dict[str, Any]:
    image = {
        "dtype": image_dtype,
        "shape": (height, width, 3),
        "names": ["height", "width", "channels"],
        "info": {"is_depth_map": False},
    }
    return {
        "observation.state": {
            "dtype": "float32",
            "shape": (ARM_DOF,),
            "names": STATE_NAMES,
        },
        "action": {"dtype": "float32", "shape": (ARM_DOF,), "names": ACTION_NAMES},
        "observation.images.world": dict(image),
        "observation.images.right_wrist": dict(image),
    }


def take_sample(
    scheduled_ns: int,
    state_source: Any,
    world: Any,
    right_wrist: Any,
) -> RightSample:
    state = state_source.snapshot(max_age_sec=0.2)
    vector = np.asarray(state.vector, dtype=np.float32)
    if vector.shape != (ARM_DOF,) or not np.isfinite(vector).all():
        raise RuntimeError(f"Invalid right-arm state: {vector}")
    return RightSample(
        scheduled_ns,
        state,
        vector,
        world.latest(max_age_sec=0.2),
        right_wrist.latest(max_age_sec=0.2),
    )


def joint_speed_deg_s(previous: RightSample, current: RightSample) -> float:
    elapsed = (current.scheduled_monotonic_ns - previous.scheduled_monotonic_ns) / 1e9
    if elapsed <= 0:
        return 0.0
    delta = current.state_vector[:7] - previous.state_vector[:7]
    return float(np.max(np.abs(np.rad2deg(delta))) / elapsed)


def feedback_action(sample: RightSample) -> RightAction:
    return RightAction(
        sample.state_vector.copy(), sample.state.monotonic_ns, "next_feedback"
    )


def command_action(sample: RightSample, receiver: ArmCommandReceiver) -> RightAction:
    command = receiver.nearest(sample.scheduled_monotonic_ns)
    vector = np.asarray(command.vector, dtype=np.float32)
    if vector.shape != (ARM_DOF,) or not np.isfinite(vector).all():
        raise RuntimeError(f"Invalid right-arm command: {vector}")
    return RightAction(
        vector,
        command.monotonic_ns,
        "controller_command",
        command.sequence,
        command.control_state,
    )


def add_frame(
    dataset: Any, sample: RightSample, action: RightAction, task: str
) -> None:
    dataset.add_frame(
        {
            "observation.state": sample.state_vector,
            "observation.images.world": sample.world.image_rgb,
            "observation.images.right_wrist": sample.right_wrist.image_rgb,
            "action": action.vector,
            "task": task,
        }
    )


def timing_record(
    index: int, sample: RightSample, action: RightAction
) -> dict[str, Any]:
    tick = sample.scheduled_monotonic_ns
    return {
        "frame_index": index,
        "scheduled_monotonic_ns": tick,
        "right_state_monotonic_ns": sample.state.monotonic_ns,
        "world_monotonic_ns": sample.world.monotonic_ns,
        "right_wrist_monotonic_ns": sample.right_wrist.monotonic_ns,
        "right_state_age_ms": (tick - sample.state.monotonic_ns) / 1e6,
        "world_age_ms": (tick - sample.world.monotonic_ns) / 1e6,
        "right_wrist_age_ms": (tick - sample.right_wrist.monotonic_ns) / 1e6,
        "action_source": action.source,
        "right_action_offset_ms": (action.monotonic_ns - tick) / 1e6,
        "right_action_sequence": action.sequence,
        "right_control_state": action.control_state,
        "world_sequence": sample.world.sequence,
        "right_wrist_sequence": sample.right_wrist.sequence,
    }


def resolved_device(value: str) -> str:
    path = Path(value)
    if not path.exists():
        raise FileNotFoundError(f"Camera device does not exist: {value}")
    return str(path.resolve())


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--can", default="can_right")
    parser.add_argument("--world-camera", required=True)
    parser.add_argument("--right-wrist-camera", required=True)
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--episode-seconds", type=float, default=20.0)
    parser.add_argument("--successful-episodes", type=int, default=50)
    parser.add_argument("--task", required=True)
    parser.add_argument("--control-profile", default="unspecified")
    parser.add_argument("--repo-id", required=True)
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
    )
    parser.add_argument("--action-socket", type=Path)
    parser.add_argument("--activity-socket", type=Path)
    parser.add_argument("--inactivity-auto-stop-seconds", type=float, default=0.0)
    parser.add_argument("--inactivity-joint-speed-deg-s", type=float, default=0.5)
    parser.add_argument("--inactivity-gripper-speed-per-s", type=float, default=0.02)
    parser.add_argument("--start-joints-deg", default="")
    parser.add_argument(
        "--no-motion-start-detection", action="store_false", dest="motion_start"
    )
    parser.set_defaults(motion_start=True)
    parser.add_argument(
        "--controller-start-detection",
        action="store_const",
        const="controller",
        default="motion",
        dest="start_detection",
        help="start on the controller velocity-tracking state instead of joint speed",
    )
    parser.add_argument("--motion-start-threshold-deg-s", type=float, default=2.0)
    parser.add_argument("--motion-start-consecutive", type=int, default=3)
    parser.add_argument("--motion-preroll-seconds", type=float, default=0.2)
    parser.add_argument("--min-episode-seconds", type=float, default=1.0)
    parser.add_argument("--managed-controller-command", default="")
    parser.add_argument("--managed-controller-startup-sec", type=float, default=2.0)
    parser.add_argument("--return-home-command", default="")
    parser.add_argument("--return-delay-seconds", type=float, default=0.5)
    parser.add_argument("--return-timeout-sec", type=float, default=75.0)
    args = parser.parse_args()
    if args.fps <= 0 or args.episode_seconds <= 0 or args.successful_episodes <= 0:
        parser.error("fps, episode-seconds, and successful-episodes must be positive")
    if args.min_episode_seconds <= 0 or args.min_episode_seconds > args.episode_seconds:
        parser.error("min-episode-seconds must be positive and within episode-seconds")
    if args.motion_start_threshold_deg_s <= 0 or args.motion_start_consecutive <= 0:
        parser.error("motion-start parameters must be positive")
    if args.motion_preroll_seconds < 0 or args.return_delay_seconds < 0:
        parser.error("preroll and return delay must be non-negative")
    if bool(args.managed_controller_command) != bool(args.return_home_command):
        parser.error(
            "managed controller and return-home commands must be configured together"
        )
    if args.action_source == "controller_command" and args.action_socket is None:
        parser.error("controller_command requires --action-socket")
    if args.inactivity_auto_stop_seconds < 0:
        parser.error("inactivity-auto-stop-seconds must be non-negative")
    if args.inactivity_auto_stop_seconds > 0 and args.activity_socket is None:
        parser.error("inactivity auto-stop requires --activity-socket")
    if (
        args.motion_start
        and args.start_detection == "controller"
        and args.action_socket is None
        and args.activity_socket is None
    ):
        parser.error("controller start detection requires an action or activity socket")
    if min(args.inactivity_joint_speed_deg_s, args.inactivity_gripper_speed_per_s) <= 0:
        parser.error("inactivity speed thresholds must be positive")
    if args.start_joints_deg:
        try:
            start_joints_deg = [
                float(item) for item in args.start_joints_deg.split(",")
            ]
        except ValueError:
            parser.error("start-joints-deg must contain seven comma-separated numbers")
        if len(start_joints_deg) != 7 or not np.isfinite(start_joints_deg).all():
            parser.error("start-joints-deg must contain seven finite numbers")
        args.start_joints_deg = start_joints_deg
    else:
        args.start_joints_deg = None
    root_exists = args.root.exists()
    root_has_data = root_exists and any(args.root.iterdir())
    if root_exists and not args.resume:
        parser.error(f"new dataset root must not exist: {args.root}")
    if args.resume and not root_has_data:
        parser.error(f"cannot resume missing or empty dataset: {args.root}")
    if args.resume:
        print(
            f"dataset parquet health check passed: {validate_existing_dataset(args.root)} files",
            flush=True,
        )
    return args


def main() -> None:
    args = parse_args()
    from lerobot.configs.video import RGBEncoderConfig
    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    use_video = args.image_storage == "video"
    encoder = RGBEncoderConfig(vcodec="h264", crf=23, preset="ultrafast", g=args.fps)
    options = {
        "root": args.root,
        "image_writer_threads": 0 if use_video else 4,
        "rgb_encoder": encoder,
        "streaming_encoding": use_video,
        "encoder_queue_maxsize": 120,
        "encoder_threads": 4,
    }
    features = dataset_features(args.height, args.width, args.image_storage)

    def open_dataset() -> Any:
        if args.resume:
            result = LeRobotDataset.resume(args.repo_id, **options)
            if result.fps != args.fps:
                result.finalize()
                raise RuntimeError(
                    f"Existing dataset is {result.fps} FPS, requested {args.fps}"
                )
            print(
                f"resuming dataset at {result.num_episodes}/{args.successful_episodes}",
                flush=True,
            )
            return result
        return LeRobotDataset.create(
            args.repo_id,
            args.fps,
            robot_type="nero_right",
            features=features,
            use_videos=use_video,
            **options,
        )

    if args.synthetic_inputs:
        from nero_neo_teleop.recording.bimanual_lerobot_recorder import (
            SyntheticStateSource,
        )

        state_source: Any = SyntheticStateSource(0.0)
        world: Any = SyntheticCameraReader("world", args.width, args.height, args.fps)
        wrist: Any = SyntheticCameraReader(
            "right_wrist", args.width, args.height, args.fps
        )
    else:
        state_source = NeroCanStateSource(args.can, "right")
        world = V4L2CameraReader(
            resolved_device(args.world_camera),
            width=args.width,
            height=args.height,
            fps=args.fps,
            name="world",
        )
        wrist = V4L2CameraReader(
            resolved_device(args.right_wrist_camera),
            width=args.width,
            height=args.height,
            fps=args.fps,
            name="right_wrist",
        )

    receiver_path = args.action_socket or args.activity_socket
    receiver = (
        ArmCommandReceiver(receiver_path, "right")
        if receiver_path is not None
        else None
    )
    sources = (world, wrist, state_source)
    dataset = None
    try:
        if receiver is not None:
            receiver.start()
        for source in sources:
            source.start()
        for source in sources:
            source.wait_ready()
        print(
            f"inputs ready: right={args.can} cameras=2 "
            f"{args.width}x{args.height}@{args.fps}fps",
            flush=True,
        )
        dataset = open_dataset()
        config_path = args.root / "recording_config.json"
        config = {
            "schema_version": 1,
            "robot_type": "nero_right",
            "training_ready": False,
            "observation_state_semantics": "7_joint_position_rad_plus_gripper_opening",
            "action_source": args.action_source,
            "action_alignment": (
                "nearest_timestamped_executed_command"
                if args.action_source == "controller_command"
                else "next_feedback_sample"
            ),
            "jepa_wms_action_semantics": "not_converted",
            "gripper_convention": "opening_0_closed_1_open",
            "fps": args.fps,
            "task": args.task,
            "control_profile": args.control_profile,
            "start_detection": (
                args.start_detection if args.motion_start else "immediate"
            ),
        }
        if args.start_joints_deg is not None:
            config["start_joints_deg"] = args.start_joints_deg
        if args.inactivity_auto_stop_seconds > 0:
            config["inactivity_auto_stop_seconds"] = args.inactivity_auto_stop_seconds
        if config_path.exists():
            if json.loads(config_path.read_text(encoding="utf-8")) != config:
                raise RuntimeError(
                    "Recording configuration does not match resumed dataset"
                )
        elif args.resume:
            raise RuntimeError("Resumed dataset has no recording_config.json")
        else:
            config_path.write_text(json.dumps(config, indent=2, sort_keys=True) + "\n")

        attempts = 0
        while dataset.num_episodes < args.successful_episodes:
            attempts += 1
            if not args.auto_start:
                input(
                    f"Press Enter to PREPARE successful episode "
                    f"{dataset.num_episodes + 1}/{args.successful_episodes} "
                    f"(attempt {attempts}); recording starts only after motion: "
                )
            controller = None
            command_started_ns = time.monotonic_ns()
            if receiver is not None:
                receiver.clear()
            if args.managed_controller_command:
                controller = start_managed_controller(
                    args.managed_controller_command,
                    args.managed_controller_startup_sec,
                )
            period_ns = round(1e9 / args.fps)
            frame_limit = round(args.episode_seconds * args.fps)
            next_tick = time.monotonic_ns()
            previous: RightSample | None = None
            diagnostics: list[dict[str, Any]] = []
            inactivity = (
                CommandInactivityAutoStop(
                    inactivity_seconds=args.inactivity_auto_stop_seconds,
                    joint_speed_deg_s=args.inactivity_joint_speed_deg_s,
                    gripper_speed_per_s=args.inactivity_gripper_speed_per_s,
                )
                if args.inactivity_auto_stop_seconds > 0
                else None
            )

            def action_for(
                observation: RightSample, next_sample: RightSample
            ) -> RightAction:
                return (
                    command_action(observation, receiver)
                    if args.action_source == "controller_command"
                    and receiver is not None
                    else feedback_action(next_sample)
                )

            try:
                if receiver is not None:
                    receiver.wait_ready(after_ns=command_started_ns, timeout_sec=15.0)
                    # Managed startup includes force calibration. Do not retain
                    # a sampling deadline from before the first live command;
                    # it makes the first controller-state lookup appear stale.
                    next_tick = time.monotonic_ns()
                if args.motion_start:
                    preroll = round(args.motion_preroll_seconds * args.fps)
                    armed: deque[RightSample] = deque(
                        maxlen=preroll + args.motion_start_consecutive + 1
                    )
                    streak = 0
                    if args.start_detection == "controller":
                        print(
                            "armed; waiting for right controller Grip/velocity_tracking",
                            flush=True,
                        )
                    else:
                        print(
                            f"armed; waiting for right arm >="
                            f"{args.motion_start_threshold_deg_s:.1f}deg/s",
                            flush=True,
                        )
                    while True:
                        require_managed_controller_alive(controller)
                        wait_until(next_tick)
                        current = take_sample(next_tick, state_source, world, wrist)
                        armed.append(current)
                        if args.start_detection == "controller":
                            assert receiver is not None
                            control_state = command_action(current, receiver).control_state
                            active = controller_motion_started(control_state)
                            streak = streak + 1 if active else 0
                            if streak >= args.motion_start_consecutive:
                                print(
                                    f"controller motion detected in {control_state}; recording",
                                    flush=True,
                                )
                                next_tick += period_ns
                                break
                        elif previous is not None:
                            speed = joint_speed_deg_s(previous, current)
                            streak = (
                                streak + 1
                                if speed >= args.motion_start_threshold_deg_s
                                else 0
                            )
                            if streak >= args.motion_start_consecutive:
                                print(
                                    f"motion detected at {speed:.2f}deg/s; recording",
                                    flush=True,
                                )
                                next_tick += period_ns
                                break
                        previous = current
                        next_tick += period_ns
                    buffered = list(armed)
                    for observation, next_sample in zip(buffered[:-1], buffered[1:]):
                        action = action_for(observation, next_sample)
                        add_frame(dataset, observation, action, args.task)
                        diagnostics.append(
                            timing_record(len(diagnostics), observation, action)
                        )
                    previous = buffered[-1]

                print(
                    f"recording task={args.task!r} max={args.episode_seconds:.1f}s; "
                    f"press Enter to finish; inactivity auto-stop="
                    f"{args.inactivity_auto_stop_seconds:.1f}s",
                    flush=True,
                )
                while len(diagnostics) < frame_limit:
                    require_managed_controller_alive(controller)
                    wait_until(next_tick)
                    current = take_sample(next_tick, state_source, world, wrist)
                    if previous is not None:
                        action = action_for(previous, current)
                        add_frame(dataset, previous, action, args.task)
                        diagnostics.append(
                            timing_record(len(diagnostics), previous, action)
                        )
                        if inactivity is not None:
                            assert receiver is not None
                            activity = command_action(previous, receiver)
                            result = inactivity.update(
                                activity.vector, previous.scheduled_monotonic_ns
                            )
                            diagnostics[-1].update(
                                inactivity_state=result.state,
                                input_joint_speed_deg_s=result.joint_speed_deg_s,
                                input_gripper_speed_per_s=result.gripper_speed_per_s,
                            )
                            if result.should_stop:
                                print(
                                    "episode auto-stopped: no arm/gripper input for "
                                    f"{args.inactivity_auto_stop_seconds:.1f}s",
                                    flush=True,
                                )
                                break
                    previous = current
                    next_tick += period_ns
                    if diagnostics and finish_requested():
                        print(
                            f"episode stopped at {len(diagnostics) / args.fps:.2f}s",
                            flush=True,
                        )
                        break
            finally:
                if controller is not None:
                    stop_managed_controller(
                        controller,
                        allow_existing_failure=sys.exc_info()[0] is not None,
                    )

            if args.return_home_command:
                if args.return_delay_seconds:
                    time.sleep(args.return_delay_seconds)
                run_home_command(args.return_home_command, args.return_timeout_sec)

            duration = len(diagnostics) / args.fps
            if duration < args.min_episode_seconds:
                dataset.clear_episode_buffer()
                print(
                    f"episode rejected: only {duration:.2f}s; replacement required",
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
                    f"success={dataset.num_episodes}/{args.successful_episodes} "
                    f"attempts={attempts} diagnostics={path}",
                    flush=True,
                )
            else:
                dataset.clear_episode_buffer()
                if decision == "quit":
                    print("pending episode discarded; quitting", flush=True)
                    break
                print("episode discarded; replacement attempt required", flush=True)
        if dataset.num_episodes == args.successful_episodes:
            print(
                f"target reached: {dataset.num_episodes} successful episodes",
                flush=True,
            )
        print(
            f"camera_rates_hz world={world.measured_hz:.1f} "
            f"right_wrist={wrist.measured_hz:.1f}",
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
        if receiver is not None:
            receiver.stop()
        for source in reversed(sources):
            source.stop()
        if dataset is not None:
            dataset.finalize()


if __name__ == "__main__":
    main()
