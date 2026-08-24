#!/usr/bin/env python3
"""Independent NERO PICO Neo 3 Servo v3: velocity IK and finite-lead CPV."""

from __future__ import annotations

import argparse
from datetime import datetime
import gc
import json
from pathlib import Path
import signal
import time

import numpy as np
from pyAgxArm import AgxArmFactory, ArmModel, NeroFW, create_agx_arm_config


from nero_vla.cpv_backend import NeroCpvPositionBackend
from nero_vla.dual_can import require_bridge_not_forwarding, require_can_role

from nero_neo_teleop.control.contact_force_guard import DownwardContactGuard, TcpForceEstimator
from nero_neo_teleop.control.gripper import GripperAnalogController
from nero_neo_teleop.control.servo_v3_core import (
    bounded_pose_target,
    bounded_transport_step,
    extrapolate_controller_state,
    FiniteLeadCommandFollower,
    PinocchioVelocityServo,
    PoseLowPassFilter,
)
from nero_neo_teleop.pico.pico_input import PicoInputMonitor
from nero_neo_teleop.pico.pose_mapper import (
    OPENXR_TO_NERO,
    ClutchedPoseMapper,
    Pose,
    head_yaw_world_to_view,
    matrix_to_quaternion,
    rpy_to_matrix,
    transform_state_to_view,
)
from nero_neo_teleop.recording.action_command_stream import ArmCommandPublisher
from nero_neo_teleop.robot.nero_io import (
    check_driver_health,
    check_gripper_health,
    flange_pose,
    wait_complete_joint_feedback,
    wait_cpv_mode,
    wait_enabled,
    wait_gripper_status,
)
from nero_neo_teleop.runtime import ARTIFACTS_DIR, NERO_URDF


def rotation_error_deg(left: np.ndarray, right: np.ndarray) -> float:
    import pinocchio as pin

    return float(np.rad2deg(np.linalg.norm(pin.log3(left.T @ right))))


def json_default(value):
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, np.ndarray):
        return value.tolist()
    raise TypeError(f"Object of type {type(value).__name__} is not JSON serializable")


def complete_feedback(robot, timeout_sec: float = 0.10) -> np.ndarray:
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        message = robot.get_joint_angles()
        if message is not None:
            value = np.asarray(message.msg, dtype=np.float64)
            if value.shape == (7,) and np.isfinite(value).all():
                return value
        time.sleep(0.001)
    raise RuntimeError("No complete seven-joint feedback in Servo v3 loop")


def complete_motor_torques(robot, timeout_sec: float = 0.10) -> np.ndarray:
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        states = [robot.get_motor_states(index) for index in range(1, 8)]
        if all(state is not None for state in states):
            values = np.asarray([state.msg.torque for state in states], dtype=np.float64)
            if values.shape == (7,) and np.isfinite(values).all():
                return values
        time.sleep(0.001)
    raise RuntimeError("No complete seven-joint motor torque feedback")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bind", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=50150)
    parser.add_argument("--hand", choices=("left", "right"), default="right")
    parser.add_argument("--can-port", default="can0")
    parser.add_argument("--duration", type=float, default=120.0)
    parser.add_argument("--rate-hz", type=float, default=30.0)
    parser.add_argument("--translation-scale", type=float, default=0.50)
    parser.add_argument("--max-translation-mm", type=float, default=300.0)
    parser.add_argument("--rotation-scale", type=float, default=0.50)
    parser.add_argument("--max-rotation-deg", type=float, default=20.0)
    parser.add_argument("--position-filter-hz", type=float, default=10.0)
    parser.add_argument("--rotation-filter-hz", type=float, default=8.0)
    parser.add_argument("--position-gain-s", type=float, default=6.0)
    parser.add_argument("--rotation-gain-s", type=float, default=4.0)
    parser.add_argument("--max-linear-speed-mm-s", type=float, default=120.0)
    parser.add_argument("--max-angular-speed-deg-s", type=float, default=60.0)
    parser.add_argument("--max-velocity-deg-s", type=float, default=15.0)
    parser.add_argument("--max-acceleration-deg-s2", type=float, default=60.0)
    parser.add_argument("--command-lead-ms", type=float, default=67.0)
    parser.add_argument("--max-command-lead-deg", type=float, default=1.25)
    parser.add_argument("--max-cpv-step-deg", type=float, default=0.85)
    parser.add_argument("--nullspace-gain-s", type=float, default=0.12)
    parser.add_argument("--orientation-limit-soft-margin-deg", type=float, default=12.0)
    parser.add_argument("--orientation-limit-hard-margin-deg", type=float, default=3.0)
    parser.add_argument("--max-executable-position-lead-mm", type=float, default=35.0)
    parser.add_argument("--max-executable-rotation-lead-deg", type=float, default=12.0)
    parser.add_argument("--grip-engage-threshold", type=float, default=0.25)
    parser.add_argument("--grip-release-threshold", type=float, default=0.12)
    parser.add_argument("--max-packet-age-ms", type=float, default=120.0)
    parser.add_argument("--network-prediction-ms", type=float, default=250.0)
    parser.add_argument("--clutch-reset-gap-ms", type=float, default=500.0)
    parser.add_argument("--gripper-open-width-mm", type=float, default=90.0)
    parser.add_argument("--gripper-closed-width-mm", type=float, default=0.0)
    parser.add_argument("--gripper-force-n", type=float, default=1.0)
    parser.add_argument(
        "--action-socket",
        default="",
        help="optional Unix datagram destination for timestamped executed commands",
    )
    parser.add_argument("--force-calibration-sec", type=float, default=1.5)
    parser.add_argument("--force-filter-hz", type=float, default=3.0)
    parser.add_argument(
        "--downward-force-guard-n",
        type=float,
        default=0.0,
        help="block further TCP descent above this estimated force; 0 only logs force",
    )
    parser.add_argument("--downward-force-release-n", type=float, default=4.0)
    parser.add_argument("--invert-forward", action="store_true")
    parser.add_argument("--invert-lateral", action="store_true")
    parser.add_argument("--execute", action="store_true")
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=ARTIFACTS_DIR / "logs/servo_v3",
    )
    args = parser.parse_args()
    if not 30.0 <= args.rate_hz <= 40.0:
        parser.error("Servo v3 control rate must be in [30, 40] Hz")
    if not 1.0 <= args.max_velocity_deg_s <= 35.0:
        parser.error("max-velocity-deg-s must be in [1, 35]")
    if not 5.0 <= args.max_acceleration_deg_s2 <= 300.0:
        parser.error("max-acceleration-deg-s2 must be in [5, 300]")
    if not 10.0 <= args.command_lead_ms <= 100.0:
        parser.error("command-lead-ms must be in [10, 100]")
    if not args.max_packet_age_ms <= args.network_prediction_ms <= 300.0:
        parser.error("network-prediction-ms must be in [max-packet-age-ms, 300]")
    if args.force_calibration_sec <= 0.0 or args.force_filter_hz <= 0.0:
        parser.error("force calibration and filter parameters must be positive")
    if args.downward_force_guard_n < 0.0:
        parser.error("downward-force-guard-n must be nonnegative")
    if args.downward_force_guard_n > 0.0 and not (
        0.0 <= args.downward_force_release_n < args.downward_force_guard_n
    ):
        parser.error("force release threshold must be below the guard threshold")
    return args


def main() -> None:
    args = parse_args()
    command_publisher = ArmCommandPublisher(args.action_socket, args.hand)
    stop_requested = False

    def stop_signal(_signum, _frame) -> None:
        nonlocal stop_requested
        if stop_requested:
            return
        stop_requested = True
        raise KeyboardInterrupt

    signal.signal(signal.SIGINT, stop_signal)
    signal.signal(signal.SIGTERM, stop_signal)
    require_can_role(args.can_port, "follower")
    require_bridge_not_forwarding()
    monitor = PicoInputMonitor(args.bind, args.port)
    monitor.start()
    robot = None
    backend = None
    rows: list[dict[str, object]] = []
    completion = "preflight"
    gc_was_enabled = gc.isenabled()
    if gc_was_enabled:
        gc.disable()
    try:
        pico_sample = monitor.wait_ready(args.hand)
        servo = PinocchioVelocityServo(
            NERO_URDF,
            position_gain_s=args.position_gain_s,
            rotation_gain_s=args.rotation_gain_s,
            max_linear_speed_m_s=args.max_linear_speed_mm_s / 1000.0,
            max_angular_speed_rad_s=np.deg2rad(args.max_angular_speed_deg_s),
            max_joint_speed_rad_s=np.deg2rad(args.max_velocity_deg_s),
            nullspace_gain_s=args.nullspace_gain_s,
            orientation_limit_soft_margin_rad=np.deg2rad(
                args.orientation_limit_soft_margin_deg
            ),
            orientation_limit_hard_margin_rad=np.deg2rad(
                args.orientation_limit_hard_margin_deg
            ),
        )
        config = create_agx_arm_config(
            robot=ArmModel.NERO,
            firmeware_version=NeroFW.V120,
            interface="socketcan",
            channel=args.can_port,
        )
        robot = AgxArmFactory.create_arm(config)
        robot.set_joint_limits_enabled(True)
        robot.connect()
        gripper = robot.init_effector(robot.OPTIONS.EFFECTOR.AGX_GRIPPER)
        firmware = robot.get_firmware(timeout=2.0, min_interval=0.0)
        measured = wait_complete_joint_feedback(robot)
        lower = servo.lower + servo.limit_margin_rad
        upper = servo.upper - servo.limit_margin_rad
        if np.any(measured <= lower) or np.any(measured >= upper):
            raise RuntimeError("Initial joints are inside the Servo v3 limit margin")
        check_driver_health(robot)
        gripper_status = wait_gripper_status(gripper)
        check_gripper_health(gripper_status)

        initial_se3 = servo.pose(measured)
        initial_pose = Pose(initial_se3.translation.copy(), initial_se3.rotation.copy())
        sdk_pose = flange_pose(robot)
        fk_position_error_mm = 1000.0 * float(
            np.linalg.norm(initial_pose.position - sdk_pose[:3])
        )
        fk_rotation_error_deg = rotation_error_deg(
            initial_pose.rotation, rpy_to_matrix(sdk_pose[3:])
        )
        if fk_position_error_mm > 1.0 or fk_rotation_error_deg > 0.2:
            raise RuntimeError(
                f"Official FK does not match SDK flange: {fk_position_error_mm:.3f}mm/"
                f"{fk_rotation_error_deg:.3f}deg"
            )

        basis = OPENXR_TO_NERO.copy()
        if args.invert_forward:
            basis[0] *= -1.0
        if args.invert_lateral:
            basis[1] *= -1.0
        mapper = ClutchedPoseMapper(
            initial_pose,
            translation_scale=args.translation_scale,
            max_translation_m=args.max_translation_mm / 1000.0,
            max_rotation_rad=np.deg2rad(args.max_rotation_deg),
            rotation_scale=args.rotation_scale,
            engage_threshold=args.grip_engage_threshold,
            release_threshold=args.grip_release_threshold,
            basis=basis,
        )
        pose_filter = PoseLowPassFilter(
            position_cutoff_hz=args.position_filter_hz,
            rotation_cutoff_hz=args.rotation_filter_hz,
        )
        pose_filter.reset(initial_pose.position, initial_pose.rotation)
        command_follower = FiniteLeadCommandFollower(
            7,
            max_velocity_rad_s=np.deg2rad(args.max_velocity_deg_s),
            max_acceleration_rad_s2=np.deg2rad(args.max_acceleration_deg_s2),
            command_lead_sec=args.command_lead_ms / 1000.0,
            max_command_lead_rad=np.deg2rad(args.max_command_lead_deg),
        )
        force_estimator = TcpForceEstimator(
            servo.model,
            servo.frame_id,
            cutoff_hz=args.force_filter_hz,
        )
        force_guard = (
            DownwardContactGuard(
                engage_force_n=args.downward_force_guard_n,
                release_force_n=args.downward_force_release_n,
            )
            if args.downward_force_guard_n > 0.0
            else None
        )
        gripper_control = GripperAnalogController(
            open_m=args.gripper_open_width_mm / 1000.0,
            closed_m=args.gripper_closed_width_mm / 1000.0,
            initial_feedback_m=float(gripper_status.msg.value),
            min_command_step_m=0.001,
        )
        sent_q = measured.copy()
        print(
            f"[{args.hand}] keep the arm unloaded/still for "
            f"{args.force_calibration_sec:.1f}s force calibration",
            flush=True,
        )
        calibration_q: list[np.ndarray] = []
        calibration_torque: list[np.ndarray] = []
        calibration_deadline = time.monotonic() + args.force_calibration_sec
        while time.monotonic() < calibration_deadline:
            calibration_q.append(complete_feedback(robot))
            calibration_torque.append(complete_motor_torques(robot))
            time.sleep(0.025)
        force_estimator.calibrate(
            np.asarray(calibration_q), np.asarray(calibration_torque)
        )
        print(f"[{args.hand}] QUEST SERVO V3 PREFLIGHT PASSED", flush=True)
        print(
            f"hand={args.hand} can={args.can_port} peer={pico_sample.peer[0]} firmware={firmware} "
            f"rate={args.rate_hz:g}Hz finite_lead={args.command_lead_ms:g}ms/"
            f"{args.max_command_lead_deg:g}deg speed/accel="
            f"{args.max_velocity_deg_s:g}/{args.max_acceleration_deg_s2:g}",
            flush=True,
        )
        if not args.execute:
            print("READ-ONLY COMPLETE. Add --execute to move.", flush=True)
            return

        wait_enabled(robot)
        backend = NeroCpvPositionBackend(
            robot, max_command_step_rad=np.deg2rad(args.max_cpv_step_deg + 0.05)
        )
        backend.prepare_hold(measured)
        cpv_error = None
        for cpv_attempt in range(1, 4):
            try:
                wait_cpv_mode(robot, timeout_sec=2.0)
                cpv_error = None
                break
            except RuntimeError as exc:
                cpv_error = exc
                if cpv_attempt == 3:
                    break
                print(
                    f"[{args.hand}] CPV handoff {cpv_attempt}/3 not confirmed: {exc}; retrying",
                    flush=True,
                )
                wait_enabled(robot)
                backend.hold()
                robot.set_motion_mode("cpv")
                backend.hold()
                time.sleep(0.15)
        if cpv_error is not None:
            raise RuntimeError(f"CPV startup failed after 3 attempts: {cpv_error}")
        print(
            f"Hold {args.hand} Grip to move; release to hold/re-anchor. "
            "Trigger controls gripper. Keep E-stop ready.",
            flush=True,
        )

        started = time.monotonic()
        previous_tick = started
        next_tick = started
        last_report = started - 1.0
        last_health = started
        command_sequence = 0
        world_to_view = None
        while time.monotonic() - started < args.duration:
            now = time.monotonic()
            if now < next_tick:
                time.sleep(next_tick - now)
                now = time.monotonic()
            period = 1.0 / args.rate_hz
            next_tick += period
            if next_tick < now - period:
                next_tick = now + period
            dt = max(1e-4, min(now - previous_tick, 2.0 * period))
            previous_tick = now
            measured = complete_feedback(robot)
            measured_se3 = servo.pose(measured)
            measured_pose = Pose(measured_se3.translation.copy(), measured_se3.rotation.copy())
            force_sample = force_estimator.update(
                measured, complete_motor_torques(robot), dt
            )
            vertical_force_n = force_sample.vertical_force_n
            contact_force_n = abs(vertical_force_n)
            relative_contact_load_n = 0.0
            force_guard_state = "monitor_only"
            latest = monitor.latest()
            packet_age_ms = float("inf")
            if latest.received_monotonic is not None:
                packet_age_ms = 1000.0 * (now - latest.received_monotonic)
            hand_state = None if latest.sample is None else latest.sample.packet.get(args.hand)
            head_state = None if latest.sample is None else latest.sample.packet.get("head")
            tracked = bool(hand_state is not None and hand_state.get("tracked", False))
            fresh = tracked and packet_age_ms <= args.max_packet_age_ms
            predicted_input = (
                tracked
                and args.max_packet_age_ms < packet_age_ms <= args.network_prediction_ms
            )
            grip = 0.0 if hand_state is None else float(hand_state.get("grip", 0.0))
            state = "input_hold"
            filtered_position = measured_pose.position.copy()
            filtered_rotation = measured_pose.rotation.copy()
            ik = None
            ik_ms = None
            desired_velocity = np.zeros(7, dtype=np.float64)
            cartesian_limited = False
            target_debt_position_mm = 0.0
            target_debt_orientation_deg = 0.0

            if fresh or predicted_input:
                effective_hand_state = hand_state
                if predicted_input:
                    effective_hand_state = extrapolate_controller_state(
                        hand_state, packet_age_ms / 1000.0
                    )
                trigger = float(hand_state.get("trigger", 0.0))
                gripper_output = gripper_control.update(
                    trigger=trigger, input_valid=not predicted_input
                )
                if not predicted_input and gripper_output.command:
                    gripper.move_gripper_m(
                        value=gripper_output.target_m, force=args.gripper_force_n
                    )
                if (
                    not mapper.engaged
                    and grip >= mapper.engage_threshold
                    and head_state is not None
                    and bool(head_state.get("tracked", False))
                ):
                    world_to_view = head_yaw_world_to_view(head_state)
                if world_to_view is not None:
                    mapped = mapper.update(
                        transform_state_to_view(effective_hand_state, world_to_view)
                    )
                    state = mapped.state
                    if state == "released_hold":
                        mapper.reset_target(measured_pose)
                        world_to_view = None
                        pose_filter.reset(measured_pose.position, measured_pose.rotation)
                        command_follower.reset()
                    elif state in {"clutch_engaged", "tracking"}:
                        filtered_position, filtered_rotation = pose_filter.update(
                            mapped.target.position, mapped.target.rotation, dt
                        )
                        if force_guard is not None:
                            filtered_position, force_guard_state = force_guard.apply(
                                filtered_position,
                                measured_pose.position,
                                vertical_force_n,
                            )
                            relative_contact_load_n = force_guard.relative_load_n
                            if force_guard.active:
                                pose_filter.reset(filtered_position, filtered_rotation)
                        executable = bounded_pose_target(
                            measured_pose.position,
                            measured_pose.rotation,
                            filtered_position,
                            filtered_rotation,
                            max_position_lead_m=(
                                args.max_executable_position_lead_mm / 1000.0
                            ),
                            max_orientation_lead_rad=np.deg2rad(
                                args.max_executable_rotation_lead_deg
                            ),
                        )
                        filtered_position = executable.position
                        filtered_rotation = executable.rotation
                        cartesian_limited = executable.limited
                        target_debt_position_mm = executable.position_debt_mm
                        target_debt_orientation_deg = executable.orientation_debt_deg
                        if executable.limited:
                            pose_filter.reset(filtered_position, filtered_rotation)
                        solve_started = time.perf_counter()
                        ik = servo.solve(measured, filtered_position, filtered_rotation)
                        ik_ms = 1000.0 * (time.perf_counter() - solve_started)
                        desired_velocity = ik.joint_velocity
                        state = "network_predict" if predicted_input else "velocity_tracking"
            else:
                gripper_control.update(trigger=0.0, input_valid=False)
                state = "network_hold" if tracked else "tracking_hold"
                # A hard input hold must discard the old controller anchor.
                # Otherwise the first recovered packet can command the full
                # Cartesian lead envelope from a stale hand pose.
                if mapper.engaged:
                    mapper.reset_target(measured_pose)
                    world_to_view = None
                    pose_filter.reset(measured_pose.position, measured_pose.rotation)
                    command_follower.reset()
                    state = "network_reset"

            followed = command_follower.step(desired_velocity, measured, dt, lower, upper)
            transport_q, transport_limited = bounded_transport_step(
                sent_q, followed.command, np.deg2rad(args.max_cpv_step_deg)
            )
            backend.send(transport_q)
            sent_q = transport_q
            command_publisher.publish(
                joint_target_rad=sent_q,
                gripper_target_normalized=np.clip(
                    (gripper_control.target_m - args.gripper_closed_width_mm / 1000.0)
                    / (
                        (args.gripper_open_width_mm - args.gripper_closed_width_mm)
                        / 1000.0
                    ),
                    0.0,
                    1.0,
                ),
                monotonic_ns=time.monotonic_ns(),
                sequence=command_sequence,
                control_state=state,
            )
            command_sequence += 1
            status = robot.get_arm_status()
            if status is not None and int(status.msg.arm_status) != 0:
                raise RuntimeError(f"Arm status became abnormal: {status.msg.arm_status}")
            command_pose = servo.pose(sent_q)
            rows.append(
                {
                    "elapsed_sec": now - started,
                    "state": state,
                    "packet_age_ms": None if not np.isfinite(packet_age_ms) else packet_age_ms,
                    "packet_sequence": (
                        None if latest.sample is None else int(latest.sample.packet["sequence"])
                    ),
                    "pico_source_monotonic_sec": (
                        None
                        if latest.sample is None
                        else float(latest.sample.packet["monotonicTimeSec"])
                    ),
                    "pico_transport_age_ms": (
                        None if latest.sample is None else latest.sample.age_ms
                    ),
                    "grip_input": grip,
                    "estimated_tcp_wrench_world": force_sample.wrench_world.tolist(),
                    "estimated_vertical_force_n": vertical_force_n,
                    "estimated_contact_force_n": contact_force_n,
                    "estimated_relative_contact_load_n": relative_contact_load_n,
                    "downward_force_guard_state": force_guard_state,
                    "ik_ms": ik_ms,
                    "position_error_mm": None if ik is None else ik.position_error_mm,
                    "orientation_error_deg": None if ik is None else ik.orientation_error_deg,
                    "minimum_singular_value": None if ik is None else ik.minimum_singular_value,
                    "damping": None if ik is None else ik.damping,
                    "nullspace_velocity_norm": None if ik is None else ik.nullspace_velocity_norm,
                    "joint_limit_margin_deg": None if ik is None else ik.joint_limit_margin_deg,
                    "orientation_limit_scale": (
                        None if ik is None else ik.orientation_limit_scale
                    ),
                    "cartesian_limited": cartesian_limited,
                    "target_debt_position_mm": target_debt_position_mm,
                    "target_debt_orientation_deg": target_debt_orientation_deg,
                    "command_lead_deg": followed.command_lead_deg,
                    "lead_limited": followed.lead_limited,
                    "transport_limited": transport_limited,
                    "limit_clamped": followed.limit_clamped,
                    "desired_joint_velocity_deg_s": np.rad2deg(desired_velocity).tolist(),
                    "limited_joint_velocity_deg_s": np.rad2deg(followed.joint_velocity).tolist(),
                    "measured_joint_deg": np.rad2deg(measured).tolist(),
                    "command_joint_deg": np.rad2deg(sent_q).tolist(),
                    "measured_tcp_position_m": measured_pose.position.tolist(),
                    "command_tcp_position_m": command_pose.translation.tolist(),
                    "filtered_target_position_m": filtered_position.tolist(),
                    "filtered_target_quaternion_xyzw": matrix_to_quaternion(filtered_rotation).tolist(),
                    "pico_rx_hz": monitor.stream.rx_hz,
                    "pico_loss_percent": monitor.stream.loss_percent,
                }
            )
            if now - last_report >= 0.5:
                print(
                    f"[{args.hand}] t={now-started:5.1f}s state={state:17s} "
                    f"age={packet_age_ms:5.1f}ms "
                    f"lead={followed.command_lead_deg:.3f}deg ik_ms={ik_ms} "
                    f"sigma={None if ik is None else round(ik.minimum_singular_value, 3)} "
                    f"margin={None if ik is None else round(ik.joint_limit_margin_deg, 1)}deg "
                    f"rot_scale={None if ik is None else round(ik.orientation_limit_scale, 2)} "
                    f"Fz={vertical_force_n:+.1f}N dF={relative_contact_load_n:.1f}N "
                    f"force_guard={force_guard_state} "
                    f"debt={target_debt_position_mm:.1f}mm/"
                    f"{target_debt_orientation_deg:.1f}deg",
                    flush=True,
                )
                last_report = now
            if now - last_health >= 1.0:
                check_driver_health(robot)
                latest_gripper = gripper.get_gripper_status()
                if latest_gripper is not None:
                    check_gripper_health(latest_gripper)
                last_health = now
        completion = "passed"
        print(f"[{args.hand}] PASS: PICO Servo v3 finished and is holding", flush=True)
    except KeyboardInterrupt:
        completion = "keyboard_interrupt"
        print(f"[{args.hand}] Keyboard interrupt: CPV holding", flush=True)
    finally:
        # The dual wrapper forwards SIGINT to both workers after the terminal
        # has already delivered it to the process group. Final CPV hold and
        # log flushing must not be interrupted by that duplicate signal.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        if backend is not None:
            try:
                backend.hold()
            except Exception:
                pass
        if robot is not None:
            robot.disconnect()
        monitor.stop()
        if gc_was_enabled:
            gc.enable()
            gc.collect()
        if rows:
            run_id = datetime.now().strftime("run_%Y%m%d_%H%M%S_%f")
            run_dir = args.output_dir / run_id
            run_dir.mkdir(parents=True, exist_ok=False)
            samples_text = "".join(
                json.dumps(row, sort_keys=True, default=json_default) + "\n"
                for row in rows
            )
            (run_dir / "samples.jsonl").write_text(samples_text, encoding="utf-8")
            (run_dir / "summary.json").write_text(
                json.dumps({"completion": completion, "samples": len(rows)}, indent=2) + "\n",
                encoding="utf-8",
            )
            print(f"[{args.hand}] Servo v3 log={run_dir}", flush=True)
        if args.action_socket:
            print(
                f"[{args.hand}] action stream sent={command_publisher.sent} "
                f"dropped={command_publisher.dropped}",
                flush=True,
            )
        command_publisher.close()


if __name__ == "__main__":
    main()
