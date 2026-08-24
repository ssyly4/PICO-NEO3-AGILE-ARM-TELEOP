#!/usr/bin/env python3
"""Preview or move both follower NERO arms to the community mirrored Home."""

from __future__ import annotations

import argparse
from datetime import datetime
import json
from pathlib import Path
import threading
import time

import numpy as np
from pyAgxArm import AgxArmFactory, ArmModel, NeroFW, create_agx_arm_config

from nero_vla.dual_can import require_bridge_not_forwarding, require_can_role
from nero_vla.robot_config import NERO_CPV_JOINT_LIMIT_OVERRIDES_RAD

from nero_neo_teleop.robot.home_config import (
    LEFT_CAN,
    LEFT_HOME_RAD,
    RIGHT_CAN,
    RIGHT_HOME_RAD,
)
from nero_neo_teleop.robot.nero_io import (
    check_driver_health,
    check_gripper_health,
    wait_complete_joint_feedback,
    wait_gripper_status,
)
from nero_neo_teleop.runtime import ARTIFACTS_DIR


CONFIRMATION = "MOVE BOTH NERO ARMS TO COMMUNITY HOME"


def wait_live_joint_feedback(robot, timeout_sec: float = 1.0) -> np.ndarray:
    """Read one complete sample while an arm is intentionally moving."""
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        feedback = robot.get_joint_angles()
        if feedback is not None:
            value = np.asarray(feedback.msg, dtype=np.float64)
            if value.shape == (7,) and np.isfinite(value).all():
                return value
        time.sleep(0.005)
    raise RuntimeError("No complete seven-joint CAN feedback during dual Home motion")


def create_robot(interface: str):
    config = create_agx_arm_config(
        robot=ArmModel.NERO,
        firmeware_version=NeroFW.V120,
        interface="socketcan",
        channel=interface,
        joint_limits=NERO_CPV_JOINT_LIMIT_OVERRIDES_RAD,
    )
    robot = AgxArmFactory.create_arm(config)
    robot.set_joint_limits_enabled(True)
    robot.connect()
    return robot, config


def limits_from_config(config) -> np.ndarray:
    return np.asarray(
        [config["joint_limits"][f"joint{index}"] for index in range(1, 8)],
        dtype=np.float64,
    )


def send_parallel(left_robot, right_robot) -> None:
    barrier = threading.Barrier(3)
    failures: list[BaseException] = []

    def send(robot, target: np.ndarray) -> None:
        try:
            barrier.wait()
            robot.move_j(target.astype(float).tolist())
        except BaseException as exc:
            failures.append(exc)

    threads = [
        threading.Thread(target=send, args=(left_robot, LEFT_HOME_RAD), daemon=True),
        threading.Thread(target=send, args=(right_robot, RIGHT_HOME_RAD), daemon=True),
    ]
    for thread in threads:
        thread.start()
    barrier.wait()
    for thread in threads:
        thread.join(timeout=3.0)
    if any(thread.is_alive() for thread in threads):
        raise TimeoutError("A parallel move_j call did not return")
    if failures:
        raise RuntimeError(f"Parallel move_j failed: {failures!r}")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--left-can", default=LEFT_CAN)
    parser.add_argument("--right-can", default=RIGHT_CAN)
    parser.add_argument("--speed-percent", type=int, default=5)
    parser.add_argument("--timeout", type=float, default=45.0)
    parser.add_argument("--max-travel-deg", type=float, default=120.0)
    parser.add_argument("--tolerance-deg", type=float, default=1.0)
    parser.add_argument("--gripper-open-mm", type=float, default=90.0)
    parser.add_argument("--gripper-force-n", type=float, default=1.0)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--confirm", default="")
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=ARTIFACTS_DIR / "logs/dual_home",
    )
    args = parser.parse_args()

    if args.left_can == args.right_can:
        parser.error("left-can and right-can must be different interfaces")
    if not 1 <= args.speed_percent <= 10:
        parser.error("speed-percent must be in [1, 10]")
    if args.timeout <= 0 or args.max_travel_deg <= 0 or args.tolerance_deg <= 0:
        parser.error("timeout, max-travel-deg and tolerance-deg must be positive")
    if not 0.0 < args.gripper_open_mm <= 100.0:
        parser.error("gripper-open-mm must be in (0, 100]")
    if not 0.1 <= args.gripper_force_n <= 3.0:
        parser.error("gripper-force-n must be in [0.1, 3]")

    require_can_role(args.left_can, "follower")
    require_can_role(args.right_can, "follower")
    require_bridge_not_forwarding()

    left_robot = right_robot = None
    record: dict = {
        "left_can": args.left_can,
        "right_can": args.right_can,
        "left_home_rad": LEFT_HOME_RAD.tolist(),
        "right_home_rad": RIGHT_HOME_RAD.tolist(),
        "execute": args.execute,
        "completed": False,
    }
    try:
        left_robot, left_config = create_robot(args.left_can)
        right_robot, right_config = create_robot(args.right_can)
        left_start = wait_complete_joint_feedback(left_robot)
        right_start = wait_complete_joint_feedback(right_robot)
        check_driver_health(left_robot)
        check_driver_health(right_robot)

        for label, target, config in (
            ("left", LEFT_HOME_RAD, left_config),
            ("right", RIGHT_HOME_RAD, right_config),
        ):
            limits = limits_from_config(config)
            if np.any((target < limits[:, 0]) | (target > limits[:, 1])):
                raise RuntimeError(f"{label} community Home violates SDK joint limits")

        left_travel = float(np.max(np.abs(np.rad2deg(LEFT_HOME_RAD - left_start))))
        right_travel = float(np.max(np.abs(np.rad2deg(RIGHT_HOME_RAD - right_start))))
        planned_travel = max(left_travel, right_travel)
        record.update(
            left_start_deg=np.rad2deg(left_start).tolist(),
            right_start_deg=np.rad2deg(right_start).tolist(),
            planned_max_travel_deg=planned_travel,
        )
        print(f"left_start_deg={np.round(np.rad2deg(left_start), 2).tolist()}")
        print(f"right_start_deg={np.round(np.rad2deg(right_start), 2).tolist()}")
        print(f"left_home_deg={np.round(np.rad2deg(LEFT_HOME_RAD), 2).tolist()}")
        print(f"right_home_deg={np.round(np.rad2deg(RIGHT_HOME_RAD), 2).tolist()}")
        print(
            f"planned_max_travel={planned_travel:.2f}deg "
            f"speed={args.speed_percent}%"
        )
        if planned_travel > args.max_travel_deg:
            raise RuntimeError(
                f"Planned travel {planned_travel:.2f}deg exceeds "
                f"guard {args.max_travel_deg:.2f}deg"
            )
        if not args.execute:
            print(
                f"PREVIEW ONLY: add --execute --confirm {CONFIRMATION!r} to move"
            )
            return
        if args.confirm != CONFIRMATION:
            raise RuntimeError(
                f"Execution not authorized; pass --confirm {CONFIRMATION!r}"
            )

        for robot in (left_robot, right_robot):
            if not robot.enable(timeout=2.0):
                raise RuntimeError("A NERO arm could not be enabled")
            robot.set_speed_percent(args.speed_percent)
            robot.set_auto_set_motion_mode_enabled(False)
            robot.set_motion_mode(robot.OPTIONS.MOTION_MODE.J)
        time.sleep(0.2)
        send_parallel(left_robot, right_robot)

        deadline = time.monotonic() + args.timeout
        stable = 0
        last_report = 0.0
        while time.monotonic() < deadline:
            left = wait_live_joint_feedback(left_robot, timeout_sec=1.0)
            right = wait_live_joint_feedback(right_robot, timeout_sec=1.0)
            left_error = np.rad2deg(LEFT_HOME_RAD - left)
            right_error = np.rad2deg(RIGHT_HOME_RAD - right)
            max_error = max(
                float(np.max(np.abs(left_error))),
                float(np.max(np.abs(right_error))),
            )
            stable = stable + 1 if max_error <= args.tolerance_deg else 0
            now = time.monotonic()
            if now >= last_report:
                print(
                    f"left_error_deg={np.round(left_error, 2).tolist()} "
                    f"right_error_deg={np.round(right_error, 2).tolist()}"
                )
                last_report = now + 1.0
            check_driver_health(left_robot)
            check_driver_health(right_robot)
            if stable >= 3:
                break
            time.sleep(0.05)
        else:
            raise TimeoutError("Dual-arm community Home motion timed out")

        left_settled = wait_complete_joint_feedback(left_robot, timeout_sec=5.0)
        right_settled = wait_complete_joint_feedback(right_robot, timeout_sec=5.0)
        settled_error = max(
            float(np.max(np.abs(np.rad2deg(LEFT_HOME_RAD - left_settled)))),
            float(np.max(np.abs(np.rad2deg(RIGHT_HOME_RAD - right_settled)))),
        )
        if settled_error > args.tolerance_deg:
            raise RuntimeError(
                "Dual Home feedback did not settle inside tolerance: "
                f"max_error_deg={settled_error:.3f}"
            )

        for label, robot in (("left", left_robot), ("right", right_robot)):
            gripper = robot.init_effector(robot.OPTIONS.EFFECTOR.AGX_GRIPPER)
            status = wait_gripper_status(gripper)
            check_gripper_health(status)
            gripper.move_gripper_m(
                value=args.gripper_open_mm / 1000.0,
                force=args.gripper_force_n,
            )
            print(f"{label} gripper opening to {args.gripper_open_mm:.1f}mm")
        record["completed"] = True
        print("PASS: both NERO arms reached the community mirrored Home")
    finally:
        for robot in (left_robot, right_robot):
            if robot is not None:
                robot.disconnect()
        args.output_dir.mkdir(parents=True, exist_ok=True)
        log_path = args.output_dir / f"home_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
        log_path.write_text(json.dumps(record, indent=2, sort_keys=True) + "\n")
        print(f"dual Home log={log_path}")


if __name__ == "__main__":
    main()
