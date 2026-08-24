#!/usr/bin/env python3
"""Move one follower NERO arm to a configured PICO teleoperation Home."""

from __future__ import annotations

import argparse
from datetime import datetime
import json
from pathlib import Path
import time

import numpy as np
from pyAgxArm import AgxArmFactory, ArmModel, NeroFW, create_agx_arm_config


from nero_vla.dual_can import require_bridge_not_forwarding, require_can_role
from nero_vla.robot_config import NERO_CPV_JOINT_LIMIT_OVERRIDES_RAD

from nero_neo_teleop.robot.home_config import LEFT_HOME_RAD, RIGHT_HOME_RAD
from nero_neo_teleop.robot.nero_io import (
    check_driver_health,
    check_gripper_health,
    wait_complete_joint_feedback,
    wait_gripper_status,
)
from nero_neo_teleop.runtime import ARTIFACTS_DIR


CONFIRMATION = "MOVE NERO ARM TO PICO HOME"


def wait_live_joint_feedback(robot, timeout_sec: float = 1.0) -> np.ndarray:
    """Read one complete sample while the arm is intentionally moving."""
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        feedback = robot.get_joint_angles()
        if feedback is not None:
            value = np.asarray(feedback.msg, dtype=np.float64)
            if value.shape == (7,) and np.isfinite(value).all():
                return value
        time.sleep(0.005)
    raise RuntimeError("No complete seven-joint CAN feedback during Home motion")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--can-port", default="can1")
    parser.add_argument("--home-side", choices=("left", "right"), default="right")
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
        default=ARTIFACTS_DIR / "logs/single_home",
    )
    args = parser.parse_args()

    if not 1 <= args.speed_percent <= 10:
        parser.error("speed-percent must be in [1, 10]")
    if args.timeout <= 0 or args.max_travel_deg <= 0 or args.tolerance_deg <= 0:
        parser.error("timeout, max-travel-deg and tolerance-deg must be positive")
    if not 0.0 < args.gripper_open_mm <= 100.0:
        parser.error("gripper-open-mm must be in (0, 100]")
    if not 0.1 <= args.gripper_force_n <= 3.0:
        parser.error("gripper-force-n must be in [0.1, 3]")

    target = RIGHT_HOME_RAD if args.home_side == "right" else LEFT_HOME_RAD
    require_can_role(args.can_port, "follower")
    require_bridge_not_forwarding()

    config = create_agx_arm_config(
        robot=ArmModel.NERO,
        firmeware_version=NeroFW.V120,
        interface="socketcan",
        channel=args.can_port,
        joint_limits=NERO_CPV_JOINT_LIMIT_OVERRIDES_RAD,
    )
    limits = np.asarray(
        [config["joint_limits"][f"joint{index}"] for index in range(1, 8)],
        dtype=np.float64,
    )
    if np.any((target < limits[:, 0]) | (target > limits[:, 1])):
        raise RuntimeError("Configured PICO Home violates SDK joint limits")

    robot = AgxArmFactory.create_arm(config)
    record: dict = {
        "can_port": args.can_port,
        "home_side": args.home_side,
        "home_deg": np.rad2deg(target).tolist(),
        "execute": args.execute,
        "completed": False,
    }
    try:
        robot.set_joint_limits_enabled(True)
        robot.connect()
        current = wait_complete_joint_feedback(robot)
        check_driver_health(robot)
        travel_deg = float(np.max(np.abs(np.rad2deg(target - current))))
        record.update(
            start_deg=np.rad2deg(current).tolist(),
            planned_max_travel_deg=travel_deg,
        )
        print(f"start_deg={np.round(np.rad2deg(current), 2).tolist()}")
        print(f"home_deg={np.round(np.rad2deg(target), 2).tolist()}")
        print(f"planned_max_travel={travel_deg:.2f}deg speed={args.speed_percent}%")
        if travel_deg > args.max_travel_deg:
            raise RuntimeError(
                f"Planned travel {travel_deg:.2f}deg exceeds guard "
                f"{args.max_travel_deg:.2f}deg"
            )
        if not args.execute:
            print(f"PREVIEW ONLY: execute requires confirmation {CONFIRMATION!r}")
            return
        if args.confirm != CONFIRMATION:
            raise RuntimeError(f"Exact confirmation required: {CONFIRMATION!r}")

        if not robot.enable(timeout=2.0):
            raise RuntimeError("NERO arm could not be enabled")
        robot.set_speed_percent(args.speed_percent)
        robot.set_auto_set_motion_mode_enabled(False)
        robot.set_motion_mode(robot.OPTIONS.MOTION_MODE.J)
        time.sleep(0.2)
        robot.move_j(target.astype(float).tolist())

        deadline = time.monotonic() + args.timeout
        stable = 0
        last_report = 0.0
        while time.monotonic() < deadline:
            feedback = wait_live_joint_feedback(robot, timeout_sec=1.0)
            error_deg = np.rad2deg(target - feedback)
            max_error = float(np.max(np.abs(error_deg)))
            stable = stable + 1 if max_error <= args.tolerance_deg else 0
            now = time.monotonic()
            if now >= last_report:
                print(f"home_error_deg={np.round(error_deg, 2).tolist()}")
                last_report = now + 1.0
            check_driver_health(robot)
            if stable >= 3:
                break
            time.sleep(0.05)
        else:
            raise TimeoutError("Single-arm PICO Home motion timed out")

        # Confirm that the arm has actually settled before disconnecting and
        # handing the same CAN interface to the CPV process.
        settled = wait_complete_joint_feedback(robot, timeout_sec=5.0)
        settled_error_deg = np.rad2deg(target - settled)
        if float(np.max(np.abs(settled_error_deg))) > args.tolerance_deg:
            raise RuntimeError(
                "Home feedback did not settle inside tolerance: "
                f"error_deg={np.round(settled_error_deg, 3).tolist()}"
            )

        gripper = robot.init_effector(robot.OPTIONS.EFFECTOR.AGX_GRIPPER)
        gripper_status = wait_gripper_status(gripper)
        check_gripper_health(gripper_status)
        gripper.move_gripper_m(
            value=args.gripper_open_mm / 1000.0,
            force=args.gripper_force_n,
        )
        record["completed"] = True
        print("PASS: NERO reached PICO Home and the gripper is opening")
    finally:
        robot.disconnect()
        args.output_dir.mkdir(parents=True, exist_ok=True)
        log_path = args.output_dir / (
            f"home_{datetime.now().strftime('%Y%m%d_%H%M%S_%f')}.json"
        )
        log_path.write_text(json.dumps(record, indent=2, sort_keys=True) + "\n")
        print(f"single Home log={log_path}")


if __name__ == "__main__":
    main()
