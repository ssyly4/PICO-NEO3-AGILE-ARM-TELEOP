# NERO Neo Teleoperation

[简体中文](README.zh-CN.md) | English

PICO Neo 3 bimanual teleoperation and LeRobot v3 data collection for AgileX
NERO seven-axis robot arms. The stack maps OpenXR controller poses to Cartesian
tool targets, solves velocity IK with Pinocchio, and streams guarded joint
commands through the NERO CPV interface.

> **Safety:** this software commands physical robots. Test one arm at low speed,
> keep the emergency stop reachable, verify Home poses and joint limits, and
> never operate near people. Read [SAFETY.md](docs/SAFETY.md) before execution.

## Features

- PICO Neo 3 Unity/OpenXR client with binary UDP streaming
- One-arm and two-arm clutch-based Cartesian teleoperation
- Pinocchio differential IK, pose filtering, finite command lead, and CPV guards
- Analog gripper control and optional downward-force guard
- Automatic SocketCAN binding and guarded dual-arm Home
- Three-camera, dual-arm LeRobot v3 recording with resumable workflows

## Architecture

```text
PICO controllers (OpenXR, 72 Hz)
        | UDP :50150
        v
PICO input + coordinate mapper
        | Cartesian target
        v
Pinocchio velocity IK + safety guards
        | 7-joint target @ 30-40 Hz
        v
NERO CPV backend -> SocketCAN -> left/right robot

CAN feedback + 3 cameras + executed commands -> LeRobot v3 dataset
```

See [Architecture](docs/ARCHITECTURE.md) for the data and control paths.

## Repository Layout

| Path | Purpose |
| --- | --- |
| `src/nero_neo_teleop/` | Host-side Python package |
| `pico_client/` | Unity/OpenXR PICO application |
| `scripts/control/` | Home and teleoperation launchers |
| `scripts/recording/` | Managed LeRobot recorder |
| `scripts/can/` | Stable gs_usb/SocketCAN setup |
| `scripts/pico/` | Build, install, and input diagnostics |
| `tests/` | Mapping, controller, Home, and recorder tests |
| `artifacts/` | Ignored local builds and logs |

## Requirements

- Linux with SocketCAN and two `gs_usb` CAN adapters
- Python 3.11+, NumPy, Pinocchio, and the NERO SDK (`pyAgxArm`)
- An existing `nero_ws` providing `nero_vla`, the NERO URDF, and SDK dependencies
- PICO Neo 3, Unity 6, Android build support, and ADB
- Three V4L2 cameras and a LeRobot environment for recording

The robot SDK, LeRobot, and the PICO Unity OpenXR SDK are external dependencies
and are not vendored here.

## Setup

```bash
git clone <repository-url> nero_neo_teleop
cd nero_neo_teleop
cp .env.example .env
# Edit .env for your SDK, CAN USB paths, cameras, and PICO host address.
python3 -m pip install -e .
```

Find persistent devices before editing `.env`:

```bash
lsusb -t
find /dev/v4l/by-path -type l
```

Download the PICO Unity OpenXR SDK from the
[official PICO developer site](https://developer.picoxr.com/document/unity-openxr/)
and place its package directory at:

```text
pico_client/LocalPackages/com.unity.xr.openxr.picoxr/
```

The package is intentionally excluded from Git because its upstream license
does not grant this repository permission to redistribute it.

Build and install the PICO client:

```bash
./scripts/pico/build.sh
./scripts/pico/install_and_launch.sh
./scripts/pico/check_input.sh
```

## Robot Operation

Preview Home without moving, then execute only after checking the printed poses:

```bash
./scripts/control/run_dual_home.sh
./scripts/control/run_dual_home.sh --execute
```

Start one-arm or two-arm teleoperation:

```bash
./scripts/control/run_servo_v3_experiment.sh --duration 120 --execute
./scripts/control/run_dual_servo_v3_experiment.sh --duration 120 --execute
```

Controls: hold **Grip** to clutch and move an arm, release it to hold and
re-anchor, and use **Trigger** for the gripper.

## Data Collection

```bash
./scripts/recording/run_bimanual_record.sh --workflow fullflow
```

Available workflows are `custom`, `fullflow`, `stage1`, and `stage23`. Press
Enter to prepare an episode; recording starts after motion detection. The
recorder stops according to the selected workflow, returns both arms Home, and
then asks whether to save or discard the attempt. Datasets are written outside
the repository to `NERO_BIMANUAL_DATA_DIR`.

## Validation

```bash
PYTHONPATH=src python3 -m compileall -q src tests
PYTHONPATH=src python3 -m unittest discover -s tests
for file in scripts/**/*.sh; do bash -n "$file"; done
```

Hardware tests are separate from unit tests. No test or default setup command
should move a robot without an explicit `--execute` argument.

## Status and Limits

This is a research prototype validated on a specific dual-NERO installation.
Home poses, CAN topology, URDF location, cameras, and force thresholds are
hardware-specific and must be calibrated. It is not a certified safety system.

The repository's original source is licensed under MIT. External PICO SDK
components retain their upstream license and are not distributed here.
