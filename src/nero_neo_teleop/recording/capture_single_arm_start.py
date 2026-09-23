"""Print a stable right-arm joint snapshot as comma-separated degrees."""

from __future__ import annotations

import argparse

import numpy as np

from nero_neo_teleop.recording.bimanual_lerobot_recorder import NeroCanStateSource


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--can", default="can_right")
    args = parser.parse_args()

    source = NeroCanStateSource(args.can, "right")
    try:
        source.start()
        state = source.wait_ready(timeout_sec=5.0)
    finally:
        source.stop()
    degrees = np.rad2deg(np.asarray(state.vector[:7], dtype=np.float64))
    print(",".join(f"{value:.6f}" for value in degrees))


if __name__ == "__main__":
    main()
