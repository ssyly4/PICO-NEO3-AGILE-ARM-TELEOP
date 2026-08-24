"""Print live PICO controller packets without commanding a robot."""

from __future__ import annotations

import argparse
import time

from nero_neo_teleop.pico.pico_input import PicoUdpStream


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bind", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=50150)
    parser.add_argument("--duration", type=float, default=0.0, help="0 runs until Ctrl+C")
    args = parser.parse_args()

    stream = PicoUdpStream(bind=args.bind, port=args.port, timeout_sec=2.0)
    print(f"PICO input probe listening on {args.bind}:{args.port}")
    deadline = None if args.duration <= 0 else time.monotonic() + args.duration
    try:
        while deadline is None or time.monotonic() < deadline:
            sample = stream.receive()
            packet = sample.packet
            left = packet["left"]
            right = packet["right"]
            age = "n/a" if sample.age_ms is None else f"{sample.age_ms:.1f}ms"
            print(
                f"peer={sample.peer[0]} age={age} "
                f"rate={stream.rx_hz:3d}Hz loss={stream.loss_percent:4.1f}% "
                f"left={left['tracked']}/{left['grip']:.2f} "
                f"right={right['tracked']}/{right['grip']:.2f}"
            )
    except KeyboardInterrupt:
        pass
    finally:
        stream.close()


if __name__ == "__main__":
    main()
