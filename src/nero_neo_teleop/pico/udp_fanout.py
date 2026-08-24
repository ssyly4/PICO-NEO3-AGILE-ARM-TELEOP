#!/usr/bin/env python3
"""Fan one PICO/OpenXR UDP stream out to the two arm controllers."""

from __future__ import annotations

import argparse
from pathlib import Path
import socket


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bind", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=50150)
    parser.add_argument("--left-host", default="127.0.0.1")
    parser.add_argument("--left-port", type=int, default=50151)
    parser.add_argument("--right-host", default="127.0.0.1")
    parser.add_argument("--right-port", type=int, default=50152)
    parser.add_argument("--ready-file", type=Path)
    args = parser.parse_args()

    source = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    source.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    source.bind((args.bind, args.port))
    targets = [
        (socket.socket(socket.AF_INET, socket.SOCK_DGRAM), (args.left_host, args.left_port)),
        (socket.socket(socket.AF_INET, socket.SOCK_DGRAM), (args.right_host, args.right_port)),
    ]
    print(
        f"PICO UDP fanout {args.bind}:{args.port} -> "
        f"{args.left_host}:{args.left_port}, {args.right_host}:{args.right_port}",
        flush=True,
    )
    first_packet = True
    try:
        while True:
            payload, peer = source.recvfrom(65535)
            if first_packet:
                first_packet = False
                print(
                    f"PICO UDP first packet: peer={peer[0]}:{peer[1]} "
                    f"bytes={len(payload)}",
                    flush=True,
                )
                if args.ready_file is not None:
                    args.ready_file.write_text(
                        f"{peer[0]}:{peer[1]} bytes={len(payload)}\n",
                        encoding="utf-8",
                    )
            for target_socket, target in targets:
                target_socket.sendto(payload, target)
    except KeyboardInterrupt:
        pass
    finally:
        source.close()
        for target_socket, _target in targets:
            target_socket.close()


if __name__ == "__main__":
    main()
