#!/usr/bin/env python3
"""Subscribe to the radar_bridge video stream (ZMQ PUB, JPEG frames) and
forward them as an MJPEG stream over a pipe for ffplay.

Usage (host, host-network container so localhost works):

    # terminal 1: run the bridge (inside container)
    ros2 run radar_bridge radar_bridge_node --ros-args \
        --params-file /workspace/ros_ws/install/radar_bringup/share/radar_bringup/config/bridge/radar_bridge.yaml

    # terminal 2: subscribe and play (host)
    python3 tools/video_zmq/zmq_mjpeg_player.py --addr tcp://localhost:5557 | ffplay -f mjpeg -i -

Options:
    --addr   ZMQ PUB address to subscribe (default tcp://localhost:5557)
    --no-pipe-length  write raw JPEG stream with 4-byte length prefix
                      (for scripts; ffplay expects raw MJPEG without it)
"""
import argparse
import struct
import sys

import zmq


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--addr", default="tcp://localhost:5557",
                    help="ZMQ PUB address of the bridge video stream")
    ap.add_argument("--length-prefix", action="store_true",
                    help="prepend 4-byte big-endian frame length (not for ffplay)")
    args = ap.parse_args()

    ctx = zmq.Context()
    sub = ctx.socket(zmq.SUB)
    sub.connect(args.addr)
    sub.setsockopt(zmq.SUBSCRIBE, b"")
    print(f"[zmq_mjpeg_player] subscribing {args.addr} ...", file=sys.stderr, flush=True)

    n = 0
    while True:
        frame = sub.recv()
        if args.length_prefix:
            sys.stdout.buffer.write(struct.pack(">I", len(frame)))
        sys.stdout.buffer.write(frame)
        sys.stdout.buffer.flush()
        n += 1
        if n % 30 == 0:
            print(f"[zmq_mjpeg_player] {n} frames ({len(frame)/1024:.0f} KiB/frame)",
                  file=sys.stderr, flush=True)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
