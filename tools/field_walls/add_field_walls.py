#!/usr/bin/env python3
"""Generate a field map with 4 boundary walls for GICP localization.

Reads an existing map PCD (field, center-origin) and appends dense wall
points along the four field edges. Walls give the GICP scan-to-map
registration strong x/y translation constraints that bare ground points
lack — verified: a 0.3 m offset converges to ~1-2 cm with walls vs
~17 cm without.

Usage: add_field_walls.py --in map.pcd --out map_walls.pcd [--wx 14.0 --wy 7.5 --dz 0.25 --zmax 1.75 --step 0.1]
"""
import argparse
import struct

import numpy as np


def load_pcd(path):
    with open(path, "rb") as f:
        head = b""
        while True:
            line = f.readline()
            head += line
            if line.startswith(b"DATA"):
                break
        rest = f.read()
    n = len(rest) // 12
    return np.frombuffer(rest, dtype=np.float32).reshape(n, 3)


def save_pcd(path, pts):
    n = len(pts)
    header = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\n"
        "FIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n"
        f"WIDTH {n}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {n}\nDATA binary\n"
    )
    with open(path, "wb") as f:
        f.write(header.encode())
        f.write(pts.astype(np.float32).tobytes())


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--in", dest="in_path", required=True)
    ap.add_argument("--out", dest="out_path", required=True)
    ap.add_argument("--wx", type=float, default=14.0, help="field half-width x (m)")
    ap.add_argument("--wy", type=float, default=7.5, help="field half-depth y (m)")
    ap.add_argument("--dz", type=float, default=0.25, help="wall layer spacing (m)")
    ap.add_argument("--zmax", type=float, default=1.75, help="wall top height (m)")
    ap.add_argument("--step", type=float, default=0.1, help="wall horizontal spacing (m)")
    args = ap.parse_args()

    orig = load_pcd(args.in_path)
    z_levels = np.arange(args.dz, args.zmax + 1e-9, args.dz)
    wall = []
    for z in z_levels:
        for y in np.arange(-args.wy, args.wy + args.step, args.step):
            wall.append((-args.wx, round(y, 3), round(z, 2)))
            wall.append((args.wx, round(y, 3), round(z, 2)))
        for x in np.arange(-args.wx, args.wx + args.step, args.step):
            wall.append((round(x, 3), -args.wy, round(z, 2)))
            wall.append((round(x, 3), args.wy, round(z, 2)))
    wall = np.array(wall)
    merged = np.vstack([orig, wall])
    save_pcd(args.out_path, merged)
    print(f"wrote {args.out_path}: {len(merged)} points "
          f"(orig {len(orig)} + walls {len(wall)}, {len(z_levels)} layers)")


if __name__ == "__main__":
    main()
