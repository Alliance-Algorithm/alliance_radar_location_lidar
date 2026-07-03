#!/usr/bin/env python3
"""Minimal PCD reader supporting ascii / binary / binary_compressed (LZF).

Returns a dict of field-name -> numpy array. Pure-stdlib + numpy, no open3d.
Usage: import read_pcd; data = read_pcd.load("file.pcd")
"""
import re
import struct
import sys

import numpy as np

_TYPE_MAP = {
    ("F", 4): "f4", ("F", 8): "f8",
    ("U", 1): "u1", ("U", 2): "u2", ("U", 4): "u4", ("U", 8): "u8",
    ("I", 1): "i1", ("I", 2): "i2", ("I", 4): "i4", ("I", 8): "i8",
}


def _lzf_decompress(src: bytes, expected: int) -> bytes:
    """Decompress PCL's LZF (liblzf) byte stream."""
    out = bytearray()
    i = 0
    n = len(src)
    while i < n:
        ctrl = src[i]
        i += 1
        if ctrl < 32:
            length = ctrl + 1
            out += src[i:i + length]
            i += length
        else:
            length = ctrl >> 5
            if length == 7:
                length += src[i]
                i += 1
            ref = len(out) - ((ctrl & 0x1f) << 8) - src[i] - 1
            i += 1
            for _ in range(length + 2):
                out.append(out[ref])
                ref += 1
    return bytes(out[:expected])


def load(path: str) -> dict:
    with open(path, "rb") as f:
        raw = f.read()

    header_end = raw.index(b"DATA")
    header_line_end = raw.index(b"\n", header_end)
    header = raw[:header_line_end].decode("ascii", "replace")
    body = raw[header_line_end + 1:]

    fields = re.search(r"FIELDS (.+)", header).group(1).split()
    sizes = [int(x) for x in re.search(r"SIZE (.+)", header).group(1).split()]
    types = re.search(r"TYPE (.+)", header).group(1).split()
    counts = [int(x) for x in re.search(r"COUNT (.+)", header).group(1).split()]
    npts = int(re.search(r"POINTS (\d+)", header).group(1))
    data_fmt = re.search(r"DATA (\w+)", header).group(1)

    dtypes = [(f, _TYPE_MAP[(t, s)]) for f, t, s in zip(fields, types, sizes)]

    if data_fmt == "binary_compressed":
        comp_size, uncomp_size = struct.unpack("II", body[:8])
        decomp = _lzf_decompress(body[8:8 + comp_size], uncomp_size)
        # binary_compressed is stored field-by-field (SoA), not point-by-point
        result = {}
        offset = 0
        for f, t, s, c in zip(fields, types, sizes, counts):
            dt = np.dtype(_TYPE_MAP[(t, s)])
            nbytes = npts * s * c
            arr = np.frombuffer(decomp[offset:offset + nbytes], dtype=dt)
            result[f] = arr
            offset += nbytes
        return result

    if data_fmt == "binary":
        arr = np.frombuffer(body, dtype=np.dtype(dtypes), count=npts)
        return {f: arr[f] for f in fields}

    # ascii
    vals = np.fromstring(body, sep=" ")
    vals = vals.reshape(npts, -1)
    result = {}
    col = 0
    for f, c in zip(fields, counts):
        result[f] = vals[:, col:col + c].squeeze()
        col += c
    return result


if __name__ == "__main__":
    d = load(sys.argv[1])
    for k, v in d.items():
        if v.dtype.kind == "f":
            print(f"{k:24s} min={v.min():10.3f} max={v.max():10.3f} mean={v.mean():8.3f}  n={len(v)}")
        else:
            print(f"{k:24s} dtype={v.dtype} n={len(v)}")
