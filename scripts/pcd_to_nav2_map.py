#!/usr/bin/env python3
"""Project a binary PCD map into a Nav2-compatible 2D occupancy map.

This utility intentionally produces a baseline static map for visual
inspection. It does not solve localization, TF ownership, local obstacle
updates, or safe robot control.
"""

from __future__ import annotations

import argparse
import math
import struct
from pathlib import Path
from typing import Dict, Iterable, List, Sequence, Tuple


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Convert a binary PCD map to Nav2 PGM/YAML files."
    )
    parser.add_argument("pcd", type=Path, help="Input binary PCD file")
    parser.add_argument(
        "output_prefix",
        type=Path,
        help="Output path without extension, for example maps/real/site_a",
    )
    parser.add_argument("--resolution", type=float, default=0.05)
    parser.add_argument("--obstacle-min-z", type=float, default=0.15)
    parser.add_argument("--obstacle-max-z", type=float, default=1.50)
    parser.add_argument(
        "--occupied-dilation",
        type=float,
        default=0.05,
        help="Expand occupied cells by this many metres",
    )
    parser.add_argument(
        "--min-points-per-cell",
        type=int,
        default=1,
        help="Minimum obstacle points required before a cell is occupied",
    )
    parser.add_argument(
        "--trim-percent",
        type=float,
        default=0.5,
        help="Trim this percentage from each x/y tail to reject distant outliers",
    )
    parser.add_argument(
        "--padding",
        type=float,
        default=0.50,
        help="Free-space padding around the retained point-cloud bounds",
    )
    return parser.parse_args()


def read_pcd_xyz(path: Path) -> List[Tuple[float, float, float]]:
    metadata: Dict[str, List[str]] = {}

    with path.open("rb") as stream:
        while True:
            raw_line = stream.readline()
            if not raw_line:
                raise ValueError("PCD header ended before a DATA line")

            line = raw_line.decode("ascii", errors="strict").strip()
            if not line or line.startswith("#"):
                continue

            parts = line.split()
            key = parts[0].upper()
            metadata[key] = parts[1:]
            if key == "DATA":
                payload = stream.read()
                break

    if metadata["DATA"][0].lower() != "binary":
        raise ValueError("Only uncompressed binary PCD files are supported")

    fields = metadata.get("FIELDS", metadata.get("FIELD", []))
    sizes = [int(value) for value in metadata["SIZE"]]
    types = metadata["TYPE"]
    counts = [int(value) for value in metadata.get("COUNT", ["1"] * len(fields))]

    if not (len(fields) == len(sizes) == len(types) == len(counts)):
        raise ValueError("PCD FIELDS/SIZE/TYPE/COUNT lengths do not match")

    offsets: Dict[str, int] = {}
    offset = 0
    for field, size, count in zip(fields, sizes, counts):
        offsets[field] = offset
        offset += size * count
    record_size = offset

    for axis in ("x", "y", "z"):
        index = fields.index(axis) if axis in fields else -1
        if index < 0 or sizes[index] != 4 or types[index].upper() != "F":
            raise ValueError(f"PCD field {axis!r} must be a 32-bit float")

    declared_points = int(metadata.get("POINTS", ["0"])[0])
    available_points = len(payload) // record_size
    point_count = min(declared_points or available_points, available_points)

    points: List[Tuple[float, float, float]] = []
    for index in range(point_count):
        base = index * record_size
        x = struct.unpack_from("<f", payload, base + offsets["x"])[0]
        y = struct.unpack_from("<f", payload, base + offsets["y"])[0]
        z = struct.unpack_from("<f", payload, base + offsets["z"])[0]
        if math.isfinite(x) and math.isfinite(y) and math.isfinite(z):
            points.append((x, y, z))

    if not points:
        raise ValueError("PCD contains no finite XYZ points")
    return points


def percentile(sorted_values: Sequence[float], fraction: float) -> float:
    position = round((len(sorted_values) - 1) * fraction)
    return sorted_values[position]


def retained_bounds(
    points: Sequence[Tuple[float, float, float]],
    trim_percent: float,
    padding: float,
) -> Tuple[float, float, float, float]:
    if not 0.0 <= trim_percent < 50.0:
        raise ValueError("--trim-percent must be in [0, 50)")

    trim_fraction = trim_percent / 100.0
    xs = sorted(point[0] for point in points)
    ys = sorted(point[1] for point in points)

    min_x = percentile(xs, trim_fraction) - padding
    max_x = percentile(xs, 1.0 - trim_fraction) + padding
    min_y = percentile(ys, trim_fraction) - padding
    max_y = percentile(ys, 1.0 - trim_fraction) + padding
    return min_x, max_x, min_y, max_y


def dilate(cells: Iterable[int], width: int, height: int, radius: int) -> set[int]:
    source = set(cells)
    if radius <= 0:
        return source

    expanded: set[int] = set()
    offsets = [
        (dx, dy)
        for dy in range(-radius, radius + 1)
        for dx in range(-radius, radius + 1)
        if dx * dx + dy * dy <= radius * radius
    ]
    for cell in source:
        x = cell % width
        y = cell // width
        for dx, dy in offsets:
            nx = x + dx
            ny = y + dy
            if 0 <= nx < width and 0 <= ny < height:
                expanded.add(ny * width + nx)
    return expanded


def write_map(
    output_prefix: Path,
    pixels: bytes,
    width: int,
    height: int,
    resolution: float,
    origin_x: float,
    origin_y: float,
) -> None:
    output_prefix.parent.mkdir(parents=True, exist_ok=True)
    pgm_path = output_prefix.with_suffix(".pgm")
    yaml_path = output_prefix.with_suffix(".yaml")
    png_path = output_prefix.with_suffix(".png")

    with pgm_path.open("wb") as stream:
        stream.write(b"P5\n")
        stream.write(b"# Generated from a Fast-LIO PCD map\n")
        stream.write(f"{width} {height}\n255\n".encode("ascii"))
        stream.write(pixels)

    yaml_path.write_text(
        "\n".join(
            [
                f"image: {pgm_path.name}",
                "mode: trinary",
                f"resolution: {resolution:.6f}",
                f"origin: [{origin_x:.6f}, {origin_y:.6f}, 0.0]",
                "negate: 0",
                "occupied_thresh: 0.65",
                "free_thresh: 0.196",
                "",
            ]
        ),
        encoding="utf-8",
    )

    try:
        from PIL import Image

        Image.frombytes("L", (width, height), pixels).save(png_path)
    except ImportError:
        print("Pillow is unavailable; skipped PNG preview")


def main() -> None:
    args = parse_args()
    if args.resolution <= 0.0:
        raise ValueError("--resolution must be positive")
    if args.obstacle_min_z >= args.obstacle_max_z:
        raise ValueError("--obstacle-min-z must be below --obstacle-max-z")
    if args.min_points_per_cell < 1:
        raise ValueError("--min-points-per-cell must be at least 1")

    points = read_pcd_xyz(args.pcd)
    min_x, max_x, min_y, max_y = retained_bounds(
        points, args.trim_percent, args.padding
    )

    width = math.ceil((max_x - min_x) / args.resolution) + 1
    height = math.ceil((max_y - min_y) / args.resolution) + 1
    counts = [0] * (width * height)
    obstacle_points = 0

    for x, y, z in points:
        if not (args.obstacle_min_z <= z <= args.obstacle_max_z):
            continue
        if not (min_x <= x <= max_x and min_y <= y <= max_y):
            continue

        grid_x = int((x - min_x) / args.resolution)
        grid_y = int((y - min_y) / args.resolution)
        counts[grid_y * width + grid_x] += 1
        obstacle_points += 1

    occupied = {
        index
        for index, count in enumerate(counts)
        if count >= args.min_points_per_cell
    }
    dilation_cells = math.ceil(args.occupied_dilation / args.resolution)
    occupied = dilate(occupied, width, height, dilation_cells)

    # PGM rows run from the map's maximum y down to its minimum y. Nav2's
    # origin remains the lower-left corner in metric coordinates.
    pixels = bytearray([254]) * (width * height)
    for cell in occupied:
        grid_x = cell % width
        grid_y = cell // width
        image_y = height - 1 - grid_y
        pixels[image_y * width + grid_x] = 0

    write_map(
        args.output_prefix,
        bytes(pixels),
        width,
        height,
        args.resolution,
        min_x,
        min_y,
    )

    print(f"Input finite points: {len(points)}")
    print(f"Obstacle-band points: {obstacle_points}")
    print(f"Occupied cells after dilation: {len(occupied)}")
    print(f"Map size: {width} x {height} cells")
    print(f"Metric bounds: x=[{min_x:.3f}, {max_x:.3f}] m")
    print(f"Metric bounds: y=[{min_y:.3f}, {max_y:.3f}] m")
    print(f"Wrote: {args.output_prefix.with_suffix('.pgm')}")
    print(f"Wrote: {args.output_prefix.with_suffix('.yaml')}")
    if args.output_prefix.with_suffix(".png").exists():
        print(f"Wrote: {args.output_prefix.with_suffix('.png')}")


if __name__ == "__main__":
    main()
