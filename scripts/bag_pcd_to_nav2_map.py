#!/usr/bin/env python3
"""Build a Nav2 map from a Fast-LIO PCD plus the source rosbag.

The PCD supplies stable occupied cells. Raw LiDAR rays synchronized with
Fast-LIO odometry supply observed free space. Cells with neither kind of
evidence remain unknown, unlike a plain point-cloud projection that marks all
non-occupied cells as free.

Run this script with the system ROS 2 Python after sourcing ROS 2 Humble and
the workspace that contains livox_ros_driver2.
"""

from __future__ import annotations

import argparse
import bisect
import math
from pathlib import Path
from typing import Iterable, List, Sequence, Tuple

import rosbag2_py
from livox_ros_driver2.msg import CustomMsg
from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message

from pcd_to_nav2_map import dilate, read_pcd_xyz, retained_bounds, write_map


Pose = Tuple[int, float, float, float, float, float, float, float]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Create an observed/free/unknown Nav2 map from PCD and rosbag."
    )
    parser.add_argument("bag", type=Path, help="Input rosbag2 directory")
    parser.add_argument("pcd", type=Path, help="Fast-LIO binary PCD map")
    parser.add_argument(
        "output_prefix",
        type=Path,
        help="Output path without extension, for example maps/real/site_observed",
    )
    parser.add_argument("--resolution", type=float, default=0.05)
    parser.add_argument("--obstacle-min-z", type=float, default=0.15)
    parser.add_argument("--obstacle-max-z", type=float, default=1.50)
    parser.add_argument("--min-points-per-cell", type=int, default=1)
    parser.add_argument("--occupied-dilation", type=float, default=0.05)
    parser.add_argument("--free-dilation", type=float, default=0.05)
    parser.add_argument("--trim-percent", type=float, default=0.5)
    parser.add_argument("--padding", type=float, default=0.50)
    parser.add_argument("--min-ray-range", type=float, default=0.50)
    parser.add_argument("--max-ray-range", type=float, default=15.0)
    parser.add_argument(
        "--ray-end-clearance",
        type=float,
        default=0.10,
        help="Do not mark this distance before each return as free",
    )
    parser.add_argument(
        "--angle-step-deg",
        type=float,
        default=0.5,
        help="Angular bin size for the conservative nearest return per scan",
    )
    parser.add_argument(
        "--scan-stride",
        type=int,
        default=2,
        help="Use every Nth LiDAR message",
    )
    parser.add_argument(
        "--point-stride",
        type=int,
        default=3,
        help="Use every Nth valid Livox point before angular binning",
    )
    parser.add_argument(
        "--max-sync-delta",
        type=float,
        default=0.05,
        help="Maximum bag-time difference between LiDAR and odometry, seconds",
    )
    parser.add_argument(
        "--extrinsic-translation",
        nargs=3,
        type=float,
        metavar=("X", "Y", "Z"),
        default=(-0.011, -0.02329, 0.04412),
        help="LiDAR origin in the Fast-LIO body/IMU frame, metres",
    )
    return parser.parse_args()


def open_bag(path: Path) -> rosbag2_py.SequentialReader:
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(path), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    return reader


def read_odometry(path: Path) -> List[Pose]:
    reader = open_bag(path)
    poses: List[Pose] = []
    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        if topic != "/Odometry":
            continue
        message = deserialize_message(data, Odometry)
        position = message.pose.pose.position
        orientation = message.pose.pose.orientation
        poses.append(
            (
                timestamp,
                position.x,
                position.y,
                position.z,
                orientation.x,
                orientation.y,
                orientation.z,
                orientation.w,
            )
        )

    if not poses:
        raise ValueError("Bag contains no /Odometry messages")
    return poses


def nearest_pose(
    poses: Sequence[Pose], pose_timestamps: Sequence[int], timestamp: int
) -> Tuple[Pose, float]:
    insertion = bisect.bisect_left(pose_timestamps, timestamp)
    candidates = []
    if insertion < len(poses):
        candidates.append(poses[insertion])
    if insertion > 0:
        candidates.append(poses[insertion - 1])
    pose = min(candidates, key=lambda item: abs(item[0] - timestamp))
    return pose, abs(pose[0] - timestamp) / 1e9


def rotate(
    x: float,
    y: float,
    z: float,
    qx: float,
    qy: float,
    qz: float,
    qw: float,
) -> Tuple[float, float, float]:
    # Quaternion-vector rotation expanded to avoid temporary matrix objects.
    tx = 2.0 * (qy * z - qz * y)
    ty = 2.0 * (qz * x - qx * z)
    tz = 2.0 * (qx * y - qy * x)
    return (
        x + qw * tx + qy * tz - qz * ty,
        y + qw * ty + qz * tx - qx * tz,
        z + qw * tz + qx * ty - qy * tx,
    )


def bresenham(
    x0: int, y0: int, x1: int, y1: int, skip_end_cells: int
) -> Iterable[Tuple[int, int]]:
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    error = dx - dy
    cells: List[Tuple[int, int]] = []

    while True:
        cells.append((x0, y0))
        if x0 == x1 and y0 == y1:
            break
        doubled = 2 * error
        if doubled > -dy:
            error -= dy
            x0 += sx
        if doubled < dx:
            error += dx
            y0 += sy

    end = max(1, len(cells) - skip_end_cells)
    yield from cells[:end]


def validate_args(args: argparse.Namespace) -> None:
    if args.resolution <= 0.0:
        raise ValueError("--resolution must be positive")
    if args.obstacle_min_z >= args.obstacle_max_z:
        raise ValueError("--obstacle-min-z must be below --obstacle-max-z")
    if args.min_points_per_cell < 1:
        raise ValueError("--min-points-per-cell must be at least 1")
    if args.min_ray_range < 0.0 or args.min_ray_range >= args.max_ray_range:
        raise ValueError("Ray range limits are invalid")
    if args.angle_step_deg <= 0.0 or args.angle_step_deg > 180.0:
        raise ValueError("--angle-step-deg must be in (0, 180]")
    if args.scan_stride < 1 or args.point_stride < 1:
        raise ValueError("Scan and point strides must be at least 1")
    if args.max_sync_delta <= 0.0:
        raise ValueError("--max-sync-delta must be positive")


def main() -> None:
    args = parse_args()
    validate_args(args)

    pcd_points = read_pcd_xyz(args.pcd)
    min_x, max_x, min_y, max_y = retained_bounds(
        pcd_points, args.trim_percent, args.padding
    )
    width = math.ceil((max_x - min_x) / args.resolution) + 1
    height = math.ceil((max_y - min_y) / args.resolution) + 1

    occupied_counts = [0] * (width * height)
    obstacle_points = 0
    for x, y, z in pcd_points:
        if not (args.obstacle_min_z <= z <= args.obstacle_max_z):
            continue
        if not (min_x <= x <= max_x and min_y <= y <= max_y):
            continue
        grid_x = int((x - min_x) / args.resolution)
        grid_y = int((y - min_y) / args.resolution)
        occupied_counts[grid_y * width + grid_x] += 1
        obstacle_points += 1

    occupied = {
        index
        for index, count in enumerate(occupied_counts)
        if count >= args.min_points_per_cell
    }
    occupied = dilate(
        occupied,
        width,
        height,
        math.ceil(args.occupied_dilation / args.resolution),
    )

    poses = read_odometry(args.bag)
    pose_timestamps = [pose[0] for pose in poses]
    angle_bins = math.ceil(360.0 / args.angle_step_deg)
    skip_end_cells = math.ceil(args.ray_end_clearance / args.resolution)
    extrinsic_x, extrinsic_y, extrinsic_z = args.extrinsic_translation

    observed_free: set[int] = set()
    reader = open_bag(args.bag)
    lidar_index = 0
    used_scans = 0
    rejected_sync = 0
    valid_points = 0
    selected_rays = 0
    maximum_sync_delta = 0.0

    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        if topic != "/livox/lidar":
            continue

        use_scan = lidar_index % args.scan_stride == 0
        lidar_index += 1
        if not use_scan:
            continue

        pose, sync_delta = nearest_pose(poses, pose_timestamps, timestamp)
        maximum_sync_delta = max(maximum_sync_delta, sync_delta)
        if sync_delta > args.max_sync_delta:
            rejected_sync += 1
            continue

        message = deserialize_message(data, CustomMsg)
        _, px, py, pz, qx, qy, qz, qw = pose
        sensor_offset = rotate(
            extrinsic_x, extrinsic_y, extrinsic_z, qx, qy, qz, qw
        )
        sensor_x = px + sensor_offset[0]
        sensor_y = py + sensor_offset[1]
        sensor_z = pz + sensor_offset[2]
        origin_x = int((sensor_x - min_x) / args.resolution)
        origin_y = int((sensor_y - min_y) / args.resolution)

        nearest_returns: List[Tuple[float, int, int] | None] = [None] * angle_bins
        accepted_index = 0
        for point in message.points:
            if point.line >= 4 or (point.tag & 0x30) not in (0x00, 0x10):
                continue
            accepted_index += 1
            if accepted_index % args.point_stride != 0:
                continue

            range_squared = point.x * point.x + point.y * point.y + point.z * point.z
            if range_squared < args.min_ray_range * args.min_ray_range:
                continue

            body_x = point.x + extrinsic_x
            body_y = point.y + extrinsic_y
            body_z = point.z + extrinsic_z
            rotated = rotate(body_x, body_y, body_z, qx, qy, qz, qw)
            world_x = px + rotated[0]
            world_y = py + rotated[1]
            world_z = pz + rotated[2]
            if not (args.obstacle_min_z <= world_z <= args.obstacle_max_z):
                continue

            delta_x = world_x - sensor_x
            delta_y = world_y - sensor_y
            planar_range = math.hypot(delta_x, delta_y)
            if not (args.min_ray_range <= planar_range <= args.max_ray_range):
                continue

            angle = math.atan2(delta_y, delta_x)
            bin_index = int((angle + math.pi) / (2.0 * math.pi) * angle_bins)
            bin_index = min(bin_index, angle_bins - 1)
            end_x = int((world_x - min_x) / args.resolution)
            end_y = int((world_y - min_y) / args.resolution)
            current = nearest_returns[bin_index]
            if current is None or planar_range < current[0]:
                nearest_returns[bin_index] = (planar_range, end_x, end_y)
            valid_points += 1

        for result in nearest_returns:
            if result is None:
                continue
            _, end_x, end_y = result
            selected_rays += 1
            for grid_x, grid_y in bresenham(
                origin_x, origin_y, end_x, end_y, skip_end_cells
            ):
                if 0 <= grid_x < width and 0 <= grid_y < height:
                    observed_free.add(grid_y * width + grid_x)
        used_scans += 1

    observed_free = dilate(
        observed_free,
        width,
        height,
        math.ceil(args.free_dilation / args.resolution),
    )
    observed_free.difference_update(occupied)

    # Nav2 trinary convention with negate=0: black occupied, white free,
    # middle gray unknown. Rows are written from maximum y to minimum y.
    pixels = bytearray([205]) * (width * height)
    for cell in observed_free:
        grid_x = cell % width
        grid_y = cell // width
        image_y = height - 1 - grid_y
        pixels[image_y * width + grid_x] = 254
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

    unknown_cells = width * height - len(occupied) - len(observed_free)
    print(f"PCD finite points: {len(pcd_points)}")
    print(f"PCD obstacle-band points: {obstacle_points}")
    print(f"LiDAR scans: {lidar_index}; used: {used_scans}")
    print(f"Scans rejected by sync limit: {rejected_sync}")
    print(f"Maximum nearest sync delta: {maximum_sync_delta:.6f} s")
    print(f"Ray-band candidate points: {valid_points}")
    print(f"Selected conservative rays: {selected_rays}")
    print(f"Occupied cells after dilation: {len(occupied)}")
    print(f"Observed free cells after dilation: {len(observed_free)}")
    print(f"Unknown cells: {unknown_cells}")
    print(f"Map size: {width} x {height} cells")
    print(f"Metric bounds: x=[{min_x:.3f}, {max_x:.3f}] m")
    print(f"Metric bounds: y=[{min_y:.3f}, {max_y:.3f}] m")
    print(f"Wrote: {args.output_prefix.with_suffix('.pgm')}")
    print(f"Wrote: {args.output_prefix.with_suffix('.yaml')}")
    if args.output_prefix.with_suffix(".png").exists():
        print(f"Wrote: {args.output_prefix.with_suffix('.png')}")


if __name__ == "__main__":
    main()
