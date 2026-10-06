#!/usr/bin/env python3
"""Plot trajectory recordings, grouped into batches of repetitions."""

import argparse
import csv
import math
import re
from collections import defaultdict
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

REPETITION_PATTERN = re.compile(r"_r(?P<repetition>\d+)$")
REQUIRED_COLUMNS = {
    "elapsed_s", "trajectory", "amplitude_m", "frequency_hz",
    "target_x_m", "target_y_m", "target_z_m",
    "measured_x_m", "measured_y_m", "measured_z_m",
}


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("inputs", nargs="+", type=Path, help="CSV files or directories")
    parser.add_argument("--recursive", action="store_true", help="search directories recursively")
    parser.add_argument("--output", type=Path, default=Path("trajectory_plots"))
    parser.add_argument(
        "--repetitions-per-figure", type=int, default=0,
        help="maximum repetitions per figure; 0 groups all repetitions",
    )
    parser.add_argument("--format", choices=("png", "pdf", "svg"), default="png")
    parser.add_argument("--dpi", type=int, default=180)
    parser.add_argument("--show", action="store_true", help="open figures after saving")
    args = parser.parse_args()
    if args.repetitions_per_figure < 0:
        parser.error("--repetitions-per-figure must be non-negative")
    if args.dpi <= 0:
        parser.error("--dpi must be positive")
    return args


def discover_csv(inputs, recursive):
    paths = set()
    for item in inputs:
        if item.is_file() and item.suffix.lower() == ".csv":
            paths.add(item.resolve())
        elif item.is_dir():
            iterator = item.rglob("*.csv") if recursive else item.glob("*.csv")
            paths.update(path.resolve() for path in iterator)
        else:
            print(f"Skipping missing or unsupported input: {item}")
    return sorted(paths)


def read_recording(path):
    with path.open(newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        columns = set(reader.fieldnames or ())
        missing = REQUIRED_COLUMNS - columns
        if missing:
            raise ValueError(f"missing columns: {', '.join(sorted(missing))}")
        rows = list(reader)
    if not rows:
        raise ValueError("file contains no samples")

    def values(name):
        return np.asarray([float(row[name]) for row in rows], dtype=float)

    target = np.column_stack([values(f"target_{axis}_m") for axis in "xyz"])
    measured = np.column_stack([values(f"measured_{axis}_m") for axis in "xyz"])
    match = REPETITION_PATTERN.search(path.stem)
    repetition = int(match.group("repetition")) if match else 0
    return {
        "path": path,
        "trajectory": rows[0]["trajectory"],
        "amplitude": float(rows[0]["amplitude_m"]),
        "frequency": float(rows[0]["frequency_hz"]),
        "repetition": repetition,
        "time": values("elapsed_s"),
        "target": target,
        "measured": measured,
    }


def batches(items, size):
    size = size or len(items)
    for start in range(0, len(items), size):
        yield items[start:start + size]


def safe_number(value):
    return f"{value:g}".replace("-", "m").replace(".", "p")


def plot_batch(group, batch, batch_index, batch_count, output, file_format, dpi):
    trajectory, amplitude, frequency = group
    figure = plt.figure(figsize=(14, 13), constrained_layout=True)
    grid = figure.add_gridspec(3, 2)
    axis_xy = figure.add_subplot(grid[0, 0])
    axis_3d = figure.add_subplot(grid[0, 1], projection="3d")
    axis_norm = figure.add_subplot(grid[1, 0])
    error_axes = (
        figure.add_subplot(grid[1, 1]),
        figure.add_subplot(grid[2, 0]),
        figure.add_subplot(grid[2, 1]),
    )
    colors = plt.colormaps["tab10"](np.linspace(0.0, 1.0, max(len(batch), 2)))

    metrics = []
    for color, recording in zip(colors, batch):
        label = f"rep {recording['repetition']}"
        target = recording["target"]
        measured = recording["measured"]
        error = measured - target
        error_norm = np.linalg.norm(error, axis=1)

        axis_xy.plot(target[:, 0], target[:, 1], "--", color=color, alpha=0.45)
        axis_xy.plot(measured[:, 0], measured[:, 1], color=color, label=label)
        axis_3d.plot(*target.T, "--", color=color, alpha=0.45)
        axis_3d.plot(*measured.T, color=color, label=label)
        axis_norm.plot(recording["time"], error_norm * 1000.0, color=color, label=label)
        for index, axis in enumerate(error_axes):
            axis.plot(recording["time"], error[:, index] * 1000.0, color=color, label=label)
        component_rmse = np.sqrt(np.mean(error**2, axis=0)) * 1000.0
        component_mean = np.mean(error, axis=0) * 1000.0
        metrics.append(
            f"{label}: mean={np.mean(error_norm) * 1000:.2f} mm, "
            f"mean XYZ=({component_mean[0]:.2f}, {component_mean[1]:.2f}, "
            f"{component_mean[2]:.2f}) mm, RMSE={math.sqrt(np.mean(error_norm**2)) * 1000:.2f} mm, "
            f"RMSE XYZ=({component_rmse[0]:.2f}, {component_rmse[1]:.2f}, "
            f"{component_rmse[2]:.2f}) mm, P95={np.percentile(error_norm, 95) * 1000:.2f} mm"
        )

    title = f"{trajectory}: amplitude={amplitude:g} m, frequency={frequency:g} Hz"
    if batch_count > 1:
        title += f" (batch {batch_index + 1}/{batch_count})"
    figure.suptitle(title)
    axis_xy.set(title="XY path", xlabel="X [m]", ylabel="Y [m]")
    axis_xy.axis("equal")
    axis_xy.grid(True, alpha=0.3)
    axis_xy.legend()
    axis_3d.set(title="3D path", xlabel="X [m]", ylabel="Y [m]", zlabel="Z [m]")
    axis_3d.legend()
    # axis_3d.set_aspect('equal')

    axis_norm.set(title="Position error", xlabel="Time [s]", ylabel="Error norm [mm]")
    axis_norm.grid(True, alpha=0.3)
    axis_norm.legend()
    for coordinate, axis in zip("XYZ", error_axes):
        axis.set(
            title=f"Signed {coordinate} error",
            xlabel="Time [s]",
            ylabel=f"{coordinate} error [mm]",
        )
        axis.axhline(0.0, color="black", linewidth=0.8, alpha=0.5)
        axis.grid(True, alpha=0.3)
        axis.legend()
    figure.text(0.5, 0.005, " | ".join(metrics), ha="center", fontsize=8)

    suffix = f"_batch{batch_index + 1:02d}" if batch_count > 1 else ""
    filename = (
        f"{trajectory}_a{safe_number(amplitude)}_f{safe_number(frequency)}"
        f"{suffix}.{file_format}"
    )
    path = output / filename
    figure.savefig(path, dpi=dpi)
    return figure, path


def main():
    args = parse_args()
    files = discover_csv(args.inputs, args.recursive)
    if not files:
        raise SystemExit("No CSV files found")

    grouped = defaultdict(list)
    for path in files:
        try:
            recording = read_recording(path)
        except (OSError, ValueError) as error:
            print(f"Skipping {path}: {error}")
            continue
        key = (recording["trajectory"], recording["amplitude"], recording["frequency"])
        grouped[key].append(recording)
    if not grouped:
        raise SystemExit("No valid trajectory recordings found")

    args.output.mkdir(parents=True, exist_ok=True)
    figures = []
    for group in sorted(grouped):
        recordings = sorted(grouped[group], key=lambda item: (item["repetition"], item["path"].name))
        grouped_batches = list(batches(recordings, args.repetitions_per_figure))
        for index, batch in enumerate(grouped_batches):
            figure, path = plot_batch(
                group, batch, index, len(grouped_batches), args.output, args.format, args.dpi
            )
            figures.append(figure)
            print(path)

    if args.show:
        plt.show()
    else:
        for figure in figures:
            plt.close(figure)


if __name__ == "__main__":
    main()
