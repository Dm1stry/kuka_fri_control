#!/usr/bin/env python3
"""Generate a compact article-ready Cartesian tracking figure."""

import argparse
import csv
import re
from collections import defaultdict
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

AXES = "xyz"
COLORS = {"x": "#0072B2", "y": "#E69F00", "z": "#009E73"}
REPETITION_PATTERN = re.compile(r"_r(?P<repetition>\d+)")


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--aligned", type=Path, default=Path("trajectory_analysis/aligned"))
    parser.add_argument("--metrics", type=Path, default=Path("trajectory_analysis/metrics_per_run.csv"))
    parser.add_argument("--output", type=Path, default=Path("trajectory_analysis/article_tracking"))
    parser.add_argument("--amplitude", type=float, default=0.02, help="representative amplitude in metres")
    parser.add_argument("--frequency", type=float, default=0.1, help="representative frequency in Hz")
    parser.add_argument("--formats", nargs="+", choices=("pdf", "svg", "png"), default=("pdf", "png"))
    parser.add_argument("--dpi", type=int, default=300)
    args = parser.parse_args()
    if args.amplitude <= 0 or args.frequency <= 0 or args.dpi <= 0:
        parser.error("amplitude, frequency, and dpi must be positive")
    return args


def read_aligned(path):
    with path.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    if len(rows) < 2:
        raise ValueError(f"not enough samples in {path}")

    def values(name):
        return np.asarray([float(row[name]) for row in rows], dtype=float)

    match = REPETITION_PATTERN.search(path.stem)
    return {
        "path": path,
        "trajectory": rows[0]["trajectory"],
        "amplitude": float(rows[0]["amplitude_m"]),
        "frequency": float(rows[0]["frequency_hz"]),
        "repetition": int(match.group("repetition")) if match else 0,
        "time": values("elapsed_s"),
        "target": np.column_stack([values(f"target_{axis}_m") for axis in AXES]),
        "measured": np.column_stack([values(f"measured_{axis}_m") for axis in AXES]),
    }


def interpolate(time, values, query):
    return np.column_stack([np.interp(query, time, values[:, index]) for index in range(3)])


def common_series(recordings):
    start = max(item["time"][0] for item in recordings)
    end = min(item["time"][-1] for item in recordings)
    dt = max(float(np.median(np.diff(item["time"]))) for item in recordings)
    time = np.arange(start, end + 0.5 * dt, dt)
    targets = np.stack([interpolate(item["time"], item["target"], time) for item in recordings])
    measured = np.stack([interpolate(item["time"], item["measured"], time) for item in recordings])
    return time, targets, measured


def select_condition(recordings, trajectory, amplitude, frequency):
    selected = [
        item for item in recordings
        if item["trajectory"] == trajectory
        and np.isclose(item["amplitude"], amplitude)
        and np.isclose(item["frequency"], frequency)
    ]
    if not selected:
        raise ValueError(
            f"no aligned recordings for {trajectory}, amplitude={amplitude:g}, frequency={frequency:g}"
        )
    return sorted(selected, key=lambda item: item["repetition"])


def plot_path(axis, recordings, panel_label, title):
    _, targets, measured = common_series(recordings)
    origins = targets[:, :1, :]
    targets_mm = (targets - origins) * 1000.0
    measured_mm = (measured - origins) * 1000.0
    reference = np.mean(targets_mm, axis=0)
    measured_mean = np.mean(measured_mm, axis=0)

    axis.plot(reference[:, 0], reference[:, 1], "k--", linewidth=1.7, label="Reference")
    for index, curve in enumerate(measured_mm):
        axis.plot(
            curve[:, 0], curve[:, 1], color="#56B4E9", linewidth=0.8, alpha=0.35,
            label="Individual repetitions" if index == 0 else None,
        )
    axis.plot(measured_mean[:, 0], measured_mean[:, 1], color="#0072B2", linewidth=2.0, label="Mean measured")
    axis.set_xlabel(r"$\Delta X$ [mm]")
    axis.set_ylabel(r"$\Delta Y$ [mm]")
    axis.set_title(f"({panel_label}) {title}", loc="left", fontweight="bold")
    axis.set_aspect("equal", adjustable="box")
    axis.grid(True, alpha=0.25)
    axis.legend(frameon=False, fontsize=8)


def plot_axis_errors(axis, recordings, panel_label):
    time, targets, measured = common_series(recordings)
    error_mm = (measured - targets) * 1000.0
    mean = np.mean(error_mm, axis=0)
    sd = np.std(error_mm, axis=0, ddof=1) if len(recordings) > 1 else np.zeros_like(mean)
    for index, coordinate in enumerate(AXES):
        color = COLORS[coordinate]
        axis.plot(time, mean[:, index], color=color, linewidth=1.5, label=coordinate.upper())
        axis.fill_between(
            time, mean[:, index] - sd[:, index], mean[:, index] + sd[:, index],
            color=color, alpha=0.16, linewidth=0,
        )
    axis.axhline(0.0, color="black", linewidth=0.8, alpha=0.6)
    axis.set_xlabel("Time [s]")
    axis.set_ylabel("Signed error [mm]")
    axis.set_title(f"({panel_label}) Time-aligned Cartesian errors", loc="left", fontweight="bold")
    axis.grid(True, alpha=0.25)
    axis.legend(title="Axis", frameon=False, ncol=3, fontsize=8, title_fontsize=8)


def read_metrics(path):
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def condition_label(trajectory, amplitude):
    names = {
        "circle_xy": "Circle XY", "circle_xyz": "Circle XYZ",
        "square_xy": "Square XY", "square_xyz": "Square XYZ",
        "line_y": "Line Y", "sine_z": "Sine Z",
    }
    return f"{names.get(trajectory, trajectory)}\nA={amplitude * 1000:g} mm"


def plot_rmse_summary(axis, rows, panel_label):
    groups = defaultdict(list)
    for row in rows:
        groups[(row["trajectory"], float(row["amplitude_m"]))].append(row)
    keys = sorted(groups, key=lambda item: (item[0], item[1]))
    x = np.arange(len(keys), dtype=float)
    offset = 0.13

    for index, key in enumerate(keys):
        group = groups[key]
        raw = np.asarray([float(row["raw_rmse_mm"]) for row in group])
        aligned = np.asarray([float(row["aligned_rmse_mm"]) for row in group])
        for raw_value, aligned_value in zip(raw, aligned):
            axis.plot(
                [x[index] - offset, x[index] + offset], [raw_value, aligned_value],
                color="0.75", linewidth=0.7, zorder=1,
            )
        axis.scatter(np.full(len(raw), x[index] - offset), raw, color="0.55", s=16, alpha=0.75, zorder=2)
        axis.scatter(np.full(len(aligned), x[index] + offset), aligned, color="#0072B2", s=16, alpha=0.8, zorder=2)
        for values, position, color in (
            (raw, x[index] - offset, "0.25"),
            (aligned, x[index] + offset, "#0072B2"),
        ):
            sd = np.std(values, ddof=1) if len(values) > 1 else 0.0
            axis.errorbar(position, np.mean(values), yerr=sd, fmt="o", color=color, capsize=3, markersize=5, zorder=3)

    axis.scatter([], [], color="0.55", label="Raw")
    axis.scatter([], [], color="#0072B2", label="Time-aligned")
    axis.set_xticks(x, [condition_label(*key) for key in keys], rotation=42, ha="right")
    axis.tick_params(axis="x", labelsize=6.5, pad=1)
    axis.set_ylabel("RMSE [mm]")
    axis.set_title(f"({panel_label}) Raw and time-aligned tracking error", loc="left", fontweight="bold")
    axis.grid(True, axis="y", alpha=0.25)
    axis.legend(frameon=False, fontsize=8)


def write_description(path, amplitude, frequency, circle_count, square_count):
    diameter = 2.0 * amplitude * 1000.0
    circle_speed = 2.0 * np.pi * amplitude * frequency * 1000.0
    square_speed = 8.0 * amplitude * frequency * 1000.0
    text = f"""Cartesian tracking figure generated from temporally aligned trajectory logs.

Panels (a) and (b) show reference paths, individual repetitions, and the mean measured path for Circle XY and Square XY trajectories. Panel (c) shows mean signed X, Y, and Z errors; shaded regions denote ±1 sample standard deviation across repetitions. Panel (d) compares raw and time-aligned RMSE for all recorded trajectory types and amplitudes; individual points are repetitions and error bars denote mean ±1 sample standard deviation.

Representative path conditions:
- Amplitude: {amplitude:g} m
- Circle diameter / square side: {diameter:g} mm
- Frequency: {frequency:g} Hz
- Nominal Circle XY speed: {circle_speed:.1f} mm/s
- Nominal Square XY speed: {square_speed:.1f} mm/s
- Circle repetitions: {circle_count}
- Square repetitions: {square_count}

One common three-dimensional temporal offset was fitted per repetition and applied equally to X, Y, and Z. This offset is an effective tracking lag incorporating filtering, step limiting, and robot dynamics; it is not communication latency.
"""
    path.write_text(text, encoding="utf-8")


def main():
    args = parse_args()
    paths = sorted(args.aligned.glob("*.csv"))
    if not paths:
        raise SystemExit(f"No aligned CSV files found in {args.aligned}")
    recordings = [read_aligned(path) for path in paths]
    circle = select_condition(recordings, "circle_xy", args.amplitude, args.frequency)
    square = select_condition(recordings, "square_xy", args.amplitude, args.frequency)
    metrics = read_metrics(args.metrics)

    plt.rcParams.update({
        "font.family": "DejaVu Sans", "font.size": 9, "axes.labelsize": 9,
        "axes.titlesize": 9, "xtick.labelsize": 8, "ytick.labelsize": 8,
        "legend.fontsize": 8, "lines.solid_capstyle": "round",
        "pdf.fonttype": 42, "ps.fonttype": 42,
    })
    figure, axes = plt.subplots(2, 2, figsize=(7.15, 6.6), constrained_layout=True)
    plot_path(axes[0, 0], circle, "a", "Circle XY")
    plot_path(axes[0, 1], square, "b", "Square XY")
    plot_axis_errors(axes[1, 0], square, "c")
    plot_rmse_summary(axes[1, 1], metrics, "d")

    args.output.parent.mkdir(parents=True, exist_ok=True)
    for file_format in dict.fromkeys(args.formats):
        path = args.output.with_suffix(f".{file_format}")
        save_options = {"bbox_inches": "tight"}
        if file_format == "png":
            save_options["dpi"] = args.dpi
        figure.savefig(path, **save_options)
        print(path)
        write_description(path.with_suffix(".txt"), args.amplitude, args.frequency, len(circle), len(square))
    plt.close(figure)


if __name__ == "__main__":
    main()
