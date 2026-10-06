#!/usr/bin/env python3
"""Time-align recorded trajectories and generate article-ready tracking tables."""

import argparse
import csv
import math
from collections import defaultdict
from pathlib import Path

import numpy as np

AXES = "xyz"
REQUIRED = {
    "elapsed_s", "trajectory", "amplitude_m", "frequency_hz",
    *(f"target_{axis}_m" for axis in AXES),
    *(f"measured_{axis}_m" for axis in AXES),
}


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path, help="directory containing trajectory CSV files")
    parser.add_argument("--output", type=Path, default=Path("trajectory_analysis"))
    parser.add_argument("--max-lag", type=float, default=0.5, help="maximum tested lag in seconds")
    parser.add_argument("--report", default="TRACKING_RESULTS.md")
    args = parser.parse_args()
    if args.max_lag <= 0:
        parser.error("--max-lag must be positive")
    return args


def read_recording(path):
    with path.open(newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        fields = set(reader.fieldnames or ())
        missing = REQUIRED - fields
        if missing:
            raise ValueError(f"missing columns: {', '.join(sorted(missing))}")
        rows = list(reader)
    if len(rows) < 3:
        raise ValueError("not enough samples")

    def values(name):
        return np.asarray([float(row[name]) for row in rows], dtype=float)

    time = values("elapsed_s")
    if np.any(np.diff(time) <= 0):
        raise ValueError("elapsed_s must be strictly increasing")
    return {
        "path": path,
        "time": time,
        "target": np.column_stack([values(f"target_{axis}_m") for axis in AXES]),
        "measured": np.column_stack([values(f"measured_{axis}_m") for axis in AXES]),
        "trajectory": rows[0]["trajectory"],
        "amplitude": float(rows[0]["amplitude_m"]),
        "frequency": float(rows[0]["frequency_hz"]),
    }


def interpolate(time, values, query):
    return np.column_stack([np.interp(query, time, values[:, index]) for index in range(3)])


def estimate_lag(recording, max_lag):
    time = recording["time"]
    dt = float(np.median(np.diff(time)))
    max_lag = min(max_lag, max(0.0, time[-1] - time[0] - 2.0 * dt))
    candidates = np.arange(0.0, max_lag + 0.5 * dt, dt)
    # Use the same target interval for every candidate so larger lags do not
    # win merely because they discard a different endpoint segment.
    reference_mask = time <= time[-1] - max_lag
    reference_time = time[reference_mask]
    reference_target = recording["target"][reference_mask]
    costs = []
    for lag in candidates:
        aligned = interpolate(time, recording["measured"], reference_time + lag)
        costs.append(float(np.mean(np.sum((aligned - reference_target) ** 2, axis=1))))
    return float(candidates[int(np.argmin(costs))])


def metrics(error):
    norm = np.linalg.norm(error, axis=1)
    return {
        "mean_mm": float(np.mean(norm) * 1000.0),
        "rmse_mm": float(math.sqrt(np.mean(norm**2)) * 1000.0),
        "p95_mm": float(np.percentile(norm, 95) * 1000.0),
        "bias_xyz_mm": np.mean(error, axis=0) * 1000.0,
        "rmse_xyz_mm": np.sqrt(np.mean(error**2, axis=0)) * 1000.0,
    }


def align_recording(recording, lag):
    time = recording["time"]
    valid = time + lag <= time[-1]
    target_time = time[valid]
    measurement_time = target_time + lag
    target = recording["target"][valid]
    measured = interpolate(time, recording["measured"], measurement_time)
    return target_time, measurement_time, target, measured


def write_aligned(path, recording, lag, aligned):
    target_time, measurement_time, target, measured = aligned
    error = measured - target
    fields = [
        "sequence", "elapsed_s", "measurement_elapsed_s", "trajectory",
        "amplitude_m", "frequency_hz", "lag_s",
        *(f"target_{axis}_m" for axis in AXES),
        *(f"measured_{axis}_m" for axis in AXES),
        "position_error_m",
    ]
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(fields)
        for index in range(len(target_time)):
            writer.writerow([
                index, target_time[index], measurement_time[index], recording["trajectory"],
                recording["amplitude"], recording["frequency"], lag,
                *target[index], *measured[index], np.linalg.norm(error[index]),
            ])


def nominal_speed(trajectory, amplitude, frequency):
    omega_speed = 2.0 * math.pi * amplitude * frequency
    if trajectory == "circle_xy":
        return omega_speed
    if trajectory == "circle_xyz":
        return math.sqrt(2.0) * omega_speed
    square_speed = 8.0 * amplitude * frequency
    if trajectory == "square_xy":
        return square_speed
    if trajectory == "square_xyz":
        return math.hypot(square_speed, omega_speed)
    if trajectory in ("line_y", "sine_z"):
        return omega_speed
    return float("nan")


def mean_sd(values):
    values = np.asarray(values, dtype=float)
    sd = np.std(values, ddof=1) if len(values) > 1 else 0.0
    return float(np.mean(values)), float(sd)


def format_pm(values, digits=2):
    mean, sd = mean_sd(values)
    return f"{mean:.{digits}f} ± {sd:.{digits}f}"


def vector_mean(group, key):
    values = np.asarray([item[key] for item in group])
    return np.mean(values, axis=0)


def build_report(groups, per_run_path, aligned_dir, max_lag):
    lines = [
        "# Time-Aligned Cartesian Tracking Results",
        "",
        "## Method",
        "",
        "For each repetition, one non-negative three-dimensional time shift was selected by minimizing the mean squared Euclidean distance between the Cartesian reference and the interpolated measured trajectory:",
        "",
        "$$",
        r"\tau^* = \arg\min_{0 \leq \tau \leq " + f"{max_lag:g}" + r"\,\mathrm{s}} \frac{1}{N}\sum_t \|\mathbf{p}_{meas}(t+\tau)-\mathbf{p}_{ref}(t)\|_2^2.",
        "$$",
        "",
        "The same lag was applied to X, Y, and Z. No axis-specific optimization was used. This fitted offset is an effective trajectory-tracking lag that combines target filtering, step limiting, and robot dynamics; it is not a measurement of UDP or command-to-motion latency. Metrics were first computed for each independent repetition and then summarized as mean ± sample standard deviation across repetitions. The complete recorded interval was analyzed; no samples were removed based on warmup or trajectory closing.",
        "",
        "## Main tracking results",
        "",
        "| Trajectory | Amplitude [m] | Nominal max speed [mm/s] | Repetitions | Raw mean error [mm] | Aligned mean error [mm] | Aligned RMSE [mm] | Aligned P95 [mm] | Lag [ms] |",
        "|---|---:|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for key in sorted(groups):
        group = groups[key]
        trajectory, amplitude, frequency = key
        lines.append(
            f"| {trajectory} | {amplitude:g} | {nominal_speed(trajectory, amplitude, frequency) * 1000:.1f} "
            f"| {len(group)} | {format_pm([x['raw_mean_mm'] for x in group])} "
            f"| {format_pm([x['aligned_mean_mm'] for x in group])} "
            f"| {format_pm([x['aligned_rmse_mm'] for x in group])} "
            f"| {format_pm([x['aligned_p95_mm'] for x in group])} "
            f"| {format_pm([x['lag_s'] * 1000 for x in group], 1)} |"
        )

    lines += [
        "",
        "## Axis-wise aligned errors",
        "",
        "Bias is the signed mean error. RMSE is reported independently for each Cartesian axis.",
        "",
        "| Trajectory | Amplitude [m] | Bias X/Y/Z [mm] | RMSE X/Y/Z [mm] |",
        "|---|---:|---:|---:|",
    ]
    for key in sorted(groups):
        group = groups[key]
        trajectory, amplitude, _ = key
        bias = vector_mean(group, "aligned_bias_xyz_mm")
        rmse = vector_mean(group, "aligned_rmse_xyz_mm")
        lines.append(
            f"| {trajectory} | {amplitude:g} | {bias[0]:.2f} / {bias[1]:.2f} / {bias[2]:.2f} "
            f"| {rmse[0]:.2f} / {rmse[1]:.2f} / {rmse[2]:.2f} |"
        )

    lines += [
        "",
        "## Recommended presentation in the paper",
        "",
        "Use one compact tracking figure in the main paper:",
        "",
        "1. Reference and measured circle in the XY plane.",
        "2. Reference and measured square in the XY plane.",
        "3. Signed X, Y, and Z errors after applying the common time shift.",
        "4. A summary plot of aligned RMSE across trajectory types and amplitudes, with repetitions shown as individual points and mean ± SD overlaid.",
        "",
        "Use a black dashed line for the reference and a solid color-blind-safe line for measured motion. Keep all labels in English, use millimetres for tracking errors, preserve equal aspect ratio for path plots, and export the final artwork as PDF or SVG.",
        "",
        "Suggested caption:",
        "",
        "> **Cartesian tracking after temporal alignment.** Reference and measured end-effector paths for circular and square trajectories. A single three-dimensional lag was estimated independently for each repetition and applied equally to all Cartesian axes. Signed axis errors show residual geometric and static tracking error after delay compensation. Reported statistics are mean ± SD across independent repetitions.",
        "",
        "## Generated article figure",
        "",
        "Generate the compact four-panel figure after running the alignment script:",
        "",
        "```bash",
        "python scripts/plot_article_tracking.py --aligned trajectory_analysis/aligned --metrics trajectory_analysis/metrics_per_run.csv --output trajectory_analysis/article_tracking --amplitude 0.02 --frequency 0.1 --formats pdf png",
        "```",
        "",
        "- Vector artwork: [`article_tracking.pdf`](article_tracking.pdf)",
        "- Markdown preview: ![Time-aligned Cartesian tracking](article_tracking.png)",
        "- Supplementary description: [`article_tracking.txt`](article_tracking.txt)",
        "",
        "## Files for reproducibility",
        "",
        f"- Per-run metrics: [`{per_run_path.name}`]({per_run_path.name})",
        f"- Time-aligned trajectories: [`{aligned_dir.name}/`]({aligned_dir.name}/)",
        "- Each aligned CSV contains the reference timestamp, corresponding shifted measurement timestamp, common lag, target position, interpolated measured position, and aligned error norm.",
        "",
        "## Reporting notes",
        "",
        "- State trajectory amplitude, geometric size, nominal speed, frequency, duration, and number of independent repetitions in the paper.",
        "- Report both raw and time-aligned errors; the difference quantifies the contribution of delay.",
        "- Do not treat sampled control points as independent repetitions.",
        "- Keep the detailed per-condition plots in the supplementary material.",
        "- Ensure that the abstract and conclusion use the values from the final repeated experiment set.",
        "",
    ]
    return "\n".join(lines)


def main():
    args = parse_args()
    paths = sorted(args.input.glob("*.csv"))
    if not paths:
        raise SystemExit(f"No CSV files found in {args.input}")
    args.output.mkdir(parents=True, exist_ok=True)
    aligned_dir = args.output / "aligned"
    aligned_dir.mkdir(exist_ok=True)

    results = []
    for path in paths:
        try:
            recording = read_recording(path)
        except (OSError, ValueError) as error:
            print(f"Skipping {path}: {error}")
            continue
        lag = estimate_lag(recording, args.max_lag)
        aligned = align_recording(recording, lag)
        raw = metrics(recording["measured"] - recording["target"])
        aligned_error = aligned[3] - aligned[2]
        shifted = metrics(aligned_error)
        output_path = aligned_dir / f"{path.stem}_aligned.csv"
        write_aligned(output_path, recording, lag, aligned)
        result = {
            "file": path.name,
            "aligned_file": output_path.name,
            "trajectory": recording["trajectory"],
            "amplitude": recording["amplitude"],
            "frequency": recording["frequency"],
            "lag_s": lag,
            "raw_mean_mm": raw["mean_mm"],
            "raw_rmse_mm": raw["rmse_mm"],
            "raw_p95_mm": raw["p95_mm"],
            "aligned_mean_mm": shifted["mean_mm"],
            "aligned_rmse_mm": shifted["rmse_mm"],
            "aligned_p95_mm": shifted["p95_mm"],
            "aligned_bias_xyz_mm": shifted["bias_xyz_mm"],
            "aligned_rmse_xyz_mm": shifted["rmse_xyz_mm"],
        }
        results.append(result)
        print(f"{path.name}: lag={lag * 1000:.1f} ms, RMSE {raw['rmse_mm']:.2f} -> {shifted['rmse_mm']:.2f} mm")

    if not results:
        raise SystemExit("No valid trajectory recordings found")

    per_run_path = args.output / "metrics_per_run.csv"
    fields = [
        "file", "aligned_file", "trajectory", "amplitude_m", "frequency_hz", "lag_ms",
        "raw_mean_mm", "raw_rmse_mm", "raw_p95_mm",
        "aligned_mean_mm", "aligned_rmse_mm", "aligned_p95_mm",
        "aligned_bias_x_mm", "aligned_bias_y_mm", "aligned_bias_z_mm",
        "aligned_rmse_x_mm", "aligned_rmse_y_mm", "aligned_rmse_z_mm",
    ]
    with per_run_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(fields)
        for item in results:
            writer.writerow([
                item["file"], item["aligned_file"], item["trajectory"], item["amplitude"],
                item["frequency"], item["lag_s"] * 1000.0,
                item["raw_mean_mm"], item["raw_rmse_mm"], item["raw_p95_mm"],
                item["aligned_mean_mm"], item["aligned_rmse_mm"], item["aligned_p95_mm"],
                *item["aligned_bias_xyz_mm"], *item["aligned_rmse_xyz_mm"],
            ])

    groups = defaultdict(list)
    for item in results:
        groups[(item["trajectory"], item["amplitude"], item["frequency"])].append(item)
    report_path = args.output / args.report
    report_path.write_text(build_report(groups, per_run_path, aligned_dir, args.max_lag), encoding="utf-8")
    print(report_path)


if __name__ == "__main__":
    main()
