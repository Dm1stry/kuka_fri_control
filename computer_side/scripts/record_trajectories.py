#!/usr/bin/env python3
"""Record repeatable Cartesian trajectories and KUKA FRI observations.

Robot motion is disabled unless --execute is explicitly supplied.
"""

import argparse
import csv
import json
import math
import signal
import time
from pathlib import Path

import numpy as np

TRAJECTORIES = ("circle_xy", "circle_xyz", "square_xy", "square_xyz", "line_y", "sine_z")
COLUMNS = (
    ["sequence", "monotonic_ns", "elapsed_s", "trajectory", "amplitude_m", "frequency_hz"]
    + [f"target_{axis}_m" for axis in "xyz"]
    + [f"measured_{axis}_m" for axis in "xyz"]
    + ["position_error_m"]
    + [f"joint_{index + 1}_rad" for index in range(7)]
    + [f"wrench_{name}" for name in ("tx", "ty", "tz", "fx", "fy", "fz")]
)


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--trajectory", nargs="+", choices=TRAJECTORIES, default=["circle_xy"])
    parser.add_argument("--amplitude", nargs="+", type=float, default=[0.02], help="metres")
    parser.add_argument("--frequency", nargs="+", type=float, default=[0.1], help="Hz")
    parser.add_argument("--duration", type=float, default=20.0, help="seconds per run")
    parser.add_argument("--rate", type=float, default=100.0, help="command/log rate in Hz")
    parser.add_argument("--warmup", type=float, default=2.0, help="smooth ramp duration in seconds")
    parser.add_argument("--settle", type=float, default=2.0, help="pause between runs in seconds")
    parser.add_argument("--repetitions", type=int, default=1)
    parser.add_argument("--output", type=Path, default=Path("trajectory_data"))
    parser.add_argument("--urdf", default="robots/iiwa2_gripper.urdf")
    parser.add_argument("--execute", action="store_true", help="connect to and move the robot")
    args = parser.parse_args()
    if min(args.amplitude) <= 0 or min(args.frequency) <= 0:
        parser.error("amplitudes and frequencies must be positive")
    if args.duration <= 0 or args.rate <= 0 or args.repetitions < 1:
        parser.error("duration/rate must be positive and repetitions >= 1")
    return args


def smoothstep(value):
    value = min(1.0, max(0.0, value))
    return value * value * (3.0 - 2.0 * value)


def target_at(origin, trajectory, amplitude, frequency, elapsed, warmup):
    target = origin.copy()
    phase = 2.0 * math.pi * frequency * elapsed
    ramp = smoothstep(elapsed / warmup) if warmup > 0 else 1.0
    if trajectory in ("circle_xy", "circle_xyz"):
        target[0] += ramp * amplitude * (1.0 - math.cos(phase))
        target[1] += ramp * amplitude * math.sin(phase)
        if trajectory == "circle_xyz":
            target[2] += ramp * amplitude * math.cos(phase)
    elif trajectory in ("square_xy", "square_xyz"):
        cycle = (frequency * elapsed) % 1.0
        square_phase = 4.0 * cycle
        side = min(3, int(square_phase))
        progress = square_phase - side
        side_length = 2.0 * amplitude
        offsets = (
            (side_length * progress, 0.0),
            (side_length, side_length * progress),
            (side_length * (1.0 - progress), side_length),
            (0.0, side_length * (1.0 - progress)),
        )
        target[0] += ramp * offsets[side][0]
        target[1] += ramp * offsets[side][1]
        if trajectory == "square_xyz":
            target[2] += ramp * amplitude * math.sin(phase)
    elif trajectory == "line_y":
        target[1] += ramp * amplitude * math.sin(phase)
    elif trajectory == "sine_z":
        target[2] += ramp * amplitude * math.sin(phase)
    return target


def run_experiment(controller, args, trajectory, amplitude, frequency, repetition, stop):
    observation = np.asarray(controller.get_observation(), dtype=float)
    origin = observation[7:10].copy()
    rotation = observation[10:19].reshape(3, 3).copy()
    stem = f"{trajectory}_a{amplitude:g}_f{frequency:g}_r{repetition:02d}"
    csv_path = args.output / f"{stem}.csv"
    period = 1.0 / args.rate
    start_ns = time.monotonic_ns()
    deadline = start_ns

    with csv_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(COLUMNS)
        sequence = 0
        while not stop[0]:
            now_ns = time.monotonic_ns()
            elapsed = (now_ns - start_ns) * 1e-9
            if elapsed >= args.duration:
                break
            target = target_at(origin, trajectory, amplitude, frequency, elapsed, args.warmup)
            controller.set_target(target, rotation)
            observation = np.asarray(controller.get_observation(), dtype=float)
            measured = observation[7:10]
            writer.writerow(
                [sequence, now_ns, elapsed, trajectory, amplitude, frequency]
                + target.tolist()
                + measured.tolist()
                + [float(np.linalg.norm(target - measured))]
                + observation[0:7].tolist()
                + observation[19:25].tolist()
            )
            sequence += 1
            deadline += int(period * 1e9)
            time.sleep(max(0.0, (deadline - time.monotonic_ns()) * 1e-9))

    controller.set_target(origin, rotation)
    return csv_path


def main():
    args = parse_args()
    matrix = [
        {"trajectory": t, "amplitude": a, "frequency": f, "repetition": r}
        for t in args.trajectory for a in args.amplitude for f in args.frequency
        for r in range(1, args.repetitions + 1)
    ]
    args.output.mkdir(parents=True, exist_ok=True)
    metadata = vars(args).copy()
    metadata["output"] = str(args.output)
    metadata["runs"] = matrix
    metadata["created_unix_ns"] = time.time_ns()
    (args.output / "experiment.json").write_text(json.dumps(metadata, indent=2), encoding="utf-8")

    if not args.execute:
        print(f"Planned {len(matrix)} run(s); metadata written to {args.output}. Add --execute to move the robot.")
        return

    import kuka_fri_py as fri
    controller = fri.KukaController(fri.ControlMode.JOINT_POSITION, args.urdf, True)
    stop = [False]
    signal.signal(signal.SIGINT, lambda *_: stop.__setitem__(0, True))
    controller.start()
    try:
        for run in matrix:
            if stop[0]:
                break
            path = run_experiment(controller, args, stop=stop, **run)
            print(path)
            time.sleep(args.settle)
    finally:
        controller.stop()


if __name__ == "__main__":
    main()
