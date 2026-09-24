# KUKA FRI Control

Research software for controlling a KUKA LBR iiwa manipulator through the
Fast Research Interface (FRI). This repository accompanies an academic
research project and contains the host-side controller, Python bindings,
trajectory-recording utilities, robot models, and an experimental KUKA
Sunrise application.

> **Safety warning**
>
> This software can command a physical industrial robot. It is provided for
> research use and has no safety certification. Test changes in simulation or
> with the robot drives disabled first, verify all joint/workspace limits and
> controller gains, keep an emergency stop accessible, and begin with reduced
> speed and small motions. The operator is responsible for the safety of the
> complete robotic system.

## Repository layout

```text
.
|-- computer_side/       C++ FRI client, controllers, Python bindings, tools
|   |-- control/         Joint- and task-space control
|   |-- ik/              Drake-based kinematics and dynamics
|   |-- kukafri/         KUKA FRI client integration
|   |-- trajectory/      Reference-trajectory generators
|   |-- udp/             JSON/UDP command transport
|   |-- scripts/         Recording, plotting, and communication utilities
|   |-- robots/          URDF models and meshes
|   `-- examples/        KUKA FRI SDK overlay examples
`-- manipulator_side/    Experimental Sunrise Java application
```

The computer-side software receives targets over UDP or generates reference
trajectories locally. A high-level control loop converts these targets into
joint-position or torque commands, while a dedicated FRI thread exchanges
commands and state with the robot. Measurements and controller signals can be
logged to CSV for subsequent analysis.

See [computer_side/README.md](computer_side/README.md) for the controller
architecture, UDP packet format, observation and log layouts, Python API, and
trajectory-recording workflow.

## Requirements

- Linux
- CMake 3.16 or newer
- A C++20 compiler
- Eigen 3.3 or newer
- Drake with its CMake package configuration
- A KUKA FRI Client SDK/library compatible with the robot controller
- KUKA Sunrise.Workbench and the matching FRI package for robot-side work
- Python 3.10+, NumPy, and pybind11 (optional, for Python bindings)

FRI headers and a prebuilt static client library are present under
`computer_side/deps/`. Their compatibility with a particular controller and
the terms under which they may be used or redistributed must be checked with
the applicable KUKA SDK license.

## Build

Build the host-side executable from the repository root:

```bash
cmake -S computer_side -B computer_side/build \
  -Ddrake_DIR=/path/to/drake/lib/cmake/drake
cmake --build computer_side/build -j2
```

If Drake is already discoverable by CMake, `drake_DIR` can be omitted. The
resulting controller executable is `computer_side/build/FRI_control`.

To build the optional Python extension:

```bash
cmake -S computer_side -B computer_side/build \
  -DBUILD_PYTHON_BINDINGS=ON \
  -Ddrake_DIR=/path/to/drake/lib/cmake/drake
cmake --build computer_side/build --target kuka_fri_py -j2
```

The bundled static FRI library must be position-independent (`-fPIC`) when it
is linked into the Python module. Alternatively, pass an ABI-compatible shared
library with `-DFRI_LIB=/path/to/libFRIClient.so`.

## Configuration and use

Before running the controller, review the configuration near the beginning of
`computer_side/main.cpp`. In particular, verify:

- the control mode (joint-position or torque overlay);
- whether targets come from UDP or the local trajectory generator;
- the local and remote IP addresses and UDP ports;
- the URDF path and end-effector model;
- trajectory type, amplitude, and frequency;
- controller gains, limits, and the robot's initial pose.

Run from the build directory because the current executable uses a relative
path to the robot model:

```bash
cd computer_side/build
./FRI_control
```

The FRI connection blocks until robot state is available. Configure and start
a compatible FRI session on the robot controller before starting an
experiment. The current `main.cpp` defaults to torque control, task-space
control, and UDP targets on localhost; these are source-level settings rather
than command-line options.

The Java sources in `manipulator_side/` are an experimental Sunrise-side
prototype, not a turnkey deployment. Several message-decoding methods in
`RobotRemoteControl.java` are placeholders and must be implemented and
validated before use. Import these sources into a Sunrise.Workbench project,
set the correct client address, and configure the FRI session for the specific
cabinet and network. For ordinary FRI operation, a standard validated Sunrise
FRI application may be used instead.

## Reproducing experiments

`computer_side/scripts/record_trajectories.py` can create experiment plans and
record commanded/measured motion, timestamps, joint state, and estimated
wrench. It performs a dry run by default:

```bash
cd computer_side
python scripts/record_trajectories.py \
  --trajectory circle_xy square_xy line_y \
  --amplitude 0.01 0.02 \
  --frequency 0.05 0.10 \
  --duration 30 \
  --rate 100 \
  --repetitions 3 \
  --output trajectory_data
```

Inspect the generated experiment metadata before adding `--execute`, which
allows the script to send commands to the robot. Record the exact commit,
robot/controller versions, FRI version, URDF/tool parameters, gains, network
configuration, and experiment arguments when producing paper results.

## Research status and limitations

- The software is under active development and does not provide hard
  real-time scheduling on the host operating system.
- Network latency, packet loss, host load, and FRI configuration affect the
  achieved update rate and should be measured for each setup.
- Estimated Cartesian wrench is reconstructed from external joint torques and
  the model Jacobian; calibrate it against an external reference before using
  it as a quantitative measurement.
- Hardware-specific addresses, model paths, and control settings are currently
  configured in source code.

## Citation

If you use this software in academic work, please cite the associated paper.
The final bibliographic reference and BibTeX entry will be added here when the
paper is published.

## License

Repository-authored code is available under the [MIT License](LICENSE).
Third-party components, including KUKA FRI files, robot assets, Drake, Eigen,
pybind11, Boost, and nlohmann JSON, remain subject to their own licenses.
