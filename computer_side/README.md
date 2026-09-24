# KUKA FRI Computer-Side Controller

This repository contains the computer-side software used to control a KUKA LBR iiwa through the Fast Research Interface (FRI). It provides a C++ control application, joint- and task-space controllers, UDP communication with an operator-side process, CSV logging, reference-trajectory generation, and Python bindings for data collection and learning pipelines.

The FRI application runs in a dedicated thread so that robot communication is kept alive independently of command processing. The higher-level controller runs in a separate loop and exchanges joint commands and robot state with the FRI client through single-producer/single-consumer ring buffers.

> **Safety notice:** This is research software that commands physical robot hardware. Validate joint limits, workspace limits, controller gains, tool geometry, collision settings, and the emergency-stop procedure before use. Begin with low velocities and small trajectory amplitudes.

## Features

- KUKA FRI joint-position and torque overlays.
- Cartesian targets converted to joint commands using Drake kinematics.
- Task-space impedance control with joint-limit handling and damped pseudoinverse kinematics.
- Non-blocking UDP transport with sequence numbers and rejection of duplicate or reordered packets.
- Built-in circle, square, line, sinusoidal, and rotational reference trajectories.
- A 25-element observation vector exposed through C++ and Python.
- CSV logging and parameter-sweep scripts for repeatable trajectory trials.

## Architecture

```text
operator-side process
        | UDP: Cartesian increment or joint target
        v
main.cpp / UDPServer
        v
KukaController control loop
        |-- Control (IK-based joint controller), or
        |-- TaskSpaceControl (Cartesian controller)
        | lock-free joint command/state queues
        v
KukaFRIController / CustomLBRClient
        | KUKA FRI
        v
KUKA LBR iiwa
```

The code does not provide hard real-time scheduling. Actual update rates and end-to-end latency depend on the host operating system, FRI configuration, controller parameters, and network load and should be measured on the target system.

## Requirements

- Linux
- CMake 3.16 or newer and a C++20 compiler
- Eigen 3.3 or newer
- Drake with a CMake package configuration
- A KUKA FRI Client SDK/library compatible with the robot controller
- Threads/pthreads
- Python 3.10+, NumPy, and pybind11 for the optional Python module

The repository currently includes FRI headers and `deps/lib/libFRIClient.a`. Users are responsible for confirming that redistribution and use of the vendor-provided FRI components comply with the applicable KUKA license.

## Building the C++ application

```bash
cmake -S . -B build -Ddrake_DIR=/path/to/drake/lib/cmake/drake
cmake --build build -j2
```

Omit `drake_DIR` if Drake is discoverable by CMake. The main executable is `build/FRI_control`. The control mode, UDP source, target representation, trajectory parameters, network addresses, and URDF are currently selected near the beginning of `main.cpp`; review them before connecting to the robot.

## Python bindings

Build the `kuka_fri_py` pybind11 module with:

```bash
cmake -S . -B build -DBUILD_PYTHON_BINDINGS=ON
cmake --build build --target kuka_fri_py -j2
```

The bundled static FRI library must have been compiled with `-fPIC`, because a Python extension is a shared object. An ABI-compatible shared library can be selected instead:

```bash
cmake -S . -B build \
  -DBUILD_PYTHON_BINDINGS=ON \
  -DFRI_LIB=/path/to/libFRIClient.so
```

The package can also be installed with pip:

```bash
CMAKE_EXECUTABLE=/usr/bin/cmake \
CMAKE_ARGS='-Ddrake_DIR=/path/to/drake/lib/cmake/drake' \
python -m pip install .
```

Minimal example:

```python
import kuka_fri_py as fri

controller = fri.KukaController(
    fri.ControlMode.JOINT_POSITION,
    "robots/iiwa.urdf",
    False,
)
controller.start()
try:
    observation = controller.get_observation()
finally:
    controller.stop()
```

## Observation format

`KukaController::getObservation()` and `KukaController.get_observation()` return 25 floating-point values:

| Indices | Quantity | Unit / representation |
|---:|---|---|
| 0--6 | Measured joint positions | rad |
| 7--9 | End-effector position | m, base frame |
| 10--18 | End-effector orientation | row-major 3 x 3 rotation matrix |
| 19--24 | Estimated external wrench | torque followed by force |

The wrench is reconstructed from external joint torques and the robot Jacobian. It should be calibrated against an external reference before being treated as a quantitative force measurement.

## UDP protocol

Sequenced packets use a JSON envelope:

```json
{
  "sequence": 42,
  "data": [0.001, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
}
```

`sequence` is an unsigned, monotonically increasing packet number. Duplicate and reordered sequenced packets are discarded. Legacy JSON arrays are accepted for migration purposes, but cannot be checked for loss, duplication, or reordering.

In Cartesian UDP mode, `data` contains 12 values: a Cartesian position increment `(dx, dy, dz)` in metres followed by a row-major 3 x 3 target rotation matrix. When `use_udp_joint_target` is enabled in `main.cpp`, the first seven values are interpreted as joint targets in degrees. `UDPServer<12, 25>` denotes the expected command and observation sizes in the current application.

## Recording reference trajectories

`scripts/record_trajectories.py` records commanded and measured Cartesian positions, position error, joint positions, estimated wrench, monotonic timestamps, and sequence numbers. It supports parameter sweeps over trajectory type, amplitude, frequency, and repetition count.

The script is safe by default: without `--execute` it writes only the experiment plan and metadata.

```bash
python scripts/record_trajectories.py \
  --trajectory circle_xy square_xy line_y \
  --amplitude 0.01 0.02 \
  --frequency 0.05 0.10 \
  --duration 30 \
  --rate 100 \
  --repetitions 3 \
  --output trajectory_data
```

After reviewing `experiment.json`, add `--execute` to connect to and command the robot. For square trajectories, `amplitude` is half the side length. `square_xyz` follows the same XY square while adding sinusoidal Z motion. Each executed run produces one CSV file.

## Repository contents

### Top-level files

| Path | Description |
|---|---|
| `main.cpp` | Application entry point; receives UDP commands or generates a reference trajectory. |
| `main_controller.hpp/.cpp` | Thread-safe high-level controller owning the FRI connection, selected controller, observation state, and logger. |
| `pybind_module.cpp` | pybind11 definitions for `KukaController` and `ControlMode`. |
| `CMakeLists.txt` | Builds the C++ executable, FRI examples, and optional Python extension. |
| `setup.py`, `pyproject.toml` | Python package metadata and CMake-backed extension build. |
| `LICENSE` | MIT license for repository-authored code. |

### Control, kinematics, and trajectories

| Path | Description |
|---|---|
| `control/control_interface.hpp` | Common interface implemented by the controllers. |
| `control/control.hpp/.cpp` | IK-based controller with target filtering, joint-step limiting, position/torque commands, and wrench estimation. |
| `control/task_space_control.hpp/.cpp` | Cartesian motion and impedance controller with damped pseudoinverses, joint-limit terms, and torque-level null-space projection. |
| `ik/drake_kinematic.hpp/.cpp` | Drake wrapper for forward/inverse kinematics, force estimation, and model-based torque calculations. |
| `ik/cost.hpp` | Custom optimization cost used by the Drake kinematics solver. |
| `trajectory/trajectory_generator.hpp/.cpp` | Time-based circles, squares, lines, translation sinusoids, and rotational trajectories. |

### FRI integration and communication

| Path | Description |
|---|---|
| `kukafri/customlbrclient.hpp/.cpp` | FRI client callbacks and state/command queue integration. |
| `kukafri/kukafricontroller.hpp/.cpp` | Owns the FRI `ClientApplication` thread and exposes joint state/command methods. |
| `kukafri/helper_functions.hpp` | Conversion helpers for standard arrays and Eigen types. |
| `kukafri/apiserver.hpp/.cpp` | Legacy/general-purpose Boost.Asio UDP API server, separate from the server used by `main.cpp`. |
| `udp/udp_server.hpp/.cpp` | Non-blocking JSON/UDP transport, sequence handling, and Eigen/JSON conversion. |
| `udp/json.hpp` | Vendored nlohmann JSON single-header implementation. |
| `lockfree/lockfree.hpp` | Single-producer/single-consumer ring buffer and timing/hash utilities. |

### Logging, scripts, and robot models

| Path | Description |
|---|---|
| `logger/logger.hpp/.cpp` | Buffered CSV logger; writes `controller_log.csv`. |
| `scripts/record_trajectories.py` | Parameterized trajectory execution and dataset recorder. |
| `scripts/fri_python.py` | Python/UDP joint-command example supporting sequenced envelopes and legacy arrays. |
| `scripts/print.py` | Exploratory plotting utility for `controller_log.csv`. |
| `scripts/read_tenso.py` | Serial monitor for an external device on `/dev/ttyUSB0`. |
| `scripts/temp.py` | Exploratory mass/value calibration plot; not part of the runtime. |
| `robots/iiwa.urdf` | Base iiwa model used by the default Python example. |
| `robots/iiwa2.urdf` | Alternative iiwa model. |
| `robots/iiwa2_gripper.urdf` | Model with tool/gripper geometry used by the C++ application. |
| `robots/meshes/` | Visual and collision meshes referenced by the URDF files. |

### Dependencies and examples

| Path | Description |
|---|---|
| `deps/include/FRI/` | KUKA FRI Client SDK headers. |
| `deps/lib/libFRIClient.a` | Bundled static FRI client library. |
| `deps/deps_autoinstall.sh` | Legacy dependency installation helper; inspect before running. |
| `examples/LBRJointSineOverlay/` | KUKA joint-position sine-overlay example. |
| `examples/LBRTorqueSineOverlay/` | KUKA torque sine-overlay example. |

`temp/` contains working material and is not required to build or run the controller.

## Log format

The C++ controller writes `controller_log.csv`. Each row contains a record type, a monotonic timestamp in microseconds from controller start, and seven payload values.

| Type | Payload |
|---:|---|
| 0 | Target joint positions |
| 1 | Measured joint positions |
| 2 | Target joint torques |
| 3 | External joint torques |
| 4 | Requested Cartesian target position |
| 5 | Measured Cartesian position |
| 6 | Estimated Cartesian force components |
| 7 | Requested joint target |

## License

Repository-authored code is provided under the MIT License. See `LICENSE`. Third-party components, including KUKA FRI files, Drake, Eigen, Boost, pybind11, and nlohmann JSON, remain subject to their respective licenses.
