# Modular Robot Simulation (MuJoCo + ROS2)

> **About this fork (Kurabayashi Lab)**
> Based on [twoo-kim/modular](https://github.com/twoo-kim/modular). The Kurabayashi Lab paddle and connector CAD (by J. Yamaguchi) is added next to Kim's model, with Python demos for two, three and four modules. Kim's original model (`modular.xml`, Kim's paddle), its fluid coefficients, densities, joint settings and scripts are unchanged, so `test_force.py` and `test_experiment.py` still compare against the `exp/` data with Kim's paddle.
>
> | Change | Files |
> |---|---|
> | Module with the Kurabayashi Lab paddle (60 mm by default; 40 mm and 80 mm included; the number is the size of the V-shaped blade, not the overall width) | `models/xml/modular_cad.xml`, `models/meshes/paddle{40,60,80}.stl` (meshes declared in `asset.xml`) |
> | Two modules with the Kurabayashi Lab paddle, joined by the tetra 1-DOF connector | `models/xml/colony/test_2_tetra.xml`, `models/meshes/tetra_connector.stl` |
> | Three CAD modules on a ring at 120° intervals | `models/xml/colony/test_3_ring.xml`, `src/controller/scripts/run_three_module_cpg.py` |
> | Four CAD modules in a ring with vertical paddle strokes and an optional fixed center | `models/xml/colony/test_4_ring.xml`, `src/controller/scripts/run_four_module_cpg.py` |
> | Original CAD (mm, not transformed) | `models/cad/` |
>
> - Paddle size: change `mesh="paddle60"` in `modular_cad.xml` to `paddle40` or `paddle80`. Models that include `modular.xml` keep Kim's paddle (`paddle.STL`).
> - The paddle meshes in `meshes/` are the original CAD scaled from mm to m (x0.001), rotated 180 deg about x, and translated by (0, -30, +1.7) mm so that the mounting hole is at the paddle hinge origin.
> - In `test_2_tetra.xml` the whole colony rotates only about the vertical axis (world z through (0, 0, 0.5) m), matching the fixed-shaft experiment. For free swimming, replace `colony_yaw` with `<joint name="base" type="free"/>`.
> - The connector uses PLA density 1240 kg/m^3 and has contact disabled; it overlaps the old capsule body.
> - Mass, inertia and fluid coefficients are not calibrated against measurements.
> - Tested: the models load and run in MuJoCo 3.3.7.
>
> **Example: two modules driven by phase oscillators** — `src/controller/scripts/run_two_module_cpg.py`
>
> | Setting | Value |
> |---|---|
> | Motor target | 40 deg x cos(phi_i) |
> | Motor control | Torque PD, kp = 0.060, kd = 0.002 |
> | Motor limits (HS-5086WP at 4.8 V) | ±0.2941995 N·m and ±5.818 rad/s (±333.3°/s) |
> | Angular frequency (default) | omega_1 = 2.5 pi, omega_2 = 2.4 pi rad/s |
> | Flex sensor value | S_i = -clip(q_i / (2 q_limit), -0.5, 0.5), q_i: passive paddle angle |
> | Phase update | dphi_i/dt = omega_i + sigma Z(phi_i) S_i (`sigma = 0` for open loop; edit `z_profile()` in `src/controller/scripts/module_cpg.py` for Z) |
>
> The CAD model caps the motor torque. The Python demos bound the motor joint speed and angle change per time step. This approximates the HS-5086WP's stall torque and no-load speed as independent limits. [Manufacturer specifications](https://hitecrcd.co.jp/products/servo35086s/)
>
> ```
> python src/controller/scripts/run_two_module_cpg.py                 # open loop, 30 s, CSV in logs/
> python src/controller/scripts/run_two_module_cpg.py --sigma 0.1     # with feedback
> mjpython src/controller/scripts/run_two_module_cpg.py --view        # viewer on macOS
> ```
>
> **Four-module Volvocine ring**
>
> Modules 1–4 are mounted on +X, +Y, −X and −Y at a radius of 60 mm, with about 85 mm between neighboring mounts. The hinge axes are tangent to the ring so the outward-facing paddles stroke up and down. For the 40, 60 and 80 mm paddle meshes, the projected CAD stays within separate 90° sectors across the full joint ranges (motor ±45°, passive paddle ±22.5°). This leaves at least 3.3 mm of clearance between neighboring modules and a central opening of about 36 mm in the plane of the ring.
>
> A massless rigid parent stands in for the module connections. The ring starts at (0, 0, 0.5) m and its free joint allows three translations and three rotations. Use `--fixed-center` to select a ball joint that holds the center while allowing all three rotations. The CAD modules supply the mass and inertia, and the center stays open.
>
> Use `--float` for a simple surface-floating mode. The ring starts flipped 180° about the X axis through its center. A vertical spring and damper approximate net buoyancy about the starting height, with horizontal motion and all three rotations free. The viewer draws a waterline 15 mm above that height (`z = 0.515 m` for this model). The fluid loads are evaluated in the same uniform medium at every height.
>
> Run from the repository root with MuJoCo 3.3.7. Use `mjpython` with `--view` on macOS.
>
> ```bash
> .venv/bin/python src/controller/scripts/run_four_module_cpg.py
> .venv/bin/python src/controller/scripts/run_four_module_cpg.py --sigma 0.1 --phi0 0 1.57 3.14 4.71 --omega1 7.9 --omega2 7.5 --omega3 7.2 --omega4 6.9
> .venv/bin/python src/controller/scripts/run_four_module_cpg.py --omega 7.5 --random-phi0
> .venv/bin/python src/controller/scripts/run_four_module_cpg.py --view
> .venv/bin/python src/controller/scripts/run_four_module_cpg.py --float --view
> .venv/bin/python src/controller/scripts/run_four_module_cpg.py --view --fixed-center
> ```
>
> Both demos use the torque PD and phase feedback above. The four-module run defaults to open loop for 30 s, with initial phases (0, π/2, π, 3π/2) rad and angular frequencies (2.5, 2.4, 2.3, 2.2) × π rad/s. Override the phases in module order with `--phi0` and set individual natural angular frequencies with `--omega1` through `--omega4`. `--omega` sets the same value in rad/s for all four modules and takes precedence over individual values.
>
> `--random-phi0` draws the four initial phases independently from a uniform distribution on `[0, 2π)` rad. Add `--seed 42` to repeat the same draw. The CSV records the initial phases at `time_s = 0`.
>
> In open loop, `--omega 7.5` gives each paddle a target period of about 0.84 s (`2π / 7.5`). With `--view`, playback is paced against wall-clock time and the camera follows the center of the ring.
>
> The default log `logs/four_module_cpg.csv` records each module's phase, motor angle, passive paddle angle and Flex value. The `colony_yaw_rad`, `colony_pitch_rad` and `colony_roll_rad` columns give the orientation in ZYX order. Free runs also record the center position in `colony_x_m`, `colony_y_m` and `colony_z_m`. Use `--out` to choose another destination.
>
> To save a plot of each module's phase minus the reference module's phase, install Matplotlib in the project `.venv` once and run the plot script with the same interpreter. The script wraps the differences to `[-π, π)` radians and saves the PNG beside the CSV. Use `--modules 2`, `--modules 3` or `--modules 4` to select the demo log, with four modules as the default. Pass a CSV path as the first argument to plot another run, and use `--out` to set the image path.
>
> ```bash
> .venv/bin/python -m pip install matplotlib
> .venv/bin/python src/controller/scripts/plot_phase_differences.py --reference 1
> ```
>
> **Three-module Volvocine ring**
>
> The three mounts start at +X and advance in 120° steps about world Z. The 60 mm mounting radius gives about 104 mm between neighboring mounts. All three paddle sizes fit through their full joint ranges with at least 24.6 mm of clearance between modules in the projection onto the ring plane.
>
> The three-module demo uses the same controller and modes as the four-module demo, including free motion, `--fixed-center` and `--float` with an inverted starting pose. `--phi0` takes three phases and defaults to (0, 2π/3, 4π/3) rad. Use `--omega` for a common frequency, or `--omega1` through `--omega3` for individual frequencies, whose defaults are (2.5, 2.4, 2.3) × π rad/s. The default output is `logs/three_module_cpg.csv`.
>
> ```bash
> .venv/bin/python src/controller/scripts/run_three_module_cpg.py --float --view --omega 7.5
> .venv/bin/python src/controller/scripts/plot_phase_differences.py --modules 3 --reference 1
> ```

Simulation of modular robots with one active motor joint and one passive paddle joint per module. Run via pure Python scripts or as ROS2 C++ nodes. Includes linear time-periodic (LTP) analysis and harmonic transfer function (HTF) tools.

## Overview
- MuJoCo-based dynamics with optional OpenGL viewer.
- Python scripts for quick experiments and plots (no ROS needed).
- ROS2 C++ simulator + per-module controllers for multi-agent setups.
- Offline C++ runner computes LTV/HTF for periodic behavior analysis.

## Quick Start

Follow these steps depending on whether you want Python-only runs or the full ROS2 C++ stack.

### A) Python-only (no ROS)
1) Install Python packages
   - `pip install mujoco matplotlib numpy`

2) Verify MuJoCo Python works
   - `python -c "import mujoco; print(mujoco.__version__)"`

3) Run examples
   - Thrust sweep: `python src/controller/scripts/test_force.py`
   - Single module time series: `python src/controller/scripts/test_module.py`
   - Compare sim vs experiment: update CSV path and run `src/controller/scripts/test_experiment.py`

Notes
- These scripts load XML from `models/xml/` directly; no ROS is required.

### B) ROS2 C++ Simulator
Prerequisites
- Installed ROS2 (any supported distro), C++ toolchain, and `colcon`.
- MuJoCo binaries and GLFW development headers.

1) Install MuJoCo binaries
   - Download from: https://github.com/google-deepmind/mujoco/releases (e.g., `mujoco-3.3.7`).
   - Place at: `$HOME/.mujoco/mujoco-3.3.7/`.
   - Add to your shell init (e.g., `~/.bashrc`):
     
     ```
     export MUJOCO_PATH=$HOME/.mujoco/mujoco-3.3.7
     export LD_LIBRARY_PATH=$MUJOCO_PATH/lib:$MUJOCO_PATH/bin:$LD_LIBRARY_PATH
     ```
   - Apply changes: `source ~/.bashrc`.

2) Install GLFW3 (viewer)
   - Ubuntu/Debian: `sudo apt-get install -y libglfw3-dev`

3) Build the packages
   - From your ROS2 workspace:
     ```
     colcon build --packages-up-to modular
     source install/setup.bash
     ```

4) Configure the simulator
   - Edit `src/controller/config/modular_params.yaml` to set:
     - `N` (number of modules), `SIM_TIME`, `TIME_STEP`, `isFixed`, `visualize`.
   - The simulator generates a colony XML under `models/xml/colony/` at runtime.

5) Run
   - `ros2 launch modular modular_launch.py`
   - This launches N controller nodes and one simulator node. The simulator publishes `/pose_topic`; each controller publishes to its own `<prefix>module/control` topic (e.g., `r1_module/control`).

Troubleshooting
- If the viewer fails, set `visualize: false` in the YAML or install/configure OpenGL/X forwarding.
- If MuJoCo libraries aren’t found, re-check `LD_LIBRARY_PATH` and `MUJOCO_PATH`.
- The CMake file uses a `MUJOCO_PATH` use absolute paths or rely on system include/lib paths.

### C) Dynamics Simulation + LTP/HTF (offline C++)
Use the standalone C++ runners to step MuJoCo, collect LTV (A/B) along a periodic trajectory, and build the harmonic transfer function.

Prerequisites
- Same as B) (MuJoCo + GLFW). Also ensure Python dev headers are available (CMake links `Python3::Python` and `Python3::NumPy`). On Ubuntu/Debian: `sudo apt-get install -y python3-dev python3-numpy`.

1) Build
   - From your ROS2 workspace:
     ```
     colcon build --packages-up-to modular
     source install/setup.bash
     ```

2) Configure analysis
   - Edit `src/controller/config/modular_params.yaml`:
     - Time: set `SIM_TIME` long enough to cover several periods at your frequency (e.g., ≥ 8–10 periods).
     - LTP/HTF: set `harmonics` (e.g., 20) and `sampling_f` (Hz) for Fourier blocks.
     - Optional: `visualize: false` for faster, headless runs.
   - IMPORTANT: The sample runners hard-code `config_path` in C++:
     - `src/controller/src/dynamics_simulation.cpp`
     - `src/controller/src/dynamics_simulation_multi.cpp`
     Update `config_path` to your local `modular_params.yaml` path, then rebuild.

3) Run
   - Single-case:\
      `ros2 run modular colony_sim`
   - Phase sweep:\
   `ros2 run modular colony_sim_multi --phases=0,30,60,90,120,150,180`
    
    - If argument parsing via `ros2 run` is problematic, run the binary directly:
       `install/modular/lib/modular/colony_sim_multi --phases=0,30,60,90`

4) Inspect outputs
   - Joint angles over time for modules.
   - Periodicity check: norms of A(2s)–A(t), B(2s)–B(t) over one period.
   - HTF magnitude vs harmonic order for selected joints (e.g., module 1 passive).

Tips
- Ensure the fundamental frequency used in the runner matches your intended study; the examples set frequency and amplitude inside the C++ files via `setFrequency()`/`setAmplitude()`.
- For stable HTF estimates, the routine ignores the initial transient (≈ first 2 s).

## Repository Structure
- `models/`:
  - `xml/modular.xml`: single-module robot definition (joints, sensors, actuator).
  - `xml/environment.xml`, `xml/asset.xml`: visuals/materials.
  - `xml/colony/`: auto-generated multi-module colony XMLs.
  - `urdf/`: URDF variants (not used by the runtime sim).
- `modular_msgs/`: ROS2 message package (`ControlMsg.msg`, `StateMsg.msg`).
- `src/controller/`:
  - `config/modular_params.yaml`: central configuration (N, dt, gains, fluids, LTP, phases).
  - `include/`: headers for config, modules, LTV, simulator, viewer, utils, XML generation.
  - `src/simulation/*.cpp`: implementations for viewer, ROS simulator, dynamics+LTP.
  - `src/*.cpp`: main entrypoints (`sim_node`, `ctrl_node`, `colony_sim`, `colony_sim_multi`).
  - `launch/modular_launch.py`: spawns N controllers + one simulator.
  - `scripts/`: Python demos (`test_force.py`, `test_module.py`, `test_experiment.py`).
- `exp/`: experimental CSVs and PDFs for validation/alignment.

## Workflows
### Python (no ROS)
1) Install: `pip install mujoco matplotlib numpy`  
2) Examples:
- Sweep thrust vs angle/frequency: `python src/controller/scripts/test_force.py`
- Single module time series: `python src/controller/scripts/test_module.py`
- Compare sim vs experiment (RMS, overlays): `python src/controller/scripts/test_experiment.py`

### ROS2 C++
1) Install MuJoCo binaries and GLFW (`libglfw3-dev`). Ensure `LD_LIBRARY_PATH` includes MuJoCo libs.  
2) Build in your ROS2 workspace: `colcon build --packages-up-to modular modular_msgs`  
3) Launch: `ros2 launch modular modular_launch.py`  
4) Tune `src/controller/config/modular_params.yaml` (e.g., `N`, `phase_gap`, `isFixed`).

### Offline LTP/HTF (C++)
Build then run:
- `modular/colony_sim`: single-case plots (joints, thrust, periodicity, HTF magnitude).
- `modular/colony_sim_multi`: phase-sweep overlays (thrust, passive angle, HTF).  
Adjust analysis in `modular_params.yaml` (`harmonics`, `sampling_f`, `SIM_TIME`).

## Configuration (YAML)
File: `src/controller/config/modular_params.yaml`
- Geometry: `N`, `RADIUS`, `FACE_RADIAL`, `YAW_OFFSET_DEG`, `ADD_DUMMY_MASS`.
- Time: `SIM_TIME`, `TIME_STEP`, `isEuler` (Euler vs RK4), `isFixed` (free base vs fixed).
- Control: `is_activate` (per-module enable), `P_gain`, `D_gain`.
- Fluid: `Cn`, `rho`, `viscosity`.
- LTP/HTF: `harmonics`, `sampling_f`, `phase_gap` (per-module phase offsets in radians).

## ROS Interfaces
- Messages:
  - `modular_msgs/msg/ControlMsg`: `index` (1-based module ID), `ctrl` (control input), `phi` (internal phase).
  - `modular_msgs/msg/StateMsg`: `index[]`, `joint_motor[]`, `joint_paddle[]`, `force[]`, `pose[]`, `twist[]`, `phase[]`.
- Topics:
  - Simulator publishes `/pose_topic`.
  - Controllers publish `<prefix>module/control` (e.g., `r1_module/control`).
- Launch: `launch/modular_launch.py` reads `N` from YAML, starts N controller nodes + one simulator.

## Key Files
- Config: `src/controller/config/modular_params.yaml` (main knobs most users change).
- XML Gen: `src/controller/include/utils/xml_utils.hpp` (creates `models/xml/colony/test_<N>_circle.xml`).
- ROS Sim: `src/controller/src/simulation/mujoco_simulator.cpp` (subscribes control, steps sim, publishes state).
- Dynamics + LTP: `src/controller/src/simulation/dynamics_simulator.cpp` (linearization, HTF building, plotting).
- Experiments: `src/controller/scripts/test_experiment.py` (CSV load, alignment, RMS, plotting).

## Design Notes
- Control: per-module PD maps desired position-style input to actuator effort; enable via `is_activate`.
- Phase: `phase_gap[]` sets module offsets (used in offline runners; extendable to controllers).
- Linearization: LTV (A/B) captured each step with `mjd_transitionFD`; HTF assembled from Fourier blocks.
- Fluid: paddle uses `fluidshape`/`fluidcoef`; global density/viscosity set via YAML → XML.

## Known Pitfalls
- Absolute paths: some offline runners hard-code config path; switch to a relative path or pass via CLI.
- MuJoCo path in CMake: avoid `~` in `MUJOCO_PATH`; use absolute path or system include/lib.
- Viewer: set `visualize: true` only with a working OpenGL/X setup. Headless runs fine otherwise.
- LTP windows: HTF uses post-transient data (≈2 s onwards). Ensure enough `SIM_TIME` for your frequency.

 
