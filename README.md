# Autonomous Intercept Drone
### Image-based Visual Servo Autonomous Intercept Drone System (Autonomous Intercept Drone with Image-based Visual Servo)

<div align="center">

**English** | [**中文**](README.zh-CN.md)

[![Bilibili](https://img.shields.io/badge/Bilibili-观看视频演示-ff69b4?style=for-the-badge&logo=bilibili)](https://www.bilibili.com/video/BV1M8QVYHE39/?spm_id_from=333.1387.homepage.video_card.click)
[![PX4](https://img.shields.io/badge/PX4-Autopilot-blue.svg)](https://px4.io/)
[![ROS2](https://img.shields.io/badge/ROS2-Humble%2FFoxy-green.svg)](https://docs.ros.org/en/humble/)
[![TensorRT](https://img.shields.io/badge/TensorRT-Accelerated-76B900.svg)](https://developer.nvidia.com/tensorrt)
[![YOLO11](https://img.shields.io/badge/YOLO-v11-yellow.svg)](https://github.com/ultralytics/ultralytics)

</div>

> This project is an experimental verification platform for autonomous drone interception tasks. It integrates visual perception, target detection, proportional navigation guidance (PNG) algorithms, and flight control methods. The project aims to provide a complete simulation reference solution for autonomous drone interception, guidance algorithm research, and visual servo control, and is suitable for learning and academic research.
---

## Project Overview

Developed in the ROS 2 environment and combined with the PX4 firmware and the Gazebo simulator, this system implements a closed loop from target discovery to simulated interception. The main features include:

*   **Visual Target Detection**: Uses a trained YOLO model and a tracking model (LightTrack) to recognize and track drone targets in the field of view in real time.
*   **Proportional Navigation Guidance**: Generates intercept trajectories based on the Proportional Navigation Guidance (PNG) law to achieve precise lead interception of dynamic targets.
*   **Simulation Verification System**: Built on PX4 software-in-the-loop (SITL), it supports offline algorithm verification and parameter tuning in the Gazebo environment.
*   **Data Analysis Tools**: Built-in kinematics and visual metrics recording scripts are provided for quantitative evaluation of guidance law accuracy and visual tracking stability.

---

## Simulation Demonstration

### 1. Full Dynamic Interception Process
<div align="center">
  <img src="assets/output.gif" alt="Interception dynamic demonstration" width="80%">
  <p><i>(The PNG algorithm predicts the target trajectory to achieve precise physical collision interception.)</i></p>
  <p><b>📺 <a href="https://www.bilibili.com/video/BV1sKBVBgEZ9/">Click here to watch the full HD demonstration video on Bilibili</a></b></p>
</div>

### 2. Small-Target Drone Detection Framework
<div align="center">
  <img src="assets/小目标无人机检测框架.png" alt="Small-target detection framework" width="80%">
  <p><i>(Vision processing pipeline optimized for drone "point targets".)</i></p>
</div>

---

## System Architecture and Logic

The system adopts a layered ROS 2 architecture, ensuring high flexibility and real-time performance:

<div align="center">
  <img src="assets/框架.png" alt="Architecture diagram" width="80%">
</div>

| Module | Description | Key Packages | Status/Notes |
| :--- | :--- | :--- | :--- |
| **Perception Layer** | Target detection, pixel error computation, line-of-sight angle extraction | `uav_vision_dectect`, `uav_vision_png` | **Current primary solution** |
| **Guidance Layer** | PNG guidance law computation, lead compensation, trajectory generation | `uav_png_intercept` | **Pure PNG solution (does not rely on vision)** |
| **Learned Guidance** | Reinforcement learning based GRU policy guidance, replacing traditional PNG | `uav_rl_guidance` | 🆕 **RL solution (replaces vision_png)** |
| **Control Layer** | PX4 Offboard interface, velocity/attitude closed-loop control | `uav_vehicle_controller`, `px4_ros_com` | Core control foundation |
| **Simulation Layer** | Target drone dynamics simulation, multiple motion mode support, intercept environment generation | `uav_target_sim` | Simulation support |
| **Earlier Solution** | Image-based visual servo control solution | `uav_ibvs_control` | **Deprecated (code kept for reference only)** |

---

## Quantitative Interception Performance Analysis

The following shows the system performance in typical interception tasks, which can serve as a reference baseline for algorithm optimization:

### Kinematic Performance
<div align="center">
  <table>
    <tr>
      <td><img src="assets/plots_output/1_3D_Trajectory.png" width="300px"><br><b>3D Interception Trajectory</b></td>
      <td><img src="assets/plots_output/2_Relative_Distance.png" width="300px"><br><b>Relative Distance Convergence</b></td>
      <td><img src="assets/plots_output/3_Velocity.png" width="300px"><br><b>Interceptor Velocity Vector</b></td>
    </tr>
    <tr>
      <td>Shows the lead interception in 3D space</td>
      <td>Verifies that the intercept distance converges to &lt;0.2m</td>
      <td>Reflects the efficient use of thrust by the guidance law</td>
    </tr>
  </table>
</div>

### Visual Tracking Performance
<div align="center">
  <table>
    <tr>
      <td><img src="assets/plots_output/4_LOS_PNG_Angles.png" width="300px"><br><b>Line-of-Sight (LOS) Angle Evolution</b></td>
      <td><img src="assets/plots_output/5_Pixel_Error.png" width="300px"><br><b>Visual Center Tracking Error</b></td>
      <td><img src="assets/plots_output/6_2D_Top_View.png" width="300px"><br><b>2D Top-Down Interception Path</b></td>
    </tr>
    <tr>
      <td>Convergence stability analysis of the guidance law</td>
      <td>Robustness of the detection algorithm during dynamic motion</td>
      <td>Typical lead compensation effect demonstration</td>
    </tr>
  </table>
</div>

---

## Quick Start

### 1. Environment Requirements
*   **ROS 2 Version**: Humble; native Ubuntu 24.04 / Jazzy simulation and visual guidance setup is described in sections 2.1 and 2.2. RL inference requires separate setup.
*   **Gazebo**: Gazebo Harmonic (gz-sim 8.12.0)
*   **PX4 Firmware**: v1.16 (**Micro-XRCE-DDS-Agent** version v2.4.x recommended)
*   **Dependencies**: OpenCV, onnx, colcon

### 2. Build the Project
```bash
# Enter the workspace and build
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
```

### 2.1 Native Ubuntu 24.04 / Jazzy

Verified combination: Jazzy, Gazebo Harmonic 8.11.0, PX4 v1.16.0, and Micro-XRCE-DDS-Agent v2.4.3. Verification covers both vehicles' GPS/position/status messages, the 1920×1080 RGB camera bridge, and dual Offboard takeoff using `uav_target_sim` + `uav_png_intercept`. Visual guidance uses the detector below; RL and a completed interception are outside this verification scope.

Version and configuration requirements:

* Pin PX4 to `v1.16.0` (`6ea3539157ca358c70a515878b77077af7d4611d`), with the default source directory `~/PX4-Autopilot`.
* Before building this version, apply `patches/px4-v1.16.0-imu-first-sample.patch`: the first IMU sample must seed the integrator, not integrate the elapsed time since boot. On a clean, unpatched PX4 checkout:

  ```bash
  git -C ~/PX4-Autopilot apply \
    ~/intercept_ws/src/Autonomous-Intercept-Drone/patches/px4-v1.16.0-imu-first-sample.patch
  ```

  Rebuild PX4 using the commands below; rebuilding ROS packages alone does not update the flight controller. Do not apply the patch twice.
* Use `px4_msgs` from `release/1.16` at `392e831c1f659429ca83902e66820d7094591410`; all 226 message definitions match this firmware. Restore submodules with `git submodule update --init px4_msgs px4_ros_com`.
* The firmware publishes `/px4_N/fmu/out/vehicle_status_v1`, not the old unversioned status topic.
* `uav_vision_png` also needs the following entry in the firmware's `src/modules/uxrce_dds_client/dds_topics.yaml` under `publications`, followed by a firmware rebuild:

  ```yaml
  - topic: /fmu/out/hover_thrust_estimate
    type: px4_msgs::msg::HoverThrustEstimate
  ```

* The DDS Agent may be installed to `~/.local/opt/intercept/agent`, with its command linked into `~/.local/bin/MicroXRCEAgent`. Reuse Jazzy's Fast-DDS 2.14 / Fast-CDR 2.2, but use upstream spdlog v1.9.2 for Agent v2.4.3 rather than the host's incompatible spdlog 1.12 / fmt 9.
* Keep a real [QGroundControl ground station](https://docs.qgroundcontrol.com/Stable_V5.1/en/qgc-user-guide/getting_started/download_and_install.html) connected. PX4 may reject arming without it; do not replace the missing connection by disabling preflight checks or forcing arming.
* `run_swarm.sh` sets the same height configuration for both vehicles: `EKF2_HGT_REF=0` (barometer), `EKF2_GPS_CTRL=5` (GNSS horizontal position and 3D velocity, without GNSS altitude fusion), `EKF2_GPS_DELAY=0` (matching Gazebo's current-time samples), and `EKF2_BARO_NOISE=0.3 m`. Both models have barometers and no downward range sensor. Measured barometric altitude noise relative to ground truth is approximately 0.15 m standard deviation, rather than the estimator's previous 3.5 m setting. These are project-specific SITL settings, not hardware recommendations; simulated sensor noise and safety checks remain enabled.
* For stationary height acceptance, wait for both ground-height and vertical-velocity estimates to converge before recording each vehicle's own ground local z. Neither `z_valid` nor subtracting independent local z values proves physical relative height; validate named-model Gazebo world z instead. Formal Vision PNG comparisons remain paused until the visual chain also passes validation.
* Link the repository once at `~/intercept_ws/src/Autonomous-Intercept-Drone`. Do not separately link the same dependency packages.

Build PX4 with an isolated Python environment instead of modifying system Python. For the prepared environment:

```bash
source /opt/ros/jazzy/setup.bash
source ~/.local/opt/intercept/px4-venv/bin/activate
export CMAKE_PREFIX_PATH="/opt/ros/jazzy${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
export GZ_DISTRO=harmonic
cmake -S ~/PX4-Autopilot -B ~/PX4-Autopilot/build/px4_sitl_default \
  -G Ninja -DCONFIG=px4_sitl_default -DPYTHON_EXECUTABLE="$VIRTUAL_ENV/bin/python" \
  -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build ~/PX4-Autopilot/build/px4_sitl_default \
  --target px4 px4_gz_plugins --parallel 4
```

Do not use `DONT_RUN=1 make ... gz_x500` as a build-only command for this tag: the Gazebo target still launches simulation. Leave the Python virtual environment before building the ROS workspace:

```bash
deactivate
source /opt/ros/jazzy/setup.bash
cd ~/intercept_ws
colcon build --packages-select \
  px4_msgs uav_common_msg uav_vehicle_controller uav_target_sim \
  uav_png_intercept uav_keyboard_control uav_vision_png uav_bc_recorder ros2_helloworld \
  --parallel-workers 2 --cmake-args -DBUILD_TESTING=OFF
source install/setup.bash
```

This build excludes `uav_vision_dectect`, `uav_rl_guidance`, and upstream `px4_ros_com` examples.

Load this environment in every ROS terminal:

```bash
source /opt/ros/jazzy/setup.bash
source ~/intercept_ws/install/setup.bash
export PATH="$HOME/.local/bin:$PATH"
export ROS_DOMAIN_ID=173
export GZ_PARTITION=intercept_jazzy
```

Run these commands in four separate terminals:

1. `~/.local/bin/qgroundcontrol` (the prepared user-local launcher; keep it open).
2. `bash "$HOME/intercept_ws/src/Autonomous-Intercept-Drone/run_swarm.sh"` (adjust to the actual repository path). Deploy the repository world first; run from the complete repository rather than copying/linking the script alone into PX4. It deploys a small self-model wrapper without replacing upstream resources. `PX4_ROOT` overrides PX4; `HEADLESS=1` omits GUI.
3. `ros2 run uav_target_sim uav_target_sim`.
4. `ros2 run uav_png_intercept uav_png_intercept`. For recording, start `experiment_recorder` from section 2.3 first with `algorithm:=pure_png`.

`run_swarm.sh` stops only its own process groups and no longer globally kills other simulations or deletes historical logs. Stopping control nodes triggers normal Offboard-loss protection; verification covers takeoff and communication, not a completed interception.


### 2.2 Jazzy Visual Detection and Guidance

**Current temporary `vision_png` default baseline: `visionpng-baseline-high08-refresh5-probation-raw`.** Adopted at the user's request, with implementation `4160914`: bounded probation and capture-attitude rotation compensation, retaining self-visual isolation, unique Gazebo partitions, Reliable QoS and native timing. Candidates.4/global acquisition strictly>.8/refresh5/support IoU.1/hold.35s/recovery2s; raw keeps KF off and control gains unchanged. Fixed-circle first hits were3/3 far and2/3 near; the near long-loss counterexample remains. This is a provisional working baseline, not a claim that stability is solved.

The historical frozen tag `visionpng-baseline-high08-refresh5-raw` retains its small-cohort GPS results (first hits3/3 at24m,2/3 at16.8m). Current post-baseline self-visual isolation and same-image support validation change behavior; old hit rates are not new-version performance or general safety guarantees. RL is outside this scope.

The vision chain is `/camera/image` → `uav_vision_dectect` → `/camera_detect_result` → `uav_vision_png`. The detector publishes no flight-control commands; do not simultaneously run `uav_png_intercept` or another interceptor controller.

Vision PNG updates LOS/PNG and yaw-PD history once per new valid image observation. The baseline maps capture time and SLERPs attitude at the image epoch rather than substituting receipt pose. New images with equal bbox coordinates are still processed. The 200 Hz controller retains the existing velocity formula, gains and vertical compensation; visual history does not advance between observations.

Loss limits are now seconds: `freshness_timeout: 0.15` and `track_lost_timeout: 0.6`, both finite with `0 < freshness_timeout < track_lost_timeout`. No-target results neither replace the cached observation nor refresh its clock, and do not immediately enter COAST. COAST starts only when monotonic age since the last valid detection receipt exceeds 0.15 s; TRACK_LOST starts at 0.6 s, including when the detector is silent. Valid detections returning within the freshness window retain LOS/PNG and yaw-PD history without reinitialization. COAST holds the last velocity/yaw-rate command and still evaluates hits; TRACK_LOST retains braking/search behavior. Recovery from these loss states reinitializes PNG and yaw difference references. The old frame-count parameters `coast_thresh`/`lost_thresh` are removed; migrate old configurations.

Receipt age is not image age or inference latency; use native-clock-aligned observation diagnostics. `uav_vision_png/test/test_visual_freshness.py` launches real nodes for freshness, recovery, repeated-coordinate frames and attitude-time alignment. Node regressions alone do not establish interception success rates.

Global first acquisition remains strictly>0.8. Refresh5 uses current-frame spatial support, but a miss now retains the tracker under a bounded steady-clock budget: `tracking_validation_hold=.35s` limits valid publication; `tracking_recovery_timeout=2s` limits internal retention. Probation checks every frame and permits .4–.8 spatial support. Prefer the current prediction; only unexpired probation can use the last confirmed YOLO box for local reanchoring. Tracker confidence never renews support, unrelated high scores cannot replace supported tracking, and expiration precedes recovery. Refresh0 disables frame cadence only, not timed authorization checks.


Use ONNX Runtime C++ GPU SDK 1.23.2 with Blackwell-capable CUDA 12.8 and cuDNN 9 runtime libraries. A full CUDA Toolkit, PyTorch, and TensorRT are not required. The SDK may reside at `~/.local/opt/intercept/onnxruntime`; keep CUDA/cuDNN libraries in an isolated user-local environment instead of changing system Python. See the [ONNX CUDA dependency documentation](https://onnxruntime.ai/docs/execution-providers/CUDA-ExecutionProvider.html).

```bash
source /opt/ros/jazzy/setup.bash
source ~/.local/opt/intercept/vision-env.bash
cd ~/intercept_ws
colcon build --symlink-install --packages-select uav_vision_dectect uav_vision_png \
  --parallel-workers 2 --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release \
  -DONNXRUNTIME_ROOT="$ONNXRUNTIME_ROOT"
source install/setup.bash
export ROS_DOMAIN_ID=0 PX4_PARAM_UXRCE_DDS_DOM_ID=0
```

Models are installed in the ROS package share directory, so startup is independent of the working directory. `--symlink-install` avoids duplicate model copies. Defaults are `GDUT_UAV.onnx` and the LightTrack init/update models; retain the accompanying `.onnx.data` file. This YOLO model outputs `[1,25200,6]`, and published bounding boxes use original 1920×1080 image coordinates, not the 640×640 network input.

Keep QGroundControl, dual SITL, and target control from the previous section running. In two terminals with the ROS/inference environment above loaded, run:

```bash
ros2 run uav_vision_dectect uav_vision_dectect
```

```bash
ros2 run uav_vision_png uav_vision_png
```

Detector parameters:

* `use_cuda` defaults to `true`. CUDA initialization/inference failures exit with an error rather than silently falling back to CPU; explicit CPU diagnostics use `-p use_cuda:=false`.
* `enable_tracking` and `show_debug_window` default to `true`; use `-p show_debug_window:=false` without a display.
* Candidates>=.4, global acquisition>.8, refresh5, NMS IoU.45 and support IoU.1 remain. New steady-clock `tracking_validation_hold=.35` and `tracking_recovery_timeout=2.0` require finite0<hold<timeout and are forwarded/recorded by the runner. Reliable/Depth1, raw, timing and controller gains are unchanged. Bounded local recovery is not lowered global acquisition.
* One result is published per processed image with its original timestamp. Lost targets use `-1` for all four bbox fields; stale boxes are not repeatedly stamped as fresh detections.

Guidance enters `SEARCHING` after reaching standby altitude and converging hover-thrust estimation, then enters `INTERCEPT` on a valid detection. `/vpng_data` exposes actual LOS and pixel errors; a running process or an existing topic alone does not prove detection or interception.

### 2.3 Unified Experiment Recording

The `uav_bc_recorder` package now provides `experiment_recorder`, replacing the old BC executable. Both PNG nodes publish `/experiment/telemetry` (`uav_common_msg/ExperimentTelemetry`) instead of writing CSV or accepting `csv_path`. Vision PNG also retains `/vpng_data`. The recorder is a subscriber only: it does not publish flight commands. Currently supported algorithms are `pure_png` and `vision_png`; only `scenario:=intercept` is implemented, and unsupported scenarios fail explicitly.

Rebuild `uav_common_msg`, both PNG packages, `uav_vision_dectect`, and `uav_bc_recorder`, then source the workspace in every terminal. Rebuild other `RectMsg` consumers too: its new `confidence` field changes the message contract, so old/new message binaries must not be mixed. Dependencies include `python3-yaml`, `python3-numpy`, `python3-opencv`, `ros-jazzy-ros2bag`, `ros-jazzy-rosbag2-transport`, and `ros-jazzy-rosbag2-storage-default-plugins`; regression tests additionally use `python3-pytest`.

| `profile` | Capture | Use |
| --- | --- | --- |
| `debug` (default) | CSV, metadata, summary; no raw-image subscription | Development |
| `compare` | Above + selected-topic rosbag; no raw-image subscription | Algorithm comparison |
| `archive` | Above + extended rosbag + `camera.mp4` | Results and archival |

Start the recorder before the chosen guidance node; never run both interceptor controllers together:

```bash
ros2 run uav_bc_recorder experiment_recorder --ros-args \
  -p out_dir:="$HOME/intercept_ws/experiment_data" \
  -p algorithm:=vision_png -p target_motion:=circle \
  -p scenario:=intercept -p profile:=debug \
  -p start_timeout:=60.0 -p max_duration:=120.0
```

`target_motion` labels the experiment; it must match the target node's actual `motion_mode` and does not command target motion. Metadata gets actual `speed_cmd`, `hit_radius`, planner/controller, and the complete parameter mapping from guidance telemetry, not independent recorder defaults. Vision PNG uses `kv`/`kz` for its vertical/horizontal angle updates; the unused `N_png` parameter was removed from Vision PNG. `config/params.yaml`, launch arguments, C++ fallbacks, and the phase-two runner now use the same defaults. Control parameters are read at startup; restart guidance when changing them.

Installed profiles can be loaded with subsequent `-p` overrides:

```bash
ros2 run uav_bc_recorder experiment_recorder --ros-args \
  --params-file "$(ros2 pkg prefix --share uav_bc_recorder)/config/archive.yaml" \
  -p algorithm:=vision_png -p target_motion:=circle
```

To orchestrate the complete simulation from the repository, load the ROS/vision environment and keep QGroundControl connected, then run:

```bash
bash uav_bc_recorder/launch_bc_collection.sh circle experiment_data 120.0 archive vision_png 60.0
# Arguments: motion, output root, interception duration, profile, algorithm, startup wait (optional, default 60.0).
# Set the fifth argument to pure_png to omit the detector.
```

The script starts recording first, then the repository's `run_swarm.sh`, target, optional detector, and guidance. Shutdown finalizes recording before stopping only its owned processes.

`duration` and `start_timeout` accept integer or decimal seconds, e.g. `60`/`180` or `60.0`/`180.0`. The script normalizes integer inputs to ROS DOUBLE literals; direct `ros2 run ... -p` commands still need floating-point values. If the recorder exits early, startup reports its status and original error instead of a misleading missing-PID `kill` error. Stop other simulations/controllers using the same PX4/vision topics before starting, keeping QGroundControl connected, to avoid consuming another run's data.

**GUI workflow for the temporary baseline (local desktop).** Stop other simulations/controllers sharing `/px4_1` and `/px4_2`; keep only one QGroundControl. The collection script starts simulation, target, detector and guidance itself, so do not launch those twice.

Build once after source changes (the current local version is already built and GUI-smoke-verified):

```bash
source /opt/ros/jazzy/setup.bash
source "$HOME/.local/opt/intercept/vision-env.bash"
cd "$HOME/intercept_ws"
colcon build --symlink-install \
  --packages-up-to uav_vision_dectect uav_target_sim uav_bc_recorder \
  --parallel-workers 2 --cmake-args -DCMAKE_BUILD_TYPE=Release
```

Terminal1, unless QGroundControl is already open:

```bash
unset QT_QPA_PLATFORM
"$HOME/.local/bin/qgroundcontrol"
```

Terminal2:

```bash
source /opt/ros/jazzy/setup.bash
source "$HOME/.local/opt/intercept/vision-env.bash"
source "$HOME/intercept_ws/install/setup.bash"
export ROS_DOMAIN_ID=0 PX4_PARAM_UXRCE_DDS_DOM_ID=0
export PX4_SELF_POSE="0,0" PX4_TARGET_POSE="20,0"
unset HEADLESS GZ_PARTITION QT_QPA_PLATFORM

bash "$HOME/Documents/GitHub/Autonomous-Intercept-Drone/uav_bc_recorder/launch_bc_collection.sh" \
  circle \
  "$HOME/intercept_ws/experiment_data/visionpng_baseline_gui" \
  60.0 \
  compare \
  vision_png \
  180.0
```

Expect Gazebo and the `Result` detection window, automatic takeoff/acquisition/interception, without manual arming. The duration is the interception recording limit, not total startup time; the final argument is startup wait. Ordinary GUI spawn/automatic takeover is not the fixed24m/16.8m distance/phase admission protocol. Compare saves CSV/meta/summary/bag, not camera video or default anomaly PNGs; change only the profile to `archive` for video. Each run creates a fresh timestamped directory. Hit/DONE/timeout finalizes automatically; to stop early press Ctrl+C once in terminal2 and wait for recorder/process cleanup before closing QGroundControl. Do not use global pkill.

Actual GUI smoke is under `~/intercept_ws/experiment_data/visionpng_probation_baseline_freeze/gui_20261016_150725/`: all three windows present, actual tracked target visible, live default parameters and native CameraInfo verified, launcher exit0 and finalized bag. This10s ordinary-entry smoke missed at1.076m; it proves GUI/default operation only and is not part of the six fixed-range flights.

**Split GUI workflow: target first, manual pursuit release.** This replaces, rather than runs alongside, the one-click script/phase2 runner. Reuse existing `require_experiment_start` and `/experiment/start`; no algorithm/gain/default-baseline change. Detection/tracking preview may run before pursuit authorization. Use the ROS/vision/workspace environment above in every ROS terminal, with `ROS_DOMAIN_ID=0 PX4_PARAM_UXRCE_DDS_DOM_ID=0` and `unset HEADLESS QT_QPA_PLATFORM`.

Keep one QGroundControl open. Run each of the following foreground commands in its own terminal, in order; wait for simulation readiness and target takeoff:

```bash
# Simulation terminal
export PX4_SELF_POSE="0,0" PX4_TARGET_POSE="20,0"
unset GZ_PARTITION
bash "$HOME/Documents/GitHub/Autonomous-Intercept-Drone/run_swarm.sh"
```

```bash
# Target terminal
ros2 run uav_target_sim uav_target_sim --ros-args -p motion_mode:=circle
```

```bash
# Preview terminal
ros2 run uav_vision_dectect uav_vision_dectect
```

```bash
# Interceptor terminal: automatic takeoff, then position hold until released
ros2 run uav_vision_png uav_vision_png --ros-args \
  -p require_experiment_start:=true
```

In a further terminal, optionally monitor `ros2 topic echo /experiment/telemetry --field state`; Ctrl+C ends only the monitor. Once SEARCHING, choose when to release:

```bash
ros2 topic pub --once /experiment/start std_msgs/msg/Bool "{data: true}"
```

Release still requires fresh valid vision and takeoff readiness. This gates first pursuit only: publishing false after interception is **not a pause or emergency stop**. Restart the scenario for a new round. Optional recording before release uses `experiment_recorder` with `algorithm:=vision_png target_motion:=circle scenario:=intercept profile:=compare start_timeout:=3600.0 max_duration:=60.0` and the desired `out_dir`. Its one-hour startup limit is configurable and does not limit controller hold. Split-start recorders do not own the other terminals; finalize recording before stopping the other nodes/world.

Actual GUI/CUDA verification `visionpng_probation_baseline_freeze/manual_release_20261016_152025/`: target airborne/moving first, then8.002s held SEARCHING with241 valid detections and fixed position-hold commands, followed by INTERCEPT after the manual CLI release. No code or baseline-tag changes; not hit-rate evidence. Full host-specific commands are also in the Chinese section.



Each startup immediately creates an independent directory beneath `out_dir`, e.g. `2026-10-15_14-30-00_vision_png_circle_001/`, containing `meta.yaml`, `timeseries.csv`, and `summary.csv`. Archive adds `camera.mp4`; compare/archive add `rosbag/` and `rosbag.log`. Atomic directory creation increments the suffix instead of overwriting runs. An optional string `experiment_id` is included in the directory name and metadata, with a separate collision-safe suffix.

**Lifecycle**

* CSV sampling defaults to 20 Hz (`sample_rate`), independently of image callbacks or zero velocity components.
* Start requires fresh `INTERCEPT` telemetry, valid self/target GPS and NED velocities. Vision PNG also requires a valid detection; purePNG requires neither camera nor detection. `topic_timeout` defaults to one second.
* `start_timeout` defaults to 60 seconds and must be finite and positive. It uses monotonic wall time from node initialization, independently of ROS simulation time. If interception prerequisites are not met by the deadline, finalize and exit with `reason: start_timeout` in metadata/summary and normally `status: no_intercept`; requested-resource finalization errors still produce `error`. Once recording starts, this deadline no longer applies; `max_duration` continues to bound interception.
* Continue through detection loss and stale topics; unavailable numeric fields become `NaN`. Detection/lost/reacquisition counters use callbacks and freshness transitions, not repeated CSV rows.
* A measured hit starts `post_hit_duration` (default 2 monotonic wall seconds) before finalization. Algorithm `DONE` also starts a tail without fabricating a hit. DONE and max duration do not truncate an active tail; external interruption reports `tail_truncated`. `intercept_time` remains first measured hit time, excluding the tail; minimum distance uses fresh GPS callbacks and sampled observations, including the tail. Repeated hit/DONE messages do not insert duplicate event rows; regular tail sampling and one final row remain. Misses still use `max_duration`; runs that never intercept report `no_intercept`.
* Bag/video capture begins at recorder startup, including takeoff/acquisition; duration starts only after interception prerequisites are met. Video uses a fixed receipt-time grid, holding the previous frame across missing slots; metadata records its ROS start/end and duplicate-frame count. Gaps beyond `max_duration` fail explicitly instead of causing unbounded backfill.
* Requested bag/video startup/finalization failures, including zero encodable images, produce error metadata and a nonzero exit rather than silently downgrading capture.

**Data semantics**

`timeseries.csv` contains self p/v, wxyz quaternion and body angular rates; ground-truth target p/v and distance; bbox/confidence/pixel error; LOS increments and PNG angles; reference velocity; thrust/torque; solver diagnostics; state; latency and events, plus per-source `*_age_s`. There is no OMMPC publisher yet, so its diagnostics remain `NaN`/empty. Unpublished thrust/torque likewise remain `NaN`, never inferred from velocity commands.

Positions use a common first-valid-self-GPS NED origin (meters, **z positive down**), not a subtraction of independent vehicle local origins. Ellipsoid altitude is preferred; MSL fallback is explicitly approximate and mixed altitude datums are rejected. Velocities are PX4 local NED in m/s; angular rates are body FRD in rad/s. Guidance angles are radians with existing `atan2(North, East)` azimuth; `dlos_*` are wrapped algorithm increments in rad/update, not angular rates. `cmd_thrust=-thrust.xyz[2]` and `cmd_tx/ty/tz` are normalized PX4 body-FRD setpoints, not SI force/torque; enable their DDS publications if needed.

Bounding boxes use original-image pixels. Confidence is the YOLO acquisition score or LightTrack tracking score, not a common calibrated probability; unavailable/lost scores are `NaN`. Detection freshness is receipt-based and does not guarantee source-image age. `vision_latency_ms` is computed only for a nonzero, nonfuture image stamp within `topic_timeout` of the recorder clock. Mixed Gazebo simulation/wall epochs produce `NaN`, not bogus latency. For meaningful latency, configure a common time domain; enable `use_sim_time` only when `/clock` is actually available and related nodes agree.

LightTrack confidence now belongs to the actual window/size-penalized candidate decoded into the published bbox. The global classification maximum is retained only as diagnostic `raw_score`, alongside `raw_argmax` and `selected`; it is not the bbox confidence.

P0 timing diagnostics do not change global `use_sim_time`. `run_swarm.sh` starts `simulation_clock_bridge`, preserving the simultaneous Gazebo Clock `sim` / `system` timestamps on `/simulation/clock_pairs`. Vision PNG interpolates this bounded history to map image capture time into the system epoch; bbox receipt is never treated as a zero-latency clock anchor. `image_time_domain` defaults to `gazebo`; explicitly select `ros` for cameras already stamped in ROS system time.

Gazebo SITL DDS timestamps can share the system epoch without being accurate sampling epochs: the smoothed DDS offset can lag changes in simulation/wall-clock rate. `px4_time_domain=gazebo_dds` (default) accepts DDS `TimesyncStatus` only, recovers integer `HRT_us = DDS_timestamp_us + estimated_offset_us`, then maps that native clock through Gazebo sim/system pairs for both attitude and GPS analysis. Status expires after 2 s, and its recovered epoch must agree with Agent `remote_timestamp` within `50 ms + RTT/2`. Missing/invalid mapping never falls back to the uncalibrated DDS epoch. Select `px4_time_domain=ros` explicitly for genuine ROS/system-domain inputs. The HRT/simulation-time relation follows PX4 `GZBridge::clockCallback()` and POSIX lockstep `hrt_absolute_time()`, not an offset fitted to receipt time.

`clock_offsets.csv` preserves both aircraft's original DDS synchronization events; geometry additionally retains the raw DDS source timestamp and offset used. Offset steps over 20 ms cut pose/waiting histories. Smaller cross-topic boundary ambiguity remains possible; a shared epoch does not prove zero synchronization error. Compare bags include both `timesync_status` topics.

The recorder also writes event-driven `observations.csv` and `geometry.csv`, including pre-interception events. Observations contain original/mapped image time, controller bbox receipt, LOS computation, first velocity-setpoint use, attitude sample times, source (0=unknown, 1=YOLO, 2=LightTrack), and LOS. Geometry contains each aircraft's GPS source/receipt timestamps and common-origin NED position. Unavailable mappings and never-consumed control times remain `NaN`. Compare bags include clock pairs, observation events, and odometry. Existing sampled `timeseries.csv` and `vision_latency_ms` semantics are unchanged.

`attitude_sync=true` is the production path: a fixed 512-sample history accepts finite, NED odometry quaternions in the compatible epoch and uses normalized shortest-arc SLERP at the mapped image epoch. Brackets must be at most 0.1 s apart; extrapolation is forbidden. Clock jumps/pauses and estimator resets cut the history. Missing mapping/brackets and duplicate/out-of-order images produce explicit rejection events without refreshing observation age or silently falling back. `attitude_sync=false` retains receipt-pose behavior only as an A/B control; `truth_los` is unaffected. P0 aligns `LOS(t_image)` but does not predict target motion to now.

Attitude packets can arrive after the bbox. The synchronized path keeps at most 16 observations and waits in image order for the right-hand pose sample, for at most `min(0.1 s, freshness_timeout)`. Expired observations and history gaps are rejected. Waiting never refreshes bbox receipt time or observation age; `image_to_control_ms` includes the wait and must be compared separately with receipt-pose latency.

`meta.yaml` records identifiers, actual parameters, git revision/dirty status/source path, recorder configuration, time/frame/unit provenance, GPS origin, and resource outcomes. `experiment_conditions.requested` stores `/experiment/conditions`; `.target` stores `/target/experiment_state`, without overwriting canonical metadata. The one-time `initial_conditions` snapshot captures actual positions/velocities, distance/horizontal distance/height difference (target below self positive), target speed/direction, self yaw, commanded and measured target-circle phase, and source ages; unavailable values are null. First-hit time, actual post-hit duration, and truncation are separate. `summary.csv` has one row per run with hit/time/distance, counts, latency aggregates, and resource errors.

Compare bags include `/experiment/telemetry`, `/vpng_data`, `/camera_detect_result`, both vehicles' GPS/local position, and self trajectory setpoint. Archive additionally records self odometry, thrust/torque setpoints, and `/clock`, covering the recommended eight topics. Raw images are not duplicated in bags; MP4 is separate. Installed `bag_qos.yaml` accepts PX4 BEST_EFFORT publishers. Unpublished topics do not produce messages; metadata reports actual counts.

Existing `assets/vpng_intercept_stats_*.csv` and plotting scripts remain historical-format artifacts, not consumers of the new unified schema.


### 2.4 Fixed-Condition Phase-Two Validation

Rebuild target, both PNG packages, and the recorder; load the workspace and vision environment, keep QGroundControl connected, and do not start other simulation/controllers. From the repository root:

```bash
ros2 run uav_bc_recorder phase2_runner --repo-root "$PWD" \
  --groups near far --trials 3 --duration 60.0
```

Each trial starts fresh real dual SITL, actual CUDA detection, target/guidance nodes, and compare recording. The supervisor never injects detections or PX4 velocity setpoints. Once the target has satisfied continuous hover stability, READY is latched; the existing `Float64` future epoch is then accepted despite a transient threshold excursion and `motion_start_ros_time` remains the simple acceptance receipt. Takeover uses a fixed motion age/phase and explicit external release.

Defaults: circle radius 5 m, initial phase 0, signed angular velocity +0.5 rad/s; release is scheduled one orbit plus commanded phase 4.4 rad after motion starts. Requested near/far distances are approximately 16.8/24 m, with target 0.3 m below self. Commanded versus measured phase are distinct metadata. Readiness, distance, height, detection, or release-time violations stop and preserve an invalid trial instead of silently counting it. Simulation spawn poses use `PX4_SELF_POSE` / `PX4_TARGET_POSE`.

The `phase2_fixed_circle_v2` entry protocol captures each vehicle's disarmed ground local z before starting controllers. It waits at most 30 seconds for approximately 2 seconds with a range no greater than 0.05 m, then uses the median as `self_ground_z`/`target_ground_z`. Defaults are `self_target_z = self_ground_z - 5.3` and `target_target_z = target_ground_z - 5.0`. Controllers receive these compensated targets; the runner checks self readiness against the same `self_target_z`. `--self-altitude` now means negative NED displacement above the vehicle's own ground datum, not absolute local z. All four values and capture ranges enter protocol/recorder experiment conditions. Relative height is still accepted using GPS geometry, never by subtracting independently referenced local z values.

Detector readiness requires the last valid receipt age to be `<0.15 s` and at least three valid detections within the last `<0.3 s`. Invalid messages neither refresh valid time nor clear the window; old timestamps expire naturally.

Diagnostic A/B modes do not count as formal hit-rate trials:

```bash
# SITL GPS geometry LOS drives the unchanged Vision PNG law.
ros2 run uav_bc_recorder phase2_runner --repo-root "$PWD" \
  --vision-mode truth_los --groups near far --trials 1

# Run every frame through YOLO, without LightTrack.
ros2 run uav_bc_recorder phase2_runner --repo-root "$PWD" \
  --vision-mode yolo_only --groups near far --trials 1
```

`truth_los` bypasses detector readiness and is explicitly marked `diagnostic_truth_los`. `yolo_only` is also diagnostic and bypasses the combined-mode entry window without changing YOLO confidence or fabricating boxes. `--diagnostic-bypass-detector-entry` provides the same explicit bypass for combined frontend diagnosis; it records a diagnostic mode and permits up to 0.8 s entry delay. Formal combined trials retain the three-valid-in-300-ms gate and 0.3 s entry-delay limit.

For the P0 single-variable comparison, keep Combined and all other protocol settings fixed and select `--attitude-sync receipt` for the receipt-pose baseline or `--attitude-sync image` (default) for capture-time SLERP. The selection is recorded in the protocol and analysis results; do not also change range, speed, gains, or detector gates.


For entry-only smoke verification, without an interception success-rate experiment:

```bash
ros2 run uav_bc_recorder phase2_runner --repo-root "$PWD" \
  --out-root "$HOME/intercept_ws/experiment_data/phase2_entry_smoke" \
  --groups near --trials 3 --smoke
```

Smoke holds the target at READY, waits for self readiness, GPS height within the original `0.3±0.18 m` tolerance, and detector-window readiness, then releases and records only 2 seconds after entry. Metadata marks `mode: entry_smoke`; these runs are not formal circle phase/range or hit-rate trials. Normal phase/range/height gates are unchanged. Failed attempts are retained and stop the batch, with no automatic retries.

Height-reference caveat: the former Drone1-only `EKF2_HGT_REF=2` selected a range sensor that these models do not provide. Both vehicles now use the barometric configuration and PX4 IMU initialization patch described in section 2.1. Ground-datum compensation does not guarantee absence of subsequent estimator drift, so GPS height acceptance remains required; the stricter stationary validation also checks Gazebo ground truth.


Sessions live beneath `~/intercept_ws/experiment_data/phase2/`, preserving protocol/result JSON, process logs, and independent recorder directories. Analyze sessions with:

```bash
ros2 run uav_bc_recorder phase2_analysis \
  "$HOME/intercept_ws/experiment_data/phase2"
```

Outputs are `analysis/metrics.csv`, `results.json`, and diagnostic window CSVs from 2 seconds before to 3 seconds after the first closest approach. After at least 1 m closure and elapsed time of at least 1 second, the first approach ends when distance rises 1 m above its running minimum for at least 0.2 seconds. No percentage-of-initial-range gate may skip an early distant turnaway in favor of a later closer pass. A first approach is distinct from a first close scrape. Windows include separation/height, bbox/detection, LOS/sampled LOS rate, yaw/yaw rate, commands, and controller state. Angular rates wrap pi crossings and do not bridge detection gaps; sampled metrics are not ground truth/contact evidence.

With P0 sidecars, analysis additionally writes `*_observations_aligned.csv` and an error-versus-distance plot when matplotlib is available. Each aircraft's GPS source timeline is interpolated independently at image and first-control-use epochs, with a maximum 0.15 s gap and no extrapolation. `los_error_image_*`, `los_error_control_*`, and the paired `receive_pose_error_image_*` baseline remain distinct. Latency statistics are event-weighted, not weighted by repeated timer rows. First-approach metrics end at the first closest point and include 0.8–2, 2–5, and 5–17 m bins, median/P95 absolute errors, and geometry coverage; uncovered values stay `NaN`/null. GPS geometry remains analysis-only.

P1 also writes `*_source_metrics.csv` split by YOLO/LightTrack, first approach, and distance bin, plus `*_los_timeline.png` showing same-image-time visual/GPS LOS from 2 s before to 0.5 s after the first closest point with source markers. Aligned rows add bbox centers, `time_since_yolo_s`, `is_yolo_refresh`, and wrapped `error_step_*_rad`; step errors subtract actual truth motion and never bridge rejected events or gaps over 0.15 s. Refresh means a consecutive LightTrack→YOLO transition, not reacquisition after a gap. High confidence does not imply low LOS error, and source scores are not a shared calibrated probability. Distance plots show absolute errors; timelines retain signed LOS to expose crossings, lag, and relocalization jumps.

**P0/P1 experiment status (controller `91fd754`).** Native-clock-calibrated near-range trials retained Combined, 16.8 m requested entry, 5 m/s, 0.8 m GPS hit threshold, fixed circle, all gains, and entry gates. Receipt-pose control hit 0/3; image-time control hit 2/3. A separate valid full native-clock verification missed at 0.982 m, so all observed valid image-time flights are 2/4, not evidence that repeatability is solved. One additional image-mode attempt failed the recent-valid detector entry gate and remains invalid, not a miss. All six matched formal flights had 100% detection-valid samples and 0% TRACK_LOST.

Median image-to-bbox latency was approximately 37–56 ms. Paired attitude correction alone had small, inconsistent median error gains; terminal errors of tens of degrees remained, including high-score LightTrack observations far from the last YOLO update. Sparse near-range YOLO samples do not establish a statistically reliable source ranking. Continue P2/P3 rather than claiming KF already repairs tracker drift or rewriting PNG/PX4. Detailed per-flight distances/errors, raw-data locations, and fixed executable hashes are recorded in [the Chinese experiment table](README.zh-CN.md).

**P2 pixel KF / delay prediction.** `--pixel-estimator raw` retains P0; `--pixel-estimator kf --attitude-sync image` updates a fixed-size `[u,v,du,dv]` constant-velocity filter once at each capture epoch and predicts to the current guidance calculation. P2 uses identical measurement noise for both sources (4 px) and acceleration noise 1000 px/s², with a 0.3 s forecast limit. Current attitude uses at most 50 ms body-FRD gyro propagation; stale/unmatched predictions are explicitly rejected. PNG/PX4/gains/entry conditions remain unchanged and raw stays the default.

Raw image `los_v/z` and actual `control_los_v/z` are separate, with explicit estimate time, predicted pixels/velocities, and pixel/attitude horizons. Analysis compares raw-held and estimated LOS against the same control-time truth and preserves negative prediction gains. The first full direct-camera CV trial (`p2_kf_validation/2026-10-16_00-53-40/`) missed at 9.949 m and never reached the 5 m window; paired first-approach median gains were −0.927° vertical and −0.319° azimuth. This is a negative result, not a reason to enable KF by default. Isolate ego-rotation-induced pixel motion before source-weighted P3 fusion.

Current `kf` removes ego rotation into a fixed virtual-camera plane before CV estimation, then reprojects to the current camera, including angular-motion pixel velocity. `reference_time_ros` identifies the anchor. The ideal stationary-world/rotating-camera replay exposed 4.724° mean / 7.550° maximum spurious LOS error in direct-camera CV; fixed-plane prediction passes rotation invariance, velocity-sign, and actual-node regressions (93 Python/node cases plus two C++ behavior tests). The first valid corrected circular flight missed at 0.944 m; a separate detector-entry-window attempt remained invalid. That flight rebuilt the filter reference 13 times over 76 first-approach observations, motivating a separate short-COAST estimator continuity check without changing PNG recovery.

Short COAST now preserves the estimator velocity/reference, while acquisition, long loss, clock/pose resets and blind gaps remain explicit reset boundaries; PNG's COAST recovery initialization is unchanged. The actual-node regression first reproduced zero velocity after a short pause, then passed with the complete 93-case suite. A full flight (`p2_coast_validation/2026-10-16_01-22-49/`) retained one reference over 99 first-approach observations but still missed at 1.574 m, with negative near-range paired median gains (−0.680° vertical, −1.176° azimuth). Estimator continuity does not repair biased observations; repeat this fixed version before source-weighted P3.

**LightTrack candidate consistency.** The old float row/column reconstruction changed 132/324 winning indices on the 18×18 map (e.g. 21→20). Selection now retains the exact integer candidate across classification, penalty and bbox regression, without an unused pscore allocation or new confidence/loss thresholds. Returning the chosen candidate's classification score matches the [official implementation](https://github.com/researchmm/LightTrack/blob/39c426f48ee674795cdf0e00301a0b4ad0785d2a/lib/tracker/lighttrack.py). Its double size blend is also present in the official reference, so window/learning-rate/size formulas were intentionally left unchanged. The complete candidate regression and real CUDA circular validation passed as an execution check, but that first flight missed at 2.145 m; it does not prove a performance gain.

**Large-image cadence.** With the same 1920×1080, 6,220,800-byte frames and Keep Last 1, a no-inference C++ receiver measured 9.82 Hz / 794 ms maximum gap with Best Effort versus 30.18 Hz / 54.5 ms with Reliable; Gazebo source windows were approximately 30 Hz. Actual CUDA bbox message output (including no-target invalid results) rose from 10.77 Hz to 30.08–30.23 Hz. A bbox-only observer added no Image reader; mean/maximum image-to-bbox age stayed around 53/74 ms rather than accumulating old frames. These are cadence measurements, not detection-accuracy results.

`image_reliability` defaults to `reliable` for this Gazebo publisher, with Depth 1 unchanged. Runner flag `--image-reliability reliable|best_effort` records the choice. Cameras offering Best Effort only require explicit `-p image_reliability:=best_effort`; no silent fallback or stale-box republishing occurs. Pixels, resolution, models, FOV, network sysctls, gains and entry gates were unchanged. The full flight in `camera_reliable_validation/2026-10-16_02-38-23/` used 1112 observations at 30.16 Hz, with 75.0 ms maximum interval and no sampled COAST/TRACK_LOST, but its first pass still missed at 2.749 m before a later hit at 36.875 s. Cadence is repaired, not first-pass interception accuracy. Probe results/source snapshots are archived in `camera_cadence_diagnostic/qos_probe.json` under the experiment-data root.

**Opt-in anomaly snapshots.** Add `--capture-anomalies` to `phase2_runner` (keep `--pixel-estimator raw`) to save under each trial's `anomaly_frames/`; standalone nodes accept `diagnostic_capture_dir`, empty/off by default with no cache/writer. A valid bbox-center jump exceeding 20% of the image diagonal triggers two preceding processed frames, the trigger frame and two following frames, provided adjacent processed frames have increasing image stamps, ≤150 ms gap and equal dimensions. This only triggers capture: it never rejects detections or changes guidance. PNGs retain native, unannotated pixels. `frames.csv` stores exact sec/ns, source and selected box; `candidates.csv` stores every post-threshold/post-NMS YOLO candidate, unclipped/clipped boxes and selection; `event.csv` records trigger/context completeness. One asynchronous writer, two waiting clips and a 16-event process limit bound resources; queue/write failures are explicit. Normal shutdown flushes partial context. These are processed frames, not guaranteed consecutive camera frames.

The actual CUDA Combined/image/raw/Reliable flight `bbox_anomaly_diagnostic/2026-10-16_08-32-48/` produced 16 complete events / 80 verified native PNGs. Its first pass missed at 1.169 m, with no hit within 60 s. In `event_frame_1463_75_472000000/frame_0.png`, the real target remains clearly visible centrally, but the **only YOLO candidate is the interceptor's own visible propeller** at `(1860,263,59,44)`, score 0.500. That frame was approximately 6.415 s into interception at 2.808 m GPS distance and 51.3 ms image-to-bbox age. It immediately replaced the tracker template; subsequent edge boxes scored 0.947/0.963. A later event also captured left-edge propeller misdetection. Review overlays are separate in `capture_review/`; originals remain untouched. First-approach image-to-bbox median/P95 was 56.4/70.4 ms; later median rose to 161.7 ms, with cause not isolated, so timing neutrality is not claimed. This confirms missed-target/self-propeller detection followed by wrong-template tracking—not a KF fix or a proven success-rate improvement. No mask, association gate, camera relocation or controller change was added.

**Perception-policy comparison.** Upstream `6f2d53b` used post-processing `>=0.4`, independent acquisition strictly `>0.8`, and no YOLO during normal tracking. Local `b74d7c7` removed the second gate; `97a27ab` added refresh and removed the raw-argmax loss gate. These were strategy changes, not all deterministic bug fixes. Runner defaults now match the frozen detector baseline: `--confidence-threshold 0.4 --acquisition-threshold 0.8 --yolo-refresh-interval 5`. The historical low-gate comparison requires explicit `--acquisition-threshold 0.0`; no-refresh requires explicit `--yolo-refresh-interval 0`. Keep the independent acquisition gate separate from post-processing. Parameters are startup-only. Opt-in snapshots also preserve first acquisition context (`trigger_reason=first_acquisition`), with explicit missing prehistory. Actual CUDA inference verified strict equality rejection and acceptance one float ULP above a threshold.

**Three-policy flights (`2237ead`).** `~/intercept_ws/experiment_data/perception_policy_ab/` contains three valid fixed-circle trials per policy, no admission failures. First-pass minima were **0.773/0.692/2.026 m** for current acquisition>0/refresh5 (first hits 2/3, final hits 2/3), **0.578/1.984/0.390 m** for acquisition>0.8/refresh5 (2/3, 3/3), and **0.894/1.618/0.698 m** for acquisition>0.8/refresh0 (1/3, 2/3). Visual review confirmed current-policy first acquisition was the own propeller in all three runs; both high-gate policies first acquired the real UAV in all runs. The no-refresh policy initialized exactly once per run, but did not produce a higher first-hit rate in this small sample.

The current-policy third run switched to a 0.7916 propeller candidate at 6.392 s / 3.126 m before its first miss at 7.077 s / 2.026 m. Separately, high-gate refresh trial 2 captured **propeller 0.814677 versus actual target 0.800589 in the same image**; highest-score refresh replaced a tracker still on the true target. That event was at 18.921 s, **after** the 6.375 s first encounter, so it is not evidence for the cause of that first miss. High thresholds reduce wrong acquisition but are not an identity guarantee. Other late frames show high-score tracking of empty sky; the area-only loss rule remains unchanged. High-gate refresh ran YOLO on 85–90% of all processed frames including startup, because unsuccessful refresh retries next frame. All results, identity evidence and phase-qualified observations are in `analysis/policy_diagnosis.json` and `review/`. Sequential n=3 groups, GPS noise/entry variations and a GPS-only 0.8 m hit criterion limit conclusions. No default strategy, camera, mask, association or KF changes were deployed.

**Historical frozen VisionPNG baseline: `visionpng-baseline-high08-refresh5-raw` (tag preserved).** It records Combined/CUDA, candidates>=0.4, acquisition>0.8, refresh5 with unsuccessful-acquisition retries, the original camera view, raw/image-epoch attitude/native clocks, unchanged gains and area-only tracker validity. Current post-baseline safety changes below are not retroactively attributed to this tag.

Historical24m trials on `5807967` are in `~/intercept_ws/experiment_data/perception_policy_24m/analysis/policy24_diagnosis.json`: refresh5 first minima .308/.418/.214m with3/3 first hits; refresh0 had0/3 despite correct initial acquisition. Reproduce that behavior by checking out the frozen tag; current main adds the safety changes below and cannot inherit its hit rate. Small cohorts and GPS-only evaluation are not general safety/contact guarantees.

Freeze verification started the default detector and actual VisionPNG launch, read their live parameters, and exercised real CUDA images: reject a low-score propeller, acquire the UAV, run five tracker updates, then refresh with YOLO. Explicit low-gate diagnostics still work. A default-runner SITL entry smoke without perception overrides passed with 0.103s entry delay. This 2s smoke has no circle motion or distance gate and is **not included in either range's hit rate**. Evidence is under `~/intercept_ws/experiment_data/visionpng_baseline_freeze/`.

**Historical single-miss revocation (`a326ea1`).** Real-CUDA negative replay previously published all160 empty-scene frames as valid (64 near-fullscreen); immediate support revocation reduced this to five intervening frames and zero fullscreen boxes, but destroyed continuity after a single missed detection. The subsequent bounded version is described below.

The launcher deploys `x500_depth_external_view`:17 self visuals use flags1, only IMX214 RGB uses mask2; target/ground retain default flags and GUI still sees self. Expanded-SDF comparison verifies unchanged camera pose/FOV/resolution, collision, inertia and depth settings. The wrapper references upstream resources without copying meshes or editing shared models. This is simulator sensing policy, not a physical camera remedy; no screen-edge mask or KF was added.

Actual PX4/Gazebo appearance/removal smoke:15s without an external UAV produced455 invalid detections, SEARCHING only and maximum translation command0. A rendered static x500 then triggered INTERCEPT; removal led to zero translation command after0.724s. Images confirm no self parts and a visible external target. This is not a circular-target hit-rate trial. Existing .15/.6s freshness/coast semantics remain, so invalidation is not immediate braking; yaw search remains. Normal startup still takes off to standby before waiting.

The safety/continuity tradeoff is explicit:24m circular regression `regression_far/2026-10-16_12-57-43` lost support, later candidates stayed below0.8, and admission was rejected for lack of a recent valid window. Preserve this invalid trial, not a control miss; do not lower the gate or transfer historical3/3 claims. Evidence is in `~/intercept_ws/experiment_data/no_target_diagnostic/`; the old frozen tag is untouched.

The16.8m moving-circle regression `regression_near/2026-10-16_13-02-36` admitted successfully, missed the first pass at1.652m and finally hit at44.057s. Sampled TRACK_LOST fraction87.4% shows recovery exists but continuity is more conservative; performance equivalence is not claimed and no thresholds/gains were loosened.

The launcher creates a fresh GZ_PARTITION by default. An explicitly occupied partition is rejected before processes start, without killing its world: an old self-model would otherwise be an external visible object to the new camera. If a partition is already exported, `unset GZ_PARTITION` before rerunning the collection command; startup prints the actual partition. Other controllers sharing PX4/ROS topics must still be stopped normally.

**Bounded probation continuity.** Separate internal template retention from valid observation authority. YOLO support alone renews .35s publication permission; later frames become invalid while the predictor remains recoverable up to2s. Expired priors cannot be revived by a late low score and global acquisition still requires>.8. Probation retries each frame and healthy low-FPS tracking checks support early enough to avoid extending permission by frame count. The2s retention covers the approximately1.3s candidate gap in the previous failed24m record.

Actual CUDA native-image replay passed single/three-frame miss recovery from a0.800867 seed using a same-position0.793742 candidate, finite empty-scene publication, internal expiry, rejection of expired low-score acquisition and later high-score acquisition. Clock-injected tests cover deadline boundaries, gaps, backward time, current-support priority and bounded anchor recovery. Real PX4/Gazebo smoke passed15s SEARCHING/zero translation with no UAV, acquisition of a rendered static target at12m, zero command after0.740s on removal, and explicit internal expiry. An18m static view scoring only .77–.79 was correctly refused by the first-acquisition gate and retained as a negative record. These are mechanism/safety checks, not hit-rate or baseline qualification. No self-visual, partition, QoS, timing or PNG fixes were reverted. Evidence is under `~/intercept_ws/experiment_data/probation_validation/`.

**Ego-rotation-compensated bounded prior.** The `fb5e6db` 24m pilot retained all three attempts: two admitted trials produced19.994m/80.3% TRACK_LOST and0.680m/0% respectively; the third never globally acquired and is entry-invalid, not a control miss. Failed frame1375 still had a0.776 YOLO candidate, but the last confirmed pixel box had zero overlap. Native-time pose reconstruction gives14.684deg of self rotation over198ms; projecting that box gives IoU≈.357 at the unchanged .1 gate. The exact preceding support frame1369 is inferred from cadence and support_age, not directly recorded YOLO data: this is bounded offline counterfactual evidence, not a recovered flight.

Only the unexpired probation fallback now rotates its last confirmed box using real `/camera/camera_info` K and capture-epoch self attitude. Existing current-tracker support retains priority and only actual YOLO evidence renews permission. Reuse ClockHistory/DdsClock/AttitudeHistory with independently consumed state callbacks and post-inference capture-time SLERP; no receipt-pose substitution, target-motion model, GPS translation, ROI expansion, lowered global gate or control change. Cross-generation/calibration changes and behind/outside projections invalidate the prior; missing pose/calibration retains the conservative old pixel fallback. PNG changes only install/export existing headers and Eigen.

The new CUDA replay passed all10 assertions. Real PX4/Gazebo `live_20261016_143920` passed15.003s empty-scene SEARCHING/zero translation, static12m acquisition, zero translation0.950s after removal and internal expiry. Actual CameraInfo is1920×1080, fx≈1397.2236/fy≈1397.2235; capture_rotation and native attitude brackets were observed. Geometry/clock regressions passed. These are not moving-circle results; the pilot remains under `formal_far/`, offline evidence under `rotation_evidence/`, and final six-flight results on one fixed source follow.

**Final six flights on fixed source `4160914`.** All three24m and three16.8m entries were valid, with identical recorded source and clean trees. Keep fixed-circle admission, raw/image/Reliable, candidate.4/global>.8, refresh5, IoU.1, hold.35s/recovery2s and all original control gains; no entry bypass.

| Requested range | Trial | First minimum m | First 0.8m hit s | Full-record TRACK_LOST |
|---|---:|---:|---:|---:|
| 24m | 1 | .788 | 8.263 | 25.9% |
| 24m | 2 | .682 | 8.180 | 23.8% |
| 24m | 3 | .401 | 7.995 | 0.0% |
| 16.8m | 1 | .612 | 6.767 | 24.0% |
| 16.8m | 2 | .990 | No hit in60s | 79.2% |
| 16.8m | 3 | .696 | 5.571 | 27.2% |

Hits use the existing GPS0.8m evaluation radius, not physical-contact proof or GPS guidance. Fractions include the existing2s post-hit tail; misses run60s. Pooled sample-weighted TRACK_LOST is16.7% far/67.7% near versus the old single near reference87.4%; this is neither paired nor statistical-significance evidence. Each far flight had exactly one global acquisition and an actual accepted rotation-compensated local recovery at .781096/.771598/.769793, IoU.3627/.3189/.4100, not a lowered first-acquisition gate.

Near trial2 recovered after .626s/.488s support gaps using .586592/.460884 candidates, then had sustained empty candidates from about4.8s, invalid publication at4.966s and internal expiry at6.600s. First-min horizontal/absolute vertical error was .660/.738m. A309×139 tracker box received weak support from a130×56 YOLO box at only.1171 IoU: overlap does not establish precise localization. **[INFERENCE]** Near-pass appearance/scale, tracker drift or leaving the FOV remain plausible; no native images cover this exact failure segment, so empty detections cannot establish a visible-target YOLO miss.

**Current decision at the user's request: temporarily adopt this version as the `vision_png` default, tagged `visionpng-baseline-high08-refresh5-probation-raw`; preserve the historical tag and do not restore unsafe behavior on main.** This is a provisional diagnostic baseline, not solved performance. Safety and24m continuity have end-to-end evidence, but one near60s miss/long loss remains. Next capture the near-approach image window on the same configuration to distinguish detection absence from tracker-coordinate drift, before KF/fusion or any further threshold change. Evidence: `motion_prior_far/2026-10-16_14-41-54`, `motion_prior_near/2026-10-16_14-46-49`, `final_validation_summary.json` and per-group `analysis/continuity_summary.json` under the probation root; all pilot negatives remain. Temporary diagnostic sources are archived in `validation_provenance.json`.


**Non-circular GUI diagnosis: two sinusoidal and two random-walk trials (`b8511b7`, implementation still `4160914`).** Evidence is under `~/intercept_ws/experiment_data/visionpng_motion_gui/2026-10-16_15-39-11/`. All four had visible Gazebo/Result, actual interception entry, completed capture and finalized bags. Ordinary automatic entry, compare profile and0,0/20,0 spawns; no baseline threshold, budget, estimator or control changes.

| Target / trial | Entry range m | First minimum m | Overall minimum m | 0.8m criterion s | Full TRACK_LOST | Pre-hit TRACK_LOST |
|---|---:|---:|---:|---:|---:|---:|
| sinusoidal 1 | 23.130 | .493 | .493 | 19.030 | 62.1% | 68.6% |
| sinusoidal 2 | 23.514 | 8.315 | .509 | 34.879 | 81.2% | 85.8% |
| random_walk 1 | 15.725 | .286 | .219 | 3.384 | 0.0% | 0.0% |
| random_walk 2 | 15.607 | .648 | .632 | 4.031 | 45.5% | 18.5% |

Sinusoidal first hits1/2 and eventual hits2/2 conceal long losses and three global acquisitions per trial. Random-walk first/eventual hits2/2 had nearer entry ranges and different motion directions/phases, so two trials cannot rank difficulty fairly. Full fractions include the2s post-hit tail. Recorded states nearest the criterion were COAST in three trials and TRACK_LOST in random_walk2: GPS-distance success is not continuous visual stability or physical-contact proof.

Do not pool these ordinary-entry results with controlled24m/16.8m,0.3m-height-difference trials; targets were still climbing at initial takeover. Original random_device seeding is unchanged and actual paths are retained. Keep the temporary baseline/tag unchanged; do not lower gates or add KF from these results. Next control entry range/takeoff stability before diagnosing FOV exit, detector misses or tracker drift from near-pass images. Preserve `results.json`, `metrics.csv`, `actual_xy_paths.png`, per-trial CSV/bags/logs and GUI screenshots; no video was recorded.


Only if the cause remains ambiguous, add matched far Pure PNG controls:

```bash
ros2 run uav_bc_recorder phase2_runner --repo-root "$PWD" \
  --algorithm pure_png --groups far --trials 2
```

Pure controls retain real detection for comparable process load and match speed/hit radius to 5 m/s and 0.8 m, rather than comparing unmatched original 8 m/s Pure flights.

### 3. Running the Interception Task (Step Sequence)
Please run each node in a separate terminal in the following order:


0. **Start the Simulation Environment**:
   Run from the repository root, retaining its complete model-wrapper assets:

   ```bash
   # Deploy the world only; do not copy run_swarm.sh alone
   cp assets/gazebo_world/grass_world.sdf \
     "${PX4_ROOT:-$HOME/PX4-Autopilot}/Tools/simulation/gz/worlds/"
   ```

   > Run the repository launcher each time; it links the independent wrapper into PX4's model directory without replacing upstream resources:
   ```bash
   bash ./run_swarm.sh
   ```

   This script automatically starts the Gazebo simulation server, two drones (px4_1 interceptor + px4_2 target), the MicroXRCEAgent, and the camera topic bridge.


1. **Start the Target Simulator**:
   ```bash
   # Default circular motion
   ros2 run uav_target_sim uav_target_sim

   # Or select the motion mode and movement range via parameters
   ros2 run uav_target_sim uav_target_sim --ros-args -p motion_mode:=sinusoidal -p max_range:=10.0

   # You can also use the launch file
   ros2 launch uav_target_sim target_sim.launch.py motion_mode:=random_walk
   ```

   **Supported target motion modes**:

   | Parameter | Motion Mode | Kinematic Formula | Characteristics |
   | :--- | :--- | :--- | :--- |
   | `circle` | Uniform circular | R=5m, w=0.5rad/s | Default mode, uniform circular motion with a radius of 5m |
   | `sinusoidal` | Sinusoidal maneuver | a=0.5*sin(0.5t) m/s<sup>2</sup> | Forward uniform 1m/s + lateral sinusoidal acceleration, periodic evasion |
   | `random_walk` | Random walk | v(t+dt) = v(t) + N(0, 0.2) | Random velocity perturbation, speed limited to 3m/s, irregular escape |

   > The `max_range` parameter (default 10m) controls the target movement range to prevent the target from flying out of the interceptor's field of view.

2. **Start Vision-Guided Interception**:
   ```bash
   # Recommended to run directly (uses the default parameters in the .hpp file)
   ros2 run uav_vision_png uav_vision_png

   # Or use the launch file to load parameters from config/params.yaml
   ros2 launch uav_vision_png vision_png.launch.py
   ```

   > **Parameter Configuration Notes**: The parameter priority of the `uav_vision_png` node is:
   >   **launch CLI > params.yaml > .hpp member initial values**
   >
   > - When started with `ros2 run`: parameters take the default values in `.hpp`; **`params.yaml` is not loaded automatically**, so you need to pass parameters manually via `--ros-args -p param_name:=value`
   > - When started with `ros2 launch`: the launch file automatically loads `config/params.yaml`, and the YAML values override the `.hpp` defaults
   > - During debugging, use `ros2 param get /uav_vision_png <param_name>` to inspect the currently effective value
   > - See `uav_vision_png/config/params.yaml` for tunable parameters, including PNG gain, FOV compensation, takeoff altitude, etc.

2-Alt. **Start RL Learned Guidance Interception (Alternative)** 🆕:
   `uav_rl_guidance` is a GRU policy guidance node trained with reinforcement learning (BC + PPO), used to replace the traditional PNG algorithm.
   It maintains the same topic interfaces, state machine, and CSV statistics format as `uav_vision_png`, so it can be used as a seamless replacement.
   It uses ONNX Runtime for inference, with zero PyTorch dependencies and a pure C++ implementation.

   ```bash
   # Method 1: Launch via the launch file (automatically loads config/params.yaml)
   ros2 launch uav_rl_guidance rl_guidance.launch.py

   # Method 2: Run directly (uses the default parameters in the code)
   ros2 run uav_rl_guidance uav_rl_guidance

   # A/B baseline mode: use the built-in PNG throughout, without loading the policy model (equivalent to uav_vision_png)
   ros2 launch uav_rl_guidance rl_guidance.launch.py fallback_png:=true

   # Bench debugging: skip takeoff and enter SEARCHING directly
   ros2 launch uav_rl_guidance rl_guidance.launch.py bench_test:=true
   ```

   > **Architecture Notes**: The RL policy outputs velocity commands during the INTERCEPT phase; if the policy behaves abnormally (watchdog), it automatically falls back to the built-in PNG controller.
   > The policy model (ONNX `.onnx`) is stored under `uav_rl_guidance/models/`.
   > The training code is available at [AeroIntercept](https://github.com/Eaglewzw/AeroIntercept). To update the model, run `python3 uav_rl_guidance/src/export_onnx.py`.

3. **Start Vision Detection**:
   ```bash
   ros2 run uav_vision_dectect uav_vision_dectect
   ```
---

---

> **Project Maintainer's Note**: If you need more details about the algorithm derivation (PNG), please refer to the `include/` header files in each package or contact the development team.
