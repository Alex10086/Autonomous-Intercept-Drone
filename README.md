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
2. `bash ~/PX4-Autopilot/run_swarm.sh`. First place the repository world at PX4's `Tools/simulation/gz/worlds/grass_world.sdf`; the launcher can be linked into the PX4 root. Set `PX4_ROOT` to override the default source directory, or `HEADLESS=1` to omit the Gazebo GUI.
3. `ros2 run uav_target_sim uav_target_sim`.
4. `ros2 run uav_png_intercept uav_png_intercept`. For recording, start `experiment_recorder` from section 2.3 first with `algorithm:=pure_png`.

`run_swarm.sh` stops only its own process groups and no longer globally kills other simulations or deletes historical logs. Stopping control nodes triggers normal Offboard-loss protection; verification covers takeoff and communication, not a completed interception.


### 2.2 Jazzy Visual Detection and Guidance

Verification passed with default CUDA model detection/tracking, actual bounding-box publication, converged hover-thrust estimation, and `uav_vision_png` transitioning from takeoff through `SEARCHING` to `INTERCEPT`. ONNX profiles recorded CUDA convolution operations for all three models, rather than merely listing a Provider or injecting fake boxes. A completed interception and RL remain unverified.

The vision chain is `/camera/image` → `uav_vision_dectect` → `/camera_detect_result` → `uav_vision_png`. The detector publishes no flight-control commands; do not simultaneously run `uav_png_intercept` or another interceptor controller.

Vision PNG updates LOS/PNG and yaw-PD history once per new valid detection callback, using an attitude snapshot taken at receipt. New messages with equal bbox coordinates are still processed; coordinates are not a frame identity. The 40 Hz PNG recomputation timer is removed. The 200 Hz controller retains the existing velocity formula, gains, and vertical compensation; visual history does not advance between new detections.

Loss limits are now seconds: `freshness_timeout: 0.15` and `track_lost_timeout: 0.6`, both finite with `0 < freshness_timeout < track_lost_timeout`. No-target results neither replace the cached observation nor refresh its clock, and do not immediately enter COAST. COAST starts only when monotonic age since the last valid detection receipt exceeds 0.15 s; TRACK_LOST starts at 0.6 s, including when the detector is silent. Valid detections returning within the freshness window retain LOS/PNG and yaw-PD history without reinitialization. COAST holds the last velocity/yaw-rate command and still evaluates hits; TRACK_LOST retains braking/search behavior. Recovery from these loss states reinitializes PNG and yaw difference references. The old frame-count parameters `coast_thresh`/`lost_thresh` are removed; migrate old configurations.

Receipt-time snapshots are not capture-time attitude interpolation, and receipt age does not measure actual inference latency. `uav_vision_png/test/test_visual_freshness.py` launches the real node on isolated topics to cover stale boxes with changing attitude, new frames with equal coordinates, brief no-target gaps preserving history, silent/no-target expiry, recovery, and hits during COAST. These node regressions alone do not establish interception success rates.

The combined frontend periodically reruns YOLO every five tracker frames and reinitializes LightTrack on a valid detection. LightTrack now uses its adaptive `penalty × score × lr` value in both size-update terms; the former raw flattened-argmax jump gate was removed because it rejected high-score, in-frame motion. `[VISION_DIAG]` logs distinguish YOLO no-candidate/acquisition and LightTrack out-of-image loss, with per-process counters.


Use ONNX Runtime C++ GPU SDK 1.23.2 with Blackwell-capable CUDA 12.8 and cuDNN 9 runtime libraries. A full CUDA Toolkit, PyTorch, and TensorRT are not required. The SDK may reside at `~/.local/opt/intercept/onnxruntime`; keep CUDA/cuDNN libraries in an isolated user-local environment instead of changing system Python. See the [ONNX CUDA dependency documentation](https://onnxruntime.ai/docs/execution-providers/CUDA-ExecutionProvider.html).

```bash
source /opt/ros/jazzy/setup.bash
source ~/.local/opt/intercept/vision-env.bash
cd ~/intercept_ws
colcon build --symlink-install --packages-select uav_vision_dectect uav_vision_png \
  --parallel-workers 2 --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release \
  -DONNXRUNTIME_ROOT="$ONNXRUNTIME_ROOT"
source install/setup.bash
export ROS_DOMAIN_ID=173
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
* Override default installed models with `yolo_model_path`, `tracker_init_model_path`, and `tracker_update_model_path`. `confidence_threshold` defaults to `0.4`; the same threshold is used for YOLO post-processing and tracker acquisition instead of discarding otherwise valid detections at a second, stricter gate. IoU remains `0.45`.
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

Only if the cause remains ambiguous, add matched far Pure PNG controls:

```bash
ros2 run uav_bc_recorder phase2_runner --repo-root "$PWD" \
  --algorithm pure_png --groups far --trials 2
```

Pure controls retain real detection for comparable process load and match speed/hit radius to 5 m/s and 0.8 m, rather than comparing unmatched original 8 m/s Pure flights.

### 3. Running the Interception Task (Step Sequence)
Please run each node in a separate terminal in the following order:


0. **Start the Simulation Environment**:
   Before starting, copy the simulation resources from this repository to the PX4-Autopilot directory:

   ```bash
   # Step 1: Copy the run_swarm.sh script from this repository to the PX4-Autopilot root directory (if not already copied)
   cp ros2_ws/src/run_swarm.sh ~/PX4-Autopilot/

   # Step 2: Copy the custom world file to the PX4 Gazebo worlds directory
   cp ros2_ws/src/assets/gazebo_world/grass_world.sdf ~/PX4-Autopilot/Tools/simulation/gz/worlds/
   ```

   > The above steps only need to be performed once. After that, to start the simulation, simply enter the PX4-Autopilot directory and run the startup script:
   ```bash
   cd ~/PX4-Autopilot
   ./run_swarm.sh
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
