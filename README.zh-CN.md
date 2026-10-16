# Autonomous Intercept Drone
### 基于视觉伺服的自主拦截无人机系统 (Autonomous Intercept Drone with Image-based Visual Servo)

<div align="center">

[**English**](README.md) | **中文**

</div>

<div align="center">

[![Bilibili](https://img.shields.io/badge/Bilibili-观看视频演示-ff69b4?style=for-the-badge&logo=bilibili)](https://www.bilibili.com/video/BV1M8QVYHE39/?spm_id_from=333.1387.homepage.video_card.click)
[![PX4](https://img.shields.io/badge/PX4-Autopilot-blue.svg)](https://px4.io/)
[![ROS2](https://img.shields.io/badge/ROS2-Humble%2FFoxy-green.svg)](https://docs.ros.org/en/humble/)
[![TensorRT](https://img.shields.io/badge/TensorRT-Accelerated-76B900.svg)](https://developer.nvidia.com/tensorrt)
[![YOLO11](https://img.shields.io/badge/YOLO-v11-yellow.svg)](https://github.com/ultralytics/ultralytics)

</div>

> 本项目是一个面向自主无人机拦截任务的实验验证平台，集成了视觉感知、目标检测、比例导引（PNG）算法及飞行控制方法。该项目旨在为无人机自主拦截、制导算法研究及视觉伺服控制提供一套完整的仿真参考方案，适用于学习与学术研究。
---

## 项目简介

该系统在 ROS 2 环境下开发，结合 PX4 固件与 Gazebo 仿真器，实现了从目标发现到仿真拦截的全链路闭环。主要功能包括：

*   **视觉目标检测**：利用训练的 YOLO 模型和跟踪模型（LightTrack）实时识别并跟踪视场内的无人机目标。
*   **比例导引制导**：基于比例导引律（Proportional Navigation Guidance, PNG）生成拦截轨迹，实现对动态目标的精确前置拦截。
*   **仿真验证体系**：基于 PX4 软件在环（SITL）搭建，支持在 Gazebo 环境中进行算法的离线验证与参数调优。
*   **数据分析工具**：内置运动学与视觉指标记录脚本，用于定量评估制导律精度与视觉追踪稳定性。

---

## 仿真演示

### 1. 动态拦截全过程
<div align="center">
  <img src="assets/output.gif" alt="拦截动态演示" width="80%">
  <p><i>（通过 PNG 算法预判目标航迹，实现精准的物理碰撞拦截）</i></p>
  <p><b>📺 <a href="https://www.bilibili.com/video/BV1sKBVBgEZ9/">点击此处前往 Bilibili 观看完整高清演示视频</a></b></p>
</div>

### 2. 小目标无人机检测框架
<div align="center">
  <img src="assets/小目标无人机检测框架.png" alt="小目标检测框架" width="80%">
  <p><i>（针对无人机“点目标”优化的视觉处理流程）</i></p>
</div>

---

## 系统架构与逻辑

系统采用 ROS 2 分层架构，确保了极高的灵活性与实时性：

<div align="center">
  <img src="assets/框架.png" alt="框架图" width="80%">
</div>

| 模块名称 | 功能描述 | 关键包 | 状态/备注 |
| :--- | :--- | :--- | :--- |
| **感知层** | 目标检测、像素误差计算、视线角提取 | `uav_vision_dectect`, `uav_vision_png` | **当前主方案** |
| **制导层** | PNG 导引率计算、前置量补、轨迹生成 | `uav_png_intercept` | **纯PNG方案 (不依赖视觉)** |
| **学习制导** | 基于强化学习的 GRU 策略制导，取代传统 PNG | `uav_rl_guidance` | 🆕 **RL 方案 (替代 vision_png)** |
| **控制层** | PX4 Offboard 接口、速度/姿态闭环控制 | `uav_vehicle_controller`, `px4_ros_com` | 核心控制底座 |
| **仿真层** | 目标机动态模拟、多运动模式支持、拦截环境生成 | `uav_target_sim` | 仿真支持 |
| **早期方案** | 基于图像的视觉伺服控制方案 | `uav_ibvs_control` | **已放弃 (代码仅供保留参考)** |

---

## 拦截性能量化分析

以下展示了系统在典型拦截任务中的表现，可作为算法优化的参考基准：

### 运动学性能
<div align="center">
  <table>
    <tr>
      <td><img src="assets/plots_output/1_3D_Trajectory.png" width="300px"><br><b>3D 拦截轨迹图</b></td>
      <td><img src="assets/plots_output/2_Relative_Distance.png" width="300px"><br><b>相对距离收敛曲线</b></td>
      <td><img src="assets/plots_output/3_Velocity.png" width="300px"><br><b>拦截机速度矢量</b></td>
    </tr>
    <tr>
      <td>展示了 3D 空间内的截击前置量</td>
      <td>验证拦截距离最终收敛至 &lt;0.2m</td>
      <td>反映了制导律对推力的高效利用</td>
    </tr>
  </table>
</div>

### 视觉追踪表现
<div align="center">
  <table>
    <tr>
      <td><img src="assets/plots_output/4_LOS_PNG_Angles.png" width="300px"><br><b>视线角 (LOS) 演变</b></td>
      <td><img src="assets/plots_output/5_Pixel_Error.png" width="300px"><br><b>视觉中心追踪误差</b></td>
      <td><img src="assets/plots_output/6_2D_Top_View.png" width="300px"><br><b>2D 俯视截击路径</b></td>
    </tr>
    <tr>
      <td>导引规律的收敛稳定性分析</td>
      <td>检测算法在动态过程中的稳健性表现</td>
      <td>典型前置量补效果展示</td>
    </tr>
  </table>
</div>

---

## 快速上手

### 1. 环境依赖
*   **ROS 2 Version**: Humble；Ubuntu 24.04 上的 Jazzy 原生仿真与视觉制导配置见 2.1、2.2 节。RL 推理需单独配置。
*   **Gazebo**:Gazebo Harmonic（gz-sim 8.12.0）
*   **PX4 Firmware**: v1.16 (**Micro-XRCE-DDS-Agent**版本推荐v2.4.x)
*   **Dependencies**: OpenCV, onnx, colcon

### 2. 编译项目
```bash
# 进入工作空间并编译
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
```

### 2.1 Ubuntu 24.04 / Jazzy 原生环境

验证组合：Jazzy、Gazebo Harmonic 8.11.0、PX4 v1.16.0、Micro-XRCE-DDS-Agent v2.4.3。已验证双机 GPS/位置/状态消息、1920×1080 RGB 相机桥接，以及 `uav_target_sim` + `uav_png_intercept` 的双机 Offboard 起飞。视觉制导使用下节的检测节点；RL 和完整拦截命中不在本次验证范围内。

版本与配置要求：

* PX4 固件固定为 `v1.16.0`（`6ea3539157ca358c70a515878b77077af7d4611d`），源码目录默认为 `~/PX4-Autopilot`。
* 构建此版本前，应用仓库中的 `patches/px4-v1.16.0-imu-first-sample.patch`：IMU 首包只初始化积分器，不能把开机以来的时间当成采样间隔。对尚未应用补丁的干净 PX4 源码执行：

  ```bash
  git -C ~/PX4-Autopilot apply \
    ~/intercept_ws/src/Autonomous-Intercept-Drone/patches/px4-v1.16.0-imu-first-sample.patch
  ```

  然后按下方命令重新构建 PX4；只重新编译 ROS 包不会更新飞控。已经应用过的源码不要重复应用。
* `px4_msgs` 使用 `release/1.16` 的 `392e831c1f659429ca83902e66820d7094591410`，其 226 个消息定义与上述固件一致。执行 `git submodule update --init px4_msgs px4_ros_com` 补齐子模块。
* 该固件的飞行状态话题为 `/px4_N/fmu/out/vehicle_status_v1`，不是无版本后缀的旧话题。
* `uav_vision_png` 需要额外发布悬停推力估计。在 PX4 的 `src/modules/uxrce_dds_client/dds_topics.yaml` 的 `publications` 中添加以下条目后重新编译：

  ```yaml
  - topic: /fmu/out/hover_thrust_estimate
    type: px4_msgs::msg::HoverThrustEstimate
  ```

* DDS Agent 可安装至 `~/.local/opt/intercept/agent`，并链接命令到 `~/.local/bin/MicroXRCEAgent`。Jazzy 的 Fast-DDS 2.14 / Fast-CDR 2.2 可复用；Agent v2.4.3 使用上游 spdlog v1.9.2，不能直接替换为本机 spdlog 1.12 / fmt 9。
* 保持真实的 [QGroundControl 地面站](https://docs.qgroundcontrol.com/Stable_V5.1/en/qgc-user-guide/getting_started/download_and_install.html)连接。未连接地面站时 PX4 可能拒绝解锁；不要用关闭预检或强制解锁代替缺失的连接。
* `run_swarm.sh` 为两机统一设置 `EKF2_HGT_REF=0`（气压）、`EKF2_GPS_CTRL=5`（GNSS 水平位置和三维速度，不融合 GNSS 高度）、`EKF2_GPS_DELAY=0`（匹配 Gazebo 的当前时刻采样）及 `EKF2_BARO_NOISE=0.3 m`。两种模型均有气压计，没有向下测距仪；气压高度相对真值的实测噪声标准差约为 0.15 m，不能沿用 3.5 m 的测量噪声配置。此设置只针对本项目的 SITL 环境，不是实机推荐参数；未移除仿真传感器噪声，也未禁用安全预检。
* 静止高度验收须先等待两机地面高度及垂向速度估计收敛，再分别采集各自的地面 local z。`z_valid` 或两机 local z 相减都不能证明真实高差；验收使用按模型名提取的 Gazebo world z。视觉链路尚未通过验证时，不恢复正式 Vision PNG 对比。
* 源码只需链接一次到 `~/intercept_ws/src/Autonomous-Intercept-Drone`；不要同时链接同一组依赖包，以免出现重复包。

使用独立 Python 环境构建 PX4，避免修改系统 Python。已有环境的构建命令：

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

不要用 `DONT_RUN=1 make ... gz_x500` 作为此版本的纯编译入口：该 Gazebo 目标仍会启动仿真。退出 Python 虚拟环境后构建 ROS 包：

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

这条构建路径不包含 `uav_vision_dectect`、`uav_rl_guidance` 或 `px4_ros_com` 的上游示例。

每个 ROS 终端先加载以下环境：

```bash
source /opt/ros/jazzy/setup.bash
source ~/intercept_ws/install/setup.bash
export PATH="$HOME/.local/bin:$PATH"
export ROS_DOMAIN_ID=173
export GZ_PARTITION=intercept_jazzy
```

分别在四个终端执行：

1. `~/.local/bin/qgroundcontrol`（本机用户级入口；保持地面站打开）。
2. `bash "$HOME/intercept_ws/src/Autonomous-Intercept-Drone/run_swarm.sh"`（按实际仓库路径调整）。先部署 `assets/gazebo_world/grass_world.sdf` 到PX4世界目录；必须运行保留完整assets的仓库脚本，不再复制或单独链接脚本到PX4根目录。启动器部署独立自机模型包装，不覆盖上游模型。`PX4_ROOT` 可覆盖PX4目录，`HEADLESS=1` 关闭GUI。
3. `ros2 run uav_target_sim uav_target_sim`。
4. `ros2 run uav_png_intercept uav_png_intercept`。需要记录时，先启动下一节的 `experiment_recorder`，设置 `algorithm:=pure_png`。

`run_swarm.sh` 仅停止自己创建的进程组，不再全局杀死其他仿真或删除历史日志。停止控制节点会触发正常的 Offboard 丢失保护；测试验证的是起飞和通信，不是完整拦截命中。


### 2.2 Jazzy 视觉检测与制导

历史冻结基线为 `visionpng-baseline-high08-refresh5-raw`，保留其24 m首次3/3、16.8 m首次2/3的小样本GPS评估。当前版本在其后加入本机RGB自见隔离和同帧YOLO支持复核，旧命中率不能直接当作新版本性能；不是物理碰撞或普遍安全保证。RL不在本次范围。

视觉输入链路为 `/camera/image` → `uav_vision_dectect` → `/camera_detect_result` → `uav_vision_png`。检测节点不发布飞控命令，不要同时运行 `uav_png_intercept` 或其他拦截机控制节点。

Vision PNG 只在新的有效检测观测上更新一次 LOS／PNG 和 yaw-PD 历史；默认按图像时间映射与四元数SLERP取得拍摄时刻姿态，不使用接收姿态替代。相同 bbox 坐标的新图像仍正常处理，不按坐标去重。200 Hz 控制循环保留现有速度公式、增益和垂向补偿；新帧之间不推进视觉历史。

失锁配置改为秒数：`freshness_timeout: 0.15`、`track_lost_timeout: 0.6`，要求有限且 `0 < freshness_timeout < track_lost_timeout`。无目标结果不更新缓存观测、不刷新计时，也不立即触发 COAST；仅在最后有效检测的单调接收年龄超过 0.15 s 时进入 COAST，达到 0.6 s 时进入 TRACK_LOST，完全没有消息时同样生效。新鲜窗口内恢复有效检测会延续 LOS／PNG 和 yaw-PD 历史，不重新初始化。COAST 保持最后速度／偏航率并继续命中评价，TRACK_LOST 保留原制动搜索策略；只有从这些失锁状态恢复时才重新初始化 PNG 和 yaw 差分参考，避免跨失锁差分。旧 `coast_thresh`／`lost_thresh` 帧数参数已删除，需迁移配置。

接收年龄不等于图像年龄或推理时延；实际时序以逐观测诊断和native时钟映射分析为准。`uav_vision_png/test/test_visual_freshness.py` 的真实节点回归覆盖过期、恢复、重复坐标新帧和姿态时间对齐等行为；节点回归本身不证明实际拦截命中率。

组合前端保持严格全局首次获取 `>0.8`，正常refresh5并要求同帧空间支持。一次没有YOLO支持不再立刻销毁Tracker：只由YOLO支持续期的 `tracking_validation_hold=0.35 s` 允许短漏检，过期即停止有效输出；内部模板最多保留 `tracking_recovery_timeout=2.0 s`，期间逐帧复核，相符0.4–0.8候选可恢复。优先当前预测；仅未过期probation允许最后确认YOLO框作有限锚点，必要时以低分同目标候选局部重建。无关高分不能抢占有支持的当前跟踪，Tracker自身高分不续期。内部过期先于恢复判断，下一帧重新全局获取仍须>0.8。`refresh=0`仅关闭按帧周期，不能绕过时间预算。


使用 ONNX Runtime C++ GPU SDK 1.23.2，以及支持 Blackwell 的 CUDA 12.8 和 cuDNN 9 运行库；不需要完整 CUDA Toolkit、PyTorch 或 TensorRT。SDK 可安装在 `~/.local/opt/intercept/onnxruntime`，CUDA/cuDNN 动态库可放在独立用户级环境，避免改动系统 Python。参见 [ONNX CUDA 依赖说明](https://onnxruntime.ai/docs/execution-providers/CUDA-ExecutionProvider.html)。

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

检测模型由包安装到 ROS share 目录，启动不依赖当前工作目录。`--symlink-install` 避免重复复制模型。默认模型为 `GDUT_UAV.onnx` 和 LightTrack 的 init/update；保留随附的 `.onnx.data` 文件。该 YOLO 模型输出为 `[1,25200,6]`，检测框坐标对应原始 1920×1080 图像，而不是 640×640 网络输入。

保持上一节的 QGroundControl、双机仿真和目标机控制运行，再在两个已加载上述 ROS/推理环境的终端分别执行：

```bash
ros2 run uav_vision_dectect uav_vision_dectect
```

```bash
ros2 run uav_vision_png uav_vision_png
```

检测节点参数：

* `use_cuda` 默认 `true`。CUDA 初始化或推理失败时退出并报告错误，不会悄悄回退 CPU；显式 CPU 诊断可传 `-p use_cuda:=false`。
* `enable_tracking` 默认 `true`；`show_debug_window` 默认 `true`。无显示环境传 `-p show_debug_window:=false`。
* 默认候选0.4、全局获取严格0.8、refresh5、NMS IoU0.45及 `tracking_validation_iou=0.1` 保持。新增 `tracking_validation_hold=0.35`、`tracking_recovery_timeout=2.0`，均用steady_clock秒，必须有限且 `0<hold<timeout`；runner同名CLI透传并写入协议/分析。图像Reliable/Depth1、raw、时间同步与控制参数不变。局部有界恢复不等于降低首次获取门槛，YOLO测量分数与Tracker分类分数仍不可当作共同校准概率。
* 每张实际处理图像发布一次结果并保留图像时间戳；丢失目标时 bbox 四个字段均为 `-1`，不重复发布旧框伪装成新检测。

视觉制导到达待机高度且悬停推力估计收敛后进入 `SEARCHING`，收到有效检测框后进入 `INTERCEPT`。`/vpng_data` 可观察真实 LOS 和像素误差；只有启动进程或看到话题存在，不等于识别或拦截成功。

### 2.3 统一实验记录

`uav_bc_recorder` 包现在提供 `experiment_recorder`，替代原来的同名 BC 可执行程序。purePNG 和 Vision PNG 不再直接创建 CSV，也不再接受 `csv_path`。两者发布 `/experiment/telemetry`（`uav_common_msg/ExperimentTelemetry`），Vision PNG 继续发布 `/vpng_data`；记录器只订阅，不参与制导或发控制指令。当前支持 `algorithm:=pure_png|vision_png` 和 `scenario:=intercept`，不支持的场景会明确报错。

更新消息后，重新构建 `uav_common_msg`、两个 PNG 包、`uav_vision_dectect` 和 `uav_bc_recorder`，并在每个终端重新加载工作空间。其他使用 `RectMsg` 的节点也应重新构建：该消息新增 `confidence`，不能混用旧版消息二进制。记录器依赖 `python3-yaml`、`python3-numpy`、`python3-opencv`、`ros-jazzy-ros2bag`、`ros-jazzy-rosbag2-transport` 和 `ros-jazzy-rosbag2-storage-default-plugins`；运行回归测试另需 `python3-pytest`。

| `profile` | 记录内容 | 适用场景 |
| --- | --- | --- |
| `debug`（默认） | CSV + meta/summary，不订阅原始图像 | 开发调试 |
| `compare` | 上述内容 + 关键 topic rosbag，不订阅原始图像 | 阶段性对比 |
| `archive` | 上述内容 + 扩展 rosbag + `camera.mp4` | 正式结果与留档 |

先启动记录器，再启动对应制导节点；不要同时运行两个拦截控制器。示例：

```bash
ros2 run uav_bc_recorder experiment_recorder --ros-args \
  -p out_dir:="$HOME/intercept_ws/experiment_data" \
  -p algorithm:=vision_png -p target_motion:=circle \
  -p scenario:=intercept -p profile:=debug \
  -p start_timeout:=60.0 -p max_duration:=120.0
```

`target_motion` 是实验标签，必须与目标机实际的 `motion_mode` 一致，不会由记录器改变目标运动。`speed_cmd`、`hit_radius` 等由制导遥测带入元数据，不在记录器里另设一套默认值；完整参数保存在 `parameters` 下。Vision PNG 的垂直／水平角度更新使用 `kv` / `kz`，未参与计算的 `N_png` 已从 Vision PNG 删除。`config/params.yaml`、launch 参数、C++ 回退值和 phase2 runner 现在使用同一组默认值。控制参数在节点启动时读取，改变配置应重启制导节点。

也可加载安装好的分档配置，之后的 `-p` 覆盖该配置：

```bash
ros2 run uav_bc_recorder experiment_recorder --ros-args \
  --params-file "$(ros2 pkg prefix --share uav_bc_recorder)/config/archive.yaml" \
  -p algorithm:=vision_png -p target_motion:=circle
```

若从仓库直接编排整套实验，可在已经加载 ROS/视觉运行库、保持 QGroundControl 连接的终端执行：

```bash
bash uav_bc_recorder/launch_bc_collection.sh circle experiment_data 120.0 archive vision_png 60.0
# 参数依次为：目标运动、输出根目录、拦截最长秒数、profile、algorithm、启动等待秒数（可省略，默认 60.0）。
# purePNG 将第 5 个参数设为 pure_png；不启动检测节点。
```

脚本先启动记录器，再运行仓库的 `run_swarm.sh`、目标机、可选检测和制导；退出时先完成记录文件，再停止自己启动的进程，不会全局 kill 其他实验。

`duration` 和 `start_timeout` 均接受整数或小数秒，例如 `60` / `180` 或 `60.0` / `180.0`；脚本会将纯整数归一为ROS DOUBLE字面量，避免 `INTEGER, expecting DOUBLE`。直接使用 `ros2 run ... -p` 时仍应写浮点值。记录器若提前退出，脚本报告其退出码和原始错误后停止，不再以 `kill: 没有那个进程` 掩盖原因。启动前请关闭其它使用同一组PX4/视觉话题的仿真与控制节点，只保留QGroundControl，避免读到上一套运行的数据。

每次启动都立即在 `out_dir` 下原子创建独立目录，不覆盖历史数据；同秒启动自动递增编号。可用字符串参数 `experiment_id` 指定标识，目录仍带防碰撞后缀。

```text
experiment_data/
└── 2026-10-15_14-30-00_vision_png_circle_001/
    ├── meta.yaml
    ├── timeseries.csv
    ├── summary.csv
    ├── camera.mp4       # 仅 archive
    ├── rosbag/          # compare / archive
    └── rosbag.log       # compare / archive 的录制进程日志
```

**触发与结束**

* CSV 默认 20 Hz，`sample_rate` 可配置；与相机回调解耦，合法零速度分量不会阻止录制。
* 开始需要新鲜的 `INTERCEPT` 遥测、自身/目标有效 GPS 及 NED 速度；Vision PNG 额外需要有效检测框，purePNG 不需要相机或检测。`topic_timeout` 默认 1 秒。
* `start_timeout` 默认 60 秒，必须为正的有限数；从节点初始化起按单调墙钟计时，不依赖 ROS 仿真时间。期限内未满足开始条件时收尾并退出，`meta.yaml` / `summary.csv` 的 `reason` 为 `start_timeout`，正常情况下状态为 `no_intercept`；已请求资源的收尾错误仍标记 `error`。开始录制后不再受此期限影响，继续使用 `max_duration`。
* 一旦开始，丢目标或话题过期仍继续采样；缺失/过期数值写 `NaN`。`detected`、`lost`、`reacquired` 区分检测状态与转换事件；计数来自回调/过期转换，不是重复累计 CSV 行。
* 真实距离首次达到 `hit_radius` 后继续录制 `post_hit_duration`（默认 2 秒，单调墙钟），再收尾；算法先进入 `DONE` 也保留该窗口，但没有实测命中时不伪造 `hit`。`DONE` 和 `max_duration` 不会截断已开始的撞后窗口，外部中断会标记 `tail_truncated`。`intercept_time` 始终是首次实测命中耗时，不包含撞后窗口；`min_distance` 是新鲜 GPS 回调和定频样本的观测最小值，包含撞后窗口。撞后仍按定频采样，重复命中／DONE 消息不重复插行，最终收尾补一行。未命中时仍按 `max_duration` 超时，未进入拦截时输出 `no_intercept`。
* bag/视频从记录器启动开始采集，包含起飞和捕获前阶段；`max_duration` 从满足拦截开始条件起算。视频采用固定接收时间网格，缺帧时保持上一帧，元数据记录起止 ROS 时间及补帧数；过长缺口明确报错，不无限补帧。
* 请求的 bag/视频无法启动或收尾、视频没有收到可编码图像等均标记 `error` 并以非零状态退出，不会悄悄降档；目录保留用于排查。

**字段与数据口径**

`timeseries.csv` 包含状态 p/v、wxyz 四元数及机体系角速度、目标真值 p/v 和距离、bbox/置信度/像素误差、LOS/dLOS/PNG 期望角、参考速度、推力/力矩、求解器诊断、算法状态、视觉延迟和事件；每路数据另有 `*_age_s`。目前没有 OMMPC 发布者，其求解器字段为 `NaN`/空状态；没有收到的推力/力矩也保持 `NaN`，不会从速度指令推算。

* 自身和目标位置统一为首个有效自身 GPS 原点的 NED，单位 m，**z 向下为正**；不直接相减两架飞机各自原点的 local position。优先使用椭球高，MSL 回退会明确标注为近似，拒绝混合高程基准。
* 速度为 PX4 local NED（m/s），角速度为 body FRD（rad/s）。`los_*`、`png_*` 单位 rad，方位角沿用算法的 `atan2(North, East)`；`dlos_*` 是算法使用的包裹后角增量（rad/update），不是角速度。
* `cmd_thrust=-thrust.xyz[2]`，`cmd_tx/ty/tz` 来自 PX4 body-FRD 归一化 torque setpoint，不是 N 或 N·m。需要这些话题时，须确保 PX4 的 DDS 配置实际发布它们。
* bbox 对应原始图像像素；`confidence` 是 YOLO 捕获分数或 LightTrack 实际选中候选的分类分数，未提供/丢失为 `NaN`，两种分数不应视为同一个校准概率。LightTrack 的全局分类最大值仅作为 `raw_score` 调试量，不能代替所发布 bbox 的分数。
* 时间为记录器 ROS 时钟；检测新鲜度按接收时间计算，不等于源图像年龄保证。`vision_latency_ms` 仅在图像 stamp 非零、非未来且与接收时钟相差不超过 `topic_timeout` 时计算；Gazebo 仿真 stamp 与墙钟混用时为 `NaN`，不会相减出巨大“延迟”。需要有效延迟时应统一时间域；只有在 `/clock` 确实可用且相关节点配置一致时才启用 `use_sim_time`。

P0 时间诊断不改变全局 `use_sim_time`。`run_swarm.sh` 同时启动 `simulation_clock_bridge`，把 Gazebo 同一条 Clock 中的 `sim` / `system` 时间发布到 `/simulation/clock_pairs`；Vision PNG 用有界历史插值将原始图像时间映射到系统时间，禁止用 bbox 接收时间反推时钟偏移。`image_time_domain` 默认为 `gazebo`；真实相机若已使用 ROS 系统时间，显式设为 `ros`。

Gazebo SITL 的 DDS 时间戳即使已落在系统 epoch，也不能直接视为精确采样时间：DDS 平滑偏移在仿真/墙钟速率变化时可能滞后。`px4_time_domain=gazebo_dds`（默认）只接受 DDS `TimesyncStatus`，以整数运算恢复 `HRT_us = DDS_timestamp_us + estimated_offset_us`，再用原生 Gazebo sim/system 时钟对映射；姿态与 GPS 分析都遵循此口径。同步状态最长保留 2 s，恢复后的状态时刻还须与 Agent `remote_timestamp` 满足 `50 ms + RTT/2` 残差界；缺失/失败不回退到未经校准的 DDS epoch。真正使用 ROS 系统域的输入需显式设 `px4_time_domain=ros`。原始 HRT 跟随 Gazebo 的依据是 PX4 `GZBridge::clockCallback()` 与 POSIX lockstep `hrt_absolute_time()`，不是按接收时刻拟合出的偏移。

`clock_offsets.csv` 保留两机原始 DDS 同步状态；`geometry.csv` 另保留原 DDS 时间与当时使用的估计偏移，以便审计状态更新/话题传输边界。超过 20 ms 的偏移跳变会切断姿态/等待历史；更小的主题边界歧义仍可能存在，不能把“epoch 匹配”当作零同步误差。compare bag 包含两机 `timesync_status`。

记录器另存逐回调的 `observations.csv` 和 `geometry.csv`，不把定频快照当作独立视觉样本。前者记录原始/映射后的图像时间、控制节点 bbox 接收时间、LOS 计算时间、首次使用该观测的速度指令时间、姿态样本时间、来源（0=未知、1=YOLO、2=LightTrack）及 LOS；后者记录两机各次 GPS 的源时间、接收时间与统一 NED 位置。无效映射/从未被控制使用的时刻为 `NaN`。compare bag 也保留时钟对、逐观测诊断和 odometry；原有 `timeseries.csv` / `vision_latency_ms` 口径不变。

`attitude_sync=true` 为正式路径：仅缓存 NED、有限且时间域匹配的 odometry quaternion，按 `timestamp_sample` 在图像映射时刻做归一化 SLERP（最短旋转弧）。姿态历史固定容量 512，插值两侧间隔不得超过 0.1 s，禁止外推；时钟回跳/暂停、估计器 reset 会切断历史。无时钟映射、无姿态包围或重复/乱序图像均发布明确拒绝状态，不刷新有效观测年龄、不静默回退到接收姿态。`attitude_sync=false` 仅用于保留原接收姿态路径的单变量 A/B；`truth_los` 路径不受该开关影响。此修复只对齐 `LOS(t_image)`，不预测目标到当前时刻。

姿态消息可能比 bbox 更晚到达。同步路径最多缓存 16 个观测，按图像顺序等待后侧姿态样本，最长等待 `min(0.1 s, freshness_timeout)`；超时或历史缺口仍拒绝。等待不会刷新 bbox 接收时间或状态机观测年龄，`image_to_control_ms` 包含这段等待；应与接收姿态基线的延迟分别比较。

`meta.yaml` 记录实验标识、实际算法参数、git commit、`git_dirty`、源码位置、记录器配置、坐标/时间/单位口径、GPS 原点、资源状态及 bag 消息数。`experiment_conditions.requested` 保存 `/experiment/conditions` 的实验协议，`.target` 保存 `/target/experiment_state` 的目标运动配置与状态；两者不覆盖标准元数据。`initial_conditions` 在首次满足拦截采样条件时只记录一次实际 p/v、距离、水平距离、高度差（目标低于自身为正）、目标速度/方向、自身偏航、目标指令相位和实测圆周相位以及各源年龄，缺失值为 null。首次命中时间、撞后实际时长和是否被截断单独保存；`summary.csv` 每次实验一行，包含命中、拦截耗时、最近距离、计数、延迟及资源错误。

compare 的 bag 包含 `/experiment/telemetry`、`/vpng_data`、`/camera_detect_result`、双机 GPS/local position 和自身 trajectory setpoint。archive 额外包含自身 odometry、thrust/torque setpoint 和 `/clock`，覆盖上述建议的八个 topic；原始图像不重复写入 bag，单独保存 MP4。安装的 `bag_qos.yaml` 使用兼容 PX4 BEST_EFFORT 的订阅；未发布话题不会凭空产生消息，实际消息数见元数据。

旧 `assets/vpng_intercept_stats_*.csv` 和绘图脚本保留为历史数据格式，不直接读取新的统一表。


### 2.4 二阶段固定条件验证

重建 `uav_target_sim`、`uav_vision_png`、`uav_png_intercept`、`uav_bc_recorder` 并加载对应工作空间及视觉运行库。保持 QGroundControl 连接，不要另开仿真或控制节点。从仓库根目录运行：

```bash
ros2 run uav_bc_recorder phase2_runner --repo-root "$PWD" \
  --groups near far --trials 3 --duration 60.0
```

每轮新建真实双机 SITL，并启动真实 CUDA 检测、目标控制、制导和 compare 记录；不会注入虚假检测或代发 PX4 速度指令。目标连续满足悬停稳定条件后锁存 READY；随后仍通过原有 `Float64` 未来 epoch 启动，短暂越过稳定阈值不会撤销 READY，`motion_start_ros_time` 继续作为简单接收回执。接管仍使用固定运动年龄、circle 相位和外部放行条件。

默认 circle 半径 5 m、初始相位 0、角速度 +0.5 rad/s；目标启动后一整圈再到指令相位 4.4 rad 时安排接管。默认请求距离为 near 约 16.8 m、far 约 24 m，高度差约 0.3 m。指令相位与目标实际运动相位分别保存；实际距离、高度、检测或入场时序不满足协议会中止并保留无效试验，不冒充有效样本。`run_swarm.sh` 可通过 `PX4_SELF_POSE` / `PX4_TARGET_POSE` 配置仿真初始位置。

入口协议 `phase2_fixed_circle_v2` 在任何控制节点启动前，分别采集两机未解锁时的地面 local z；要求连续约 2 秒波动不超过 0.05 m，最多等待 30 秒，再取中位数作为 `self_ground_z`／`target_ground_z`。默认生成 `self_target_z = self_ground_z - 5.3`、`target_target_z = target_ground_z - 5.0`，将补偿后的值传给控制节点，并让 runner 的 self ready 检查使用同一个 `self_target_z`。`--self-altitude` 现在表示相对本机地面的负 NED 位移，不是绝对 local z。四个高度值及采样范围会进入 `protocol.json` 和 recorder 的实验条件；真实相对高度仍由 GPS geometry 验收，不相减两机 local z。

检测就绪条件为最后有效检测接收年龄 `<0.15 s`，且最近 `<0.3 s` 内至少收到 3 次有效检测。invalid 不刷新有效时间、不清空窗口；旧时间戳会自然过期。

以下 A/B 仅用于定位，不计入正式命中率：

```bash
# 用 SITL GPS geometry LOS 驱动不变的 Vision PNG 控制律。
ros2 run uav_bc_recorder phase2_runner --repo-root "$PWD" \
  --vision-mode truth_los --groups near far --trials 1

# 每帧只运行 YOLO，不进入 LightTrack。
ros2 run uav_bc_recorder phase2_runner --repo-root "$PWD" \
  --vision-mode yolo_only --groups near far --trials 1
```

`truth_los` 跳过检测入口门槛并标记为 `diagnostic_truth_los`；`yolo_only` 同样是诊断模式，不改变 YOLO 置信度、不伪造框。组合前端诊断可显式追加 `--diagnostic-bypass-detector-entry`，记录为诊断模式并允许最多 0.8 s 接管延迟。正式 combined 试验仍保留 300 ms 内三次有效检测和 0.3 s 接管延迟门槛。

P0 姿态对照保持 Combined 与其余协议不变，分别运行 `--attitude-sync receipt`（接收姿态基线）和 `--attitude-sync image`（默认，图像时刻 SLERP）。该选项同时写入实验协议及分析结果；不要同时改变入场距离、速度、增益或检测门槛。


只验证入口、暂不做命中率实验时：

```bash
ros2 run uav_bc_recorder phase2_runner --repo-root "$PWD" \
  --out-root "$HOME/intercept_ws/experiment_data/phase2_entry_smoke" \
  --groups near --trials 3 --smoke
```

`--smoke` 让目标保持起飞后的 READY 位置，等待 self ready、GPS 高差在原有 `0.3±0.18 m` 范围内及检测窗口就绪，然后放行，仅记录接管后的 2 秒。元数据标记 `mode: entry_smoke`，不执行正式 circle 相位／距离对照，不计入命中率。正式模式的相位、距离及高度门槛不变；失败尝试保留并中止本批次，不自动补跑。

高度参考说明：此前仅 Drone1 设置的 `EKF2_HGT_REF=2` 选择了模型未提供的测距传感器；现已改为两机统一的气压高度配置，并配合 2.1 节的 PX4 IMU 初始化补丁。地面基准补偿仍不能保证估计器后续不会漂移，GPS 高差门槛保持不变；更严格的静止高度验收还需核对 Gazebo 真值。


结果默认保存在 `~/intercept_ws/experiment_data/phase2/时间戳/`。每轮保留 `protocol.json`、`result.json`、各进程日志及独立 recorder 目录；重复组别/次数均可由参数指定。对该会话目录运行：

```bash
ros2 run uav_bc_recorder phase2_analysis \
  "$HOME/intercept_ws/experiment_data/phase2"
```

分析输出 `analysis/metrics.csv`、`results.json` 和各次首次最近点前 2 秒至后 3 秒的窗口 CSV。首次接近定义为入场后至少缩短 1 m、经过至少 1 秒，再出现“距离较运行最小值回升 1 m 并持续至少 0.2 秒”；不要求先缩短到某个比例，避免跳过远处的首次折返、把后续更好的接近当成首次成功。首次接近不等于首次近距离擦过，两者必须区分。窗口包含距离/水平距离/高度误差、bbox/检测、LOS/采样 LOS rate、yaw/yaw rate、速度指令和控制状态；视觉角速度跨 ±π 正确包裹，检测缺口不桥接。它们是定频采样指标，不替代同步真值或 Gazebo 接触事件。

有 P0 sidecar 时，分析另输出 `*_observations_aligned.csv` 和可用时的误差—距离图：分别在图像映射时刻、首次控制使用时刻，对两机 GPS 源时间独立插值（最大间隔 0.15 s，禁止外推）。`los_error_image_*` 与 `los_error_control_*` 不可混为一个指标；`receive_pose_error_image_*` 是同一观测用接收姿态计算的配对基线。延迟按观测事件统计，不按 20 Hz 重复采样行加权。首次接近仅取入场至首次最近点，报告 0.8–2、2–5、5–17 m 分箱、绝对误差中位数/P95及真值覆盖率；没有覆盖时保留 `NaN`/null，不用陈旧 GPS 填补。GPS geometry 仅供分析，不进入正式视觉制导。

P1 另输出 `*_source_metrics.csv`（按 YOLO/LightTrack、首次接近及距离分箱统计）与 `*_los_timeline.png`（首次最近点前 2 s 至后 0.5 s，同图像时刻的视觉/GPS LOS，并标记观测来源）。对齐 CSV 增加 bbox 中心、`time_since_yolo_s`、`is_yolo_refresh` 和包裹后的 `error_step_*_rad`；后者从视觉跳变中扣除了同时间真值变化，不跨拒绝帧或超过 0.15 s 的间隔。只有连续 LightTrack→YOLO 观测才标记 refresh，不能把间隔后的重新捕获当成 refresh。高置信度不等于低 LOS 误差，两个来源的分数也不作为共同校准概率。误差—距离图绘制绝对误差；时间曲线保留带符号 LOS，以区分跨边、滞后和重定位跳变。

**P0/P1 固定圆周实测（控制器 `91fd754`）**

全部保持 Combined、请求入场 16.8 m、5 m/s、命中阈值 0.8 m、circle 半径 5 m / 角速度 +0.5 rad/s、原 PNG/FOV 增益与入口门槛。下表仅含原生时钟恢复后的有效正式试验；命中是既有 GPS 距离判据，不是额外的 Gazebo 接触证明。P95 为首次接近 0.8–5 m 的同图像时刻绝对 LOS 误差。

| 姿态路径 | 轮次 | 首次最近距离 m | 命中 | 图像→bbox 中位 ms | 图像→控制中位 ms | 垂直误差 P95 ° | 方位误差 P95 ° |
|---|---:|---:|---|---:|---:|---:|---:|
| receipt | 1 | 1.082 | 否 | 44.3 | 46.6 | 6.65 | 34.20 |
| receipt | 2 | 4.344 | 否 | 43.3 | 45.6 | 2.51 | 4.75 |
| receipt | 3 | 1.368 | 否 | 55.6 | 58.3 | 15.44 | 42.58 |
| image | 1 | 1.355 | 否 | 45.3 | 48.2 | 45.83 | 22.71 |
| image | 2 | 0.626 | 是 | 36.9 | 39.6 | 1.71 | 14.04 |
| image | 3 | 0.752 | 是 | 54.2 | 56.5 | 45.74 | 20.12 |

六轮检测有效比例均为 100%、TRACK_LOST 比例均为 0%。另有一轮 image 正式入口因缺少 recent-valid detection window 而 invalid，保留但不计作未命中；原生时钟修复的先行完整验证也是有效飞行，最近 0.982 m、未命中，不能从总记录中删除。因此正式对照是 receipt 0/3、image 2/3；连同先行验证，已观察到的有效 image 飞行是 2/4，尚不能声称重复性基本解决或统计显著提升。

在这组硬件/模型条件下，实测视觉处理链路的中位延迟约 37–56 ms，而不是未经测量的 100–300 ms。image 同一观测的配对姿态修正中位收益很小且符号不一致，末端仍有几十度残差。P1 的来源 CSV/时间图显示大残差可集中在高分 LightTrack 输出、且距最后 YOLO 约 1 s 的阶段；近接 YOLO 样本很少，不足以宣称来源整体精度的统计结论。下一步继续 P2/P3，不能把 KF 当作已证实能修复 tracker 漂移的结论，也不据此重写 PNG/PX4。

数据根目录为 `~/intercept_ws/experiment_data/p0_native_ab/`（receipt、image invalid、image_repeat 及统一 analysis）与 `p0_native_clock_validation/2026-10-16_00-14-52/`。此前未撤销 DDS 平滑偏移的记录仅供排查，不用于同步精度改善结论。固定制导二进制 SHA256 为 `711aea5041d9ad3907de005824edf985643dc5c0a7544b0d5a63066ed3b4b09d`，检测器为 `6e1c6c47ddd9137f7d03c94d3f61405502cf9aff7029985b92d56a608478ce6a`；本阶段只增加离线分析，未改变这两个二进制。

**P2 像素 KF / 延迟预测**

`--pixel-estimator raw` 保留 P0 路径；`--pixel-estimator kf --attitude-sync image` 启用 `[u,v,du,dv]` 常速度 KF，每张图像只在其映射后的拍摄时刻更新一次，再预测到本次控制计算时刻。P2 两种观测来源使用相同噪声：`pixel_measurement_std=4 px`、`pixel_acceleration_std=1000 px/s²`；预测最长 `pixel_prediction_horizon=0.3 s`，失锁/时钟重置切断历史。控制时刻姿态由最新有效 odometry quaternion 与 body-FRD 角速度传播，最长 50 ms；不匹配、过期或预测失败时显式拒绝，不回退成未经声明的 raw 控制。没有改变 PNG/FOV 增益、Yaw PD 的 dt 口径、PX4 控制接口或目标条件，默认仍为 raw。

`los_v/z` 始终保留原始图像 LOS；新增 `control_los_v/z` 是实际进入制导的 LOS，`estimate_time_ros`、`estimated_u/v`、`estimated_du/dv`、`pixel_prediction_dt`、`attitude_prediction_dt` 使预测时刻/来源可追溯。分析在相同控制真值上比较“保持原图像 LOS”与“预测 LOS”，`*_prediction_*_improvement_deg_*` 正值才表示预测改善，不把不同时间的误差当作同一指标。预测模式的误差—距离图也使用同一控制时刻的成对残差。

直接在旋转相机像素上使用 CV 的首轮完整试验（`p2_kf_validation/2026-10-16_00-53-40/`）未命中，首次最近距离 9.949 m，没有进入 5 m 近接窗口；全首次接近的成对预测收益中位数为垂直 −0.927°、方位 −0.319°，是负向结果，不能填补成末端成功。该轮图像→bbox / 控制中位延迟分别 66.4 / 68.7 ms。接下来先隔离自身旋转引入的像素运动，再继续评估；不把坏的预测路径设为默认，也不靠 P3 噪声加权掩盖 P2 的模型问题。

当前 `kf` 已改为固定参考相机平面的 CV：先用图像时刻姿态去旋转测量，预测参考像素，再投回控制时刻相机；相机像素速度也显式包含自身角速度项。`reference_time_ros` 记录参考平面锚点，不将“参考平面速度”误报成当前相机速度。理想固定世界 LOS、相机俯仰旋转、60 ms 延迟的复现中，原直接相机 CV 平均/最大伪误差为 4.724°/7.550°；改后通过固定世界 LOS 不变性、相机速度符号和真实 ROS 节点回归。93 项 Python/节点回归及两项 C++ 行为测试通过。

`p2_stabilized_validation/2026-10-16_01-11-49/` 的首轮有效圆周验证最近 0.944 m、未命中；此前一轮因入口检测窗口不足而 invalid，保留且未放宽门槛。与直接相机 CV 首轮 9.949 m 相比，这轮回到 0.944 m，但单轮不能证明该变化由去旋转修正独立造成，也不能证明命中率改善。该轮首次接近 76 条接受观测对应 13 次 KF 参考重建，完整记录含 157/1195 个 COAST 采样；下一步单独检验短暂 COAST 不应丢弃滤波运动状态，而 PNG 本身恢复逻辑保持不变。

短暂 COAST 现在只保持估计器状态，不清零其速度或重建参考平面；SEARCHING/TRACK_LOST、时钟/姿态 reset 仍会重置，图像间隔超过 0.5 s 的 CV 状态也不跨盲区携带旧速度。PNG 的 COAST 恢复初始化未改。真实节点回归已先复现“短暂停顿后 `du=0`”，修正后与其余 93 项回归一同通过。`p2_coast_validation/2026-10-16_01-22-49/` 首次接近 99 条接受观测只使用一个参考平面，但最近距离仍为 1.574 m、未命中；近接成对预测中位收益为垂直 −0.680°、方位 −1.176°。状态连续性修复不等于观测漂移被修复，接下来保留这一版本做重复对照，再推进 P3 来源融合。

**LightTrack 候选一致性修复**

P2 对照期间发现，原 C++ 先选出 window/size-penalized 评分最高的扁平索引，又通过浮点行列还原索引；18×18 网格在 Release 复现中有 132/324 个位置发生截断错位（例如 21→20）。当前直接保留整数索引，分类、尺度惩罚、bbox 回归严格对应同一候选，并移除不再需要的 pscore 临时数组。置信度也改为该候选的分类分数；日志同时保留 `raw_argmax` / `raw_score` 与 `selected`，不新增置信度过滤或假失锁门槛。

这与[官方 LightTrack 的候选选择及返回分数语义](https://github.com/researchmm/LightTrack/blob/39c426f48ee674795cdf0e00301a0b4ad0785d2a/lib/tracker/lighttrack.py)一致。官方也保留两次尺度混合，因此本阶段没有凭 `lr²` 的外观修改尺寸公式、窗口权重或学习率。修复前数据里的高 `track_score` 可能是另一位置的全局最大分数，不应事后当作实际 bbox 的校准置信度。

324 个候选位置、窗口/尺度评分优先级和并列边界的确定性回归通过。仅改前端候选关联、保持 `7b086dc` 的 image/raw 控制与全部条件不变的首轮真实 CUDA 圆周验证（`tracker_index_validation/2026-10-16_01-40-52/`）最近 2.145 m、未命中；确定性错误已修正，但该轮不能证明拦截性能提升。先以此正确前端继续对照，不用 KF 把错误索引隐藏起来。

**大图像接收节拍修复**

入口失效记录中，释放前 0.65 s 仅有 5 条有效框，最后两张处理图像的 stamp 相隔 297 ms，但每帧处理链路本身仅约 33 ms。随后在无推理场景同时检查 Gazebo 源流与 ROS 接收：原生源稳定窗口约 30 Hz；低开销 C++ 接收端保留 Depth 1，仅切换可靠性便得到下列结果。该现象不能简单用“YOLO 慢”或 Python `ros2 topic hz` 开销解释，也不泛化为所有 ROS 环境的 Best Effort 都有同样问题。

| 测量场景 | 图像订阅可靠性 | 接收/输出频率 Hz | 最大间隔 ms | image→bbox 均值/最大 ms |
|---|---|---:|---:|---:|
| 无推理，C++ Image 接收 | Best Effort | 9.82 | 794.2 | — |
| 无推理，C++ Image 接收 | Reliable | 30.18 | 54.5 | — |
| 真实 CUDA 推理，仅读取 bbox | Best Effort | 10.77 | 366.9 | 47.5 / 52.5 |
| 真实 CUDA 推理，仅读取 bbox，测量 1 | Reliable | 30.23 | 55.0 | 52.9 / 71.5 |
| 真实 CUDA 推理，仅读取 bbox，测量 2 | Reliable | 30.08 | 59.1 | 53.4 / 74.0 |

每帧为原始 1920×1080、6,220,800 bytes，桥接发布者实际提供 Reliable。推理吞吐探针在地面无目标场景运行，计数包含真实模型给出的 invalid 结果，**不代表检测准确率或命中率**；bbox 探针不订阅 Image，避免额外 Reliable reader 改变被测链路。没有改分辨率、像素、模型、FOV、PNG/PX4、操作系统网络缓冲或入口门槛。

检测节点新增 `image_reliability=reliable|best_effort`，本项目 Gazebo 默认使用 `reliable`，History 仍为 Keep Last 1。runner 的同名选择通过 `--image-reliability` 传入并记入协议/结果。真实相机若只发布 Best Effort，须显式传 `-p image_reliability:=best_effort`，否则可靠性不兼容会收不到图像；不会静默切换或重复发布旧框。单帧延迟与接收间隔必须分开测量，不能仅检查“话题存在/有效框比例”。

`camera_reliable_validation/2026-10-16_02-38-23/` 的完整固定圆周实飞中，1112 条接受观测达到 30.16 Hz，接收间隔中位/P95/最大为 32.9/48.6/75.0 ms，采样状态没有 COAST 或 TRACK_LOST。image→bbox / 控制中位为 43.9/46.4 ms，没有靠旧帧积压换取高频。**首次近接仍在 2.749 m 擦过，后续才于 36.875 s 命中**，因此接收节拍问题已解决，但不能宣称首次拦截精度基本解决。测量 JSON、测量源码快照与正式飞行引用保存在 `~/intercept_ws/experiment_data/camera_cadence_diagnostic/qos_probe.json`。

**异常帧取证（默认关闭，不改变选框或制导）**

```bash
ros2 run uav_bc_recorder phase2_runner --repo-root "$PWD" \
  --out-root "$HOME/intercept_ws/experiment_data/bbox_anomaly_diagnostic" \
  --groups near --trials 1 --pixel-estimator raw \
  --image-reliability reliable --capture-anomalies
```

runner 将 `diagnostic_capture_dir` 指向该轮 `anomaly_frames/`；单独启动检测节点也可用 `-p diagnostic_capture_dir:=/绝对路径`。空目录参数默认关闭，不创建缓存或写图线程。仅当相邻处理帧的有效框中心位移大于图像对角线的 20%（1920×1080 时约 440.6 px），且同尺寸、图像 stamp 严格递增、间隔不超过 150 ms 时触发。此阈值**只决定取证，不拒绝框、不影响控制**；不会跨 invalid 或时间空档比较。

每事件保存前两张、触发张、后两张已处理原图 PNG（不是保证相机连续帧），原图不画框、不缩放。`frames.csv` 记录原始 sec/ns、处理帧号、来源、最终框/分数；`candidates.csv` 记录 YOLO 经置信度筛选与 NMS 后的全部候选、裁剪前后坐标、类别/分数及选中标记，不是全部原始 anchors。`event.csv` 最后写入，记录触发位移、时间间隔和可用上下文；正常关闭时也保存未满两张后帧的 partial 事件。单后台 writer、最多两个等待事件、每进程最多 16 事件；队列满或写入失败明确报错，不等待 PNG 压缩阻塞推理。检查原图时应按 sec/ns 与 `observations.csv` 对齐，离线叠框另存，不能覆盖原图。

实测 `bbox_anomaly_diagnostic/2026-10-16_08-32-48/` 为有效 Combined/image/raw/Reliable 飞行，16 个完整事件、80 张原生分辨率 PNG 已逐张解码检查。首次最近距离 **1.169 m，未命中**，60 s 内也未最终命中，不能作为修复成功率的证据。

**已由图片确认：本机入镜桨叶被 YOLO 误检为目标。** `event_frame_1463_75_472000000/frame_0.png` 中真正目标仍在画面中部，唯一 YOLO 候选却是右边桨叶 `(1860,263,59,44)`、score 0.500；并非“正确候选也在，但最高分选错”。图像时刻距 INTERCEPT 约 6.415 s、GPS 距离 2.808 m、image→bbox 为 51.3 ms，错误方位残差约 −27.1°。该框立即重建 LightTrack；后两帧仍在右边缘，分数为 0.947/0.963。`event_frame_1540_78_13000000/` 另有左边桨叶误检 score 0.613。离线可视化位于该轮 `capture_review/`。本轮首次接近 image→bbox 中位/P95 为 56.4/70.4 ms；后续飞行中位增至 161.7 ms，尚未隔离其原因，不能声称取证对所有阶段时序零影响。

当前结论是“YOLO 漏检真实目标并误检自身桨叶 → 刷新直接覆盖模板 → Tracker 高分延续错误目标”。尚未改变相机安装、添加边缘遮罩、目标关联、失锁门槛或 KF；后续应先针对自身入镜与目标身份验证，不用滤波掩盖错误目标。

**原仓库感知策略三组对照**

上游 `6f2d53b` 在正常跟踪时不运行 YOLO，检测候选阈值为 `>=0.4`，获取模板另要求严格 `>0.8`。本地 `b74d7c7` 取消第二道门槛、`97a27ab` 增加周期刷新并删除 raw argmax 跳变失锁判据；这是策略变化，不应全部称为确定性 bug 修复。对照保留整数候选索引、时间同步、Reliable/Depth1、raw 及现有失锁判据，不整提交回滚。

runner 的 `--confidence-threshold` 默认0.4、`--acquisition-threshold`现默认0.8、`--yolo-refresh-interval`默认5，与检测节点直接启动一致。获取比较为 `score > acquisition_threshold`。下表记录 `2237ead` 时期的策略对照，低门槛参考现在须显式传 `--acquisition-threshold 0.0`，无刷新对照显式传 `--yolo-refresh-interval 0`；不能用提高检测阈值替代独立获取门槛。参数仅在启动时读取。

| 对照 | 检测候选 | 获取 | refresh |
|---|---:|---:|---:|
| 当前参考 | >=0.4 | >0.0（候选已过0.4） | 5 |
| 高获取门槛 | >=0.4 | >0.8 | 5 |
| 原式获取/持续跟踪 | >=0.4 | >0.8 | 0 |

例如原式分工：在固定圆周 runner 命令上加 `--confidence-threshold 0.4 --acquisition-threshold 0.8 --yolo-refresh-interval 0 --capture-anomalies`。开启取证时也保存第一次成功获取的前后上下文，`event.csv.trigger_reason=first_acquisition`，以排查“启动就认错”；没有历史时明确记录0张前帧，不伪造。真实 CUDA 冻结原图验证：score 0.4999756515 在默认门槛下接受、与门槛相等时拒绝、比门槛高一个 float ULP 时接受、0.8门槛下拒绝；首次获取无历史的图片回归通过。三组只验证获取/刷新分工，尚未恢复原索引失锁门槛。

**三组实测结果（代码 `2237ead`）**

`~/intercept_ws/experiment_data/perception_policy_ab/` 下三组各完成3个有效固定圆周试验，本批入口失败0；Combined/image/raw/Reliable、模型、相机、控制参数、当前面积失锁判据不变。按原式分工→高获取门槛→当前参考顺序运行，非随机化；入口实际高度及GPS噪声仍有差异，小样本结果不能当显著性或部署结论。命中仍为GPS距离0.8 m，不是实体接触证明。

| 策略 | 三轮首次最近距离 m | 首次命中 | 后续最终命中 | 首次获取真实目标 |
|---|---|---:|---:|---:|
| 当前参考，获取>0、refresh5 | 0.773 / 0.692 / 2.026 | 2/3 | 2/3 | 0/3 |
| 高获取门槛>0.8、refresh5 | 0.578 / 1.984 / 0.390 | 2/3 | 3/3 | 3/3 |
| 高获取门槛>0.8、refresh0 | 0.894 / 1.618 / 0.698 | 1/3 | 2/3 | 3/3 |

首次获取图片已逐组人工查看：参考组三轮分别以0.6637/0.6857/0.6961获取本机桨叶，之后可被刷新纠正；两个高门槛组均获取真实无人机。无刷新组三轮全程仅初始化一次，拦截期间无YOLO覆盖；首次接近image→bbox中位分别39.2/40.0/42.7 ms。恢复持续跟踪减少了主动模板覆盖，但本批并未显示更高首次命中率，不能把平滑跟踪等同撞击精度解决。

参考组第3轮 `event_frame_1451_74_350000000` 在拦截后6.392 s、距目标3.126 m时，YOLO以0.7916切到本机桨叶；首次在7.077 s、2.026 m擦过。这条覆盖路径会被0.8获取门槛阻止，但不能据单帧反事实推算改变轨迹后的命中率。

高门槛组第2轮 `event_frame_1847_89_761000000` 更明确显示：同帧桨叶0.814677、真实目标0.800589均过门槛，最高分桨叶覆盖了仍跟随真目标的Tracker。发生于18.921 s，**晚于该轮6.375 s首次近接**，不能拿它解释该轮首次未命中；它证明0.8并非身份保证，周期刷新仍保留错误覆盖入口。另有Tracker跟空天空却报大框和高分的后段图片，当前面积判据仍不足以判断真正失锁。

高门槛+refresh组实际YOLO调用占整个检测进程处理帧的85%–90%（含起飞/等待），并非始终每6帧一次：刷新未获取成功时计数不清零、下一帧重试。此行为在实验中保持不变。完整九轮指标、初始身份图片、真假候选和时刻分解在 `analysis/results.json`、`analysis/policy_diagnosis.json`、`review/`。本批不启用KF、不改默认策略、不追加遮罩或关联机制；原raw argmax失锁门槛仍是尚未单独验证的变量。

**历史冻结 VisionPNG 基线：`visionpng-baseline-high08-refresh5-raw`（标签不覆写）**

| 范围 | 冻结值 |
|---|---|
| 感知 | Combined（YOLO获取/刷新 + LightTrack）；默认CUDA、GDUT_UAV及现有LightTrack模型 |
| 候选/模板门槛 | YOLO候选 `>=0.4`；获取/覆盖模板严格 `>0.8`；IoU 0.45 |
| 刷新 | 5个Tracker帧后尝试YOLO；失败后保留现有逐帧重试行为 |
| 图像输入 | IMX214 1920×1080，Reliable，KeepLast 1，现有内参及安装不变 |
| 观测/姿态 | `pixel_estimator=raw`（KF关闭）；`attitude_sync=true`；Gazebo/native DDS时间映射 |
| 控制/评估 | speed 5 m/s，kv=kz=4，d_gain=2，yaw 0.0008/0.0002，k_ey=0.0056；GPS命中半径0.8 m |
| 失锁 | 保留当前Tracker面积判据；控制观测超时0.15/0.6 s；未恢复原raw argmax门槛 |

冻结依据追加24 m试验（实验版本 `5807967`，`~/intercept_ws/experiment_data/perception_policy_24m/analysis/policy24_diagnosis.json`）：高门槛refresh5三轮首次最近距离0.308/0.418/0.214 m，首次/最终均3/3；无刷新为21.027/21.131/20.872 m、0/3，初始均获取真目标但持续跟踪退化后未失锁。刷新组原图可见YOLO纠正背景跟踪。结合16.8 m结果，选择高门槛refresh5作为当前基线，不宣称已全面解决：高分桨叶仍可能覆盖目标，小目标/姿态变化下Tracker仍可能失效；样本每组仅3次，GPS命中不等同接触证明。

以上为历史冻结标签的默认参数与复现实验入口；重现旧行为须切换该标签。当前主线仍保持高获取门槛/refresh5/raw，但增加了下述传感器过滤与复核，不能把旧3/3成绩视为当前版本。普通launch的起飞设置不冒充固定圆周入口，取证仍显式开启。

冻结验证：实际启动默认检测节点及VisionPNG launch并读取运行参数；真实CUDA原图序列确认低分桨叶被拒绝、真目标获取、连续5次Tracker后再YOLO，显式低门槛覆盖仍可用于诊断。默认runner不传任何感知覆盖参数的SITL入口烟测也通过，接管延迟0.103 s；该2 s烟测不启动圆周运动、不启用距离闸门，**不计入16.8/24 m命中率**。验证日志与参数位于 `~/intercept_ws/experiment_data/visionpng_baseline_freeze/`。

**历史：无目标错误接管修复（`a326ea1`，单帧撤销版本）**

用户GUI记录中，接近整屏的错误Tracker框仍以高分被制导接受。真实CUDA负向回放中，160张无外部目标图像原先全部发布有效，其中64张近整屏；修复后仅最初5个Tracker间隔帧有效，随后撤销，整屏框0张，真目标重新出现仍可获取。该回放有刻意场景切换，不冒充实飞轨迹。

按用户选择，`run_swarm.sh` 使用独立 `x500_depth_external_view` 自机包装：17个本机visual设flags=1，只有IMX214 RGB camera mask=2；目标/地面默认flags不变，GUI全mask仍可显示本机。深度传感器参数未改，但未另行验证深度画面。不按像素位置涂黑，不移动相机或修改内参、collision、inertia；实际SDF展开比较确认除模型根名称及可见性字段外相同。包装只引用上游资源，不复制mesh，不修改共享x500_base/OakD；启动时部署repo资源符号链接，冲突报错，不覆盖用户目录。这是SITL成像策略，不等价于实机安装修复。

真实PX4/Gazebo静态目标出现/消失验证：15 s无目标阶段455条检测全部无效，保持SEARCHING，最大平移速度指令0；加入真实渲染x500后进入INTERCEPT，移除后0.724 s开始发送零平移指令。原图确认自身部件已排除、目标仍可见。此场景不是圆周命中率试验。控制端仍有0.15/0.6 s失效/COAST时序，并非立即停车；yaw搜索保留。脚本仍会先起飞到待机高度，再等待目标，不是“没有目标就不起飞”。

复核是新的安全性/连续性取舍，不会保证弱目标总能重获取。24 m圆周回归 `regression_far/2026-10-16_12-57-43` 在一次无候选复核后失效，后续候选长期低于0.8，因缺近期有效检测窗口拒绝接管；保留为入口无效，不记作控制未命中、不降低门槛救场。因此旧24 m的3/3不能迁移到本修复，尚不能宣称等性能替换冻结版。证据均在 `~/intercept_ws/experiment_data/no_target_diagnostic/`。

16.8 m移动圆周回归 `regression_near/2026-10-16_13-02-36` 入口有效，首次最近1.652 m未命中，随后44.057 s最终命中；采样TRACK_LOST占87.4%，表明恢复路径存在但连续性明显更保守，不能宣称性能不变。没有为此降低0.8门槛、扩大入口容差或添加KF。

启动器默认生成本次独立GZ_PARTITION；显式分区若已有world clock则在创建进程前失败，不复用旧模型、更不会杀其他仿真。旧世界残留的另一架自机属于外部visual，可能绕过本机过滤，因此不能继续复用。若已有 `GZ_PARTITION` 环境变量，先 `unset GZ_PARTITION` 再运行原采集命令；启动日志会打印实际分区。共享PX4/ROS话题的其他控制节点仍须先正常关闭。

**有界待确认连续性修复**

`a326ea1` 将一次YOLO缺支持、撤销制导观测和清除模板绑在同一帧；24m弱目标因此只能重过>0.8。现分离“可继续找同目标”和“可发布有效制导观测”：0.35s未支持宽限覆盖单次/短漏检；超过宽限publish invalid但不清内部预测；2s无支持后清上下文，禁止过期低分先验复活。2s基于旧失败记录约1.3s的候选空档选取，不是永久记忆。YOLO支持更新last-supported，Tracker正框/高分绝不更新它。正常帧接近宽限时也提前复核，probation每帧复核，避免低FPS延长授权。

真实CUDA原图回放已验证：score0.800867全局获取后，1/3帧缺支持都能以同位0.793742恢复；长期空图超hold输出invalid，超timeout清模板，过期低分图不能获取，新高分图可以。已有clock-injected回归覆盖边界、丢帧、倒退时钟、当前支持优先和有限锚点。真实PX4/Gazebo15s空场景保持SEARCHING/零平移，12m静态目标出现可获、消失后0.740s发送零平移命令，2s后内部明确过期；这不是移动目标命中率。另一次18m静态场景候选仅0.77–0.79，按首次门槛拒绝，保留记录，不降低阈值。

本机visual隔离、独立世界、Reliable、姿态/时钟同步、raw与PNG参数均未回退。此处只证明机制与有界性，不把它当作新baseline资格或旧命中率复现。日志区分global_acquired、probation_enter/recovered、local_recovery、publication_expired和recovery_expired；低分局部重建也标为YOLO测量，不能误解为低分全局首次获取。验证数据在 `~/intercept_ws/experiment_data/probation_validation/`。

仅在视觉与 PNG 本体原因仍不清楚时，使用相同协议补 Pure PNG：

```bash
ros2 run uav_bc_recorder phase2_runner --repo-root "$PWD" \
  --algorithm pure_png --groups far --trials 2
```

Pure 对照仍运行真实检测以保持进程负载接近，并将速度上限/命中半径匹配为 5 m/s / 0.8 m；不是调参优化，也不直接拿原始 8 m/s Pure 结果与视觉结果混比。

### 3. 运行拦截任务 (步骤序列)
请按以下顺序在不同终端中运行各节点：


0. **启动仿真环境**:
   从仓库根目录部署世界文件，并保留仓库内模型包装资源：

   ```bash
   # 只部署世界文件；不要单独复制run_swarm.sh
   cp assets/gazebo_world/grass_world.sdf \
     "${PX4_ROOT:-$HOME/PX4-Autopilot}/Tools/simulation/gz/worlds/"
   ```

   > 每次从保留完整assets的仓库目录运行。启动器自动将独立模型包装链接到PX4模型目录，不覆盖上游资源：
   ```bash
   bash ./run_swarm.sh
   ```

   该脚本会自动启动 Gazebo 仿真服务器、两架无人机（px4_1 拦截机 + px4_2 目标机）、MicroXRCEAgent 以及相机话题桥接。


1. **启动目标机仿真**:
   ```bash
   # 默认圆周运动
   ros2 run uav_target_sim uav_target_sim

   # 或通过参数选择运动模式和活动范围
   ros2 run uav_target_sim uav_target_sim --ros-args -p motion_mode:=sinusoidal -p max_range:=10.0

   # 也可使用 launch 文件
   ros2 launch uav_target_sim target_sim.launch.py motion_mode:=random_walk
   ```

   **支持的目标运动模式**：

   | 参数值 | 运动模式 | 运动学公式 | 特点 |
   | :--- | :--- | :--- | :--- |
   | `circle` | 匀速圆周 | R=5m, w=0.5rad/s | 默认模式，半径5m匀速圆周运动 |
   | `sinusoidal` | 正弦机动 | a=0.5*sin(0.5t) m/s<sup>2</sup> | 前向匀速1m/s + 侧向正弦加速度，周期性规避 |
   | `random_walk` | 随机游走 | v(t+dt) = v(t) + N(0, 0.2) | 速度随机扰动，限速3m/s，无规律逃逸 |

   > `max_range` 参数（默认10m）控制目标活动范围，防止目标飞出拦截机视野。

2. **启动视觉制导拦截**:
   ```bash
   # 推荐直接运行（使用 .hpp 中的默认参数）
   ros2 run uav_vision_png uav_vision_png

   # 或通过 launch 文件加载 config/params.yaml 中的参数
   ros2 launch uav_vision_png vision_png.launch.py
   ```

   > **参数配置说明**：`uav_vision_png` 节点的参数优先级为：
   >   **launch 命令行 > params.yaml > .hpp 成员初始值**
   >
   > - 使用 `ros2 run` 直接启动：参数取 `.hpp` 中的默认值，**`params.yaml` 不会自动加载**，需手动传参 `--ros-args -p 参数名:=值`
   > - 使用 `ros2 launch` 启动：launch 文件自动加载 `config/params.yaml`，YAML 中的值会覆盖 `.hpp` 默认值
   > - 调试时可用 `ros2 param get /uav_vision_png <参数名>` 查看当前实际生效的值
   > - 可调参数详见 `uav_vision_png/config/params.yaml`，含 PNG 增益、视场补偿、起飞高度等

2-备. **启动 RL 学习制导拦截 (替代方案)** 🆕:
   `uav_rl_guidance` 是基于强化学习（BC + PPO）训练的 GRU 策略制导节点，用于取代传统 PNG 算法。
   它与 `uav_vision_png` 保持相同的话题接口、状态机和 CSV 统计格式，可无缝替换。
   使用 ONNX Runtime 推理，零 PyTorch 依赖，纯 C++ 实现。

   ```bash
   # 方式1：launch 启动（自动加载 config/params.yaml）
   ros2 launch uav_rl_guidance rl_guidance.launch.py

   # 方式2：直接运行（使用代码默认参数）
   ros2 run uav_rl_guidance uav_rl_guidance

   # A/B 基线模式：全程使用内置 PNG，不加载策略模型（与 uav_vision_png 等价）
   ros2 launch uav_rl_guidance rl_guidance.launch.py fallback_png:=true

   # 台架调试：跳过起飞，直接进入 SEARCHING
   ros2 launch uav_rl_guidance rl_guidance.launch.py bench_test:=true
   ```

   > **架构说明**：RL 策略在 INTERCEPT 阶段输出速度指令；若策略出现异常（watchdog），自动回退到内置 PNG 控制器。
   > 策略模型（ONNX `.onnx`）存放在 `uav_rl_guidance/models/` 下，
   > 训练代码见 [AeroIntercept](https://github.com/Eaglewzw/AeroIntercept)，更新模型时执行 `python3 uav_rl_guidance/src/export_onnx.py`。

3. **启动视觉检测**:
   ```bash
   ros2 run uav_vision_dectect uav_vision_dectect
   ```
---

---

> **项目维护者注**：如果您需要更多关于算法推导（PNG）的细节，请参考各功能包下的 `include/` 头文件或联系开发团队。
