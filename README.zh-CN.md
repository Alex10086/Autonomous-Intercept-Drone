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
* `px4_msgs` 使用 `release/1.16` 的 `392e831c1f659429ca83902e66820d7094591410`，其 226 个消息定义与上述固件一致。执行 `git submodule update --init px4_msgs px4_ros_com` 补齐子模块。
* 该固件的飞行状态话题为 `/px4_N/fmu/out/vehicle_status_v1`，不是无版本后缀的旧话题。
* `uav_vision_png` 需要额外发布悬停推力估计。在 PX4 的 `src/modules/uxrce_dds_client/dds_topics.yaml` 的 `publications` 中添加以下条目后重新编译：

  ```yaml
  - topic: /fmu/out/hover_thrust_estimate
    type: px4_msgs::msg::HoverThrustEstimate
  ```

* DDS Agent 可安装至 `~/.local/opt/intercept/agent`，并链接命令到 `~/.local/bin/MicroXRCEAgent`。Jazzy 的 Fast-DDS 2.14 / Fast-CDR 2.2 可复用；Agent v2.4.3 使用上游 spdlog v1.9.2，不能直接替换为本机 spdlog 1.12 / fmt 9。
* 保持真实的 [QGroundControl 地面站](https://docs.qgroundcontrol.com/Stable_V5.1/en/qgc-user-guide/getting_started/download_and_install.html)连接。未连接地面站时 PX4 可能拒绝解锁；不要用关闭预检或强制解锁代替缺失的连接。
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
  uav_png_intercept uav_keyboard_control uav_vision_png ros2_helloworld \
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
2. `bash ~/PX4-Autopilot/run_swarm.sh`。先将本仓库的世界文件放入 PX4 的 `Tools/simulation/gz/worlds/grass_world.sdf`；可将启动脚本链接到 PX4 根目录。`PX4_ROOT` 可覆盖默认源码目录，`HEADLESS=1` 可关闭 Gazebo GUI。
3. `ros2 run uav_target_sim uav_target_sim`。
4. `ros2 run uav_png_intercept uav_png_intercept --ros-args -p csv_path:="$HOME/intercept_ws/intercept_data.csv"`。

`run_swarm.sh` 仅停止自己创建的进程组，不再全局杀死其他仿真或删除历史日志。停止控制节点会触发正常的 Offboard 丢失保护；测试验证的是起飞和通信，不是完整拦截命中。


### 2.2 Jazzy 视觉检测与制导

实测已通过默认 CUDA 模式的真实模型检测/跟踪、检测框发布、悬停推力收敛，以及 `uav_vision_png` 从起飞进入 `SEARCHING` / `INTERCEPT`。三个模型的 ONNX profiling 均记录了 CUDA 卷积运算；验证不是假检测框，也不是只检查 Provider 名称。完整拦截命中和 RL 仍未验证。

视觉输入链路为 `/camera/image` → `uav_vision_dectect` → `/camera_detect_result` → `uav_vision_png`。检测节点不发布飞控命令，不要同时运行 `uav_png_intercept` 或其他拦截机控制节点。

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
ros2 run uav_vision_png uav_vision_png \
  --ros-args -p csv_path:="$HOME/intercept_ws/vpng_intercept_stats.csv"
```

检测节点参数：

* `use_cuda` 默认 `true`。CUDA 初始化或推理失败时退出并报告错误，不会悄悄回退 CPU；显式 CPU 诊断可传 `-p use_cuda:=false`。
* `enable_tracking` 默认 `true`；`show_debug_window` 默认 `true`。无显示环境传 `-p show_debug_window:=false`。
* `yolo_model_path`、`tracker_init_model_path`、`tracker_update_model_path` 可覆盖安装目录中的默认模型。
* 保留原算法的 0.4 推理过滤、0.45 IoU 阈值和严格 `>0.8` 的目标捕获条件。发布一次实际处理图像的检测框并保留图像时间戳；丢失目标时 bbox 四个字段均为 `-1`，不重复发布旧框伪装成新检测。

视觉制导到达待机高度且悬停推力估计收敛后进入 `SEARCHING`，收到有效检测框后进入 `INTERCEPT`。`/vpng_data` 可观察真实 LOS 和像素误差；只有启动进程或看到话题存在，不等于识别或拦截成功。

### 3. 运行拦截任务 (步骤序列)
请按以下顺序在不同终端中运行各节点：


0. **启动仿真环境**:
   启动前，需将本仓库的仿真资源复制到 PX4-Autopilot 目录下：

   ```bash
   # 步骤1: 将本仓库的 run_swarm.sh 复制到 PX4-Autopilot 根目录（若尚未复制）
   cp ros2_ws/src/run_swarm.sh ~/PX4-Autopilot/

   # 步骤2: 将自定义世界文件复制到 PX4 Gazebo 世界目录
   cp ros2_ws/src/assets/gazebo_world/grass_world.sdf ~/PX4-Autopilot/Tools/simulation/gz/worlds/
   ```

   > 以上步骤只需执行一次。之后每次启动仿真，直接进入 PX4-Autopilot 目录运行启动脚本即可：
   ```bash
   cd ~/PX4-Autopilot
   ./run_swarm.sh
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
