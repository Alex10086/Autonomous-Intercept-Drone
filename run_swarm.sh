#!/bin/bash

# =======================================================
# PX4 双机仿真启动脚本
# Drone 1: gz_x500_depth_external_view (RGB 排除本机 visual, Instance 1)
# Drone 2: gz_x500       (普通版,     Instance 2)
# 世界: grass_world
# =======================================================

set -euo pipefail

# Resolve repository assets before switching to the external PX4 working directory.
REPO_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"

export PATH="$HOME/.local/bin:$PATH"
PX4_ROOT="${PX4_ROOT:-$HOME/PX4-Autopilot}"
# Reproducible experiments may override spawn poses without editing PX4 files.
PX4_SELF_POSE="${PX4_SELF_POSE:-0,0}"
PX4_TARGET_POSE="${PX4_TARGET_POSE:-20,0}"
# Both models have barometers but no downward range sensor. Keep one height datum.
# Gazebo GNSS is stamped at receipt; retain horizontal/3D-velocity aiding, not altitude.
# 0.3 m conservatively covers the measured ~0.15 m barometric altitude noise.
export PX4_PARAM_EKF2_HGT_REF=0
export PX4_PARAM_EKF2_GPS_CTRL=5
export PX4_PARAM_EKF2_GPS_DELAY=0
export PX4_PARAM_EKF2_BARO_NOISE=0.3
cd "$PX4_ROOT"

# -------------------------------------------------------
# 0. 检查工作目录 & 依赖工具
# -------------------------------------------------------
if [ ! -f "build/px4_sitl_default/bin/px4" ]; then
    echo "错误：未找到 $PX4_ROOT/build/px4_sitl_default/bin/px4"
    echo "请先构建 PX4 SITL，或用 PX4_ROOT 指定已构建的源码目录"
    exit 1
fi

if ! command -v rotatelogs &>/dev/null; then
    echo "错误：未找到 rotatelogs，请安装 apache2-utils，或放入 ~/.local/bin"
    exit 1
fi

# 日志轮转配置：每个日志最大 5MB，保留最近 2 个文件
LOG_MAX_SIZE="5M"
LOG_MAX_FILES=2

for command in MicroXRCEAgent ros2 setsid gz; do
    command -v "$command" >/dev/null || {
        echo "错误：未找到 $command，请先加载 ROS 和本地工具环境"
        exit 1
    }
done

# A leftover model is external to the new self instance and would still be seen.
# Use a fresh world by default; an explicit occupied partition must fail closed.
export GZ_PARTITION="${GZ_PARTITION:-intercept_${UID}_$$}"
echo "Gazebo partition: $GZ_PARTITION"
GZ_TOPICS="$(gz topic -l)"
while IFS= read -r topic; do
    case "$topic" in
        /world/*/clock)
            echo "错误：该Gazebo分区已有世界；拒绝复用遗留模型：$GZ_PARTITION" >&2
            echo "请unset GZ_PARTITION或使用全新分区；本脚本不会关闭其他仿真。" >&2
            exit 1
            ;;
    esac
done <<< "$GZ_TOPICS"

# Deploy only the repository-owned wrapper; never replace upstream or user models.
MODEL_NAME=x500_depth_external_view
MODEL_SOURCE_DIR="$REPO_ROOT/assets/gazebo_models/$MODEL_NAME"
PX4_MODELS_DIR="$PWD/Tools/simulation/gz/models"
MODEL_DEPLOY_PATH="$PX4_MODELS_DIR/$MODEL_NAME"
for resource in model.sdf model.config; do
    if [ ! -r "$MODEL_SOURCE_DIR/$resource" ] || [ ! -s "$MODEL_SOURCE_DIR/$resource" ]; then
        echo "错误：缺少或无法读取本机 visual 过滤模型资源：$MODEL_SOURCE_DIR/$resource" >&2
        exit 1
    fi
done
if [ ! -d "$PX4_MODELS_DIR" ]; then
    echo "错误：未找到 PX4 Gazebo 模型目录：$PX4_MODELS_DIR" >&2
    exit 1
fi
for dependency in x500 x500_base OakD-Lite; do
    if [ ! -r "$PX4_MODELS_DIR/$dependency/model.sdf" ]; then
        echo "错误：缺少或无法读取上游模型：$PX4_MODELS_DIR/$dependency/model.sdf" >&2
        exit 1
    fi
done
if [ -e "$MODEL_DEPLOY_PATH" ] || [ -L "$MODEL_DEPLOY_PATH" ]; then
    if [ ! -L "$MODEL_DEPLOY_PATH" ] || \
        [ "$(readlink -f -- "$MODEL_DEPLOY_PATH")" != "$MODEL_SOURCE_DIR" ]; then
        echo "错误：模型路径已存在且不是预期的 repo 资源符号链接：$MODEL_DEPLOY_PATH" >&2
        echo "预期指向：$MODEL_SOURCE_DIR；保留现有文件，请手动处理冲突。" >&2
        exit 1
    fi
elif ! ln -sT -- "$MODEL_SOURCE_DIR" "$MODEL_DEPLOY_PATH"; then
    echo "错误：无法部署本机 visual 过滤模型：$MODEL_DEPLOY_PATH" >&2
    exit 1
fi

CONFIG_DIR=$(mktemp -d)
PIDS=()
cleanup() {
    trap - EXIT INT TERM
    for pid in "${PIDS[@]}"; do
        kill -TERM -- "-$pid" 2>/dev/null || true
    done
    sleep 1
    for pid in "${PIDS[@]}"; do
        kill -KILL -- "-$pid" 2>/dev/null || true
    done
    rm -rf "$CONFIG_DIR"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM


# -------------------------------------------------------
# 2. 启动 MicroXRCEAgent (ROS 2 通信桥)
# -------------------------------------------------------
echo ""
echo "📡 正在启动 MicroXRCEAgent..."
setsid bash -o pipefail -c 'MicroXRCEAgent udp4 -p 8888 2>&1 | rotatelogs -n "$1" agent.log "$2"' \
    bash "$LOG_MAX_FILES" "$LOG_MAX_SIZE" &
PID_AGENT=$!
PIDS+=("$PID_AGENT")
echo "   -> Agent PID: $PID_AGENT (日志: agent.log, 最大 ${LOG_MAX_SIZE} x${LOG_MAX_FILES})"
sleep 2

# -------------------------------------------------------
# 3. 启动第一架无人机 (带深度相机, Instance 1)
#    同时拉起 Gazebo 仿真服务器 + grass_world
# -------------------------------------------------------
echo ""
echo "🚀 正在启动 Drone 1 (gz_x500_depth_external_view, Instance 1)..."
PX4_GZ_WORLD=grass_world \
PX4_SYS_AUTOSTART=4002 \
PX4_GZ_MODEL_POSE="$PX4_SELF_POSE" \
PX4_SIM_MODEL=gz_x500_depth_external_view \
    setsid bash -o pipefail -c './build/px4_sitl_default/bin/px4 -d -i 1 2>&1 | rotatelogs -n "$1" drone1.log "$2"' \
    bash "$LOG_MAX_FILES" "$LOG_MAX_SIZE" &
PID1=$!
PIDS+=("$PID1")
echo "   -> Drone 1 PID: $PID1 (日志: drone1.log, 最大 ${LOG_MAX_SIZE} x${LOG_MAX_FILES})"

echo ""
echo "⏳ 等待 Gazebo 世界加载 (15 秒)..."
sleep 15

	# -------------------------------------------------------
	# 4. 启动 ROS 2 相机桥接 (YAML 内嵌，直接映射到 /camera/image)
	# -------------------------------------------------------
	echo ""
	echo "🌉 正在启动 ROS 2 相机桥接..."

	cat > "$CONFIG_DIR/camera_bridge.yaml" << 'YEOF'
- ros_topic_name: "/camera/image"
  gz_topic_name: "/world/grass_world/model/x500_depth_external_view_1/link/camera_link/sensor/IMX214/image"
  ros_type_name: "sensor_msgs/msg/Image"
  gz_type_name: "gz.msgs.Image"
  direction: GZ_TO_ROS
YEOF

	setsid bash -o pipefail -c 'ros2 run ros_gz_bridge parameter_bridge \
	    --ros-args -p config_file:="$1" 2>&1 | rotatelogs -n "$2" bridge.log "$3"' \
	    bash "$CONFIG_DIR/camera_bridge.yaml" "$LOG_MAX_FILES" "$LOG_MAX_SIZE" &
	PID_BRIDGE=$!
	PIDS+=("$PID_BRIDGE")
	echo "   -> Bridge PID: $PID_BRIDGE (bridge.log)"
	echo "   -> /camera/image"
	# Preserve Gazebo sim/system pairs without changing control-node ROS clocks.
	setsid ros2 run uav_vision_png simulation_clock_bridge &
	PIDS+=("$!")
	sleep 2
echo ""
echo "🚀 正在启动 Drone 2 (gz_x500, Instance 2)..."
PX4_GZ_STANDALONE=1 \
PX4_GZ_WORLD=grass_world \
PX4_SYS_AUTOSTART=4001 \
PX4_GZ_MODEL_POSE="$PX4_TARGET_POSE" \
PX4_SIM_MODEL=gz_x500 \
    setsid bash -o pipefail -c './build/px4_sitl_default/bin/px4 -d -i 2 2>&1 | rotatelogs -n "$1" drone2.log "$2"' \
    bash "$LOG_MAX_FILES" "$LOG_MAX_SIZE" &
PID2=$!
PIDS+=("$PID2")
echo "   -> Drone 2 PID: $PID2 (日志: drone2.log, 最大 ${LOG_MAX_SIZE} x${LOG_MAX_FILES})"
sleep 3

# -------------------------------------------------------
# 6. 启动摘要
# -------------------------------------------------------
echo ""
echo "✅ 所有系统启动完成！"
echo "================================================"
echo "  MicroXRCEAgent : PID $PID_AGENT  (port 8888)"
echo "  Drone 1 (相机)  : PID $PID1      Instance 1  UXRCE_DDS_KEY=2  /px4_1/"
echo "  Drone 2 (普通)  : PID $PID2      Instance 2  UXRCE_DDS_KEY=3  /px4_2/"
echo "  Camera Bridge  : PID $PID_BRIDGE"
echo "------------------------------------------------"
echo "  日志限制        : 每个最大 ${LOG_MAX_SIZE}，保留 ${LOG_MAX_FILES} 个文件"
echo "  相机话题        : /camera/image"
echo "  世界地图        : grass_world"
echo "  Drone 1 位置   : $PX4_SELF_POSE"
echo "  Drone 2 位置   : $PX4_TARGET_POSE"
echo "================================================"
echo ""
echo "  验证命令:"
echo "  ros2 topic list | grep camera"
echo "  ros2 topic hz /camera/image"
echo "  ros2 topic list | grep px4"
echo ""
echo "按 Ctrl+C 关闭所有仿真进程"


# 等待主进程退出
wait "$PID1" "$PID2" "$PID_AGENT" "$PID_BRIDGE"
