/**
 * vision_png_control.cpp
 *
 * 基于视觉的比例导引（Vision-based PNG）实现
 *
 * ┌──────────────────────────────────────────────────────────────┐
 * │  导引算法（production=视觉；truth_los=显式SITL诊断）             │
 * │    1. 从像素坐标 → 相机系 LOS 向量                           │
 * │    2. R_c2b * R_b2n 旋转到 NED 系                           │
 * │    3. 速度角 += kv/kz * LOS 角增量                           │
 * │    4. 速度指令：vx/vy/vz                                    │
 * ├──────────────────────────────────────────────────────────────┤
 * │  GPS 命中评估（不参与视觉导引计算）                             │
 * │    双方 GPS → LocalCartesian → 真实距离 → DONE 悬停           │
 * └──────────────────────────────────────────────────────────────┘
 * 遥测独立以 20 Hz 发布至 /experiment/telemetry；文件记录由 recorder 负责。
 *
 * 参考：
 *   uav_ibvs_control/src/los_control.cpp    —— 视觉 LOS 计算方式
 *   uav_png_intercept/src/png_control.cpp   —— PNG 状态机框架
 *   include/uav_vision_png/vision_png_control.hpp
 */

#include "uav_vision_png/vision_png_control.hpp"
#include <iomanip>
#include <cmath>
#include <limits>
#include <sstream>
#include <stdexcept>

// ============================================================
//  构造函数
// ============================================================
VisionPNG::VisionPNG() : Node("uav_vision_png")
{
    // ---- QoS（与 PX4 MiddleWare 一致，sensor_data 模式）----
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(
        rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

    // ---- 从参数服务器读取参数（允许 launch 文件覆盖）----
    this->declare_parameter("focal_length",      focal_length_);
    this->declare_parameter("image_width",        image_width_);
    this->declare_parameter("image_height",       image_height_);
    this->declare_parameter("speed_cmd",          speed_cmd_);
    this->declare_parameter("d_gain",             d_gain_);
    this->declare_parameter("kv",                 kv_);
    this->declare_parameter("kz",                 kz_);
    this->declare_parameter("k1_yaw",             k1_yaw_);
    this->declare_parameter("k2_yaw",             k2_yaw_);
    this->declare_parameter("k_ey",               k_ey_);
    this->declare_parameter("standby_altitude",   standby_altitude_);
    this->declare_parameter("freshness_timeout", freshness_timeout_);
    this->declare_parameter("track_lost_timeout", track_lost_timeout_);
    this->declare_parameter("hit_radius",         (double)hit_radius_);
    guidance_source_ = declare_parameter<std::string>("guidance_source", "vision");
    if (guidance_source_ != "vision" && guidance_source_ != "truth_los") {
        throw std::invalid_argument("guidance_source must be vision or truth_los");
    }

    focal_length_      = this->get_parameter("focal_length").as_double();
    image_width_       = this->get_parameter("image_width").as_int();
    image_height_      = this->get_parameter("image_height").as_int();
    speed_cmd_         = this->get_parameter("speed_cmd").as_double();
    d_gain_            = this->get_parameter("d_gain").as_double();
    kv_                = this->get_parameter("kv").as_double();
    kz_                = this->get_parameter("kz").as_double();
    k1_yaw_            = this->get_parameter("k1_yaw").as_double();
    k2_yaw_            = this->get_parameter("k2_yaw").as_double();
    k_ey_              = this->get_parameter("k_ey").as_double();
    standby_altitude_  = this->get_parameter("standby_altitude").as_double();
    freshness_timeout_ = this->get_parameter("freshness_timeout").as_double();
    track_lost_timeout_ = this->get_parameter("track_lost_timeout").as_double();
    if (!std::isfinite(freshness_timeout_) || freshness_timeout_ <= 0.0 ||
        !std::isfinite(track_lost_timeout_) || track_lost_timeout_ <= freshness_timeout_) {
        throw std::invalid_argument(
            "Require finite 0 < freshness_timeout < track_lost_timeout (seconds)");
    }
    hit_radius_        = (float)this->get_parameter("hit_radius").as_double();
    require_experiment_start_ = declare_parameter<bool>("require_experiment_start", false);
    takeoff_altitude_tolerance_ = declare_parameter<double>("takeoff_altitude_tolerance", 0.5);
    takeoff_speed_tolerance_ = declare_parameter<double>("takeoff_speed_tolerance", 0.0);
    takeoff_settle_time_ = declare_parameter<double>("takeoff_settle_time", 0.0);
    standby_yaw_ = declare_parameter<double>("standby_yaw", standby_yaw_);
    if (!std::isfinite(standby_altitude_) ||
        !std::isfinite(takeoff_altitude_tolerance_) || takeoff_altitude_tolerance_ <= 0.0 ||
        !std::isfinite(takeoff_speed_tolerance_) || takeoff_speed_tolerance_ < 0.0 ||
        !std::isfinite(takeoff_settle_time_) || takeoff_settle_time_ < 0.0 ||
        (!std::isfinite(standby_yaw_) && !std::isnan(standby_yaw_))) {
        throw std::invalid_argument("Invalid standby/takeoff parameters: finite altitude, positive altitude tolerance, nonnegative speed tolerance (0 disables) and settle time, finite or NaN yaw required");
    }
    experiment_start_sub_ = create_subscription<std_msgs::msg::Bool>(
        "/experiment/start", rclcpp::QoS(10).reliable(),
        [this](const std_msgs::msg::Bool::SharedPtr msg) { experiment_start_ = msg->data; });
    precise_standby_ = require_experiment_start_ || takeoff_settle_time_ > 0.0 ||
        takeoff_speed_tolerance_ > 0.0 || takeoff_altitude_tolerance_ != 0.5 ||
        std::isfinite(standby_yaw_);

    // ---- 相机→机体旋转矩阵（与 los_control.cpp 保持一致）----
    // Cx → By,  Cy → Bz,  Cz → Bx
    R_c2b_ << 0, 0, 1,
               1, 0, 0,
               0, 1, 0;

    // ============================================================
    //  发布者
    // ============================================================
    offboard_pub_ = create_publisher<OffboardControlMode>(
        "/px4_1/fmu/in/offboard_control_mode", 10);
    traj_pub_ = create_publisher<TrajectorySetpoint>(
        "/px4_1/fmu/in/trajectory_setpoint", 10);
    cmd_pub_ = create_publisher<VehicleCommand>(
        "/px4_1/fmu/in/vehicle_command", 10);
    data_pub_ = create_publisher<DataMsg>("/vpng_data", 10);
    telemetry_pub_ = create_publisher<uav_common_msg::msg::ExperimentTelemetry>(
        "/experiment/telemetry", 10);

    // ============================================================
    //  订阅者 —— 导引专用
    // ============================================================

    // 视觉检测结果（核心输入）
    detect_sub_ = create_subscription<RectMsg>(
        "/camera_detect_result", 10,
        [this](const RectMsg::UniquePtr msg) { detection_callback(*msg); });

    // 自身姿态（四元数 → 欧拉角）
    odometry_sub_ = create_subscription<VehicleOdometry>(
        "/px4_1/fmu/out/vehicle_odometry", qos,
        [this](const VehicleOdometry::SharedPtr msg) {
            double q0 = msg->q[0], q1 = msg->q[1];
            double q2 = msg->q[2], q3 = msg->q[3];
            roll_  = std::atan2(2*(q0*q1 + q2*q3), 1 - 2*(q1*q1 + q2*q2));
            pitch_ = std::asin (2*(q0*q2 - q3*q1));
            yaw_   = std::atan2(2*(q0*q3 + q1*q2), 1 - 2*(q2*q2 + q3*q3));
        });

    // 自身速度与本地高度
    local_pos_sub_ = create_subscription<VehicleLocalPosition>(
        "/px4_1/fmu/out/vehicle_local_position", qos,
        [this](const VehicleLocalPosition::SharedPtr msg) {
            vx_ = msg->vx;
            vy_ = msg->vy;
            vz_ = msg->vz;
            local_z_ = msg->z;
            local_position_received_ = msg->z_valid &&
                (takeoff_speed_tolerance_ == 0.0 || (msg->v_xy_valid && msg->v_z_valid));
        });

    // 飞行状态（判断 Offboard）
    status_sub_ = create_subscription<VehicleStatus>(
        "/px4_1/fmu/out/vehicle_status_v1", qos,
        [this](const VehicleStatus::SharedPtr msg) {
            offboard_active_ =
                (msg->nav_state == msg->NAVIGATION_STATE_OFFBOARD);
        });

    // 悬停推力估计（判断起飞完成）
    hover_sub_ = create_subscription<HoverThrustEstimate>(
        "/px4_1/fmu/out/hover_thrust_estimate", qos,
        [this](const HoverThrustEstimate::SharedPtr msg) {
            if (msg->hover_thrust_var < 0.0025f) {
                hover_thrust_    = msg->hover_thrust;
                hover_thrust_ok_ = true;
            }
        });

    // ============================================================
    //  订阅者 —— 纯统计用（导引算法严禁使用这些数据）
    // ============================================================

    // 自身 GPS → 建立统计坐标系
    self_gps_sub_ = create_subscription<SensorGps>(
        "/px4_1/fmu/out/vehicle_gps_position", qos,
        [this](const SensorGps::SharedPtr msg) {
            if (!lc_stats_init_) {
                // 以首帧 GPS 作为 LocalCartesian 原点（仅统计用）
                lc_stats_ = GeographicLib::LocalCartesian(
                    msg->latitude_deg,
                    msg->longitude_deg,
                    msg->altitude_msl_m);
                lc_stats_init_ = true;
                RCLCPP_INFO(get_logger(),
                    "[统计] 坐标系原点初始化: lat=%.7f lon=%.7f",
                    msg->latitude_deg, msg->longitude_deg);
            }
            // 更新自身位置（ENU → NED）
            double e, n, u;
            lc_stats_.Forward(msg->latitude_deg, msg->longitude_deg,
                              msg->altitude_msl_m, e, n, u);
            self_pos_stats_ = {(float)n, (float)e, (float)(-u)};
            last_self_gps_observation_ = std::chrono::steady_clock::now();
        });

    // 目标 GPS（px4_2，仅供统计！导引算法不得使用）
    target_gps_sub_ = create_subscription<SensorGps>(
        "/px4_2/fmu/out/vehicle_gps_position", qos,
        [this](const SensorGps::SharedPtr msg) {
            if (!lc_stats_init_) return;
            double e, n, u;
            lc_stats_.Forward(msg->latitude_deg, msg->longitude_deg,
                              msg->altitude_msl_m, e, n, u);
            target_pos_stats_  = {(float)n, (float)e, (float)(-u)};
            target_pos_ok_    = true;
            last_target_gps_observation_ = std::chrono::steady_clock::now();
            ++truth_generation_;
        });

    // ============================================================
    //  定时器
    // ============================================================
    // 200 Hz 主控制循环（心跳 + 状态机调度）
    timer_control_ = create_wall_timer(
        std::chrono::microseconds(5000),
        std::bind(&VisionPNG::control_loop, this));

    // Cache startup parameters, including effective launch overrides and ROS parameters.
    std::ostringstream yaml;
    yaml << std::setprecision(17);
    for (const auto &name : list_parameters({}, 0).names) {
        const auto parameter = get_parameter(name);
        yaml << std::quoted(name) << ": ";
        if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
            yaml << std::quoted(parameter.as_string());
        } else if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
            const double value = parameter.as_double();
            if (std::isnan(value)) yaml << ".nan";
            else if (std::isinf(value)) yaml << (value > 0.0 ? ".inf" : "-.inf");
            else yaml << value;
        } else {
            yaml << parameter.value_to_string();
        }
        yaml << '\n';
    }
    yaml << "controller: \"px4_velocity\"\nplanner: \"none\"\n";
    parameters_yaml_ = yaml.str();
    timer_telemetry_ = create_wall_timer(std::chrono::milliseconds(50),
        std::bind(&VisionPNG::publish_telemetry, this));

    RCLCPP_INFO(get_logger(),
        "VisionPNG 节点已启动 | source=%s focal=%.1f img=%dx%d kv=%.1f kz=%.1f spd=%.1f",
        guidance_source_.c_str(), focal_length_, image_width_, image_height_,
        kv_, kz_, speed_cmd_);
}


// ============================================================
//  200 Hz 主控制循环
// ============================================================
void VisionPNG::control_loop()
{
    update_truth_guidance();
    update_visual_state();
    switch (state_) {
    case VpngState::TAKE_OFF:    handle_takeoff();    break;
    case VpngState::SEARCHING:   handle_searching();  break;
    case VpngState::INTERCEPT:   handle_intercept();  break;
    case VpngState::COAST:       handle_intercept();  break;
    case VpngState::TRACK_LOST:  handle_track_lost(); break;
    case VpngState::DONE:        handle_done();       break;
    }
}

void VisionPNG::update_truth_guidance()
{
    if (guidance_source_ != "truth_los" ||
        state_ == VpngState::TAKE_OFF || state_ == VpngState::DONE) {
        return;
    }
    const auto now = std::chrono::steady_clock::now();
    const bool geometry_fresh = lc_stats_init_ && target_pos_ok_ &&
        now - last_self_gps_observation_ <=
            std::chrono::duration<double>(freshness_timeout_) &&
        now - last_target_gps_observation_ <=
            std::chrono::duration<double>(freshness_timeout_);
    if (!geometry_fresh || truth_generation_ == processed_truth_generation_) {
        return;
    }
    if (state_ == VpngState::SEARCHING &&
        ((require_experiment_start_ && !experiment_start_) ||
         (precise_standby_ && !takeoff_ready()))) {
        return;
    }

    const Eigen::Vector3d relative =
        (target_pos_stats_ - self_pos_stats_).cast<double>();
    if (!relative.allFinite() || relative.norm() < 1e-6) {
        return;
    }
    Eigen::AngleAxisd Ryaw(yaw_, Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd Rpitch(pitch_, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd Rroll(roll_, Eigen::Vector3d::UnitX());
    const Eigen::Matrix3d R_b2n = Ryaw.toRotationMatrix() *
        Rpitch.toRotationMatrix() * Rroll.toRotationMatrix();
    const Eigen::Vector3d camera =
        R_c2b_.transpose() * R_b2n.transpose() * relative;
    double ex;
    double ey;
    if (camera.z() > 1e-6) {
        ex = focal_length_ * camera.x() / camera.z();
        ey = focal_length_ * camera.y() / camera.z();
    } else {
        ex = std::copysign(static_cast<double>(image_width_), camera.x());
        ey = std::copysign(static_cast<double>(image_height_), camera.y());
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
            "[TRUTH_LOS] target behind camera; using saturated ideal pixel");
    }
    detect_x_ = static_cast<int>(std::lround(image_width_ / 2.0 + ex));
    detect_y_ = static_cast<int>(std::lround(image_height_ / 2.0 + ey));
    detect_w_ = 0;
    detect_h_ = 0;
    if (state_ != VpngState::INTERCEPT) {
        png_initialized_ = false;
    }
    state_ = VpngState::INTERCEPT;
    last_visual_observation_ = now;
    processed_truth_generation_ = truth_generation_;
    png_calculate(roll_, pitch_, yaw_);
}

double VisionPNG::observation_age() const
{
    return std::chrono::duration<double>(
        std::chrono::steady_clock::now() - last_visual_observation_).count();
}

void VisionPNG::update_visual_state()
{
    if (state_ != VpngState::INTERCEPT && state_ != VpngState::COAST) return;
    const double age = observation_age();
    if (age >= track_lost_timeout_) {
        state_ = VpngState::TRACK_LOST;
    } else if (age > freshness_timeout_) {
        state_ = VpngState::COAST;
    }
}

void VisionPNG::detection_callback(const RectMsg &msg)
{
    if (guidance_source_ != "vision") return;
    // Check the previous observation before a new arrival can refresh its age.
    update_visual_state();
    // Invalid results neither replace the cached observation nor refresh its age.
    if (msg.width <= 0 || msg.height <= 0) return;
    const double roll = roll_, pitch = pitch_, yaw = yaw_;
    detect_x_ = msg.x;
    detect_y_ = msg.y;
    detect_w_ = msg.width;
    detect_h_ = msg.height;
    last_visual_observation_ = std::chrono::steady_clock::now();
    if (state_ == VpngState::TAKE_OFF || state_ == VpngState::DONE) return;
    if (state_ == VpngState::SEARCHING &&
        ((require_experiment_start_ && !experiment_start_) ||
         (precise_standby_ && !takeoff_ready()))) return;

    // First acquisition and either loss recovery use the existing initialization.
    if (state_ != VpngState::INTERCEPT) png_initialized_ = false;
    state_ = VpngState::INTERCEPT;
    png_calculate(roll, pitch, yaw);
}

// ============================================================
//  每个有效检测回调仅计算一次，姿态固定为接收时刻快照
//  严格参考 los_control.cpp 中的 LOS_calculate()
//  输入：像素坐标 detect_x_, detect_y_, detect_w_, detect_h_
//  输出：d_v_angle_v_, d_v_angle_z_（期望速度角度）
// ============================================================
void VisionPNG::png_calculate(double roll, double pitch, double yaw)
{

    // ---------- Step 1: 计算像素误差（相对图像中心）----------
    double cx = image_width_  / 2.0;
    double cy = image_height_ / 2.0;
    // bbox 中心
    ex_ = (detect_x_ + detect_w_ / 2.0) - cx;
    ey_ = (detect_y_ + detect_h_ / 2.0) - cy;

    // ---------- Step 2: 构造相机系 LOS 向量 ----------
    // 与 los_control.cpp 一致：Nt = [ex, ey, focal]
    Eigen::Vector3d Nt(ex_, ey_, focal_length_);

    // ---------- Step 3: 相机系 → 机体系 → NED 系 ----------
    // 机体→NED 旋转矩阵（ZYX 欧拉角，与 los_control.cpp 完全一致）
    Eigen::AngleAxisd Ryaw  (yaw,   Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd Rpitch(pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd Rroll (roll,  Eigen::Vector3d::UnitX());
    Eigen::Matrix3d R_b2n = Ryaw.toRotationMatrix()
                          * Rpitch.toRotationMatrix()
                          * Rroll.toRotationMatrix();

    // LOS 向量：相机系 → 机体系 → NED 系
    Eigen::Vector3d Nt_ned = R_b2n * R_c2b_ * Nt;


    // ---------- Step 4: 计算 LOS 角度 ----------
    double rxy = std::sqrt(Nt_ned(0)*Nt_ned(0) + Nt_ned(1)*Nt_ned(1));
    LOS_angle_v_ = std::atan2(Nt_ned(2), rxy);       // 仰角（NED Down 为正）
    LOS_angle_z_ = std::atan2(Nt_ned(0), Nt_ned(1)); // 方位角（北=0，顺时针）

    // ---------- Step 5: PNG 角度更新 ----------
    // 仅首次进入时初始化，用 LOS 方向作为初始速度方向
    // 注意：不再用 |V|<1 重置，避免加速阶段反复重置导致振荡
    Eigen::Vector3d Vt(vx_, vy_, vz_);
    if (!png_initialized_) {
        d_v_angle_v_      = LOS_angle_v_;
        d_v_angle_z_      = LOS_angle_z_;
        last_LOS_angle_v_ = LOS_angle_v_;
        last_LOS_angle_z_ = LOS_angle_z_;
        last_v_angle_v_   = LOS_angle_v_;  // 用 LOS 而非实测速度，避免起飞残余污染
        last_v_angle_z_   = LOS_angle_z_;
        png_initialized_  = true;
        used_dlos_v_ = 0.0;
        used_dlos_z_ = 0.0;
        RCLCPP_INFO(get_logger(),
            "[PNG 初始化] LOS(v=%.3f z=%.3f) |V|=%.2f",
            LOS_angle_v_, LOS_angle_z_, Vt.norm());
        last_ex_ = ex_;  // No yaw derivative across acquisition or loss recovery.
    }

    double diff_LOS_v = LOS_angle_v_ - last_LOS_angle_v_;
    double diff_LOS_z = LOS_angle_z_ - last_LOS_angle_z_;

    // 处理 ±π 穿越（方位角）
    while (diff_LOS_z >  M_PI) diff_LOS_z -= 2.0 * M_PI;
    while (diff_LOS_z < -M_PI) diff_LOS_z += 2.0 * M_PI;
    while (diff_LOS_v >  M_PI) diff_LOS_v -= 2.0 * M_PI;
    while (diff_LOS_v < -M_PI) diff_LOS_v += 2.0 * M_PI;

    // LOS 变化量足够大才更新（与 los_control.cpp 阈值 0.02 一致）
    if (std::abs(diff_LOS_v) > 0.02 || std::abs(diff_LOS_z) > 0.02) {
        // 用「上一帧的期望速度角」作为 PNG 基准（固定参考，不跟随实测速度）
        // 这样可以避免大机动时实测速度滞后导致的 PNG 发散
        // PNG 核心公式（与 los_control.cpp 完全一致）
        d_v_angle_v_ = kv_ * diff_LOS_v + last_v_angle_v_;
        d_v_angle_z_ = kz_ * diff_LOS_z + last_v_angle_z_;
        // Retain the wrapped increments used by PNG before advancing LOS references.
        used_dlos_v_ = diff_LOS_v;
        used_dlos_z_ = diff_LOS_z;

        // 限制仰角范围（±45°），防止仰角过大导致无人机猛冲高空
        d_v_angle_v_ = std::max(-M_PI/4.0, std::min(M_PI/4.0, d_v_angle_v_));

        // 更新 last_v_angle 为本次期望值（迭代更新）
        last_v_angle_v_ = d_v_angle_v_;
        last_v_angle_z_ = d_v_angle_z_;

        last_LOS_angle_v_ = LOS_angle_v_;
        last_LOS_angle_z_ = LOS_angle_z_;

        RCLCPP_INFO(get_logger(),
            "[PNG] ex=%.1f ey=%.1f | LOS(v=%.3f z=%.3f) | dLOS(v=%.3f z=%.3f) | dV(v=%.3f z=%.3f)",
            ex_, ey_,
            LOS_angle_v_, LOS_angle_z_,
            diff_LOS_v, diff_LOS_z,
            d_v_angle_v_, d_v_angle_z_);
    }

    // ---------- Step 6: 主动偏航跟踪（FOV Lock-On）----------
    // 将水平像素误差 ex 直接转换为偏航角速率指令，强制机头对准目标
    // d_yaw = k1 * ex + k2 * d(ex)/dt   [PD 控制，单位 rad/s]
    // ex > 0 表示目标在图像右侧 → 需要正偏航（顺时针 in NED）
    {
        double d_ex = ex_ - last_ex_;
        d_yaw_   = k1_yaw_ * ex_ + k2_yaw_ * d_ex;
        // 限制偏航角速率到 ±1.0 rad/s（约 57°/s），避免过激旋转
        d_yaw_ = std::max(-1.0, std::min(1.0, d_yaw_));
        last_ex_ = ex_;
    }

    // ---------- Step 7: 发布 /vpng_data 调试话题 ----------
    DataMsg data{};
    data.d_v_angle_v   = (float)d_v_angle_v_;
    data.d_v_angle_z   = (float)d_v_angle_z_;
    data.v_angle_v     = (float)std::atan2(Vt(2), std::sqrt(Vt(1)*Vt(1)+Vt(0)*Vt(0)));
    data.v_angle_z     = (float)std::atan2(Vt(0), Vt(1));
    data.los_angle_v   = (float)LOS_angle_v_;
    data.los_angle_z   = (float)LOS_angle_z_;
    data.diff_los_angle_v = (float)diff_LOS_v;
    data.diff_los_angle_z = (float)diff_LOS_z;
    data.ex            = (float)ex_;
    data.ey            = (float)ey_;
    data.yaw           = (float)yaw;
    data.pitch         = (float)pitch;
    data.roll          = (float)roll;
    data.pixel_u       = (float)(detect_x_ + detect_w_ / 2.0);
    data.pixel_v       = (float)(detect_y_ + detect_h_ / 2.0);
    data.vel_total     = (float)Vt.norm();
    data_pub_->publish(data);

}

// ============================================================
//  状态机：TAKE_OFF
//  位置控制爬升到 standby_altitude_，等待悬停推力收敛
// ============================================================
bool VisionPNG::takeoff_ready()
{
    const double now = get_clock()->now().seconds();
    const double speed = std::sqrt(vx_ * vx_ + vy_ * vy_ + vz_ * vz_);
    const bool ready = local_position_received_ && hover_thrust_ok_ &&
        std::isfinite(local_z_) &&
        std::abs(local_z_ - standby_altitude_) < takeoff_altitude_tolerance_ &&
        (takeoff_speed_tolerance_ == 0.0 ||
         (std::isfinite(speed) && speed <= takeoff_speed_tolerance_));
    if (!ready) {
        settled_since_ = -1.0;
        return false;
    }
    if (settled_since_ < 0.0 || now < settled_since_) settled_since_ = now;
    return now - settled_since_ >= takeoff_settle_time_;
}

void VisionPNG::handle_takeoff()
{
    // 持续发送 arm + offboard 心跳
    arm();
    set_offboard_mode();
    publish_offboard_position_mode();
    publish_position_setpoint(0.0f, 0.0f, (float)standby_altitude_);

    if (takeoff_counter_ < takeoff_frames_) {
        takeoff_counter_++;
        return;
    }

    // Require continuous altitude/speed settling before entering SEARCHING.
    if (takeoff_ready()) {
        RCLCPP_INFO(get_logger(),
            "→ 起飞完成 z=%.2f hover_thrust=%.3f，进入 SEARCHING",
            local_z_, hover_thrust_);
        state_ = VpngState::SEARCHING;
    } else {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
            "[TAKEOFF] z=%.2f/%.2f hover_ok=%s",
            local_z_, (float)standby_altitude_,
            hover_thrust_ok_ ? "YES" : "NO");
    }
}

// ============================================================
//  状态机：SEARCHING
//  已到达待机高度，等待视觉检测到目标
// ============================================================
void VisionPNG::handle_searching()
{
    // Experimental waiting uses position hold, including ongoing altitude correction.
    if (precise_standby_) {
        publish_offboard_position_mode();
        publish_position_setpoint(0.0f, 0.0f, static_cast<float>(standby_altitude_));
    } else {
        publish_offboard_velocity_mode();
        publish_velocity_setpoint(0.0f, 0.0f, 0.0f, 0.0f);
    }

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
        "[SEARCHING] 等待视觉检测目标... detect_w=%d", detect_w_);

    // Only a new detection callback may initialize and enter INTERCEPT.
}

// ============================================================
//  状态机：INTERCEPT
//  基于视觉 PNG 导引冲向目标
// ============================================================
void VisionPNG::handle_intercept()
{
    arm();
    set_offboard_mode();

    if (state_ == VpngState::COAST) {
        publish_offboard_velocity_mode();
        publish_velocity_setpoint(coast_vx_, coast_vy_, coast_vz_, (float)d_yaw_);
        evaluate_hit();
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 100,
            "[COAST] observation_age=%.3f v=(%.1f,%.1f,%.1f)",
            observation_age(), coast_vx_, coast_vy_, coast_vz_);
        return;
    }

    // ---- 正常拦截：计算速度指令 ----
    Eigen::Vector3d Vt(vx_, vy_, vz_);
    double d_v = Vt.norm() + d_gain_;
    d_v = std::min(d_v, speed_cmd_);
    d_v = std::max(d_v, 2.0);

    // 从 PNG 角度合成 NED 速度指令
    float vx_cmd = (float)(std::cos(d_v_angle_v_) * std::sin(d_v_angle_z_) * d_v);  // N
    float vy_cmd = (float)(std::cos(d_v_angle_v_) * std::cos(d_v_angle_z_) * d_v);  // E
    float vz_cmd = (float)(std::sin(d_v_angle_v_) * d_v);                            // D

    // ---- ey 垂直视场补偿（抵消前飞低头导致的相机下倾）----
    // ey < 0：目标在画面上方（图像坐标 y 轴向下，负值=目标偏上）
    //        → UAV 需要上升 → NED vz_cmd 减小（更负）
    //        → vz_ey = k_ey_ * ey_ < 0，叠加后 vz_cmd 变小（上升）✓
    // ey > 0：目标在画面下方 → UAV 需要下降 → vz_ey > 0，vz_cmd 增大（下降）✓
    // 补偿量限幅至 ±3 m/s
    {
        float vz_ey = (float)(k_ey_ * ey_);
        vz_ey = std::max(-3.0f, std::min(3.0f, vz_ey));
        vz_cmd += vz_ey;
    }

    // 缓存速度供惯性续飞使用
    coast_vx_  = vx_cmd;
    coast_vy_  = vy_cmd;
    coast_vz_  = vz_cmd;

    publish_offboard_velocity_mode();
    publish_velocity_setpoint(vx_cmd, vy_cmd, vz_cmd, (float)d_yaw_);

    // GPS only evaluates the hit; it never enters the visual command calculation.
    evaluate_hit();

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 200,
        "[INTERCEPT] v_cmd=(%.2f,%.2f,%.2f) yaw_r=%.3f |V|=%.2f ex=%.0f ey=%.0f",
        vx_cmd, vy_cmd, vz_cmd, d_yaw_, Vt.norm(), ex_, ey_);
}

// ============================================================
//  状态机：TRACK_LOST
//  有效视觉观测超时，制动减速并原地慢旋寻找目标
// ============================================================
void VisionPNG::handle_track_lost()
{
    // 制动：发送零速度，但保持上一次的偏航旋转方向寻找目标
    // 若 d_yaw_ 为 0（已对中），则轻微旋转扫描
    float search_yaw = (std::abs(d_yaw_) > 0.01) ? (float)d_yaw_ : 0.2f;
    publish_offboard_velocity_mode();
    publish_velocity_setpoint(0.0f, 0.0f, 0.0f, search_yaw);

    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
        "[TRACK_LOST] 减速旋转搜索目标... yaw_rate=%.2f", search_yaw);

    // 恢复逻辑由 detect_sub_ 回调处理
}

// ============================================================
//  状态机：DONE
//  拦截完成，悬停
// ============================================================
void VisionPNG::handle_done()
{
    publish_offboard_velocity_mode();
    publish_velocity_setpoint(0.0f, 0.0f, 0.0f, 0.0f);
    RCLCPP_INFO_ONCE(get_logger(), "★★★ 拦截完成，悬停中 ★★★");
}

// GPS hit evaluation is active in both INTERCEPT and COAST, never in guidance.
void VisionPNG::evaluate_hit()
{
    if (!target_pos_ok_) return;
    const float dist = (target_pos_stats_ - self_pos_stats_).norm();
    if (dist < hit_radius_) {
        RCLCPP_WARN(get_logger(), "★ 检测到命中！真实距离=%.3f m (GPS 评估)", dist);
        state_ = VpngState::DONE;
    }
}

void VisionPNG::publish_telemetry()
{
    uav_common_msg::msg::ExperimentTelemetry msg{};
    msg.header.stamp = now();
    msg.header.frame_id = "ned";
    msg.algorithm = "vision_png";
    switch (state_) {
    case VpngState::TAKE_OFF: msg.state = "TAKEOFF"; break;
    case VpngState::SEARCHING: msg.state = "SEARCHING"; break;
    case VpngState::INTERCEPT: msg.state = "INTERCEPT"; break;
    case VpngState::COAST: msg.state = "COAST"; break;
    case VpngState::TRACK_LOST: msg.state = "TRACK_LOST"; break;
    case VpngState::DONE: msg.state = "DONE"; break;
    }
    msg.parameters_yaml = parameters_yaml_;
    msg.guidance_valid = state_ == VpngState::INTERCEPT &&
        observation_age() <= freshness_timeout_ &&
        png_initialized_ && std::isfinite(LOS_angle_v_) && std::isfinite(LOS_angle_z_) &&
        std::isfinite(d_v_angle_v_) && std::isfinite(d_v_angle_z_);
    const double nan = std::numeric_limits<double>::quiet_NaN();
    msg.los_v = msg.guidance_valid ? LOS_angle_v_ : nan;
    msg.los_z = msg.guidance_valid ? LOS_angle_z_ : nan;
    msg.dlos_v = msg.guidance_valid ? used_dlos_v_ : nan;
    msg.dlos_z = msg.guidance_valid ? used_dlos_z_ : nan;
    msg.png_v = msg.guidance_valid ? d_v_angle_v_ : nan;
    msg.png_z = msg.guidance_valid ? d_v_angle_z_ : nan;
    msg.solver_time_ms = nan;
    msg.cost = nan;
    telemetry_pub_->publish(msg);
}

// ============================================================
//  PX4 发布辅助函数
// ============================================================
void VisionPNG::arm()
{
    publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f);
}

void VisionPNG::set_offboard_mode()
{
    publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0f, 6.0f);
}

void VisionPNG::publish_offboard_position_mode()
{
    OffboardControlMode msg{};
    msg.position  = true;
    msg.velocity  = false;
    msg.timestamp = get_clock()->now().nanoseconds() / 1000;
    offboard_pub_->publish(msg);
}

void VisionPNG::publish_offboard_velocity_mode()
{
    OffboardControlMode msg{};
    msg.position  = false;
    msg.velocity  = true;
    msg.timestamp = get_clock()->now().nanoseconds() / 1000;
    offboard_pub_->publish(msg);
}

void VisionPNG::publish_position_setpoint(float x, float y, float z)
{
    TrajectorySetpoint msg{};
    msg.position  = {x, y, z};
    msg.yaw       = static_cast<float>(std::isfinite(standby_yaw_) ? standby_yaw_ : yaw_);
    msg.timestamp = get_clock()->now().nanoseconds() / 1000;
    traj_pub_->publish(msg);
}

void VisionPNG::publish_velocity_setpoint(float vx, float vy, float vz, float yaw_rate)
{
    TrajectorySetpoint msg{};
    msg.position  = {std::nanf(""), std::nanf(""), std::nanf("")};
    msg.velocity  = {vx, vy, vz};
    msg.yaw       = std::nanf("");
    msg.yawspeed  = yaw_rate;
    msg.timestamp = get_clock()->now().nanoseconds() / 1000;
    traj_pub_->publish(msg);
}

void VisionPNG::publish_vehicle_command(uint16_t command, float p1, float p2)
{
    VehicleCommand msg{};
    msg.param1           = p1;
    msg.param2           = p2;
    msg.command          = command;
    msg.target_system    = 2;   // px4_1 的 MAVLink system ID（Instance 1 → DDS key 2）
    msg.target_component = 1;
    msg.source_system    = 1;
    msg.source_component = 1;
    msg.from_external    = true;
    msg.timestamp        = get_clock()->now().nanoseconds() / 1000;
    cmd_pub_->publish(msg);
}

// ============================================================
//  main
// ============================================================
int main(int argc, char* argv[])
{
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VisionPNG>());
    rclcpp::shutdown();
    return 0;
}
