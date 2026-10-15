#include "uav_target_sim.hpp"

#include <functional>
#include <chrono>
#include <iomanip>
#include <limits>
#include <locale>
#include <sstream>
#include <stdexcept>
using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;


UavTargetControl::UavTargetControl() : Node("uav_target_sim")
{
    this->declare_parameter<std::string>("motion_mode", "circle");
    max_range_ = static_cast<float>(this->declare_parameter<double>("max_range", 10.0));
    radius_ = this->declare_parameter<double>("circle_radius", 5.0);
    angular_velocity_ = this->declare_parameter<double>("circle_angular_velocity", 0.5);
    initial_phase_ = this->declare_parameter<double>("circle_initial_phase", 0.0);
    target_altitude_ = this->declare_parameter<double>("target_altitude", -5.0);
    defer_motion_ = this->declare_parameter<bool>("defer_motion", false);
    takeoff_position_tolerance_ =
        this->declare_parameter<double>("takeoff_position_tolerance", 0.1);
    takeoff_speed_tolerance_ =
        this->declare_parameter<double>("takeoff_speed_tolerance", 0.2);
    stability_duration_ = this->declare_parameter<double>("stability_duration", 2.0);

    if (!std::isfinite(radius_) || radius_ <= 0.0 ||
        radius_ > std::numeric_limits<float>::max() ||
        !std::isfinite(angular_velocity_) || !std::isfinite(initial_phase_) ||
        !std::isfinite(target_altitude_) ||
        std::abs(target_altitude_) > std::numeric_limits<float>::max() ||
        !std::isfinite(takeoff_position_tolerance_) || takeoff_position_tolerance_ <= 0.0 ||
        !std::isfinite(takeoff_speed_tolerance_) || takeoff_speed_tolerance_ < 0.0 ||
        !std::isfinite(stability_duration_) || stability_duration_ < 0.0) {
        throw std::invalid_argument("Invalid circle, altitude, or takeoff stability parameters");
    }
    const std::string mode_str = this->get_parameter("motion_mode").as_string();

    if (mode_str == "circle")           motion_mode_ = MotionMode::CIRCLE;
    else if (mode_str == "sinusoidal")  motion_mode_ = MotionMode::SINUSOIDAL;
    else if (mode_str == "random_walk") motion_mode_ = MotionMode::RANDOM_WALK;
    else {
        RCLCPP_WARN(this->get_logger(),
            "Unknown motion_mode '%s', defaulting to 'circle'", mode_str.c_str());
        motion_mode_ = MotionMode::CIRCLE;
    }

    RCLCPP_INFO(this->get_logger(), "=== Motion mode: %s | max_range: %.1fm ===",
        mode_str.c_str(), max_range_);

    // --- 发布者配置 ---
    offboard_control_mode_publisher_ =
        this->create_publisher<OffboardControlMode>("/px4_2/fmu/in/offboard_control_mode", 10);
    trajectory_setpoint_publisher_ =
        this->create_publisher<TrajectorySetpoint>("/px4_2/fmu/in/trajectory_setpoint", 10);
    vehicle_command_publisher_ =
        this->create_publisher<VehicleCommand>("/px4_2/fmu/in/vehicle_command", 10);

    experiment_state_publisher_ = this->create_publisher<std_msgs::msg::String>(
        "/target/experiment_state", rclcpp::QoS(10).reliable());

    const auto qos = rclcpp::SensorDataQoS();
    local_position_subscription_ = this->create_subscription<VehicleLocalPosition>(
        "/px4_2/fmu/out/vehicle_local_position", qos,
        std::bind(&UavTargetControl::local_position_callback, this, std::placeholders::_1));
    vehicle_status_subscription_ = this->create_subscription<VehicleStatus>(
        "/px4_2/fmu/out/vehicle_status_v1", qos,
        std::bind(&UavTargetControl::vehicle_status_callback, this, std::placeholders::_1));
    target_start_subscription_ = this->create_subscription<std_msgs::msg::Float64>(
        "/experiment/target_start", rclcpp::QoS(10).reliable(),
        std::bind(&UavTargetControl::target_start_callback, this, std::placeholders::_1));

    theta_ = initial_phase_;
    // PX4 local setpoints and feedback are NED relative to this vehicle's EKF
    // origin, not Gazebo world ENU or the interceptor's local frame.
    pos_x_ = static_cast<float>(radius_ * std::cos(initial_phase_));
    pos_y_ = static_cast<float>(radius_ * std::sin(initial_phase_));
    pos_z_ = static_cast<float>(target_altitude_);
    if (motion_mode_ == MotionMode::SINUSOIDAL) {
        vel_x_ = base_speed_;
    }
    timer_ = this->create_wall_timer(100ms, std::bind(&UavTargetControl::control_tick, this));
}

void UavTargetControl::control_tick()
{
    // Never stop the offboard heartbeat or hover setpoints while waiting.
    publish_offboard_control_mode();
    update_stability();
    if (offboard_setpoint_counter_ < 15) {
        ++offboard_setpoint_counter_;
        publish_trajectory_setpoint(pos_x_, pos_y_, pos_z_);
    } else if (offboard_setpoint_counter_ == 15) {
        RCLCPP_INFO(this->get_logger(), "Switching to Offboard and Arming...");
        publish_trajectory_setpoint(pos_x_, pos_y_, pos_z_);
        publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
        arm();
        last_arm_request_time_ = SteadyClock::now();
        ++offboard_setpoint_counter_;
    } else {
        if (defer_motion_ && !armed_offboard() &&
            SteadyClock::now() - last_arm_request_time_ >= 1s) {
            // Retry ordinary requests until PX4 accepts; never bypass preflight checks.
            if (!status_received_ ||
                vehicle_status_.nav_state != VehicleStatus::NAVIGATION_STATE_OFFBOARD) {
                publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
            }
            if (!status_received_ ||
                vehicle_status_.arming_state != VehicleStatus::ARMING_STATE_ARMED) {
                arm();
            }
            last_arm_request_time_ = SteadyClock::now();
        }
        const double now = this->get_clock()->now().seconds();
        if (!defer_motion_ && !motion_start_ros_time_) {
            // Match the legacy first circular step (+0.05 rad at 10 Hz).
            motion_start_ros_time_ = now - dt_;
        }
        if (motion_start_ros_time_ && now >= *motion_start_ros_time_) {
            motion_started_ = true;
            stable_ = false;
            stable_since_.reset();
            execute_motion();
        } else {
            publish_trajectory_setpoint(pos_x_, pos_y_, pos_z_);
        }
    }
    publish_experiment_state();
}

bool UavTargetControl::local_position_valid() const
{
    return local_position_received_ && local_position_.xy_valid && local_position_.z_valid &&
        local_position_.v_xy_valid && local_position_.v_z_valid &&
        std::isfinite(local_position_.x) && std::isfinite(local_position_.y) &&
        std::isfinite(local_position_.z) && std::isfinite(local_position_.vx) &&
        std::isfinite(local_position_.vy) && std::isfinite(local_position_.vz) &&
        SteadyClock::now() - last_local_position_time_ <= 1s;
}

bool UavTargetControl::armed_offboard() const
{
    return status_received_ && SteadyClock::now() - last_status_time_ <= 1s &&
        vehicle_status_.arming_state == VehicleStatus::ARMING_STATE_ARMED &&
        vehicle_status_.nav_state == VehicleStatus::NAVIGATION_STATE_OFFBOARD;
}

void UavTargetControl::update_stability()
{
    const auto now = SteadyClock::now();
    const double dx = local_position_.x - radius_ * std::cos(initial_phase_);
    const double dy = local_position_.y - radius_ * std::sin(initial_phase_);
    const double dz = local_position_.z - target_altitude_;
    const double position_error = std::hypot(std::hypot(dx, dy), dz);
    const double speed = std::hypot(
        std::hypot(local_position_.vx, local_position_.vy), local_position_.vz);
    const bool converged = !motion_started_ && local_position_valid() && armed_offboard() &&
        control_published_ && now - last_control_time_ <= 500ms &&
        position_error <= takeoff_position_tolerance_ && speed <= takeoff_speed_tolerance_;
    if (!converged) {
        stable_since_.reset();
        stable_ = false;
        return;
    }
    if (!stable_since_) {
        stable_since_ = now;
    }
    stable_ = std::chrono::duration<double>(now - *stable_since_).count() >= stability_duration_;
}

void UavTargetControl::local_position_callback(const VehicleLocalPosition::SharedPtr msg)
{
    // An interruption between feedback samples must restart the settling interval.
    if (local_position_received_ && SteadyClock::now() - last_local_position_time_ > 1s) {
        stable_since_.reset();
    }
    local_position_ = *msg;
    local_position_received_ = true;
    last_local_position_time_ = SteadyClock::now();
    update_stability();
}

void UavTargetControl::vehicle_status_callback(const VehicleStatus::SharedPtr msg)
{
    if (status_received_ && SteadyClock::now() - last_status_time_ > 1s) {
        stable_since_.reset();
    }
    vehicle_status_ = *msg;
    status_received_ = true;
    last_status_time_ = SteadyClock::now();
    update_stability();
}

void UavTargetControl::target_start_callback(const std_msgs::msg::Float64::SharedPtr msg)
{
    if (!defer_motion_ || motion_start_ros_time_) {
        return;  // Accepted schedules are immutable, including repeated commands.
    }
    update_stability();
    if (!stable_ || !std::isfinite(msg->data) ||
        msg->data <= this->get_clock()->now().seconds()) {
        RCLCPP_WARN(this->get_logger(), "Ignoring target start: requires stable hover and future epoch");
        return;
    }
    motion_start_ros_time_ = msg->data;
    RCLCPP_INFO(this->get_logger(), "Target motion scheduled for ROS epoch %.9f", msg->data);
}

const char *UavTargetControl::motion_mode_name() const
{
    switch (motion_mode_) {
        case MotionMode::CIRCLE: return "circle";
        case MotionMode::SINUSOIDAL: return "sinusoidal";
        case MotionMode::RANDOM_WALK: return "random_walk";
    }
    return "circle";
}

void UavTargetControl::publish_experiment_state()
{
    const char *state = motion_started_ ? "MOVING" :
        motion_start_ros_time_ ? "WAITING_START" : stable_ ? "READY" : "TAKEOFF";
    std::ostringstream json;
    json.imbue(std::locale::classic());
    json << std::setprecision(17);
    const auto number = [&json](double value) {
        if (std::isfinite(value)) {
            json << value;
        } else {
            json << "null";
        }
    };
    json << "{\"timestamp\":";
    number(this->get_clock()->now().seconds());
    json << ",\"state\":\"" << state << "\",\"stable\":" << (stable_ ? "true" : "false")
         << ",\"motion_mode\":\"" << motion_mode_name() << "\",\"circle_radius\":";
    number(radius_);
    json << ",\"circle_angular_velocity\":";
    number(angular_velocity_);
    json << ",\"circle_initial_phase\":";
    number(initial_phase_);
    json << ",\"motion_start_ros_time\":";
    if (motion_start_ros_time_) {
        number(*motion_start_ros_time_);
    } else {
        json << "null";
    }
    json << ",\"phase\":";
    number(theta_);
    json << ",\"target_altitude\":";
    number(target_altitude_);
    json << ",\"takeoff_position_tolerance\":";
    number(takeoff_position_tolerance_);
    json << ",\"takeoff_speed_tolerance\":";
    number(takeoff_speed_tolerance_);
    json << ",\"stability_duration\":";
    number(stability_duration_);
    json << ",\"defer_motion\":" << (defer_motion_ ? "true" : "false")
         << ",\"coordinate_frame\":\"LOCAL_NED\",\"armed_offboard\":"
         << (armed_offboard() ? "true" : "false")
         << ",\"local_position_valid\":" << (local_position_valid() ? "true" : "false")
         << ",\"local_position\":";
    if (local_position_valid()) {
        json << '[';
        number(local_position_.x);
        json << ',';
        number(local_position_.y);
        json << ',';
        number(local_position_.z);
        json << "],\"local_velocity\":[";
        number(local_position_.vx);
        json << ',';
        number(local_position_.vy);
        json << ',';
        number(local_position_.vz);
        json << ']';
    } else {
        json << "null,\"local_velocity\":null";
    }
    json << '}';
    std_msgs::msg::String msg;
    msg.data = json.str();
    experiment_state_publisher_->publish(msg);
}

void UavTargetControl::arm() {
    publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
    RCLCPP_INFO(this->get_logger(), "Arm command sent");
}

void UavTargetControl::publish_offboard_control_mode() {
    OffboardControlMode msg{};
    msg.position = true;
    msg.velocity = false;
    msg.acceleration = false;
    msg.attitude = false;
    msg.body_rate = false;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    if (control_published_ && SteadyClock::now() - last_control_time_ > 500ms) {
        stable_since_.reset();
        stable_ = false;
    }
    offboard_control_mode_publisher_->publish(msg);
    last_control_time_ = SteadyClock::now();
    control_published_ = true;
}

void UavTargetControl::publish_trajectory_setpoint(float x, float y, float z) {
    TrajectorySetpoint msg{};
    msg.position = {x, y, z};
    msg.yaw = 0.0;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    trajectory_setpoint_publisher_->publish(msg);
}

// ============================================================
//  边界约束：超出 max_range_ 时施加回复力
//  目标靠近边界会被平滑拉回，保证不飞出拦截机视野
// ============================================================
void UavTargetControl::apply_boundary() {
    float dx = pos_x_ - boundary_center_x_;
    float dy = pos_y_ - boundary_center_y_;
    float dist = std::sqrt(dx * dx + dy * dy);

    if (dist > max_range_ * 0.8f && dist > 0.01f) {
        // 从80%处开始施加渐进回复力
        float overshoot = (dist - max_range_ * 0.8f) / (max_range_ * 0.2f);
        float restore = 3.0f * overshoot * overshoot;
        float nx = dx / dist;
        float ny = dy / dist;
        vel_x_ -= restore * nx * dt_;
        vel_y_ -= restore * ny * dt_;
    }
}

// ============================================================
//  运动模式调度
// ============================================================
void UavTargetControl::execute_motion() {
    switch (motion_mode_) {
        case MotionMode::CIRCLE:       move_in_circle();    break;
        case MotionMode::SINUSOIDAL:   move_sinusoidal();   break;
        case MotionMode::RANDOM_WALK:  move_random_walk();  break;
    }
}

// ============================================================
//  1. 匀速圆周运动 (原有, 天然有界无需约束)
// ============================================================
void UavTargetControl::move_in_circle() {
    theta_ = initial_phase_ + angular_velocity_ *
        (this->get_clock()->now().seconds() - *motion_start_ros_time_);
    const float x = static_cast<float>(radius_ * std::cos(theta_));
    const float y = static_cast<float>(radius_ * std::sin(theta_));
    TrajectorySetpoint msg{};
    msg.position = {x, y, static_cast<float>(target_altitude_)};
    msg.yaw = static_cast<float>(std::remainder(theta_, 2.0 * M_PI));
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    trajectory_setpoint_publisher_->publish(msg);
}

// ============================================================
//  2. 正弦机动: a_T = A*sin(ωt), A=0.5m/s², ω=0.5rad/s
//     前向匀速 + 侧向正弦加速度 + 边界约束
// ============================================================
void UavTargetControl::move_sinusoidal() {
    sim_time_ += dt_;

    float a_y = sin_amplitude_ * std::sin(sin_omega_ * sim_time_);
    vel_y_ += a_y * dt_;

    apply_boundary();

    pos_x_ += vel_x_ * dt_;
    pos_y_ += vel_y_ * dt_;

    TrajectorySetpoint msg{};
    msg.position = {pos_x_, pos_y_, pos_z_};
    msg.yaw = std::atan2(vel_y_, vel_x_);
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    trajectory_setpoint_publisher_->publish(msg);
}

// ============================================================
//  3. 随机游走: v_T(t+Δt) = v_T(t) + N(0, σ_v), σ_v=0.2m/s
//     无规律逃逸 + 边界约束
// ============================================================
void UavTargetControl::move_random_walk() {
    vel_x_ += sigma_v_ * normal_dist_(rng_);
    vel_y_ += sigma_v_ * normal_dist_(rng_);

    float speed = std::sqrt(vel_x_ * vel_x_ + vel_y_ * vel_y_);
    if (speed > max_speed_) {
        vel_x_ *= max_speed_ / speed;
        vel_y_ *= max_speed_ / speed;
    }

    apply_boundary();

    pos_x_ += vel_x_ * dt_;
    pos_y_ += vel_y_ * dt_;

    TrajectorySetpoint msg{};
    msg.position = {pos_x_, pos_y_, pos_z_};
    msg.yaw = std::atan2(vel_y_, vel_x_);
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    trajectory_setpoint_publisher_->publish(msg);
}

// ============================================================
//  Vehicle command & GPS callback
// ============================================================
void UavTargetControl::publish_vehicle_command(uint16_t command, float param1, float param2)
{
    VehicleCommand msg{};
    msg.param1 = param1;
    msg.param2 = param2;
    msg.command = command;
    msg.target_system = 3;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    vehicle_command_publisher_->publish(msg);
}

