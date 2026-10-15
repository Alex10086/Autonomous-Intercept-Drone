//
// Created by verse on 24-10-16.
//

#ifndef UAV_TARGET_SIM_H
#define UAV_TARGET_SIM_H

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <stdint.h>
#include <cmath>
#include <random>
#include <optional>
#include <string>
#include <chrono>
 
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>

using namespace std::chrono_literals;
using namespace px4_msgs::msg;

enum class MotionMode {
    CIRCLE,         // 匀速圆周运动
    SINUSOIDAL,     // 正弦机动 (周期性规避)
    RANDOM_WALK,    // 随机游走 (无规律逃逸)
};

class UavTargetControl : public rclcpp::Node
{
public:
    UavTargetControl();

    void arm();

private:
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
    rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr experiment_state_publisher_;
    rclcpp::Subscription<VehicleLocalPosition>::SharedPtr local_position_subscription_;
    rclcpp::Subscription<VehicleStatus>::SharedPtr vehicle_status_subscription_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr target_start_subscription_;

    MotionMode motion_mode_ = MotionMode::CIRCLE;

    // ---- 圆周运动参数 ----
    double radius_ = 5.0;
    double angular_velocity_ = 0.5;
    double initial_phase_ = 0.0;
    double theta_ = 0.0;
    double target_altitude_ = -5.0;
    bool defer_motion_ = false;
    double takeoff_position_tolerance_ = 0.1;
    double takeoff_speed_tolerance_ = 0.2;
    double stability_duration_ = 2.0;
    bool stable_ = false;
    bool ready_latched_ = false;
    bool motion_started_ = false;
    std::optional<double> motion_start_ros_time_;
    using SteadyClock = std::chrono::steady_clock;
    std::optional<SteadyClock::time_point> stable_since_;
    SteadyClock::time_point last_local_position_time_{};
    SteadyClock::time_point last_status_time_{};
    SteadyClock::time_point last_control_time_{};
    SteadyClock::time_point last_arm_request_time_{};
    VehicleLocalPosition local_position_{};
    VehicleStatus vehicle_status_{};
    bool local_position_received_ = false;
    bool status_received_ = false;
    bool control_published_ = false;

    // ---- 通用运动状态 ----
    float pos_x_ = 5.0f;
    float pos_y_ = 0.0f;
    float pos_z_ = -5.0f;
    float vel_x_ = 0.0f;
    float vel_y_ = 0.0f;
    float sim_time_ = 0.0f;
    static constexpr float dt_ = 0.1f;

    // ---- 活动范围约束 ----
    float max_range_ = 10.0f;       // 最大活动半径 (m), 可通过参数调节
    float boundary_center_x_ = 0.0f;
    float boundary_center_y_ = 0.0f;

    // ---- 正弦机动: a_T = A*sin(ωt) ----
    float sin_amplitude_ = 0.5f;   // A = 0.5 m/s²
    float sin_omega_ = 0.5f;       // ω = 0.5 rad/s
    float base_speed_ = 1.0f;      // 前向基础速度 m/s

    // ---- 随机游走: v_T(t+Δt) = v_T(t) + N(0, σ_v) ----
    float sigma_v_ = 0.2f;
    float max_speed_ = 3.0f;
    std::mt19937 rng_{std::random_device{}()};
    std::normal_distribution<float> normal_dist_{0.0f, 1.0f};

    uint64_t offboard_setpoint_counter_ = 0;

    void publish_offboard_control_mode();
    void publish_trajectory_setpoint(float x, float y, float z);
    void control_tick();
    void update_stability();
    void publish_experiment_state();
    void target_start_callback(const std_msgs::msg::Float64::SharedPtr msg);
    void local_position_callback(const VehicleLocalPosition::SharedPtr msg);
    void vehicle_status_callback(const VehicleStatus::SharedPtr msg);
    bool local_position_valid() const;
    bool armed_offboard() const;
    void execute_motion();
    void move_in_circle();
    void move_sinusoidal();
    void move_random_walk();
    void apply_boundary();

    void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);
    const char *motion_mode_name() const;
};

#endif //UAV_TARGET_SIM_H
