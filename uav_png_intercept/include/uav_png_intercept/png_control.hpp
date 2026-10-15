#ifndef PNG_CONTROL_HPP
#define PNG_CONTROL_HPP

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/sensor_gps.hpp>

#include <GeographicLib/LocalCartesian.hpp>
#include <Eigen/Dense>
#include <chrono>
#include <cmath>

#include <uav_common_msg/msg/experiment_telemetry.hpp>
#include <string>
#include <limits>
#include <std_msgs/msg/bool.hpp>

using namespace px4_msgs::msg;
using namespace std::chrono_literals;

enum class InterceptState { INIT, TAKEOFF, INTERCEPT, DONE };

class PngInterceptor : public rclcpp::Node
{
public:
    PngInterceptor();
    ~PngInterceptor() = default;

private:
    // -------- 主控制循环 --------
    void control_loop();

    void handle_init();
    void handle_takeoff();
    bool takeoff_ready();
    void handle_intercept();

    // -------- 比例导引（与 IBVS 代码相同的角度更新公式）--------
    Eigen::Vector3f compute_png_velocity(const Eigen::Vector3f& target_pos_ned);

    // -------- PX4 发布辅助 --------
    void arm();
    void set_offboard_mode();
    void publish_offboard_velocity_mode();
    void publish_offboard_position_mode();
    void publish_velocity_setpoint(float vx, float vy, float vz, float yaw);
    void publish_position_setpoint(float x, float y, float z, float yaw);
    void publish_vehicle_command(uint16_t command, float p1 = 0.0f, float p2 = 0.0f);

    // -------- 发布者 --------
    rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_pub_;
    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr  traj_pub_;
    rclcpp::Publisher<VehicleCommand>::SharedPtr      cmd_pub_;

    // -------- 订阅者 --------
    rclcpp::Subscription<SensorGps>::SharedPtr           self_gps_sub_;   // px4_1 绝对位置
    rclcpp::Subscription<SensorGps>::SharedPtr           target_gps_sub_; // px4_2 绝对位置
    rclcpp::Subscription<VehicleLocalPosition>::SharedPtr self_vel_sub_;   // px4_1 速度
    rclcpp::Subscription<VehicleStatus>::SharedPtr        status_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr experiment_start_sub_;

    rclcpp::TimerBase::SharedPtr timer_;
    void publish_telemetry();
    rclcpp::TimerBase::SharedPtr telemetry_timer_;
    rclcpp::Publisher<uav_common_msg::msg::ExperimentTelemetry>::SharedPtr telemetry_pub_;
    std::string parameters_yaml_;
    float los_v_{0.0f}, los_z_{0.0f};
    float used_dlos_v_{0.0f}, used_dlos_z_{0.0f};
    bool guidance_valid_{false};

    // -------- 状态 --------
    InterceptState state_{InterceptState::INIT};
    bool offboard_active_{false};
    bool status_received_{false};
    int  nav_state_{-1};     // PX4 nav_state 原始值（14=OFFBOARD）
    int  arming_state_{-1};  // PX4 arming_state 原始值（2=ARMED）
    int  init_counter_{0};

    // 自身位置（NED，相对于自身第一个GPS点）
    Eigen::Vector3f self_pos_{0.0f, 0.0f, 0.0f};
    Eigen::Vector3f self_vel_{0.0f, 0.0f, 0.0f};
    bool self_pos_ok_{false};
    float local_z_{0.0f};             // 本地高度 (VehicleLocalPosition.z)

    // 目标位置（NED，转换到自身坐标系后）
    Eigen::Vector3f target_pos_{0.0f, 0.0f, 0.0f};
    bool target_pos_ok_{false};

    // 坐标系：以自身第一个GPS为原点的 LocalCartesian
    GeographicLib::LocalCartesian lc_;
    bool lc_initialized_{false};

    // -------- PNG 状态（角度更新法，与 IBVS 保持一致）--------
    bool  png_initialized_{false};
    float last_LOS_angle_v_{0.0f};  // LOS 仰角（上一帧）
    float last_LOS_angle_z_{0.0f};  // LOS 方位角（上一帧）
    float d_v_angle_v_{0.0f};       // 期望速度仰角
    float d_v_angle_z_{0.0f};       // 期望速度方位角
    float last_v_angle_v_{0.0f};    // 上一帧速度仰角
    float last_v_angle_z_{0.0f};    // 上一帧速度方位角
    float target_yaw_{0.0f};        // 目标方位角（用于机头指向目标）

    // -------- 导引参数 --------
    float N_{4.0f};               // N_png
    float intercept_speed_{8.0f}; // speed_cmd, m/s
    double takeoff_alt_{-5.0};  // standby_altitude, NED
    bool require_experiment_start_{false};
    bool experiment_start_{false};
    double takeoff_altitude_tolerance_{0.5};
    double takeoff_speed_tolerance_{0.0};  // Zero disables the speed check.
    double takeoff_settle_time_{0.0};
    double standby_yaw_{std::numeric_limits<double>::quiet_NaN()};
    double settled_since_{-1.0};
    bool local_position_received_{false};
    float hit_radius_{0.5f};      // hit_radius, m

};

#endif // PNG_CONTROL_HPP
