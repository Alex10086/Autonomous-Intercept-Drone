#include <gz/msgs/clock.pb.h>
#include <gz/transport/Node.hh>
#include <rclcpp/rclcpp.hpp>
#include <uav_common_msg/msg/simulation_clock.hpp>

#include <memory>
#include <stdexcept>
#include <string>

class SimulationClockBridge : public rclcpp::Node {
public:
    SimulationClockBridge() : rclcpp::Node("simulation_clock_bridge")
    {
        topic_ = declare_parameter<std::string>("gz_clock_topic", "/world/grass_world/clock");
        publisher_ = create_publisher<uav_common_msg::msg::SimulationClock>(
            "/simulation/clock_pairs", rclcpp::SensorDataQoS().keep_last(100));
        if (!transport_.Subscribe(topic_, &SimulationClockBridge::receive, this)) {
            throw std::runtime_error("Cannot subscribe to Gazebo clock: " + topic_);
        }
        RCLCPP_INFO(get_logger(), "Gazebo sim/system clock pairs: %s", topic_.c_str());
    }

    ~SimulationClockBridge() override { transport_.Unsubscribe(topic_); }

private:
    void receive(const gz::msgs::Clock &clock)
    {
        uav_common_msg::msg::SimulationClock message;
        message.header.stamp = now();
        message.sim_time.sec = clock.sim().sec();
        message.sim_time.nanosec = clock.sim().nsec();
        message.system_time.sec = clock.system().sec();
        message.system_time.nanosec = clock.system().nsec();
        publisher_->publish(message);
    }

    std::string topic_;
    rclcpp::Publisher<uav_common_msg::msg::SimulationClock>::SharedPtr publisher_;
    gz::transport::Node transport_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimulationClockBridge>());
    rclcpp::shutdown();
    return 0;
}
