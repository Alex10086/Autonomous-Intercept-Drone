//
// Created by verse on 24-10-9.
//
#include <exception>
#include <iostream>
#include <memory>
#include <rclcpp/rclcpp.hpp>


#include "uav_topic_subscrib.hpp"


int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);

    int exit_code = 0;
    try {
        auto detector = std::make_shared<UavTopicSubscrib>();
        rclcpp::executors::SingleThreadedExecutor executor;
        executor.add_node(detector);
        executor.spin();
    } catch (const std::exception& e) {
        std::cerr << "Vision initialization/inference failed: " << e.what() << '\n';
        exit_code = 1;
    }
    rclcpp::shutdown();
    return exit_code;
}
