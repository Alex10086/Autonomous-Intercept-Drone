#pragma once

#include <memory>
#include <opencv2/core/mat.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>

#include "LightTrack.h"
#include "uav_common_msg/msg/rect_msg.hpp"
#include "yolo_detector.hpp"

class UavTopicSubscrib : public rclcpp::Node
{
public:
    UavTopicSubscrib();
    ~UavTopicSubscrib() override;

private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr uav_image_sub_;
    rclcpp::Publisher<uav_common_msg::msg::RectMsg>::SharedPtr uav_detect_result_publisher_;
    std::unique_ptr<LightTrack> siam_tracker;
    std::unique_ptr<YoloDetector> yolo_detector_;
    bool enable_tracking_ = true;
    bool show_debug_window_ = true;
    bool tracking_ = false;
    cv::Size tracked_image_size_;

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);
    void cxy_wh_2_rect(const cv::Point& pos, const cv::Point2f& sz, cv::Rect& rect);
};
