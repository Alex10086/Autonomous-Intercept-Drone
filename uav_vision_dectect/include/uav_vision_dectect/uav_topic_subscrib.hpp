#pragma once
#include <cstdint>

#include <memory>
#include <opencv2/core/mat.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>

#include "LightTrack.h"
#include "uav_common_msg/msg/rect_msg.hpp"
#include "yolo_detector.hpp"
#include "frame_capture.hpp"

#include "tracking_validation.hpp"
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
    std::unique_ptr<vision_diagnostics::FrameCapture> frame_capture_;
    bool enable_tracking_ = true;
    bool show_debug_window_ = true;
    bool tracking_ = false;
    int yolo_refresh_interval_ = 5;
    float acquisition_threshold_ = 0.8f;
    double tracking_validation_iou_ = 0.1;
    int tracker_frames_since_yolo_ = 0;
    cv::Size tracked_image_size_;
    vision_tracking::ContinuityBudget continuity_{0.35, 2.0};
    cv::Rect confirmed_yolo_box_{-1, -1, -1, -1};
    cv::Rect last_prediction_box_{-1, -1, -1, -1};
    bool publication_expired_ = false;
    std::uint64_t frame_count_ = 0;
    std::uint64_t yolo_frame_count_ = 0;
    std::uint64_t acquisition_count_ = 0;
    std::uint64_t tracker_frame_count_ = 0;
    std::uint64_t tracker_loss_count_ = 0;
    std::uint64_t global_acquired_count_ = 0;
    std::uint64_t matched_refresh_count_ = 0;
    std::uint64_t low_score_support_count_ = 0;
    std::uint64_t probation_enter_count_ = 0;
    std::uint64_t probation_recovered_count_ = 0;
    std::uint64_t local_recovery_count_ = 0;
    std::uint64_t publication_expired_count_ = 0;
    std::uint64_t recovery_expired_count_ = 0;

    void reset_tracking();

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);
    void track_frame(cv::Mat& frame, cv::Rect& result_rect, float& result_confidence,
                     std::string& status_text, cv::Scalar& color);
    void cxy_wh_2_rect(const cv::Point& pos, const cv::Point2f& sz, cv::Rect& rect);
};
