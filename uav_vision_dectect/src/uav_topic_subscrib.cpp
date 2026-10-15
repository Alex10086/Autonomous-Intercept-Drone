#include "uav_topic_subscrib.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <algorithm>
#include <functional>
#include <limits>
#include <string>

UavTopicSubscrib::UavTopicSubscrib() : Node("uav_vision_dectect")
{
    const auto model_dir = ament_index_cpp::get_package_share_directory("uav_vision_dectect") + "/model/";
    DetectorConfig config;
    config.modelPath = declare_parameter<std::string>("yolo_model_path", model_dir + "yolov5/GDUT_UAV.onnx");
    const auto init_model = declare_parameter<std::string>("tracker_init_model_path", model_dir + "light_track/lighttrack_init.onnx");
    const auto update_model = declare_parameter<std::string>("tracker_update_model_path", model_dir + "light_track/lighttrack_update.onnx");
    config.useCuda = declare_parameter<bool>("use_cuda", true);
    enable_tracking_ = declare_parameter<bool>("enable_tracking", true);
    show_debug_window_ = declare_parameter<bool>("show_debug_window", true);
    // Apply one confidence threshold from YOLO post-processing through acquisition.
    config.confThreshold = static_cast<float>(
        declare_parameter<double>("confidence_threshold", 0.4));
    config.iouThreshold = 0.45f;

    RCLCPP_INFO(get_logger(), "Loading vision models with %s execution requested", config.useCuda ? "CUDA" : "CPU");
    yolo_detector_ = std::make_unique<YoloDetector>(config);
    if (enable_tracking_) {
        siam_tracker = std::make_unique<LightTrack>(init_model.c_str(), update_model.c_str(), config.useCuda);
    }
    RCLCPP_INFO(get_logger(), "Vision sessions initialized; confidence threshold >= %.3f",
                config.confThreshold);

    uav_detect_result_publisher_ = create_publisher<uav_common_msg::msg::RectMsg>("/camera_detect_result", 10);
    // Keep only the most recent camera frame while inference is busy. Sensor QoS
    // also accepts best-effort camera publishers.
    auto image_qos = rclcpp::SensorDataQoS();
    image_qos.keep_last(1);
    uav_image_sub_ = create_subscription<sensor_msgs::msg::Image>(
        "/camera/image", image_qos,
        std::bind(&UavTopicSubscrib::image_callback, this, std::placeholders::_1));
    if (show_debug_window_) {
        cv::namedWindow("Result", cv::WINDOW_NORMAL);
        cv::resizeWindow("Result", 640, 640);
    }
}

UavTopicSubscrib::~UavTopicSubscrib()
{
    if (show_debug_window_) {
        cv::destroyWindow("Result");
    }
}

void UavTopicSubscrib::image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (const cv_bridge::Exception& e) {
        RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }
    cv::Mat& frame = cv_ptr->image;
    if (frame.empty()) {
        return;
    }
    // LightTrack wraps the packed buffer using dimensions recorded at init.
    if (tracking_ && frame.size() != tracked_image_size_) {
        tracking_ = false;
    }

    cv::Rect result_rect(-1, -1, -1, -1);
    float result_confidence = std::numeric_limits<float>::quiet_NaN();
    std::string status_text;
    cv::Scalar color(0, 255, 0);
    const double start = cv::getTickCount();
    const bool need_detection = !enable_tracking_ || !tracking_;
    if (need_detection) {
        const auto results = yolo_detector_->detect(frame);
        const Detection* best = nullptr;
        for (const auto& det : results) {
            if (!best || det.conf > best->conf) {
                best = &det;
            }
        }
        if (best) {
            const cv::Rect safe_rect = best->box & cv::Rect(0, 0, frame.cols, frame.rows);
            if (safe_rect.width > 0 && safe_rect.height > 0 && safe_rect.area() >= 10) {
                if (enable_tracking_) {
                    Bbox box{static_cast<float>(safe_rect.x), static_cast<float>(safe_rect.y),
                             static_cast<float>(safe_rect.x + safe_rect.width),
                             static_cast<float>(safe_rect.y + safe_rect.height)};
                    siam_tracker->init(frame.data, box, frame.rows, frame.cols);
                    tracked_image_size_ = frame.size();
                    tracking_ = true;
                    status_text = "Global Track Init";
                    color = cv::Scalar(255, 0, 0);
                } else {
                    status_text = "YOLO Detect (Score: " + std::to_string(best->conf).substr(0, 4) + ")";
                    color = cv::Scalar(0, 0, 255);
                }
                result_rect = safe_rect;
                result_confidence = best->conf;
            }
        }
    } else {
        siam_tracker->track(frame.data);
        cv::Rect rect;
        cxy_wh_2_rect(siam_tracker->target_pos, siam_tracker->target_sz, rect);
        const cv::Rect safe_rect = rect & cv::Rect(0, 0, frame.cols, frame.rows);
        if (safe_rect.area() > 0 && !siam_tracker->target_pos_change()) {
            result_rect = safe_rect;
            result_confidence = siam_tracker->track_score;
            status_text = "Tracking";
        } else {
            tracking_ = false;
            status_text = "Track Lost";
        }
    }

    // Publish only inference-backed results, using the camera capture stamp and
    // original-image pixels. No racing thread or freshly stamped stale boxes.
    uav_common_msg::msg::RectMsg result;
    result.header = msg->header;
    result.x = result_rect.x;
    result.y = result_rect.y;
    result.width = result_rect.width;
    result.height = result_rect.height;
    result.depth = 0;
    result.confidence = result_confidence;
    uav_detect_result_publisher_->publish(result);

    if (show_debug_window_) {
        if (result_rect.width > 0 && result_rect.height > 0) {
            cv::rectangle(frame, result_rect, color, 2);
        }
        if (!status_text.empty()) {
            cv::putText(frame, status_text, cv::Point(20, 40), cv::FONT_HERSHEY_SIMPLEX, 0.8, color, 2);
        }
        const double fps = cv::getTickFrequency() / (cv::getTickCount() - start);
        cv::putText(frame, "FPS: " + std::to_string(fps).substr(0, 5), cv::Point(20, 80),
                    cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 255, 0), 2);
        cv::imshow("Result", frame);
        cv::waitKey(1);
    }
}

void UavTopicSubscrib::cxy_wh_2_rect(const cv::Point& pos, const cv::Point2f& sz, cv::Rect& rect)
{
    rect.x = pos.x - static_cast<int>(sz.x / 2);
    rect.y = pos.y - static_cast<int>(sz.y / 2);
    rect.width = static_cast<int>(sz.x);
    rect.height = static_cast<int>(sz.y);
}
