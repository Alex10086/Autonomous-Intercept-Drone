#include "uav_topic_subscrib.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <algorithm>
#include <functional>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>

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
    yolo_refresh_interval_ = declare_parameter<int>("yolo_refresh_interval", 5);
    const auto capture_directory = declare_parameter<std::string>("diagnostic_capture_dir", "");
    if (!capture_directory.empty()) {
        frame_capture_ = std::make_unique<vision_diagnostics::FrameCapture>(capture_directory);
        RCLCPP_INFO(get_logger(), "Anomaly snapshots enabled: %s", capture_directory.c_str());
    }
    const auto image_reliability = declare_parameter<std::string>("image_reliability", "reliable");
    if (image_reliability != "reliable" && image_reliability != "best_effort") {
        throw std::invalid_argument("image_reliability must be reliable or best_effort");
    }
    if (yolo_refresh_interval_ < 0) {
        throw std::invalid_argument("yolo_refresh_interval must be nonnegative");
    }
    // Apply one confidence threshold from YOLO post-processing through acquisition.
    config.confThreshold = static_cast<float>(
        declare_parameter<double>("confidence_threshold", 0.4));
    config.iouThreshold = 0.45f;

    RCLCPP_INFO(get_logger(), "Loading vision models with %s execution requested", config.useCuda ? "CUDA" : "CPU");
    yolo_detector_ = std::make_unique<YoloDetector>(config);
    if (enable_tracking_) {
        siam_tracker = std::make_unique<LightTrack>(init_model.c_str(), update_model.c_str(), config.useCuda);
    }
    RCLCPP_INFO(get_logger(),
                "Vision sessions initialized; confidence threshold >= %.3f, YOLO refresh=%d frames",
                config.confThreshold, yolo_refresh_interval_);

    uav_detect_result_publisher_ = create_publisher<uav_common_msg::msg::RectMsg>("/camera_detect_result", 10);
    // Keep one recent frame. Best-effort delivery showed large-image gaps in SITL.
    // Reliable matches Gazebo's offer; best_effort is explicit for BE publishers.
    auto image_qos = rclcpp::SensorDataQoS();
    image_qos.keep_last(1);
    if (image_reliability == "reliable") image_qos.reliable();
    RCLCPP_INFO(get_logger(), "Camera QoS: reliability=%s depth=1", image_reliability.c_str());
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
    RCLCPP_INFO(get_logger(),
        "[VISION_DIAG] summary frames=%llu yolo_frames=%llu acquisitions=%llu "
        "tracker_frames=%llu tracker_losses=%llu",
        static_cast<unsigned long long>(frame_count_),
        static_cast<unsigned long long>(yolo_frame_count_),
        static_cast<unsigned long long>(acquisition_count_),
        static_cast<unsigned long long>(tracker_frame_count_),
        static_cast<unsigned long long>(tracker_loss_count_));
    if (show_debug_window_) {
        cv::destroyWindow("Result");
    }
}

void UavTopicSubscrib::image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    ++frame_count_;
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
    const bool refresh_yolo = enable_tracking_ && tracking_ &&
        yolo_refresh_interval_ > 0 &&
        tracker_frames_since_yolo_ >= yolo_refresh_interval_;
    const bool need_detection = !enable_tracking_ || !tracking_ || refresh_yolo;
    bool acquired = false;
    std::vector<Detection> candidates;
    int selected_candidate = -1;
    if (need_detection) {
        ++yolo_frame_count_;
        candidates = yolo_detector_->detect(frame);
        const Detection* best = nullptr;
        for (const auto& det : candidates) {
            if (!best || det.conf > best->conf) {
                best = &det;
            }
        }
        if (!best) {
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                "[VISION_DIAG] source=YOLO outcome=no_candidate frame=%llu "
                "refresh=%s yolo_frames=%llu acquisitions=%llu",
                static_cast<unsigned long long>(frame_count_),
                refresh_yolo ? "true" : "false",
                static_cast<unsigned long long>(yolo_frame_count_),
                static_cast<unsigned long long>(acquisition_count_));
        } else {
            const cv::Rect safe_rect = best->box & cv::Rect(0, 0, frame.cols, frame.rows);
            if (safe_rect.width > 0 && safe_rect.height > 0 && safe_rect.area() >= 10) {
                ++acquisition_count_;
                acquired = true;
                if (frame_capture_) {
                    selected_candidate = static_cast<int>(best - candidates.data());
                }
                tracker_frames_since_yolo_ = 0;
                RCLCPP_INFO(get_logger(),
                    "[VISION_DIAG] source=YOLO outcome=acquired frame=%llu refresh=%s "
                    "candidates=%zu confidence=%.4f bbox=%d,%d,%d,%d acquisition=%llu",
                    static_cast<unsigned long long>(frame_count_),
                    refresh_yolo ? "true" : "false", candidates.size(), best->conf,
                    safe_rect.x, safe_rect.y, safe_rect.width, safe_rect.height,
                    static_cast<unsigned long long>(acquisition_count_));
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
            } else {
                RCLCPP_WARN(get_logger(),
                    "[VISION_DIAG] source=YOLO outcome=rejected_box frame=%llu "
                    "confidence=%.4f raw_bbox=%d,%d,%d,%d clipped_bbox=%d,%d,%d,%d",
                    static_cast<unsigned long long>(frame_count_), best->conf,
                    best->box.x, best->box.y, best->box.width, best->box.height,
                    safe_rect.x, safe_rect.y, safe_rect.width, safe_rect.height);
            }
        }
        if (refresh_yolo && !acquired) {
            track_frame(frame, result_rect, result_confidence, status_text, color);
        }
    } else {
        track_frame(frame, result_rect, result_confidence, status_text, color);
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
    result.source = result_rect.width <= 0 ? result.UNKNOWN :
        (acquired ? result.YOLO : result.LIGHTTRACK);
    uav_detect_result_publisher_->publish(result);

    if (frame_capture_) {
        frame_capture_->observe(frame_count_, msg->header.stamp.sec, msg->header.stamp.nanosec,
            frame, result_rect, result_confidence, result.source,
            std::move(candidates), selected_candidate, show_debug_window_);
    }

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

void UavTopicSubscrib::track_frame(
    cv::Mat& frame, cv::Rect& result_rect, float& result_confidence,
    std::string& status_text, cv::Scalar& color)
{
    ++tracker_frame_count_;
    ++tracker_frames_since_yolo_;
    siam_tracker->track(frame.data);
    cv::Rect rect;
    cxy_wh_2_rect(siam_tracker->target_pos, siam_tracker->target_sz, rect);
    const cv::Rect safe_rect = rect & cv::Rect(0, 0, frame.cols, frame.rows);
    if (safe_rect.area() > 0) {
        result_rect = safe_rect;
        result_confidence = siam_tracker->track_score;
        status_text = "Tracking";
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
            "[VISION_DIAG] source=LightTrack outcome=tracking frame=%llu "
            "score=%.4f raw_score=%.4f bbox=%d,%d,%d,%d raw_argmax=%d selected=%d",
            static_cast<unsigned long long>(frame_count_), siam_tracker->track_score,
            siam_tracker->raw_track_score,
            safe_rect.x, safe_rect.y, safe_rect.width, safe_rect.height,
            siam_tracker->cls_score_position, siam_tracker->selected_score_position);
        return;
    }
    ++tracker_loss_count_;
    RCLCPP_WARN(get_logger(),
        "[VISION_DIAG] source=LightTrack outcome=lost frame=%llu "
        "reason=bbox_outside_image score=%.4f raw_bbox=%d,%d,%d,%d "
        "raw_argmax=%d loss=%llu",
        static_cast<unsigned long long>(frame_count_), siam_tracker->track_score,
        rect.x, rect.y, rect.width, rect.height, siam_tracker->cls_score_position,
        static_cast<unsigned long long>(tracker_loss_count_));
    tracking_ = false;
    status_text = "Track Lost";
    color = cv::Scalar(0, 255, 255);
}

void UavTopicSubscrib::cxy_wh_2_rect(const cv::Point& pos, const cv::Point2f& sz, cv::Rect& rect)
{
    rect.x = pos.x - static_cast<int>(sz.x / 2);
    rect.y = pos.y - static_cast<int>(sz.y / 2);
    rect.width = static_cast<int>(sz.x);
    rect.height = static_cast<int>(sz.y);
}
