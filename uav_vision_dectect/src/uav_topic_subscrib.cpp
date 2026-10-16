#include "uav_topic_subscrib.hpp"
#include "tracking_validation.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>

namespace {
double stamp_seconds(const builtin_interfaces::msg::Time& stamp)
{
    return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
}
}

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
    // Detection candidates and permission to replace a tracker are separate.
    config.confThreshold = static_cast<float>(
        declare_parameter<double>("confidence_threshold", 0.4));
    config.iouThreshold = 0.45f;
    const double acquisition_threshold = declare_parameter<double>("acquisition_threshold", 0.8);
    if (!std::isfinite(acquisition_threshold) ||
        acquisition_threshold < 0.0 || acquisition_threshold > 1.0) {
        throw std::invalid_argument("acquisition_threshold must be finite and in [0,1]");
    }
    acquisition_threshold_ = static_cast<float>(acquisition_threshold);
    tracking_validation_iou_ = declare_parameter<double>("tracking_validation_iou", 0.1);
    if (!std::isfinite(tracking_validation_iou_) ||
        tracking_validation_iou_ <= 0.0 || tracking_validation_iou_ > 1.0) {
        throw std::invalid_argument("tracking_validation_iou must be finite and in (0,1]");
    }
    const double validation_hold = declare_parameter<double>("tracking_validation_hold", 0.35);
    const double recovery_timeout = declare_parameter<double>("tracking_recovery_timeout", 2.0);
    continuity_ = vision_tracking::ContinuityBudget(validation_hold, recovery_timeout);

    RCLCPP_INFO(get_logger(), "Loading vision models with %s execution requested", config.useCuda ? "CUDA" : "CPU");
    yolo_detector_ = std::make_unique<YoloDetector>(config);
    if (enable_tracking_) {
        siam_tracker = std::make_unique<LightTrack>(init_model.c_str(), update_model.c_str(), config.useCuda);
    }
    RCLCPP_INFO(get_logger(),
                "Vision sessions initialized; detection >= %.3f, acquisition > %.3f, "
                "YOLO refresh=%d frames, tracking validation IoU >= %.3f (spatial support only), "
                "validation_hold=%.3fs recovery_timeout=%.3fs (steady clock)",
                config.confThreshold, acquisition_threshold_, yolo_refresh_interval_,
                tracking_validation_iou_, validation_hold, recovery_timeout);
    if (enable_tracking_ && yolo_refresh_interval_ == 0) {
        RCLCPP_WARN(get_logger(),
            "YOLO refresh=0: frame-count refresh disabled only; "
            "steady-clock authorization budgets still force YOLO support validation");
    }

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
    state_callback_group_ = create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive, false);
    rclcpp::SubscriptionOptions state_options;
    state_options.callback_group = state_callback_group_;
    const auto state_qos = rclcpp::SensorDataQoS().keep_last(1);
    clock_sub_ = create_subscription<uav_common_msg::msg::SimulationClock>(
        "/simulation/clock_pairs", state_qos,
        [this](const uav_common_msg::msg::SimulationClock& msg) {
            const double received = now().seconds();
            std::lock_guard<std::mutex> lock(state_mutex_);
            rotation_history_.clock(stamp_seconds(msg.sim_time),
                stamp_seconds(msg.system_time), received);
        }, state_options);
    timesync_sub_ = create_subscription<px4_msgs::msg::TimesyncStatus>(
        "/px4_1/fmu/out/timesync_status", state_qos,
        [this](const px4_msgs::msg::TimesyncStatus& msg) {
            if (msg.source_protocol != msg.SOURCE_PROTOCOL_DDS) return;
            const double received = now().seconds();
            std::lock_guard<std::mutex> lock(state_mutex_);
            rotation_history_.timesync(msg.timestamp, msg.remote_timestamp,
                msg.estimated_offset, msg.round_trip_time, received);
        }, state_options);
    odometry_sub_ = create_subscription<px4_msgs::msg::VehicleOdometry>(
        "/px4_1/fmu/out/vehicle_odometry", state_qos,
        [this](const px4_msgs::msg::VehicleOdometry& msg) {
            if (msg.pose_frame != msg.POSE_FRAME_NED) return;
            const double received = now().seconds();
            const Eigen::Quaterniond attitude(msg.q[0], msg.q[1], msg.q[2], msg.q[3]);
            std::lock_guard<std::mutex> lock(state_mutex_);
            rotation_history_.odometry(msg.timestamp_sample, attitude, msg.reset_counter, received);
        }, state_options);
    camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
        "/camera/camera_info", state_qos,
        [this](const sensor_msgs::msg::CameraInfo& msg) {
            std::lock_guard<std::mutex> lock(state_mutex_);
            camera_intrinsics_.k = msg.k;
            camera_intrinsics_.width = msg.width <= static_cast<unsigned>(std::numeric_limits<int>::max()) ?
                static_cast<int>(msg.width) : 0;
            camera_intrinsics_.height = msg.height <= static_cast<unsigned>(std::numeric_limits<int>::max()) ?
                static_cast<int>(msg.height) : 0;
            camera_info_received_ = true;
        }, state_options);
    state_executor_.add_callback_group(state_callback_group_, get_node_base_interface());
    // Start last: no callback observes a partially constructed node.
    state_thread_ = std::thread([this]() {
        // A stop flag also covers destruction before the thread enters spin.
        while (!state_stopping_.load(std::memory_order_acquire) && rclcpp::ok())
            state_executor_.spin_once(std::chrono::milliseconds(50));
    });
}

UavTopicSubscrib::~UavTopicSubscrib()
{
    state_stopping_.store(true, std::memory_order_release);
    state_executor_.cancel();
    if (state_thread_.joinable()) state_thread_.join();
    RCLCPP_INFO(get_logger(),
        "[VISION_DIAG] summary frames=%llu yolo_frames=%llu acquisitions=%llu "
        "tracker_frames=%llu tracker_losses=%llu",
        static_cast<unsigned long long>(frame_count_),
        static_cast<unsigned long long>(yolo_frame_count_),
        static_cast<unsigned long long>(acquisition_count_),
        static_cast<unsigned long long>(tracker_frame_count_),
        static_cast<unsigned long long>(tracker_loss_count_));
    RCLCPP_INFO(get_logger(),
        "[VISION_DIAG] continuity global_acquired=%llu matched_refresh=%llu "
        "low_score_support=%llu probation_enter=%llu probation_recovered=%llu "
        "local_recovery=%llu publication_expired=%llu recovery_expired=%llu",
        static_cast<unsigned long long>(global_acquired_count_),
        static_cast<unsigned long long>(matched_refresh_count_),
        static_cast<unsigned long long>(low_score_support_count_),
        static_cast<unsigned long long>(probation_enter_count_),
        static_cast<unsigned long long>(probation_recovered_count_),
        static_cast<unsigned long long>(local_recovery_count_),
        static_cast<unsigned long long>(publication_expired_count_),
        static_cast<unsigned long long>(recovery_expired_count_));
    if (show_debug_window_) {
        cv::destroyWindow("Result");
    }
}

void UavTopicSubscrib::reset_tracking()
{
    tracking_ = false;
    continuity_.reset();
    confirmed_yolo_box_ = cv::Rect(-1, -1, -1, -1);
    confirmed_rotation_ = FrameRotation{};
    last_prediction_box_ = cv::Rect(-1, -1, -1, -1);
    tracked_image_size_ = cv::Size();
    tracker_frames_since_yolo_ = 0;
    publication_expired_ = false;
}

UavTopicSubscrib::FrameRotation UavTopicSubscrib::frame_rotation(double image_sim)
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    FrameRotation result;
    result.attitude = rotation_history_.sample(image_sim, now().seconds());
    result.camera = camera_intrinsics_;
    result.camera_available = camera_info_received_;
    return result;
}

cv::Rect UavTopicSubscrib::rotation_prior(const FrameRotation& current,
    const cv::Size& image_size, bool& compensated, const char*& reason) const
{
    compensated = false;
    const auto invalid = cv::Rect(-1, -1, -1, -1);
    if (confirmed_rotation_.attitude.generation != current.attitude.generation) {
        reason = "generation_changed";
        return invalid;
    }
    if (!confirmed_rotation_.camera_available || !current.camera_available) {
        reason = "no_camera_info";
        return confirmed_yolo_box_;
    }
    if (!(confirmed_rotation_.camera == current.camera) ||
        current.camera.width != image_size.width || current.camera.height != image_size.height) {
        reason = "calibration_changed_or_size_mismatch";
        return invalid;
    }
    if (!confirmed_rotation_.attitude.valid || !current.attitude.valid) {
        reason = "no_capture_attitude_bracket";
        return confirmed_yolo_box_;
    }
    cv::Rect projected;
    if (!vision_tracking::rotate_camera_prior(confirmed_yolo_box_, current.camera,
        confirmed_rotation_.attitude.b2n, current.attitude.b2n, projected)) {
        reason = "projection_invalid_or_outside";
        return invalid;
    }
    compensated = true;
    reason = "capture_rotation";
    return projected;
}

void UavTopicSubscrib::image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    ++frame_count_;
    using Budget = vision_tracking::ContinuityBudget;
    const auto frame_time = Budget::Clock::now();
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (const cv_bridge::Exception& e) {
        RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
        reset_tracking();
        return;
    }
    cv::Mat& frame = cv_ptr->image;
    if (frame.empty()) {
        reset_tracking();
        return;
    }
    // LightTrack wraps the packed buffer using dimensions recorded at init.
    const bool size_reset = tracking_ && frame.size() != tracked_image_size_;
    if (size_reset) reset_tracking();

    cv::Rect result_rect(-1, -1, -1, -1);
    float result_confidence = std::numeric_limits<float>::quiet_NaN();
    std::string status_text;
    cv::Scalar color(0, 255, 0);
    const double start = cv::getTickCount();
    cv::Rect prediction_rect(-1, -1, -1, -1);
    float prediction_confidence = std::numeric_limits<float>::quiet_NaN();
    bool acquired = false;
    std::vector<Detection> candidates;
    bool detection_ran = false;
    int selected_candidate = -1;
    vision_tracking::DetectionDecision decision;
    cv::Rect prior_rect = confirmed_yolo_box_;
    FrameRotation current_rotation;
    bool motion_compensated = false;
    const char* prior_reason = "not_queried";
    std::uint64_t prior_generation = confirmed_rotation_.attitude.generation;
    // Transition-only diagnostics retain both the prediction and the bounded
    // anchor. A timeout before inference explicitly has no current prediction.
    const auto log_transition = [&](const char* outcome, double age) {
        int maximum = -1;
        for (std::size_t index = 0; index < candidates.size(); ++index) {
            if (maximum < 0 || candidates[index].conf > candidates[maximum].conf)
                maximum = static_cast<int>(index);
        }
        const cv::Rect maximum_box = maximum >= 0 ? candidates[maximum].box :
            cv::Rect(-1, -1, -1, -1);
        RCLCPP_INFO(get_logger(),
            "[VISION_DIAG] outcome=%s frame=%llu support_age=%.6f "
            "prediction_current=%s prediction_bbox=%d,%d,%d,%d "
            "last_prediction_bbox=%d,%d,%d,%d anchor_bbox=%d,%d,%d,%d "
            "detection_ran=%s candidates=%zu candidate_max=%.6f candidate_max_bbox=%d,%d,%d,%d "
            "matched=%s support_candidate=%d support_iou=%.4f local=%s "
            "motion_compensated=%s prior_bbox=%d,%d,%d,%d prior_generation=%llu "
            "frame_generation=%llu prior_reason=%s",
            outcome, static_cast<unsigned long long>(frame_count_), age,
            prediction_rect.width > 0 ? "true" : "false",
            prediction_rect.x, prediction_rect.y, prediction_rect.width, prediction_rect.height,
            last_prediction_box_.x, last_prediction_box_.y,
            last_prediction_box_.width, last_prediction_box_.height,
            confirmed_yolo_box_.x, confirmed_yolo_box_.y,
            confirmed_yolo_box_.width, confirmed_yolo_box_.height,
            detection_ran ? "true" : "false",
            candidates.size(), maximum >= 0 ? candidates[maximum].conf :
                std::numeric_limits<float>::quiet_NaN(),
            maximum_box.x, maximum_box.y, maximum_box.width, maximum_box.height,
            decision.action != vision_tracking::DetectionAction::Invalid ? "true" : "false",
            decision.candidate_index, decision.support_iou,
            decision.local_recovery ? "true" : "false",
            motion_compensated ? "true" : "false",
            prior_rect.x, prior_rect.y, prior_rect.width, prior_rect.height,
            static_cast<unsigned long long>(prior_generation),
            static_cast<unsigned long long>(current_rotation.attitude.generation), prior_reason);
    };
    const double initial_age = continuity_.active() ? continuity_.support_age(frame_time) : 0.0;
    const auto expiry = continuity_.begin_frame(frame_time);
    bool defer_acquisition = size_reset || expiry != Budget::Expiry::None;
    if (expiry != Budget::Expiry::None) {
        ++recovery_expired_count_;
        ++tracker_loss_count_;
        log_transition(expiry == Budget::Expiry::NonMonotonicClock ?
            "recovery_expired_clock_reset" : "recovery_expired", initial_age);
        reset_tracking();
        status_text = "Track Lost (Recovery Expired)";
    }
    const bool had_tracker = tracking_;
    const bool validate = tracking_ &&
        (continuity_.validation_due(frame_time) ||
         (yolo_refresh_interval_ > 0 && tracker_frames_since_yolo_ >= yolo_refresh_interval_));
    if (tracking_) {
        track_frame(frame, prediction_rect, prediction_confidence, status_text, color);
        if (!tracking_) defer_acquisition = true;
        result_rect = prediction_rect;
        result_confidence = prediction_confidence;
    }
    const bool need_detection = !defer_acquisition &&
        (!enable_tracking_ || !had_tracker || validate);
    if (need_detection) {
        ++yolo_frame_count_;
        candidates = yolo_detector_->detect(frame);
        detection_ran = true;
        tracker_frames_since_yolo_ = 0;
        // Inference gives the independent state executor time to collect clock
        // and pose brackets; never replace capture-time attitude with receipt pose.
        current_rotation = frame_rotation(stamp_seconds(msg->header.stamp));
        if (had_tracker && continuity_.in_probation()) {
            prior_rect = rotation_prior(current_rotation, frame.size(),
                motion_compensated, prior_reason);
        }
        decision = had_tracker ?
            vision_tracking::select_tracking_detection(candidates,
                cv::Rect(0, 0, frame.cols, frame.rows), prediction_rect,
                prior_rect, continuity_, frame_time,
                tracking_validation_iou_, acquisition_threshold_) :
            vision_tracking::select_detection(candidates,
                cv::Rect(0, 0, frame.cols, frame.rows), nullptr,
                tracking_validation_iou_, acquisition_threshold_);
        if (had_tracker && continuity_.in_probation()) {
            RCLCPP_INFO(get_logger(),
                "[VISION_DIAG] outcome=probation_prior frame=%llu motion_compensated=%s "
                "prior_bbox=%d,%d,%d,%d prior_generation=%llu frame_generation=%llu "
                "prior_reason=%s frame_epoch=%.9f attitude_before=%.9f attitude_after=%.9f "
                "local=%s support_candidate=%d support_iou=%.4f",
                static_cast<unsigned long long>(frame_count_), motion_compensated ? "true" : "false",
                prior_rect.x, prior_rect.y, prior_rect.width, prior_rect.height,
                static_cast<unsigned long long>(prior_generation),
                static_cast<unsigned long long>(current_rotation.attitude.generation), prior_reason,
                current_rotation.attitude.epoch, current_rotation.attitude.before,
                current_rotation.attitude.after, decision.local_recovery ? "true" : "false",
                decision.candidate_index, decision.support_iou);
        }
        if (had_tracker) {
            const auto evidence_time = Budget::Clock::now();
            const double evidence_age = continuity_.support_age(evidence_time);
            if (continuity_.expire(evidence_time) != Budget::Expiry::None) {
                ++recovery_expired_count_;
                ++tracker_loss_count_;
                log_transition("recovery_expired", evidence_age);
                reset_tracking();
                decision.action = vision_tracking::DetectionAction::Invalid;
            }
        }
        const Detection* best = decision.candidate_index >= 0 ?
            &candidates[decision.candidate_index] : nullptr;
        if (decision.action != vision_tracking::DetectionAction::Invalid &&
            (!had_tracker || tracking_)) {
            const bool recovered = had_tracker && continuity_.in_probation();
            const double supported_age = had_tracker ? continuity_.support_age(frame_time) : 0.0;
            if (had_tracker) {
                continuity_.support(frame_time);
            } else if (enable_tracking_) {
                continuity_.acquire(frame_time);
            }
            if (recovered) {
                ++probation_recovered_count_;
                log_transition("probation_recovered", supported_age);
            }
            confirmed_yolo_box_ = decision.box;
            confirmed_rotation_ = current_rotation;
            publication_expired_ = false;
            if (decision.action == vision_tracking::DetectionAction::Acquire) {
                selected_candidate = decision.candidate_index;
                acquired = true;
                ++acquisition_count_;
                if (!had_tracker) ++global_acquired_count_;
                else if (decision.local_recovery) ++local_recovery_count_;
                else ++matched_refresh_count_;
                const char* outcome = !had_tracker ? "global_acquired" :
                    (decision.local_recovery ? "local_recovery" : "matched_refresh");
                if (!had_tracker || decision.local_recovery) {
                    log_transition(outcome, supported_age);
                } else {
                    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                        "[VISION_DIAG] source=YOLO outcome=matched_refresh frame=%llu "
                        "selected=%d confidence=%.6f support_iou=%.4f",
                        static_cast<unsigned long long>(frame_count_), selected_candidate,
                        best->conf, decision.support_iou);
                }
                if (enable_tracking_) {
                    const auto& box = decision.box;
                    Bbox tracker_box{static_cast<float>(box.x), static_cast<float>(box.y),
                                     static_cast<float>(box.x + box.width),
                                     static_cast<float>(box.y + box.height)};
                    siam_tracker->init(frame.data, tracker_box, frame.rows, frame.cols);
                    tracked_image_size_ = frame.size();
                    tracking_ = true;
                    last_prediction_box_ = decision.box;
                }
                result_rect = decision.box;
                result_confidence = best->conf;
                status_text = !had_tracker ? "Global Track Init" :
                    (decision.local_recovery ? "Local Track Recovery" : "Matched Track Refresh");
                color = cv::Scalar(255, 0, 0);
            } else {
                ++low_score_support_count_;
                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                    "[VISION_DIAG] source=LightTrack outcome=low_score_support frame=%llu "
                    "support_candidate=%d support_iou=%.4f detection_confidence=%.6f",
                    static_cast<unsigned long long>(frame_count_), decision.candidate_index,
                    decision.support_iou, best->conf);
                status_text = "Tracking (YOLO Support)";
            }
        } else if (had_tracker && tracking_) {
            if (!continuity_.in_probation()) {
                ++probation_enter_count_;
                log_transition("probation_enter", continuity_.support_age(frame_time));
            }
            continuity_.miss();
            status_text = "Tracking (Probation)";
            color = cv::Scalar(0, 255, 255);
        } else if (!had_tracker) {
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                "[VISION_DIAG] source=YOLO outcome=%s frame=%llu candidates=%zu "
                "selected=%d confidence=%.6f required_gt=%.6f",
                !best ? "no_candidate" :
                    (best->conf > acquisition_threshold_ ? "rejected_box" : "below_acquisition"),
                static_cast<unsigned long long>(frame_count_), candidates.size(),
                decision.candidate_index, best ? best->conf : 0.0f, acquisition_threshold_);
        }
    }

    // Inference itself can take time. Never publish beyond the wall-time
    // permission, even on tracker-only frames or a slow successful refresh.
    // Revoking this local output must not overwrite LightTrack's internal box.
    const auto publish_time = Budget::Clock::now();
    const double final_age = continuity_.active() ? continuity_.support_age(publish_time) : 0.0;
    const auto final_expiry = continuity_.expire(publish_time);
    if (final_expiry != Budget::Expiry::None) {
        ++recovery_expired_count_;
        ++tracker_loss_count_;
        log_transition("recovery_expired", final_age);
        reset_tracking();
    }
    if (enable_tracking_ &&
        (!tracking_ || !continuity_.publication_allowed(publish_time))) {
        if (tracking_ && !publication_expired_) {
            ++publication_expired_count_;
            log_transition("publication_expired", final_age);
            publication_expired_ = true;
        }
        result_rect = cv::Rect(-1, -1, -1, -1);
        result_confidence = std::numeric_limits<float>::quiet_NaN();
        acquired = false;
        selected_candidate = -1;
        status_text = tracking_ ? "Track Output Expired (Recovering)" : "Track Lost";
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
            std::move(candidates), selected_candidate, show_debug_window_,
            acquired && acquisition_count_ == 1);
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
        last_prediction_box_ = safe_rect;
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
    reset_tracking();
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
