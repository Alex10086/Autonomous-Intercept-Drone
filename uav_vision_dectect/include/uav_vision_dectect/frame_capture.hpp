#pragma once

#include "yolo_detector.hpp"

#include <array>
#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace vision_diagnostics {

// Observe on the image-processing thread, after publishing and before any drawing.
// An empty directory disables capture at the caller: do not construct this object.
class FrameCapture {
public:
    explicit FrameCapture(const std::string& directory);
    ~FrameCapture() noexcept;

    FrameCapture(const FrameCapture&) = delete;
    FrameCapture& operator=(const FrameCapture&) = delete;

    // Images retain shared cv::Mat ownership. Set copy_image only when the caller
    // will modify those pixels (for example, to draw a debug window).
    void observe(std::uint64_t frame_number, std::int32_t stamp_sec,
        std::uint32_t stamp_nanosec, const cv::Mat& raw_bgr,
        const cv::Rect& selected, float confidence, int source,
        std::vector<Detection> candidates, int selected_candidate,
        bool copy_image = false);

private:
    struct Frame {
        std::uint64_t number = 0;
        std::int32_t stamp_sec = 0;
        std::uint32_t stamp_nanosec = 0;
        cv::Mat image;
        cv::Rect selected;
        float confidence = 0.0f;
        int source = 0;
        std::vector<Detection> candidates;
        int selected_candidate = -1;
    };

    struct Clip {
        std::array<std::shared_ptr<Frame>, 5> frames{};
        std::size_t count = 0;
        std::size_t pre_frames = 0;
        std::size_t post_frames = 0;
        double jump_pixels = 0.0;
        double threshold_pixels = 0.0;
        double stamp_delta_seconds = 0.0;
    };

    void enqueue_active(bool wait_for_space);
    void writer_loop() noexcept;
    void write_clip(const Clip& clip);
    void report_error(const char* context, const char* message) noexcept;

    std::filesystem::path directory_;
    std::array<std::shared_ptr<Frame>, 2> history_{};
    std::size_t history_next_ = 0;
    std::size_t history_count_ = 0;
    Clip active_clip_;
    bool active_ = false;
    std::size_t event_count_ = 0;

    // Exactly two waiting clips, in addition to the writer's current clip.
    std::array<Clip, 2> pending_{};
    std::size_t pending_head_ = 0;
    std::size_t pending_count_ = 0;
    std::mutex mutex_;
    std::condition_variable work_ready_;
    std::condition_variable space_ready_;
    bool stopping_ = false;
    std::atomic<bool> writer_failed_{false};
    std::atomic<std::uint64_t> error_count_{0};
    std::thread writer_;
};

}  // namespace vision_diagnostics
