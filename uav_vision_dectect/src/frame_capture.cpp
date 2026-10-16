#include "frame_capture.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <locale>
#include <stdexcept>
#include <system_error>
#include <utility>

namespace vision_diagnostics {
namespace {
constexpr std::size_t max_events = 16;
constexpr std::int64_t max_stamp_gap_ns = 150000000;

std::int64_t nanoseconds(std::int32_t sec, std::uint32_t ns)
{
    return static_cast<std::int64_t>(sec) * 1000000000LL + ns;
}

bool valid_box(const cv::Rect& box)
{
    return box.width > 0 && box.height > 0;
}

cv::Rect clipped_box(const cv::Rect& box, const cv::Size& size)
{
    const auto left = std::clamp<std::int64_t>(box.x, 0, size.width);
    const auto top = std::clamp<std::int64_t>(box.y, 0, size.height);
    const auto right = std::clamp<std::int64_t>(
        static_cast<std::int64_t>(box.x) + box.width, 0, size.width);
    const auto bottom = std::clamp<std::int64_t>(
        static_cast<std::int64_t>(box.y) + box.height, 0, size.height);
    if (right <= left || bottom <= top) return {};
    return cv::Rect(static_cast<int>(left), static_cast<int>(top),
        static_cast<int>(right - left), static_cast<int>(bottom - top));
}

std::ofstream csv_file(const std::filesystem::path& path)
{
    std::ofstream stream;
    stream.exceptions(std::ios::failbit | std::ios::badbit);
    stream.imbue(std::locale::classic());
    stream.open(path);
    stream << std::setprecision(std::numeric_limits<double>::max_digits10);
    return stream;
}

void validate_directory(const std::filesystem::path& directory)
{
    std::filesystem::create_directories(directory);
    if (!std::filesystem::is_directory(directory)) {
        throw std::runtime_error("not a directory");
    }

    // Prove that both event directories and files can be created before starting
    // the writer. create_directory, rather than a guessed filename, claims it.
    std::filesystem::path probe;
    for (std::uint64_t suffix = 0;; ++suffix) {
        probe = directory / (".frame_capture_probe_" + std::to_string(suffix));
        std::error_code error;
        if (std::filesystem::create_directory(probe, error)) break;
        if (error && error != std::errc::file_exists) {
            throw std::filesystem::filesystem_error("create capture probe", probe, error);
        }
    }
    try {
        auto stream = csv_file(probe / "probe");
        stream << "frame capture write probe\n";
        stream.close();
        std::filesystem::remove_all(probe);
    } catch (...) {
        std::error_code ignored;
        std::filesystem::remove_all(probe, ignored);
        throw;
    }
}
}  // namespace

FrameCapture::FrameCapture(const std::string& directory) : directory_(directory)
{
    if (directory.empty()) {
        throw std::invalid_argument("FrameCapture requires a nonempty capture directory");
    }
    try {
        validate_directory(directory_);
    } catch (const std::exception& error) {
        throw std::runtime_error("FrameCapture directory '" + directory_.string()
            + "' is unusable: " + error.what());
    }
    writer_ = std::thread(&FrameCapture::writer_loop, this);
}

FrameCapture::~FrameCapture() noexcept
{
    // Shutdown may wait for queue space: unlike observe(), it must preserve a
    // partial event even when two complete clips are already waiting.
    if (active_) enqueue_active(true);
    {
        std::lock_guard<std::mutex> lock(mutex_);
        stopping_ = true;
    }
    work_ready_.notify_one();
    if (writer_.joinable()) writer_.join();
}

void FrameCapture::observe(std::uint64_t frame_number, std::int32_t stamp_sec,
    std::uint32_t stamp_nanosec, const cv::Mat& raw_bgr,
    const cv::Rect& selected, float confidence, int source,
    std::vector<Detection> candidates, int selected_candidate, bool copy_image)
{
    if (event_count_ >= max_events && !active_) return;

    const Frame* previous = history_count_ == 0 ? nullptr
        : history_[(history_next_ + 1) % history_.size()].get();
    std::int64_t delta_ns = 0;
    double threshold = 0.0;
    double jump = 0.0;
    bool trigger = false;
    if (!active_ && previous && !raw_bgr.empty()
        && frame_number > previous->number && frame_number - previous->number == 1
        && raw_bgr.size() == previous->image.size()
        && valid_box(selected) && valid_box(previous->selected)) {
        delta_ns = nanoseconds(stamp_sec, stamp_nanosec)
            - nanoseconds(previous->stamp_sec, previous->stamp_nanosec);
        if (delta_ns > 0 && delta_ns <= max_stamp_gap_ns) {
            threshold = 0.2 * std::hypot(raw_bgr.cols, raw_bgr.rows);
            jump = std::hypot(static_cast<double>(selected.x) + selected.width * 0.5
                    - previous->selected.x - previous->selected.width * 0.5,
                static_cast<double>(selected.y) + selected.height * 0.5
                    - previous->selected.y - previous->selected.height * 0.5);
            trigger = jump > threshold;
        }
    }

    if (trigger) {
        active_clip_ = Clip{};
        active_clip_.pre_frames = history_count_;
        active_clip_.jump_pixels = jump;
        active_clip_.threshold_pixels = threshold;
        active_clip_.stamp_delta_seconds = delta_ns / 1000000000.0;
        const std::size_t first = history_count_ == history_.size() ? history_next_ : 0;
        for (std::size_t i = 0; i < history_count_; ++i) {
            active_clip_.frames[active_clip_.count++] = history_[(first + i) % history_.size()];
        }
        active_ = true;
        ++event_count_;
    }

    // Reuse a history record unless a clip still owns it. The Mat always shares
    // the original pixels except when the caller explicitly requests a copy.
    auto& slot = history_[history_next_];
    auto frame = slot && slot.use_count() == 1 ? std::move(slot) : std::make_shared<Frame>();
    frame->number = frame_number;
    frame->stamp_sec = stamp_sec;
    frame->stamp_nanosec = stamp_nanosec;
    frame->image = copy_image ? raw_bgr.clone() : raw_bgr;
    frame->selected = selected;
    frame->confidence = confidence;
    frame->source = source;
    frame->candidates = std::move(candidates);
    frame->selected_candidate = selected_candidate;

    if (active_) {
        active_clip_.frames[active_clip_.count++] = frame;
        if (!trigger) ++active_clip_.post_frames;
        if (active_clip_.post_frames == 2) enqueue_active(false);
    }
    slot = std::move(frame);
    history_next_ = (history_next_ + 1) % history_.size();
    history_count_ = std::min(history_count_ + 1, history_.size());
}

void FrameCapture::enqueue_active(bool wait_for_space)
{
    std::unique_lock<std::mutex> lock(mutex_);
    if (wait_for_space) {
        space_ready_.wait(lock, [this] {
            return pending_count_ < pending_.size() || writer_failed_.load();
        });
    }
    if (writer_failed_.load() || pending_count_ == pending_.size()) {
        const bool failed = writer_failed_.load();
        active_clip_ = Clip{};
        active_ = false;
        lock.unlock();
        report_error("capture event skipped", failed ? "writer is unavailable"
            : "two-clip waiting queue is full; inference was not blocked");
        return;
    }
    pending_[(pending_head_ + pending_count_) % pending_.size()] = std::move(active_clip_);
    ++pending_count_;
    active_clip_ = Clip{};
    active_ = false;
    lock.unlock();
    work_ready_.notify_one();
}

void FrameCapture::writer_loop() noexcept
{
    try {
        for (;;) {
            Clip clip;
            {
                std::unique_lock<std::mutex> lock(mutex_);
                work_ready_.wait(lock, [this] { return stopping_ || pending_count_ != 0; });
                if (pending_count_ == 0 && stopping_) return;
                clip = std::move(pending_[pending_head_]);
                pending_[pending_head_] = Clip{};
                pending_head_ = (pending_head_ + 1) % pending_.size();
                --pending_count_;
            }
            space_ready_.notify_one();
            try {
                write_clip(clip);
            } catch (const std::exception& error) {
                report_error("writing event failed", error.what());
            } catch (...) {
                report_error("writing event failed", "unknown exception");
            }
        }
    } catch (const std::exception& error) {
        report_error("writer stopped unexpectedly", error.what());
    } catch (...) {
        report_error("writer stopped unexpectedly", "unknown exception");
    }
    try {
        // Synchronize the predicate with shutdown's space_ready_ wait.
        std::lock_guard<std::mutex> lock(mutex_);
        writer_failed_.store(true);
    } catch (...) {
        writer_failed_.store(true);
    }
    space_ready_.notify_all();
}

void FrameCapture::report_error(const char* context, const char* message) noexcept
{
    const auto count = error_count_.fetch_add(1) + 1;
    try {
        std::cerr << "[FrameCapture] ERROR #" << count << ": " << context
                  << ": " << message << '\n';
    } catch (...) {
        // Logging must never let an exception escape the writer thread.
    }
}

void FrameCapture::write_clip(const Clip& clip)
{
    const Frame& trigger = *clip.frames[clip.pre_frames];
    const Frame& previous = *clip.frames[clip.pre_frames - 1];
    const std::string base = "event_frame_" + std::to_string(trigger.number) + "_"
        + std::to_string(trigger.stamp_sec) + "_" + std::to_string(trigger.stamp_nanosec);
    std::filesystem::path event_directory;
    for (std::uint64_t suffix = 0;; ++suffix) {
        event_directory = directory_ / (base + (suffix == 0 ? "" : "_" + std::to_string(suffix)));
        std::error_code error;
        if (std::filesystem::create_directory(event_directory, error)) break;
        if (error && error != std::errc::file_exists) {
            throw std::filesystem::filesystem_error("create event directory", event_directory, error);
        }
    }

    auto frames = csv_file(event_directory / "frames.csv");
    auto candidates = csv_file(event_directory / "candidates.csv");
    frames << "relative_position,frame_number,stamp_sec,stamp_nanosec,source,confidence,"
              "selected_x,selected_y,selected_width,selected_height,image_file,"
              "candidate_count,selected_candidate\n";
    candidates << "frame_number,stamp_sec,stamp_nanosec,candidate_index,class,score,"
                  "box_x,box_y,box_width,box_height,clipped_x,clipped_y,clipped_width,"
                  "clipped_height,is_selected\n";
    for (std::size_t i = 0; i < clip.count; ++i) {
        const Frame& frame = *clip.frames[i];
        const int relative = static_cast<int>(i) - static_cast<int>(clip.pre_frames);
        const std::string image_name = "frame_" + std::to_string(relative) + ".png";
        if (!cv::imwrite((event_directory / image_name).string(), frame.image)) {
            throw std::runtime_error("PNG write returned false: " + event_directory.string()
                + "/" + image_name);
        }
        if (relative > 0) frames << '+';
        frames << relative << ',' << frame.number << ',' << frame.stamp_sec << ','
               << frame.stamp_nanosec << ',' << frame.source << ',' << frame.confidence << ','
               << frame.selected.x << ',' << frame.selected.y << ',' << frame.selected.width << ','
               << frame.selected.height << ',' << image_name << ',' << frame.candidates.size()
               << ',' << frame.selected_candidate << '\n';
        for (std::size_t index = 0; index < frame.candidates.size(); ++index) {
            const Detection& candidate = frame.candidates[index];
            const cv::Rect clipped = clipped_box(candidate.box, frame.image.size());
            candidates << frame.number << ',' << frame.stamp_sec << ',' << frame.stamp_nanosec
                << ',' << index << ',' << candidate.classId << ',' << candidate.conf << ','
                << candidate.box.x << ',' << candidate.box.y << ',' << candidate.box.width << ','
                << candidate.box.height << ',' << clipped.x << ',' << clipped.y << ','
                << clipped.width << ',' << clipped.height << ','
                << (frame.selected_candidate >= 0
                    && index == static_cast<std::size_t>(frame.selected_candidate) ? 1 : 0) << '\n';
        }
    }
    frames.close();
    candidates.close();

    // Written last so a successfully closed event.csv describes a flushed clip.
    auto event = csv_file(event_directory / "event.csv");
    event << "trigger_frame_number,trigger_stamp_sec,trigger_stamp_nanosec,"
             "previous_frame_number,previous_stamp_sec,previous_stamp_nanosec,jump_pixels,"
             "jump_threshold_pixels,stamp_delta_seconds,image_width,image_height,"
             "requested_pre_frames,requested_post_frames,available_pre_frames,"
             "available_post_frames,partial,shutdown_partial\n";
    event << trigger.number << ',' << trigger.stamp_sec << ',' << trigger.stamp_nanosec << ','
          << previous.number << ',' << previous.stamp_sec << ',' << previous.stamp_nanosec << ','
          << clip.jump_pixels << ',' << clip.threshold_pixels << ',' << clip.stamp_delta_seconds << ','
          << trigger.image.cols << ',' << trigger.image.rows << ",2,2," << clip.pre_frames << ','
          << clip.post_frames << ',' << (clip.pre_frames != 2 || clip.post_frames != 2 ? 1 : 0)
          << ',' << (clip.post_frames < 2 ? 1 : 0) << '\n';
    event.close();
}

}  // namespace vision_diagnostics
