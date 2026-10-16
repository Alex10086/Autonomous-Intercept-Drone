#include "frame_capture.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace {
namespace fs = std::filesystem;
using vision_diagnostics::FrameCapture;

void require(bool value, const char* message)
{
    if (!value) throw std::runtime_error(message);
}

class TemporaryDirectory {
public:
    TemporaryDirectory()
    {
        const auto tick = std::chrono::steady_clock::now().time_since_epoch().count();
        for (std::uint64_t suffix = 0;; ++suffix) {
            path = fs::temp_directory_path()
                / ("frame_capture_test_" + std::to_string(tick) + "_" + std::to_string(suffix));
            if (fs::create_directory(path)) break;
        }
    }

    ~TemporaryDirectory()
    {
        std::error_code ignored;
        fs::remove_all(path, ignored);
    }

    fs::path path;
};

std::vector<std::string> fields(const std::string& line)
{
    std::vector<std::string> result;
    std::istringstream stream(line);
    std::string field;
    while (std::getline(stream, field, ',')) result.push_back(field);
    return result;
}

struct Csv {
    explicit Csv(const fs::path& path)
    {
        std::ifstream stream(path);
        require(stream.is_open(), "CSV file is missing");
        std::string line;
        require(static_cast<bool>(std::getline(stream, line)), "CSV header is missing");
        header = fields(line);
        while (std::getline(stream, line)) {
            if (line.empty()) continue;
            rows.push_back(fields(line));
            require(rows.back().size() == header.size(), "CSV row has the wrong field count");
        }
        require(!stream.bad(), "CSV read failed");
    }

    const std::string& cell(std::size_t row, const char* column) const
    {
        const auto found = std::find(header.begin(), header.end(), column);
        require(found != header.end(), "CSV column is missing");
        return rows.at(row).at(static_cast<std::size_t>(found - header.begin()));
    }

    long long integer(std::size_t row, const char* column) const
    {
        return std::stoll(cell(row, column));
    }

    double number(std::size_t row, const char* column) const
    {
        return std::stod(cell(row, column));
    }

    std::vector<std::string> header;
    std::vector<std::vector<std::string>> rows;
};

std::vector<fs::path> event_directories(const fs::path& directory)
{
    std::vector<fs::path> result;
    for (const auto& entry : fs::directory_iterator(directory)) {
        if (entry.is_directory() && entry.path().filename().string().rfind("event_frame_", 0) == 0) {
            result.push_back(entry.path());
        }
    }
    std::sort(result.begin(), result.end());
    return result;
}

cv::Mat image(int frame, int offset = 0, cv::Size size = {40, 30})
{
    cv::Mat result(size, CV_8UC3, cv::Scalar(frame + offset, frame + offset + 20,
        frame + offset + 40));
    result.at<cv::Vec3b>(4, 4) = cv::Vec3b(9, static_cast<unsigned char>(frame + offset), 200);
    return result;
}

cv::Rect selected_box(int frame)
{
    return frame == 3 || frame == 5 ? cv::Rect(25, 15, 4, 4) : cv::Rect(2, 2, 4, 4);
}

std::vector<Detection> detections(const cv::Rect& selected)
{
    return {{cv::Rect(-3, 2, 9, 8), 0.2f, 5}, {selected, 0.95f, 1},
        {cv::Rect(38, 27, 8, 9), 0.7f, 3}};
}

void record_full_window(const fs::path& directory, int offset)
{
    FrameCapture capture(directory.string());
    for (int frame = 1; frame <= 5; ++frame) {
        auto raw = image(frame, offset);
        const auto box = selected_box(frame);
        auto candidates = frame == 4 ? std::vector<Detection>{} : detections(box);
        const float confidence = frame == 5 ? std::numeric_limits<float>::quiet_NaN()
            : 0.5f + 0.01f * frame;
        capture.observe(frame, 42, static_cast<std::uint32_t>(frame * 50000000), raw,
            box, confidence, frame == 4 ? 2 : 1, std::move(candidates), frame == 4 ? -1 : 1,
            frame == 4);
        // Simulate debug drawing only on the explicitly copied frame. Other
        // inputs go out of scope, exercising retained cv::Mat ownership.
        if (frame == 4) raw.setTo(cv::Scalar(255, 255, 255));
    }
}

void check_full_window(const fs::path& directory, int offset)
{
    const Csv event(directory / "event.csv");
    require(event.rows.size() == 1, "event metadata must contain exactly one event");
    require(event.integer(0, "trigger_frame_number") == 3, "wrong trigger frame");
    require(event.integer(0, "trigger_stamp_sec") == 42
        && event.integer(0, "trigger_stamp_nanosec") == 150000000, "wrong trigger stamp");
    require(event.integer(0, "previous_frame_number") == 2, "wrong previous frame");
    require(std::abs(event.number(0, "jump_threshold_pixels") - 10.0) < 1e-9,
        "threshold is not 0.2 times the image diagonal");
    require(event.number(0, "jump_pixels") > event.number(0, "jump_threshold_pixels"),
        "recorded jump does not exceed the threshold");
    require(std::abs(event.number(0, "stamp_delta_seconds") - 0.05) < 1e-9,
        "wrong trigger time interval");
    require(event.integer(0, "available_pre_frames") == 2
        && event.integer(0, "available_post_frames") == 2
        && event.integer(0, "partial") == 0 && event.integer(0, "shutdown_partial") == 0,
        "full context is incorrectly marked partial");

    const Csv frames(directory / "frames.csv");
    require(frames.rows.size() == 5, "complete event must contain five frames");
    for (int frame = 1; frame <= 5; ++frame) {
        const auto row = static_cast<std::size_t>(frame - 1);
        const auto box = selected_box(frame);
        require(frames.integer(row, "relative_position") == frame - 3, "wrong frame order");
        if (frame > 3) {
            require(frames.cell(row, "relative_position").front() == '+',
                "positive relative position needs an explicit plus sign");
        }
        require(frames.integer(row, "frame_number") == frame, "wrong frame number");
        require(frames.integer(row, "stamp_sec") == 42
            && frames.integer(row, "stamp_nanosec") == frame * 50000000, "wrong frame stamp");
        require(frames.integer(row, "source") == (frame == 4 ? 2 : 1), "wrong detector source");
        require(frames.integer(row, "selected_x") == box.x
            && frames.integer(row, "selected_y") == box.y
            && frames.integer(row, "selected_width") == box.width
            && frames.integer(row, "selected_height") == box.height, "selected box changed");
        require(frames.integer(row, "candidate_count") == (frame == 4 ? 0 : 3)
            && frames.integer(row, "selected_candidate") == (frame == 4 ? -1 : 1),
            "candidate count or selected index changed");
        if (frame == 5) {
            require(std::isnan(frames.number(row, "confidence")), "NaN confidence was not preserved");
        } else {
            require(std::abs(frames.number(row, "confidence") - (0.5 + 0.01 * frame)) < 1e-6,
                "confidence changed");
        }
        const auto saved = cv::imread((directory / frames.cell(row, "image_file")).string(),
            cv::IMREAD_UNCHANGED);
        require(saved.type() == CV_8UC3 && saved.size() == cv::Size(40, 30),
            "PNG dimensions or type changed");
        require(cv::norm(saved, image(frame, offset), cv::NORM_INF) == 0,
            "PNG pixels differ from the raw image (or contain debug drawing)");
    }

    const Csv candidates(directory / "candidates.csv");
    require(candidates.rows.size() == 12, "all post-NMS candidates must be retained");
    std::size_t row = 0;
    for (int frame : {1, 2, 3, 5}) {
        const auto expected = detections(selected_box(frame));
        for (std::size_t index = 0; index < expected.size(); ++index, ++row) {
            const auto& detection = expected[index];
            const auto clipped = detection.box & cv::Rect(0, 0, 40, 30);
            require(candidates.integer(row, "frame_number") == frame
                && candidates.integer(row, "stamp_sec") == 42
                && candidates.integer(row, "stamp_nanosec") == frame * 50000000,
                "candidate timestamp or frame association changed");
            require(candidates.integer(row, "candidate_index") == static_cast<long long>(index)
                && candidates.integer(row, "class") == detection.classId
                && std::abs(candidates.number(row, "score") - detection.conf) < 1e-6,
                "candidate order, class or score changed");
            require(candidates.integer(row, "box_x") == detection.box.x
                && candidates.integer(row, "box_y") == detection.box.y
                && candidates.integer(row, "box_width") == detection.box.width
                && candidates.integer(row, "box_height") == detection.box.height,
                "original candidate box was clipped or changed");
            require(candidates.integer(row, "clipped_x") == clipped.x
                && candidates.integer(row, "clipped_y") == clipped.y
                && candidates.integer(row, "clipped_width") == clipped.width
                && candidates.integer(row, "clipped_height") == clipped.height,
                "clipped candidate box is incorrect");
            require(candidates.integer(row, "is_selected") == (index == 1 ? 1 : 0),
                "candidate selection marker changed");
        }
    }
}

void test_full_window_and_existing_directory()
{
    TemporaryDirectory temporary;
    record_full_window(temporary.path, 0);
    auto events = event_directories(temporary.path);
    require(events.size() == 1, "jumps during an active event must not retrigger");
    const auto first = events.front();
    check_full_window(first, 0);

    record_full_window(temporary.path, 50);
    events = event_directories(temporary.path);
    require(events.size() == 2, "identical trigger frame/stamp must create a new event directory");
    for (const auto& event : events) check_full_window(event, event == first ? 0 : 50);
}

void observe(FrameCapture& capture, std::uint64_t frame, std::uint32_t ns,
    const cv::Rect& box, cv::Size size = {100, 100})
{
    capture.observe(frame, 10, ns, image(1, 0, size), box, 0.8f, 1, {}, -1);
}

void test_invalid_and_gaps()
{
    TemporaryDirectory temporary;
    const cv::Rect near(0, 0, 10, 10), far(70, 70, 10, 10);
    {
        FrameCapture capture(temporary.path.string());
        observe(capture, 1, 0, near);
        observe(capture, 2, 50000000, {});
        observe(capture, 3, 100000000, far);  // Do not bridge an invalid box.
        observe(capture, 4, 150000000, far);
        observe(capture, 5, 400000000, near);  // Timestamp gap exceeds 0.15 s.
        observe(capture, 6, 450000000, near);
        observe(capture, 7, 450000000, far);  // Equal timestamps are not adjacent in time.
        observe(capture, 8, 440000000, near); // Nor are decreasing timestamps.
        observe(capture, 9, 490000000, near);
        observe(capture, 12, 500000000, far); // Missing processed frame numbers.
        observe(capture, 13, 550000000, far);
        observe(capture, 14, 600000000, near, {101, 100});
        observe(capture, 15, 650000000, far); // Do not compare different image sizes.
        observe(capture, 16, 700000000, far);
    }
    require(event_directories(temporary.path).empty(), "invalid/gap frames caused a jump event");
}

void test_threshold_and_time_boundaries()
{
    TemporaryDirectory temporary;
    const auto run_pair = [&](const char* name, int delta_x, std::uint32_t delta_ns) {
        const auto directory = temporary.path / name;
        {
            FrameCapture capture(directory.string());
            observe(capture, 1, 0, {1, 1, 2, 2}, {40, 30});
            observe(capture, 2, delta_ns, {1 + delta_x, 1, 2, 2}, {40, 30});
        }
        return event_directories(directory);
    };
    require(run_pair("exact_threshold", 10, 50000000).empty(),
        "jump equal to the threshold must not trigger");
    require(run_pair("over_time_gap", 11, 150000001).empty(),
        "timestamp gap just above 0.15 s must not trigger");
    const auto events = run_pair("inclusive_time_gap", 11, 150000000);
    require(events.size() == 1, "0.15 s boundary should permit a strictly larger jump");
    const Csv event(events.front() / "event.csv");
    const Csv frames(events.front() / "frames.csv");
    require(event.integer(0, "available_pre_frames") == 1
        && event.integer(0, "available_post_frames") == 0
        && event.integer(0, "partial") == 1 && event.integer(0, "shutdown_partial") == 1,
        "early shutdown must report the actually available context");
    require(frames.rows.size() == 2 && frames.integer(0, "relative_position") == -1
        && frames.integer(1, "relative_position") == 0, "early partial window is mislabeled");
}

void test_shutdown_partial()
{
    TemporaryDirectory temporary;
    {
        FrameCapture capture(temporary.path.string());
        for (int frame = 1; frame <= 4; ++frame) {
            const auto box = frame < 3 ? cv::Rect(2, 2, 4, 4) : cv::Rect(25, 15, 4, 4);
            capture.observe(frame, 5, static_cast<std::uint32_t>(frame * 50000000),
                image(frame), box, 0.9f, 1, detections(box), 1);
        }
    }
    const auto events = event_directories(temporary.path);
    require(events.size() == 1, "shutdown must flush the active partial event");
    const Csv event(events.front() / "event.csv");
    const Csv frames(events.front() / "frames.csv");
    require(event.integer(0, "available_pre_frames") == 2
        && event.integer(0, "available_post_frames") == 1
        && event.integer(0, "partial") == 1 && event.integer(0, "shutdown_partial") == 1,
        "shutdown partial context is incorrect");
    require(frames.rows.size() == 4, "shutdown partial should preserve all four available frames");
    for (std::size_t row = 0; row < frames.rows.size(); ++row) {
        require(frames.integer(row, "relative_position") == static_cast<long long>(row) - 2,
            "shutdown partial relative positions are incorrect");
        const auto saved = cv::imread((events.front() / frames.cell(row, "image_file")).string());
        require(cv::norm(saved, image(static_cast<int>(row) + 1), cv::NORM_INF) == 0,
            "shutdown partial raw pixels changed");
    }
}

void wait_for_completed_events(const fs::path& directory, std::size_t expected)
{
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
    while (std::chrono::steady_clock::now() < deadline) {
        std::size_t completed = 0;
        for (const auto& event : event_directories(directory)) {
            try {
                if (Csv(event / "event.csv").rows.size() == 1) ++completed;
            } catch (const std::exception&) {
                // The writer may have claimed the directory but not closed CSV yet.
            }
        }
        if (completed >= expected) return;
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    throw std::runtime_error("writer did not complete the expected events");
}

void test_event_limit()
{
    TemporaryDirectory temporary;
    {
        FrameCapture capture(temporary.path.string());
        std::uint64_t frame = 1;
        const cv::Rect near(2, 2, 4, 4), far(25, 15, 4, 4);
        auto box = near;
        const auto send = [&] {
            const std::uint64_t time_ns = frame * 50000000;
            capture.observe(frame, static_cast<std::int32_t>(time_ns / 1000000000),
                static_cast<std::uint32_t>(time_ns % 1000000000), image(1), box, 0.9f, 1, {}, -1);
            ++frame;
        };
        send();
        send();
        for (std::size_t event = 1; event <= 16; ++event) {
            box = box == near ? far : near;
            send();  // Trigger.
            send();
            send();
            // Consume each real event before generating the next, so this test
            // exercises the 16-event cap independently of writer queue pressure.
            wait_for_completed_events(temporary.path, event);
        }
        for (int extra = 0; extra < 9; ++extra) {
            box = box == near ? far : near;
            send();
        }
    }
    require(event_directories(temporary.path).size() == 16, "capture exceeded the 16-event cap");
}

void test_unusable_directory()
{
    TemporaryDirectory temporary;
    const auto file = temporary.path / "not_a_directory";
    {
        std::ofstream stream(file);
        stream << "occupied\n";
    }
    bool rejected = false;
    try {
        FrameCapture capture(file.string());
    } catch (const std::exception& error) {
        rejected = std::string(error.what()).find("unusable") != std::string::npos;
    }
    require(rejected, "an unusable capture directory needs an explicit initialization error");
}
void test_writer_error_is_reported()
{
    TemporaryDirectory temporary;
    const auto directory = temporary.path / "capture";
    std::ostringstream errors;
    auto* original = std::cerr.rdbuf(errors.rdbuf());
    try {
        {
            FrameCapture capture(directory.string());
            // Make a previously validated destination unusable. The asynchronous
            // filesystem exception must be reported, not escape or terminate.
            fs::remove(directory);
            std::ofstream blocker(directory);
            blocker << "occupied\n";
            blocker.close();
            observe(capture, 1, 0, {0, 0, 10, 10});
            observe(capture, 2, 50000000, {70, 70, 10, 10});
            observe(capture, 3, 100000000, {70, 70, 10, 10});
            observe(capture, 4, 150000000, {70, 70, 10, 10});
        }
    } catch (...) {
        std::cerr.rdbuf(original);
        throw;
    }
    std::cerr.rdbuf(original);
    require(errors.str().find("[FrameCapture] ERROR #1: writing event failed") != std::string::npos,
        "writer filesystem failure was not visibly reported");
}

void test_first_acquisition_without_history()
{
    TemporaryDirectory temporary;
    {
        FrameCapture capture(temporary.path.string());
        const cv::Rect box(2, 2, 4, 4);
        capture.observe(1, 10, 0, image(1), box, 0.9f, 1, detections(box), 1, false, true);
        capture.observe(2, 10, 33000000, image(2), box, 0.99f, 2, {}, -1);
        capture.observe(3, 10, 66000000, image(3), box, 0.99f, 2, {}, -1);
    }
    const auto paths = event_directories(temporary.path);
    require(paths.size() == 1, "first acquisition did not produce exactly one clip");
    const Csv event(paths[0] / "event.csv");
    require(event.cell(0, "trigger_reason") == "first_acquisition", "acquisition reason lost");
    require(event.integer(0, "available_pre_frames") == 0, "invented acquisition history");
    require(event.integer(0, "available_post_frames") == 2, "acquisition post-context missing");
    require(event.integer(0, "previous_frame_number") == 0, "invented previous frame");
    const Csv frames(paths[0] / "frames.csv");
    require(frames.rows.size() == 3 && frames.integer(0, "relative_position") == 0,
        "first-frame acquisition window is misaligned");
    const auto saved = cv::imread((paths[0] / "frame_0.png").string());
    require(cv::norm(saved, image(1), cv::NORM_INF) == 0, "acquisition raw pixels changed");
}

}  // namespace

int main()
{
    test_full_window_and_existing_directory();
    test_invalid_and_gaps();
    test_threshold_and_time_boundaries();
    test_shutdown_partial();
    test_event_limit();
    test_unusable_directory();
    test_writer_error_is_reported();
    test_first_acquisition_without_history();
    std::cout << "FrameCapture: raw PNG pixels, full/partial context, candidates, adjacency, "
                 "unique directories, event limit and visible writer errors passed\n";
}
