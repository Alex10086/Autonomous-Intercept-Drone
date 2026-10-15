#include "uav_vision_png/time_alignment.hpp"

#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace {
constexpr std::uint64_t epoch_us = 1700000000000000ULL;
constexpr double epoch = 1700000000.0;
constexpr std::uint64_t status_sim = 10125000;
constexpr std::uint64_t remote = epoch_us + 93750;
// Deliberately reproduce a DDS timestamp 200ms behind Agent wall time.
constexpr std::uint64_t status_dds = remote - 200000;
constexpr std::int64_t offset = static_cast<std::int64_t>(status_sim) -
                                static_cast<std::int64_t>(status_dds);
constexpr std::uint64_t sample_sim = 10093750;
constexpr std::uint64_t sample_dds = static_cast<std::uint64_t>(
    static_cast<std::int64_t>(sample_sim) - offset);

void check(bool condition, const char *message)
{
    if (!condition) throw std::runtime_error(message);
}

void near(double actual, double expected, const char *message)
{
    check(std::isfinite(actual) && std::abs(actual - expected) < 1e-6, message);
}

void populate(vision_time::ClockHistory &history)
{
    // Native rate changes deliberately: a constant sim/system offset or a
    // receipt-derived offset cannot reproduce the intermediate capture time.
    check(!history.add(10.0, epoch, epoch), "first native pair");
    check(!history.add(10.0625, epoch + 0.03125, epoch + 0.03125), "second native pair");
    check(!history.add(10.125, epoch + 0.09375, epoch + 0.09375), "third native pair");
    check(!history.add(10.1875, epoch + 0.1875, epoch + 0.1875), "fourth native pair");
    check(!history.add(10.25, epoch + 0.25, epoch + 0.25), "fifth native pair");
}

void recovery_and_variable_rate()
{
    vision_time::ClockHistory history;
    populate(history);
    vision_time::DdsClock clock;
    check(!clock.update(status_dds, remote, offset, 4000, epoch + 0.3), "initial status resets");
    double system;
    check(clock.map(sample_dds, epoch + 0.3, history, system), "recover drifting DDS clock");
    near(system, epoch + 0.0625, "variable native rate capture time");
    check(clock.map(sample_dds, epoch + 0.45, history, system), "later receipt mapping");
    near(system, epoch + 0.0625, "receipt delay must not shift capture time");
    check(!clock.map(sample_dds, epoch + 0.299, history, system) && std::isnan(system),
          "reject negative status age");
    check(!clock.map(sample_dds, epoch + 2.301, history, system) && std::isnan(system),
          "reject stale status");
}

void validation_and_missing_mapping()
{
    vision_time::ClockHistory history;
    vision_time::DdsClock clock;
    double system;
    check(!clock.map(sample_dds, epoch + 0.3, history, system) && std::isnan(system),
          "missing status is NaN");
    clock.update(status_dds, remote, offset, 4000, epoch + 0.3);
    check(!clock.map(sample_dds, epoch + 0.3, history, system) && std::isnan(system),
          "missing native pairs is NaN");
    populate(history);
    check(clock.map(sample_dds, epoch + 0.3, history, system), "retry validation after native arrival");
    check(!clock.map(static_cast<std::uint64_t>(1000000 - offset), epoch + 0.3, history, system) &&
          std::isnan(system), "never extrapolate sample");
    check(!clock.map(sample_dds, epoch + 0.51, history, system) && std::isnan(system),
          "stale native history is NaN");

    // Compatible-looking wall epochs alone do not validate a status. The
    // native mapping of the original status HRT must match unmodified remote.
    clock.update(status_dds, epoch_us + 300000, offset, 4000, epoch + 0.3);
    check(!clock.map(sample_dds, epoch + 0.3, history, system) && std::isnan(system),
          "reject wrong remote clock despite compatible epoch");
    clock.clear();
    clock.update(status_dds, remote + 62500, offset, 25000, epoch + 0.3);
    check(clock.map(sample_dds, epoch + 0.3, history, system), "RTT half-bound inclusive");
    clock.update(status_dds, remote + 62501, offset, 25000, epoch + 0.3);
    check(!clock.map(sample_dds, epoch + 0.3, history, system) && std::isnan(system),
          "reject residual beyond RTT half-bound");
}

void invalid_status_and_integer_bounds()
{
    vision_time::ClockHistory history;
    populate(history);
    vision_time::DdsClock clock;
    clock.update(status_dds, remote, offset, 4000, epoch + 0.3);
    double system;
    const auto preserved = [&] {
        check(clock.map(sample_dds, epoch + 0.3, history, system), "invalid update poisoned status");
        near(system, epoch + 0.0625, "invalid update changed offset");
    };
    check(!clock.update(status_dds, 10, offset, 4000, epoch + 0.3), "invalid remote epoch reset");
    preserved();
    check(!clock.update(0, remote, offset, 4000, epoch + 0.3), "zero timestamp reset");
    preserved();
    check(!clock.update(std::numeric_limits<std::uint64_t>::max(), remote, offset, 4000,
                        epoch + 0.3), "unsigned timestamp overflow reset");
    preserved();
    check(!clock.update(1, remote, std::numeric_limits<std::int64_t>::max(), 4000,
                        epoch + 0.3), "signed addition overflow reset");
    preserved();
    check(!clock.update(1, remote, std::numeric_limits<std::int64_t>::min(), 4000,
                        epoch + 0.3), "negative recovered HRT reset");
    preserved();
    check(!clock.update(status_dds, remote, offset, 4000,
                        std::numeric_limits<double>::quiet_NaN()), "NaN receipt reset");
    preserved();
    check(!clock.map(1, epoch + 0.3, history, system) && std::isnan(system), "invalid sample is NaN");
    check(!clock.map(sample_dds, std::numeric_limits<double>::quiet_NaN(), history, system) &&
          std::isnan(system), "NaN now is NaN");
}

void reset_and_offset_isolation()
{
    vision_time::ClockHistory history;
    populate(history);
    vision_time::DdsClock clock;
    clock.update(status_dds, remote, offset, 4000, epoch + 0.3);
    double system;
    check(clock.map(sample_dds, epoch + 0.3, history, system), "validate initial status");
    check(clock.update(status_dds - 300000, remote, offset + 300000, 4000, epoch + 0.3),
          "large offset step resets");
    check(!clock.map(sample_dds, epoch + 0.3, history, system) && std::isnan(system),
          "old-offset sample cannot cross native boundary");
    check(clock.map(sample_dds - 300000, epoch + 0.3, history, system), "new offset maps");
    near(system, epoch + 0.0625, "new status retained after reset");
    // Successful validation must not survive a subsequent update.
    check(!clock.update(status_dds - 300000, remote + 100000, offset + 300000,
                        4000, epoch + 0.3), "forward remote is not reset");
    check(!clock.map(sample_dds - 300000, epoch + 0.3, history, system) && std::isnan(system),
          "offset update invalidates cached native validation");
    check(clock.update(status_dds - 300000, remote, offset + 300000, 4000, epoch + 0.3),
          "backwards remote resets");
    check(clock.update(status_dds - 300000, remote, offset + 300000, 4000, epoch + 0.299),
          "backwards receipt resets");
    check(!clock.update(status_dds - 320000, remote, offset + 320000, 4000, epoch + 0.3),
          "exactly 20ms offset step is not reset");
    clock.clear();
    check(!clock.map(sample_dds - 320000, epoch + 0.3, history, system) && std::isnan(system),
          "clear removes status and cached validation");
    check(!clock.update(status_dds, remote, offset, 4000, epoch + 0.3), "first update after clear");
}
}  // namespace

int main()
{
    try {
        recovery_and_variable_rate();
        validation_and_missing_mapping();
        invalid_status_and_integer_bounds();
        reset_and_offset_isolation();
    } catch (const std::exception &error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
    return 0;
}
