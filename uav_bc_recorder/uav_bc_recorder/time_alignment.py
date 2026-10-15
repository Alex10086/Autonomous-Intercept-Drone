"""Bounded native Gazebo/PX4 clock recovery; never estimate from receipt time."""
from bisect import bisect_left
import math

NAN = float("nan")
INT64_MAX = (1 << 63) - 1
INT64_MIN = -(1 << 63)


def recover_sim_us(timestamp, estimated_offset):
    """Undo DDS translation as signed integers before converting to seconds."""
    if not 0 < timestamp <= INT64_MAX or not INT64_MIN <= estimated_offset <= INT64_MAX:
        return None
    recovered = timestamp + estimated_offset
    return recovered if 0 < recovered <= INT64_MAX else None


class ClockHistory:
    CAPACITY = 4096

    def __init__(self):
        self.clear()

    def clear(self):
        self.samples = []
        self.last_received = NAN

    def add(self, sim, system, received):
        if (not all(math.isfinite(value) for value in (sim, system, received))
                or sim < 0 or system <= 0 or not -0.02 <= received - system <= 0.5):
            return False
        reset = bool(self.samples and (sim <= self.samples[-1][0]
                                      or system <= self.samples[-1][1]))
        if reset:
            self.clear()
        self.samples.append((sim, system))
        if len(self.samples) > self.CAPACITY:
            del self.samples[0]
        self.last_received = received
        return reset

    def map(self, sim, now):
        if (len(self.samples) < 2 or not math.isfinite(sim) or not math.isfinite(now) or sim <= 0
                or not 0 <= now - self.last_received <= 0.25
                or not self.samples[0][0] <= sim <= self.samples[-1][0]):
            return NAN
        high = max(1, bisect_left(self.samples, (sim,)))
        a, b = self.samples[high - 1], self.samples[high]
        if b[0] - a[0] > 0.1 or b[1] - a[1] > 0.2:
            return NAN
        system = a[1] + (sim - a[0]) / (b[0] - a[0]) * (b[1] - a[1])
        return system if 0 <= now - system <= 2.0 else NAN


class DdsClock:
    """One latest status per aircraft, validated against the native clock pairs."""
    def __init__(self):
        self.clear()

    def clear(self):
        self.status = None
        self.validated = False

    def update(self, timestamp, remote_timestamp, estimated_offset, round_trip_time, received):
        # Receipt establishes only epoch compatibility/freshness, never the offset.
        sim_us = recover_sim_us(timestamp, estimated_offset)
        if (sim_us is None or not 0 < remote_timestamp <= INT64_MAX
                or not 0 <= round_trip_time <= (1 << 32) - 1
                or not math.isfinite(received)
                or abs(remote_timestamp * 1e-6 - received) > 2.0):
            return False
        reset = False
        if self.status is not None:
            _, old_remote, old_offset, _, old_received = self.status
            # DDS topics can cross a Timesync update boundary; do not hide large steps.
            reset = (abs(estimated_offset - old_offset) > 20_000
                     or remote_timestamp < old_remote or received < old_received)
        self.status = (sim_us, remote_timestamp, estimated_offset, round_trip_time, received)
        self.validated = False
        return reset

    def offset(self, now):
        if self.status is None or not 0 <= now - self.status[4] <= 2.0:
            return NAN
        return self.status[2]

    def map(self, sample, now, history):
        offset = self.offset(now)
        if not math.isfinite(offset):
            return NAN
        sim_us = recover_sim_us(sample, offset)
        if sim_us is None:
            return NAN
        if not self.validated:
            status_sim, remote, _, rtt, _ = self.status
            system = history.map(status_sim * 1e-6, now)
            if (not math.isfinite(system)
                    or abs(system - remote * 1e-6) > 0.05 + rtt * 0.5e-6):
                return NAN
            self.validated = True
        return history.map(sim_us * 1e-6, now)
