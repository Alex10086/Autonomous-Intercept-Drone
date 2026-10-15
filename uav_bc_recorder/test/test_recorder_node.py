"""Deterministic receipt-clock scenarios exercise actual CSV and summary output."""
import csv
import json
import math

import pytest
import rclpy
from px4_msgs.msg import SensorGps, TrajectorySetpoint, VehicleLocalPosition, VehicleOdometry
from std_msgs.msg import String
from uav_common_msg.msg import ExperimentTelemetry, RectMsg
import yaml

from uav_bc_recorder import recorder_node as recorder_module
from uav_bc_recorder.recorder_node import DEFAULTS, ExperimentRecorder


@pytest.fixture
def recorder(tmp_path, monkeypatch):
    monkeypatch.setitem(DEFAULTS, "out_dir", str(tmp_path))
    monkeypatch.setitem(DEFAULTS, "algorithm", "pure_png")
    rclpy.init(args=[])
    node = ExperimentRecorder()
    clock = [100.0]
    node._now = lambda: clock[0]
    node.last_time = clock[0]
    try:
        yield node, clock
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_geometry_retains_pre_origin_events_and_rejects_wrong_clock(recorder):
    node, clock = recorder
    clock[0] = 1_800_000_000.0
    target = SensorGps()
    target.fix_type = 3
    target.latitude_deg, target.longitude_deg = 47.0001, 8.0
    target.altitude_ellipsoid_m = 547.0
    target.timestamp = 42_000_000
    target.timestamp_sample = int(clock[0] * 1e6)
    node._gps_cb("target_gps", target)
    clock[0] += 0.1
    own = SensorGps()
    own.fix_type = 3
    own.latitude_deg, own.longitude_deg = 47.0, 8.0
    own.altitude_ellipsoid_m = 547.0
    own.timestamp = int((clock[0] - 0.05) * 1e6)
    node._gps_cb("self_gps", own)
    target.timestamp = int((clock[0] + 10.0) * 1e6)
    node._gps_cb("target_gps", target)
    assert node.intercept_start is None
    with (node.storage.path / "geometry.csv").open() as stream:
        rows = list(csv.DictReader(stream))
    assert len(rows) == 3
    assert rows[0]["vehicle"] == "target"
    assert float(rows[0]["receive_time"]) == 1_800_000_000.0
    assert math.isnan(float(rows[0]["source_time"]))
    assert float(rows[0]["x"]) > 10.0
    assert float(rows[1]["source_time"]) == pytest.approx(clock[0] - 0.05)
    assert float(rows[1]["x"]) == pytest.approx(0.0)
    assert math.isnan(float(rows[2]["source_time"]))


def ready(node, clock, algorithm="pure_png", target_altitude=547.0,
          target_velocity=(8.0, 0.0, 0.0), self_heading=0.0):
    for key, latitude in (("self_gps", 47.0), ("target_gps", 47.0001)):
        gps = SensorGps()
        gps.fix_type, gps.latitude_deg, gps.longitude_deg = 3, latitude, 8.0
        gps.altitude_ellipsoid_m = 547.0 if key == "self_gps" else target_altitude
        gps.altitude_msl_m = 500.0
        node._gps_cb(key, gps)
    for key in ("self_local", "target_local"):
        local = VehicleLocalPosition()
        local.v_xy_valid = local.v_z_valid = True
        local.vx, local.vy, local.vz = (8.0, 0.0, 0.0) if key == "self_local" else target_velocity
        local.heading = self_heading
        node._cache(key, local)
    command = TrajectorySetpoint()
    command.velocity = [8.0, 0.0, 0.0]
    node._cache("trajectory", command)
    telemetry = ExperimentTelemetry()
    telemetry.algorithm, telemetry.state = algorithm, "INTERCEPT"
    telemetry.header.stamp.sec = int(clock[0])
    telemetry.parameters_yaml = "N_png: 4.0\nspeed_cmd: 8.0\nhit_radius: 0.8\n"
    telemetry.solver_time_ms = telemetry.cost = math.nan
    node._telemetry_cb(telemetry)


def test_horizontal_pure_png_records_without_camera_and_times_out_on_sensor_loss(recorder):
    node, clock = recorder
    ready(node, clock)
    node._tick()
    assert node.intercept_start == 100.0
    clock[0] += node.config["max_duration"] + 0.1
    node._tick()
    assert node.finished
    with (node.storage.path / "timeseries.csv").open() as stream:
        rows = list(csv.DictReader(stream))
    assert tuple(float(rows[0][key]) for key in ("ref_vx", "ref_vy", "ref_vz")) == (8.0, 0.0, 0.0)
    assert float(rows[0]["distance"]) > 10.0
    assert math.isnan(float(rows[-1]["distance"]))
    assert math.isnan(float(rows[-1]["ref_vx"]))
    with (node.storage.path / "summary.csv").open() as stream:
        summary = next(csv.DictReader(stream))
    assert summary["reason"] == "timeout" and summary["hit"] == "False"
    assert not (node.storage.path / "camera.mp4").exists()
    assert not (node.storage.path / "rosbag").exists()


def test_sim_stamped_detections_trigger_and_count_loss_once(recorder):
    node, clock = recorder
    node.config["algorithm"] = "vision_png"
    # Gazebo image stamp and recorder wall clock need not share an epoch.
    clock[0] = 1_800_000_000.0
    node.last_time = clock[0]
    ready(node, clock, "vision_png")
    assert node.intercept_start is None
    box = RectMsg()
    box.header.stamp.sec = 42
    box.x, box.y, box.width, box.height, box.confidence = 10, 20, 30, 40, 0.9
    node._detect_cb(box)
    node._tick()
    assert node.intercept_start == clock[0]
    clock[0] += node.config["topic_timeout"] + 0.1
    node._tick()
    node._tick()
    assert node.lost_count == 1
    node._detect_cb(box)
    node._tick()
    assert node.reacquire_count == 1
    node.finish("shutdown")
    with (node.storage.path / "timeseries.csv").open() as stream:
        rows = list(csv.DictReader(stream))
    assert math.isnan(float(rows[0]["vision_latency_ms"]))
    assert sum(row["lost"] == "True" for row in rows) == 1
    assert sum(row["reacquired"] == "True" for row in rows) == 1


def test_detection_count_uses_one_consistent_receipt_timestamp(recorder):
    node, clock = recorder

    def advancing_clock():
        clock[0] += 0.000001
        return clock[0]

    node._now = advancing_clock
    box = RectMsg()
    box.x, box.y, box.width, box.height = 10, 20, 30, 40
    node._detect_cb(box)
    assert node.detected
    assert node.detection_count == 1


def test_start_timeout_expires_with_frozen_ros_clock(recorder, monkeypatch):
    node, _ = recorder
    deadline = node.start_wait_monotonic + node.config["start_timeout"]
    monkeypatch.setattr(recorder_module, "monotonic", lambda: deadline - 0.001)
    assert not node._check_start_timeout()
    monkeypatch.setattr(recorder_module, "monotonic", lambda: deadline)
    assert node._check_start_timeout()
    with (node.storage.path / "summary.csv").open() as stream:
        summary = next(csv.DictReader(stream))
    assert summary["reason"] == "start_timeout"
    assert summary["status"] == "no_intercept" and summary["samples"] == "0"
    with (node.storage.path / "timeseries.csv").open() as stream:
        assert list(csv.DictReader(stream)) == []


def test_late_prerequisites_cannot_start_after_start_timeout(recorder, monkeypatch):
    node, clock = recorder
    deadline = node.start_wait_monotonic + node.config["start_timeout"]
    monkeypatch.setattr(recorder_module, "monotonic", lambda: deadline)
    ready(node, clock)
    assert node.finished and node.intercept_start is None
    with (node.storage.path / "summary.csv").open() as stream:
        assert next(csv.DictReader(stream))["reason"] == "start_timeout"


def test_start_timeout_does_not_stop_active_interception(recorder, monkeypatch):
    node, clock = recorder
    ready(node, clock)
    deadline = node.start_wait_monotonic + node.config["start_timeout"]
    monkeypatch.setattr(recorder_module, "monotonic", lambda: deadline + 1.0)
    assert not node._check_start_timeout()
    node._tick()
    assert node.samples == 1 and not node.finished
    clock[0] += node.config["max_duration"]
    node._tick()
    assert node.finished
    with (node.storage.path / "summary.csv").open() as stream:
        assert next(csv.DictReader(stream))["reason"] == "timeout"


@pytest.fixture
def wall_clock(recorder, monkeypatch):
    node, _ = recorder
    wall = [node.start_wait_monotonic + 1.0]
    monkeypatch.setattr(recorder_module, "monotonic", lambda: wall[0])
    return wall


def measured_hit(node):
    gps = SensorGps()
    gps.fix_type, gps.latitude_deg, gps.longitude_deg = 3, 47.0, 8.0
    gps.altitude_ellipsoid_m, gps.altitude_msl_m = 547.0, 500.0
    node._gps_cb("self_gps", gps)
    node._gps_cb("target_gps", gps)


def done(node, clock):
    telemetry = ExperimentTelemetry()
    telemetry.algorithm, telemetry.state = node.config["algorithm"], "DONE"
    telemetry.header.stamp.sec = int(clock[0])
    telemetry.parameters_yaml = node.last_parameters_yaml
    node._telemetry_cb(telemetry)


def summary(node):
    with (node.storage.path / "summary.csv").open() as stream:
        return next(csv.DictReader(stream))


def test_first_hit_tail_survives_done_and_max_duration(recorder, wall_clock):
    node, clock = recorder
    ready(node, clock)
    node.config["max_duration"] = 0.3
    clock[0] = 100.25
    measured_hit(node)
    hit_wall = wall_clock[0]
    assert node.hit and not node.finished
    assert node.first_hit_ros_time == 100.25
    clock[0] = 100.8
    done(node, clock)
    node._tick()
    assert not node.finished
    assert node.first_hit_ros_time == 100.25
    wall_clock[0] = hit_wall + 1.999
    assert not node._check_wall_time()
    clock[0] = 102.25
    wall_clock[0] = hit_wall + 2.0
    assert node._check_wall_time()
    result = summary(node)
    assert result["reason"] == "hit" and result["hit"] == "True"
    assert float(result["intercept_time"]) == pytest.approx(0.25)
    assert float(result["elapsed_s"]) == pytest.approx(2.25)
    assert float(result["post_hit_duration_actual"]) == pytest.approx(2.0)
    assert result["tail_truncated"] == "False"
    assert float(result["min_distance"]) == pytest.approx(0.0)
    with (node.storage.path / "timeseries.csv").open() as stream:
        rows = list(csv.DictReader(stream))
    assert float(rows[-1]["timestamp"]) == 102.25
    assert all(row["hit"] == "True" for row in rows)


def test_hit_tail_finishes_with_frozen_ros_time_and_keeps_bag_open(recorder, wall_clock):
    node, clock = recorder
    ready(node, clock)

    class Capture:
        def __init__(self):
            self.closed_at = None
            self.status = {"requested": True, "status": "recording", "exit_code": None, "errors": []}

        def poll(self):
            return self.closed_at is None

        def close(self):
            self.closed_at = wall_clock[0]
            self.status.update(status="finalized", exit_code=0)

    node.bag = Capture()
    node.resources["bag"] = node.bag.status
    measured_hit(node)
    start_wall = wall_clock[0]
    count = node.samples
    wall_clock[0] += 1.0
    assert not node._check_wall_time()
    assert node.samples > count and node.bag.closed_at is None
    wall_clock[0] = start_wall + 2.0
    assert node._check_wall_time()
    assert node.bag.closed_at == start_wall + 2.0
    assert clock[0] == 100.0
    assert summary(node)["reason"] == "hit"
    assert float(summary(node)["intercept_time"]) == 0.0


def test_done_without_measured_hit_records_tail_without_claiming_hit(recorder, wall_clock):
    node, clock = recorder
    ready(node, clock)
    done(node, clock)
    assert not node.finished and not node.hit
    start_wall = wall_clock[0]
    wall_clock[0] += 1.0
    done(node, clock)
    assert node.tail_start_monotonic == start_wall
    wall_clock[0] = start_wall + 2.0
    assert node._check_wall_time()
    result = summary(node)
    assert result["reason"] == "done" and result["hit"] == "False"
    assert math.isnan(float(result["intercept_time"]))
    assert node.storage.meta["first_hit_ros_time"] is None


def test_measured_hit_after_done_gets_full_hit_tail(recorder, wall_clock):
    node, clock = recorder
    ready(node, clock)
    done(node, clock)
    wall_clock[0] += 1.0
    clock[0] += 0.2
    measured_hit(node)
    hit_wall = wall_clock[0]
    wall_clock[0] += 1.0
    assert not node._check_wall_time()
    wall_clock[0] = hit_wall + 2.0
    assert node._check_wall_time()
    assert summary(node)["reason"] == "hit"
    assert float(summary(node)["intercept_time"]) == pytest.approx(0.2)


@pytest.mark.parametrize("reason", ["shutdown", "clock_reset", "bag_error"])
def test_interrupted_hit_tail_is_reported_as_truncated(recorder, wall_clock, reason):
    node, clock = recorder
    ready(node, clock)
    measured_hit(node)
    wall_clock[0] += 0.4
    if reason == "clock_reset":
        clock[0] -= 1.0
        node._check_wall_time()
    elif reason == "bag_error":
        class FailedBag:
            status = {"requested": True, "status": "error", "exit_code": 1, "errors": ["capture failed"]}

            def poll(self):
                return False

            def close(self):
                pass
        node.bag = FailedBag()
        node.resources["bag"] = node.bag.status
        node._check_wall_time()
    else:
        node.finish(reason)
    result = summary(node)
    assert result["reason"] == reason and result["tail_truncated"] == "True"
    assert float(result["post_hit_duration_actual"]) == pytest.approx(0.4)


def test_initial_conditions_are_measured_once_and_separate_from_requested(recorder, wall_clock):
    node, clock = recorder
    requested = {"algorithm": "not-canonical", "distance": 17.0, "target_phase": 0.0}
    node._conditions_cb(String(data=json.dumps(requested)))
    target_state = {"state": "MOVING", "phase": 1.2, "timestamp": 100.0}
    node._target_state_cb(String(data=json.dumps(target_state)))
    odometry = VehicleOdometry()
    odometry.angular_velocity = [0.0, 0.0, 0.4]
    node._cache("odom", odometry)
    clock[0] += 0.5
    ready(node, clock, target_altitude=546.7, target_velocity=(0.0, 2.0, 0.0), self_heading=0.7)
    initial = node.storage.meta["initial_conditions"]
    assert initial["ros_time"] == 100.5
    assert initial["self_position_ned"] == pytest.approx([0.0, 0.0, 0.0])
    assert initial["self_velocity_ned"] == [8.0, 0.0, 0.0]
    assert initial["target_velocity_ned"] == [0.0, 2.0, 0.0]
    assert initial["distance"] == pytest.approx(11.123, abs=0.02)
    assert initial["horizontal_distance"] == pytest.approx(11.119, abs=0.02)
    assert initial["height_difference"] == pytest.approx(0.3, abs=0.001)
    assert initial["relative_position_ned"] == initial["target_position_ned"]
    assert initial["target_speed"] == 2.0
    assert initial["target_horizontal_speed"] == 2.0
    assert initial["target_heading_rad"] == pytest.approx(math.pi / 2)
    assert initial["self_yaw_rad"] == pytest.approx(0.7)
    assert initial["self_body_yaw_rate_rad_s"] == pytest.approx(0.4)
    assert initial["target_phase_rad"] == 1.2
    assert initial["target_state"] == "MOVING"
    assert initial["source_ages_s"]["target_state"] == 0.5
    assert initial["source_ages_s"]["self_gps"] == 0.0
    assert initial["source_ages_s"]["detection"] is None
    assert node.storage.meta["algorithm"] == "pure_png"
    assert node.storage.meta["experiment_conditions"]["requested"] == requested
    node._target_state_cb(String(data='{"state":"MOVING","phase":2.4}'))
    node._conditions_cb(String(data='{"distance":24.0}'))
    clock[0] += 0.1
    ready(node, clock, target_velocity=(3.0, 0.0, 0.0))
    node.finish("shutdown")
    saved = yaml.safe_load((node.storage.path / "meta.yaml").read_text())
    assert saved["initial_conditions"] == initial
    assert saved["initial_conditions"]["target_experiment_state"] == target_state
    assert saved["experiment_conditions"]["target"]["phase"] == 2.4
    assert saved["experiment_conditions"]["requested"]["distance"] == 24.0


def test_unavailable_snapshot_optional_fields_are_null(recorder, wall_clock):
    node, clock = recorder
    node._target_state_cb(String(data='{"state":"MOVING","phase":1.2}'))
    clock[0] += node.config["topic_timeout"] + 0.1
    ready(node, clock, target_velocity=(0.0, 0.0, 0.0), self_heading=math.nan)
    initial = node.storage.meta["initial_conditions"]
    assert initial["target_heading_rad"] is None
    assert initial["self_yaw_rad"] is None
    assert initial["self_body_yaw_rate_rad_s"] is None
    assert initial["target_phase_rad"] is None
    assert initial["target_state"] is None
    assert initial["target_experiment_state"] is None


@pytest.mark.parametrize("payload", ['[]', '{"distance":NaN}', '{"phase":1e999}', 'not json'])
def test_invalid_experiment_json_does_not_replace_valid_metadata(recorder, payload):
    node, _ = recorder
    node._conditions_cb(String(data='{"distance":17.0}'))
    node._conditions_cb(String(data=payload))
    assert node.storage.meta["experiment_conditions"]["requested"] == {"distance": 17.0}


@pytest.mark.parametrize("duration", [-1.0, math.nan, math.inf])
def test_post_hit_duration_rejects_invalid_values(recorder, duration):
    node, _ = recorder
    node.config["post_hit_duration"] = duration
    with pytest.raises(ValueError, match="post_hit_duration"):
        node._validate_config()
    node.config["post_hit_duration"] = 2.0


def test_zero_post_hit_duration_preserves_immediate_completion(recorder, wall_clock):
    node, clock = recorder
    node.config["post_hit_duration"] = 0.0
    ready(node, clock)
    measured_hit(node)
    assert node.finished
    assert summary(node)["reason"] == "hit"
    assert summary(node)["tail_truncated"] == "False"


def test_minimum_distance_includes_tail_without_moving_first_hit_time(recorder, wall_clock):
    node, clock = recorder
    ready(node, clock)
    clock[0] += 0.2
    gps = SensorGps()
    gps.fix_type, gps.latitude_deg, gps.longitude_deg = 3, 47.000005, 8.0
    gps.altitude_ellipsoid_m, gps.altitude_msl_m = 547.0, 500.0
    node._gps_cb("target_gps", gps)
    first_distance = node.minimum_distance
    hit_wall = wall_clock[0]
    assert 0.5 < first_distance < 0.7
    wall_clock[0] += 0.5
    clock[0] += 0.5
    measured_hit(node)
    assert node.minimum_distance == pytest.approx(0.0)
    assert node.first_hit_ros_time == pytest.approx(100.2)
    assert node.tail_start_monotonic == hit_wall
    wall_clock[0] = hit_wall + 2.0
    node._check_wall_time()
    assert float(summary(node)["min_distance"]) == pytest.approx(0.0)
    assert float(summary(node)["intercept_time"]) == pytest.approx(0.2)


def test_repeated_hit_and_done_callbacks_do_not_burst_tail_rows(recorder, wall_clock):
    node, clock = recorder
    ready(node, clock)
    measured_hit(node)
    initial_samples = node.samples
    for _ in range(2):
        clock[0] += 0.01
        wall_clock[0] += 0.01
        measured_hit(node)
        done(node, clock)
        node._tick()
    assert node.samples == initial_samples
    wall_clock[0] = node.tail_start_monotonic + node.config["post_hit_duration"]
    node._check_wall_time()
    with (node.storage.path / "timeseries.csv").open() as stream:
        rows = list(csv.DictReader(stream))
    assert len(rows) == initial_samples + 1
    assert float(summary(node)["intercept_time"]) == 0.0
