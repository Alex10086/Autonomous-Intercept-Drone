"""Deterministic receipt-clock scenarios exercise actual CSV and summary output."""
import csv
import math

import pytest
import rclpy
from px4_msgs.msg import SensorGps, TrajectorySetpoint, VehicleLocalPosition
from uav_common_msg.msg import ExperimentTelemetry, RectMsg

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


def ready(node, clock, algorithm="pure_png"):
    for key, latitude in (("self_gps", 47.0), ("target_gps", 47.0001)):
        gps = SensorGps()
        gps.fix_type, gps.latitude_deg, gps.longitude_deg = 3, latitude, 8.0
        gps.altitude_ellipsoid_m, gps.altitude_msl_m = 547.0, 500.0
        node._gps_cb(key, gps)
    for key in ("self_local", "target_local"):
        local = VehicleLocalPosition()
        local.v_xy_valid = local.v_z_valid = True
        local.vx, local.vy, local.vz = 8.0, 0.0, 0.0
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
