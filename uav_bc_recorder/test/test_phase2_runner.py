"""Receipt-window readiness and independent local ground datums."""
import math

import pytest
import rclpy
from px4_msgs.msg import SensorGps, VehicleLocalPosition, VehicleStatus
from uav_common_msg.msg import ExperimentTelemetry, RectMsg

from uav_bc_recorder import phase2_runner as runner


@pytest.fixture
def monitor(monkeypatch):
    clock = [100.0]
    monkeypatch.setattr(runner.time, 'monotonic', lambda: clock[0])
    rclpy.init(args=[])
    node = runner.TrialMonitor()
    try:
        yield node, clock
    finally:
        node.destroy_node()
        rclpy.shutdown()


def detection(valid=True):
    message = RectMsg()
    message.width, message.height = (80, 60) if valid else (-1, -1)
    return message


def test_ros_command_loads_shared_params_before_explicit_overrides():
    command = runner.ros_command(
        'pkg', 'node', {'kv': 4.0, 'enabled': True},
        parameter_files=('/tmp/shared.yaml',))
    assert command == [
        'ros2', 'run', 'pkg', 'node', '--ros-args',
        '--params-file', '/tmp/shared.yaml',
        '-p', 'kv:=4.0',
        '-p', 'enabled:=true',
    ]


def test_brief_invalid_retains_recent_valid_history_but_does_not_refresh_age(monitor):
    node, clock = monitor
    for timestamp in (100.0, 100.1, 100.2):
        clock[0] = timestamp
        node._detect(detection())
    clock[0] = 100.23
    node._detect(detection(False))
    assert node.detector_ready()
    clock[0] = 100.351
    node._detect(detection(False))
    assert not node.detector_ready()


def test_old_detections_cannot_satisfy_recent_three_frame_requirement(monitor):
    node, clock = monitor
    for timestamp in (100.0, 100.01, 100.02, 100.35):
        clock[0] = timestamp
        node._detect(detection())
    assert not node.detector_ready()
    clock[0] = 100.4
    node._detect(detection())
    assert not node.detector_ready()
    clock[0] = 100.45
    node._detect(detection())
    assert node.detector_ready()
    clock[0] = 100.751
    assert not node.detector_ready()


def test_independent_ground_datums_and_compensated_readiness_use_gps_geometry(monitor):
    node, _ = monitor
    for ident, z in ((1, 1.4), (2, -0.7)):
        local = VehicleLocalPosition()
        local.z, local.z_valid = z, True
        local.v_xy_valid = local.v_z_valid = True
        status = VehicleStatus()
        status.arming_state = VehicleStatus.ARMING_STATE_DISARMED
        node._receive(f'{ident}:vehicle_local_position', local)
        node._receive(f'{ident}:vehicle_status_v1', status)
    self_target_z = node.ground_z(1) - 5.3
    target_target_z = node.ground_z(2) - 5.0
    assert self_target_z == pytest.approx(-3.9)
    assert target_target_z == pytest.approx(-5.7)

    local.z = target_target_z
    node._receive('2:vehicle_local_position', local)
    own = VehicleLocalPosition()
    own.z, own.z_valid = self_target_z, True
    own.v_xy_valid = own.v_z_valid = True
    node._receive('1:vehicle_local_position', own)
    status = VehicleStatus()
    status.arming_state = VehicleStatus.ARMING_STATE_ARMED
    status.nav_state = VehicleStatus.NAVIGATION_STATE_OFFBOARD
    node._receive('1:vehicle_status_v1', status)
    telemetry = ExperimentTelemetry()
    telemetry.state = 'SEARCHING'
    node._telemetry(telemetry)
    assert node.self_ready(self_target_z)
    assert not node.self_ready(-5.3)
    assert node.ground_z(1) is None

    for ident, altitude in ((1, 505.3), (2, 505.0)):
        gps = SensorGps()
        gps.fix_type, gps.latitude_deg, gps.longitude_deg = 3, 47.0, 8.0
        gps.altitude_ellipsoid_m = altitude
        node._receive(f'{ident}:vehicle_gps_position', gps)
    geometry = node.gps_geometry()
    assert geometry['height_difference'] == pytest.approx(0.3, abs=1e-5)
    assert geometry['distance'] == pytest.approx(0.3, abs=1e-5)


@pytest.mark.parametrize('z,valid', [(math.nan, True), (1.0, False)])
def test_ground_capture_rejects_invalid_height(monitor, z, valid):
    node, _ = monitor
    local = VehicleLocalPosition()
    local.z, local.z_valid = z, valid
    status = VehicleStatus()
    status.arming_state = VehicleStatus.ARMING_STATE_DISARMED
    node._receive('1:vehicle_local_position', local)
    node._receive('1:vehicle_status_v1', status)
    assert node.ground_z(1) is None
