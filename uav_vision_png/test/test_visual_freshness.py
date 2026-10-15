"""Exercise the real node with isolated topics; never send commands to PX4."""
import math
import os
from pathlib import Path
import subprocess
import time
import uuid

import pytest
import rclpy
from ament_index_python.packages import get_package_prefix
from px4_msgs.msg import (
    HoverThrustEstimate, SensorGps, TrajectorySetpoint,
    VehicleLocalPosition, VehicleOdometry,
)
from uav_common_msg.msg import Data, ExperimentTelemetry, RectMsg


@pytest.fixture
def flight(tmp_path, request):
    context = rclpy.context.Context()
    rclpy.init(args=[], context=context)
    prefix = '/freshness_' + uuid.uuid4().hex
    node = rclpy.create_node('visual_freshness_probe', context=context)
    executor = rclpy.executors.SingleThreadedExecutor(context=context)
    executor.add_node(node)
    topics = [
        '/camera_detect_result', '/experiment/start', '/vpng_data',
        '/experiment/telemetry',
        '/px4_1/fmu/in/offboard_control_mode',
        '/px4_1/fmu/in/trajectory_setpoint', '/px4_1/fmu/in/vehicle_command',
        '/px4_1/fmu/out/vehicle_odometry',
        '/px4_1/fmu/out/vehicle_local_position',
        '/px4_1/fmu/out/vehicle_status_v1',
        '/px4_1/fmu/out/hover_thrust_estimate',
        '/px4_1/fmu/out/vehicle_gps_position',
        '/px4_2/fmu/out/vehicle_gps_position',
    ]
    executable = os.environ.get('VISION_PNG_EXECUTABLE') or str(
        Path(get_package_prefix('uav_vision_png')) / 'lib/uav_vision_png/uav_vision_png')
    config = request.param if isinstance(getattr(request, 'param', None), dict) else {}
    freshness_timeout = config.get('freshness_timeout', getattr(request, 'param', 0.25))
    guidance_source = config.get('guidance_source', 'vision')
    command = [executable, '--ros-args', '-p', f'freshness_timeout:={freshness_timeout}',
               '-p', 'track_lost_timeout:=1.0',
               '-p', f'guidance_source:={guidance_source}']
    for topic in topics:
        command += ['-r', f'{topic}:={prefix}{topic}']
    messages = {'data': [], 'telemetry': [], 'command': []}
    subscriptions = []
    for key, message_type, topic in [
        ('data', Data, '/vpng_data'),
        ('telemetry', ExperimentTelemetry, '/experiment/telemetry'),
        ('command', TrajectorySetpoint, '/px4_1/fmu/in/trajectory_setpoint'),
    ]:
        subscriptions.append(node.create_subscription(
            message_type, prefix + topic, messages[key].append, 100))
    publishers = {}
    for key, message_type, topic in [
        ('box', RectMsg, '/camera_detect_result'),
        ('pose', VehicleOdometry, '/px4_1/fmu/out/vehicle_odometry'),
        ('local', VehicleLocalPosition, '/px4_1/fmu/out/vehicle_local_position'),
        ('hover', HoverThrustEstimate, '/px4_1/fmu/out/hover_thrust_estimate'),
        ('self_gps', SensorGps, '/px4_1/fmu/out/vehicle_gps_position'),
        ('target_gps', SensorGps, '/px4_2/fmu/out/vehicle_gps_position'),
    ]:
        publishers[key] = node.create_publisher(message_type, prefix + topic, 10)

    def pump(duration, action=None):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            if action:
                action()
            executor.spin_once(timeout_sec=0.005)

    def wait(predicate, timeout=4.0, action=None):
        deadline = time.monotonic() + timeout
        while not predicate():
            assert process.poll() is None, 'Vision PNG exited unexpectedly'
            assert time.monotonic() < deadline, 'Timed out waiting for node behavior'
            pump(0.01, action)

    def state(name):
        return bool(messages['telemetry']) and messages['telemetry'][-1].state == name

    pose = VehicleOdometry()
    pose.q = [1.0, 0.0, 0.0, 0.0]
    local = VehicleLocalPosition()
    local.z, local.z_valid = -6.0, True
    hover = HoverThrustEstimate()
    hover.hover_thrust, hover.hover_thrust_var = 0.5, 0.001

    def sensors():
        for key, message in [('pose', pose), ('local', local), ('hover', hover)]:
            publishers[key].publish(message)

    def box(x=1060, y=500, valid=True):
        message = RectMsg()
        message.header.stamp = node.get_clock().now().to_msg()
        message.x, message.y = x, y
        message.width, message.height = (80, 60) if valid else (-1, -1)
        publishers['box'].publish(message)

    log_path = tmp_path / 'vision.log'
    with log_path.open('w') as log:
        process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT)
        try:
            wait(lambda: state('SEARCHING'), timeout=8.0, action=sensors)
            yield messages, publishers, pose, local, pump, wait, state, box
        finally:
            process.terminate()
            try:
                process.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=3.0)
            executor.shutdown()
            node.destroy_node()
            context.shutdown()
            print(log_path.read_text())


def test_one_observation_is_not_reprojected_as_attitude_changes(flight):
    messages, publishers, pose, _, pump, wait, state, box = flight
    box()
    wait(lambda: state('INTERCEPT') and messages['telemetry'][-1].guidance_valid)
    first = messages['telemetry'][-1]
    assert first.los_z == pytest.approx(math.atan2(1397.2, 140.0), abs=1e-5)
    pose.q = [math.cos(0.25), 0.0, 0.0, math.sin(0.25)]
    pump(0.12, lambda: publishers['pose'].publish(pose))
    telemetry = messages['telemetry'][-1]
    assert telemetry.los_z == pytest.approx(first.los_z, abs=1e-5)
    assert telemetry.png_z == pytest.approx(first.png_z, abs=1e-5)
    # Equal pixel coordinates from another frame must still use its own new pose.
    box()
    wait(lambda: bool(messages['data']) and
         abs(messages['data'][-1].los_angle_z - first.los_z) > 0.1)
    assert messages['data'][-1].los_angle_z == pytest.approx(first.los_z - 0.5, abs=1e-5)


@pytest.mark.parametrize('flight', [0.15], indirect=True)
def test_brief_invalid_detection_preserves_guidance_and_history(flight):
    messages, _, _, _, pump, wait, state, box = flight
    box()
    wait(lambda: state('INTERCEPT') and messages['telemetry'][-1].guidance_valid)
    first = messages['data'][-1]
    data_count = len(messages['data'])
    telemetry_count = len(messages['telemetry'])

    pump(0.04, lambda: box(valid=False))
    assert len(messages['data']) == data_count
    assert state('INTERCEPT')
    assert messages['telemetry'][-1].guidance_valid
    for telemetry in messages['telemetry'][telemetry_count:]:
        assert telemetry.state == 'INTERCEPT'
        assert telemetry.guidance_valid
        assert telemetry.los_z == pytest.approx(first.los_angle_z, abs=1e-5)
        assert telemetry.png_z == pytest.approx(first.d_v_angle_z, abs=1e-5)

    box(x=1200)
    wait(lambda: len(messages['data']) > data_count)
    next_frame = messages['data'][-1]
    delta = next_frame.los_angle_z - first.los_angle_z
    assert abs(delta) > 0.02
    assert next_frame.diff_los_angle_z == pytest.approx(delta, abs=1e-5)
    assert next_frame.d_v_angle_z == pytest.approx(
        first.d_v_angle_z + 4.0 * delta, abs=1e-5)
    expected_yaw_rate = 0.0008 * 280.0 + 0.0002 * (280.0 - 140.0)
    wait(lambda: abs(messages['command'][-1].yawspeed - expected_yaw_rate) < 1e-5)


@pytest.mark.parametrize('invalid_stream', [False, True], ids=['silent-detector', 'no-target'])
def test_loss_coast_recovery_and_hit_evaluation(flight, invalid_stream):
    messages, publishers, pose, local, pump, wait, state, box = flight
    box()
    wait(lambda: state('INTERCEPT') and bool(messages['data']))
    box(x=750)
    wait(lambda: abs(messages['data'][-1].ex + 170.0) < 0.01)
    action = (lambda: box(valid=False)) if invalid_stream else None
    wait(lambda: state('COAST'), action=action)
    command = messages['command'][-1]
    cached = tuple(command.velocity), command.yawspeed
    local.vx = 4.0
    pump(0.1, lambda: publishers['local'].publish(local))
    assert tuple(messages['command'][-1].velocity) == pytest.approx(cached[0])
    assert messages['command'][-1].yawspeed == pytest.approx(cached[1])
    # A COAST recovery reinitializes both LOS increments and yaw pixel history.
    box(x=1200)
    wait(lambda: state('INTERCEPT') and abs(messages['data'][-1].ex - 280.0) < 0.01)
    recovered = messages['data'][-1]
    assert recovered.diff_los_angle_v == pytest.approx(0.0)
    assert recovered.diff_los_angle_z == pytest.approx(0.0)
    assert recovered.d_v_angle_z == pytest.approx(recovered.los_angle_z)
    wait(lambda: abs(messages['command'][-1].yawspeed - 0.0008 * 280.0) < 1e-5)
    wait(lambda: state('TRACK_LOST'), action=action)
    assert tuple(messages['command'][-1].velocity) == pytest.approx((0.0, 0.0, 0.0))
    # Long-loss recovery also starts a new visual history, not a cross-gap delta.
    pose.q = [math.cos(0.2), 0.0, 0.0, math.sin(0.2)]
    pump(0.04, lambda: publishers['pose'].publish(pose))
    box(x=900)
    wait(lambda: state('INTERCEPT') and abs(messages['data'][-1].ex + 20.0) < 0.01)
    assert messages['data'][-1].diff_los_angle_z == pytest.approx(0.0)
    assert messages['data'][-1].d_v_angle_z == pytest.approx(messages['data'][-1].los_angle_z)
    wait(lambda: abs(messages['command'][-1].yawspeed + 0.0008 * 20.0) < 1e-5)
    box(valid=False)
    wait(lambda: state('COAST'))
    gps = SensorGps()
    gps.latitude_deg, gps.longitude_deg, gps.altitude_msl_m = 47.0, 8.0, 500.0
    pump(0.04, lambda: publishers['self_gps'].publish(gps))
    pump(0.04, lambda: publishers['target_gps'].publish(gps))
    wait(lambda: state('DONE'))
    assert tuple(messages['command'][-1].velocity) == pytest.approx((0.0, 0.0, 0.0))


@pytest.mark.parametrize(
    'flight', [{'freshness_timeout': 0.25, 'guidance_source': 'truth_los'}],
    indirect=True)
def test_truth_los_uses_geometry_without_detection_messages(flight):
    messages, publishers, _, _, _, wait, state, _ = flight
    own = SensorGps()
    own.latitude_deg, own.longitude_deg, own.altitude_msl_m = 47.0, 8.0, 500.0
    target = SensorGps()
    target.latitude_deg = own.latitude_deg + 0.0001
    target.longitude_deg, target.altitude_msl_m = own.longitude_deg, own.altitude_msl_m

    def geometry():
        publishers['self_gps'].publish(own)
        publishers['target_gps'].publish(target)

    wait(lambda: state('INTERCEPT') and messages['telemetry'][-1].guidance_valid
         and bool(messages['data']), timeout=4.0, action=geometry)
    assert messages['data'][-1].los_angle_z == pytest.approx(math.pi / 2.0, abs=2e-3)
    assert messages['data'][-1].ex == pytest.approx(0.0, abs=2.0)
