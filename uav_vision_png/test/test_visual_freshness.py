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
    VehicleLocalPosition, VehicleOdometry, TimesyncStatus,
)
from uav_common_msg.msg import Data, ExperimentTelemetry, RectMsg, VisionObservation, SimulationClock


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
        '/vision/observations', '/simulation/clock_pairs',
        '/px4_1/fmu/in/offboard_control_mode',
        '/px4_1/fmu/in/trajectory_setpoint', '/px4_1/fmu/in/vehicle_command',
        '/px4_1/fmu/out/vehicle_odometry',
        '/px4_1/fmu/out/timesync_status',
        '/px4_1/fmu/out/vehicle_local_position',
        '/px4_1/fmu/out/vehicle_status_v1',
        '/px4_1/fmu/out/hover_thrust_estimate',
        '/px4_1/fmu/out/vehicle_gps_position',
        '/px4_2/fmu/out/vehicle_gps_position',
    ]
    executable = os.environ.get('VISION_PNG_EXECUTABLE') or str(
        Path(get_package_prefix('uav_vision_png')) / 'lib/uav_vision_png/uav_vision_png')
    parameter = getattr(request, 'param', 0.25)
    config = parameter if isinstance(parameter, dict) else {}
    freshness_timeout = config.get('freshness_timeout', 0.25) if isinstance(parameter, dict) else parameter
    guidance_source = config.get('guidance_source', 'vision')
    command = [executable, '--ros-args', '-p', f'freshness_timeout:={freshness_timeout}',
               '-p', 'track_lost_timeout:=1.0',
               '-p', f'guidance_source:={guidance_source}',
               '-p', 'image_time_domain:=ros',
               '-p', f'px4_time_domain:={config.get("px4_time_domain", "ros")}',
               '-p', f'attitude_sync:={str(config.get("attitude_sync", False)).lower()}']
    for topic in topics:
        command += ['-r', f'{topic}:={prefix}{topic}']
    messages = {'data': [], 'telemetry': [], 'command': [], 'observations': []}
    subscriptions = []
    for key, message_type, topic in [
        ('data', Data, '/vpng_data'),
        ('telemetry', ExperimentTelemetry, '/experiment/telemetry'),
        ('command', TrajectorySetpoint, '/px4_1/fmu/in/trajectory_setpoint'),
        ('observations', VisionObservation, '/vision/observations'),
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
        ('clock', SimulationClock, '/simulation/clock_pairs'),
        ('timesync', TimesyncStatus, '/px4_1/fmu/out/timesync_status'),
    ]:
        publishers[key] = node.create_publisher(message_type, prefix + topic, 100)

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
    pose.pose_frame = VehicleOdometry.POSE_FRAME_NED
    local = VehicleLocalPosition()
    local.z, local.z_valid = -6.0, True
    hover = HoverThrustEstimate()
    hover.hover_thrust, hover.hover_thrust_var = 0.5, 0.001

    def sensors():
        pose.timestamp_sample = pose.timestamp = time.time_ns() // 1000
        for key, message in [('pose', pose), ('local', local), ('hover', hover)]:
            publishers[key].publish(message)

    def box(x=1060, y=500, valid=True, stamp_ns=None):
        message = RectMsg()
        stamp_ns = time.time_ns() if stamp_ns is None else stamp_ns
        message.header.stamp.sec, message.header.stamp.nanosec = divmod(stamp_ns, 1_000_000_000)
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


@pytest.mark.parametrize('flight', [{'attitude_sync': True}], indirect=True)
def test_delayed_image_uses_shortest_arc_slerp_and_duplicate_does_not_refresh(flight):
    messages, publishers, pose, _, pump, wait, state, box = flight
    pump(0.15)
    before = time.time_ns() - 100_000_000
    for stamp, yaw in ((before, math.radians(170)), (before + 80_000_000, math.radians(-170))):
        pose.timestamp_sample = pose.timestamp = stamp // 1000
        pose.reset_counter = 1
        pose.q = [math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2)]
        publishers['pose'].publish(pose)
        pump(0.025)
    image_stamp = before + 40_000_000
    box(x=920, y=510, stamp_ns=image_stamp)
    wait(lambda: messages['observations'] and messages['observations'][-1].accepted)
    event = messages['observations'][-1]
    assert event.attitude_synced
    assert event.los_z == pytest.approx(-math.pi / 2, abs=1e-4)
    assert event.receive_pose_los_z == pytest.approx(math.radians(-100), abs=1e-4)
    assert event.los_v == pytest.approx(0.0, abs=1e-5)
    # Replayed old images cannot keep INTERCEPT fresh, even while boxes keep arriving.
    pump(0.35, lambda: box(x=920, y=510, stamp_ns=image_stamp))
    wait(lambda: state('COAST'))
    assert messages['observations'][-1].status == VisionObservation.OUT_OF_ORDER
    assert not messages['observations'][-1].accepted


@pytest.mark.parametrize('flight', [{'attitude_sync': True}], indirect=True)
@pytest.mark.parametrize('outside_history', [False, True], ids=['large-gap', 'no-extrapolation'])
def test_unmatched_attitude_never_falls_back_to_receipt_pose(flight, outside_history):
    messages, publishers, pose, _, pump, wait, state, box = flight
    before = time.time_ns() - 250_000_000
    for stamp in (before, before + 200_000_000):
        pose.timestamp_sample = pose.timestamp = stamp // 1000
        pose.reset_counter = 1
        publishers['pose'].publish(pose)
        pump(0.025)
    image_stamp = before - 10_000_000 if outside_history else before + 100_000_000
    box(stamp_ns=image_stamp)
    wait(lambda: bool(messages['observations']))
    assert messages['observations'][-1].status == VisionObservation.ATTITUDE_UNAVAILABLE
    assert not messages['observations'][-1].accepted
    assert state('SEARCHING')


@pytest.mark.parametrize('flight', [{'attitude_sync': True}], indirect=True)
@pytest.mark.parametrize('future_arrives', [True, False], ids=['late-bracket', 'wait-expiry'])
def test_late_attitude_is_bounded_without_receipt_pose_fallback(flight, future_arrives):
    messages, publishers, pose, _, pump, wait, state, box = flight
    before = time.time_ns() - 40_000_000
    pose.timestamp_sample = pose.timestamp = before // 1000
    pose.reset_counter = 1
    publishers['pose'].publish(pose)
    pump(0.02)
    box(x=920, y=510, stamp_ns=before + 20_000_000)
    pump(0.035)
    assert state('SEARCHING')
    assert not any(event.accepted for event in messages['observations'])
    if future_arrives:
        pose.timestamp_sample = pose.timestamp = (before + 40_000_000) // 1000
        pose.q = [math.cos(0.2), 0.0, 0.0, math.sin(0.2)]
        publishers['pose'].publish(pose)
        wait(lambda: messages['observations'] and messages['observations'][-1].accepted)
        event = messages['observations'][-1]
        assert event.los_z == pytest.approx(math.pi / 2 - 0.2, abs=1e-4)
        assert event.control_time - event.bbox_receive_time >= 0.025
    else:
        wait(lambda: bool(messages['observations']))
        assert messages['observations'][-1].status == VisionObservation.ATTITUDE_UNAVAILABLE
        assert not messages['observations'][-1].accepted
        pose.timestamp_sample = pose.timestamp = (before + 40_000_000) // 1000
        publishers['pose'].publish(pose)
        pump(0.04)
        assert state('SEARCHING')


@pytest.mark.parametrize('flight', [
    {'attitude_sync': True, 'px4_time_domain': 'gazebo_dds'}], indirect=True)
def test_native_clock_recovers_drifting_dds_epoch_before_slerp(flight):
    messages, publishers, pose, _, pump, wait, _, box = flight
    epoch_us = time.time_ns() // 1000
    # DDS says samples are 200ms older, but raw PX4 HRT is Gazebo simulation time.
    estimated_offset = -(epoch_us - 10_000_000 - 200_000)
    for sim_us in range(9_800_000, 10_000_001, 5_000):
        clock = SimulationClock()
        clock.sim_time.sec, rem = divmod(sim_us, 1_000_000)
        clock.sim_time.nanosec = rem * 1000
        system_us = epoch_us + sim_us - 10_000_000
        clock.system_time.sec, rem = divmod(system_us, 1_000_000)
        clock.system_time.nanosec = rem * 1000
        publishers['clock'].publish(clock)
    status = TimesyncStatus()
    status.source_protocol = TimesyncStatus.SOURCE_PROTOCOL_DDS
    status.estimated_offset = estimated_offset
    status.timestamp = 9_950_000 - estimated_offset
    status.remote_timestamp = epoch_us - 50_000
    status.round_trip_time = 1000
    publishers['timesync'].publish(status)
    pump(0.03)
    for sim_us, yaw in ((9_900_000, 0.0), (9_980_000, 0.4)):
        pose.timestamp_sample = pose.timestamp = sim_us - estimated_offset
        pose.reset_counter = 2
        pose.q = [math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2)]
        publishers['pose'].publish(pose)
        pump(0.025)
    image_ns = (epoch_us - 60_000) * 1000
    box(x=920, y=510, stamp_ns=image_ns)
    wait(lambda: messages['observations'] and messages['observations'][-1].accepted)
    event = messages['observations'][-1]
    assert event.los_z == pytest.approx(math.pi / 2 - 0.2, abs=1e-4)
    assert event.attitude_before_time <= event.image_time_ros <= event.attitude_after_time
    assert event.attitude_after_time - event.attitude_before_time == pytest.approx(0.08, abs=1e-5)
