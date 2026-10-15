"""Verify that a transient hover excursion cannot invalidate an accepted start schedule."""
import json
import os
from pathlib import Path
import subprocess
import time
import uuid

from ament_index_python.packages import get_package_prefix
from px4_msgs.msg import VehicleLocalPosition, VehicleStatus
import rclpy
from std_msgs.msg import Float64, String


def test_ready_is_latched_for_simple_float64_schedule(tmp_path):
    context = rclpy.context.Context()
    rclpy.init(args=[], context=context)
    prefix = '/schedule_latch_' + uuid.uuid4().hex
    node = rclpy.create_node('schedule_latch_probe', context=context)
    executor = rclpy.executors.SingleThreadedExecutor(context=context)
    executor.add_node(node)

    executable = os.environ.get('TARGET_SIM_EXECUTABLE') or str(
        Path(get_package_prefix('uav_target_sim')) / 'lib/uav_target_sim/uav_target_sim')
    topics = [
        '/experiment/target_start', '/target/experiment_state',
        '/px4_2/fmu/in/offboard_control_mode',
        '/px4_2/fmu/in/trajectory_setpoint', '/px4_2/fmu/in/vehicle_command',
        '/px4_2/fmu/out/vehicle_local_position', '/px4_2/fmu/out/vehicle_status_v1',
    ]
    command = [
        executable, '--ros-args',
        '-p', 'defer_motion:=true', '-p', 'stability_duration:=0.2',
        '-p', 'takeoff_position_tolerance:=0.1',
        '-p', 'takeoff_speed_tolerance:=0.2',
    ]
    for topic in topics:
        command += ['-r', f'{topic}:={prefix}{topic}']

    states = []
    state_sub = node.create_subscription(
        String, prefix + '/target/experiment_state',
        lambda message: states.append(json.loads(message.data)), 10)
    local_pub = node.create_publisher(
        VehicleLocalPosition, prefix + '/px4_2/fmu/out/vehicle_local_position', 10)
    status_pub = node.create_publisher(
        VehicleStatus, prefix + '/px4_2/fmu/out/vehicle_status_v1', 10)
    start_pub = node.create_publisher(
        Float64, prefix + '/experiment/target_start', 10)

    local = VehicleLocalPosition()
    local.x, local.y, local.z = 5.0, 0.0, -5.0
    local.vx = local.vy = local.vz = 0.0
    local.xy_valid = local.z_valid = local.v_xy_valid = local.v_z_valid = True
    status = VehicleStatus()
    status.arming_state = VehicleStatus.ARMING_STATE_ARMED
    status.nav_state = VehicleStatus.NAVIGATION_STATE_OFFBOARD

    def pump(duration, publish_feedback=True):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            if publish_feedback:
                local_pub.publish(local)
                status_pub.publish(status)
            executor.spin_once(timeout_sec=0.01)

    def wait(predicate, process, timeout=5.0):
        deadline = time.monotonic() + timeout
        while not predicate():
            assert process.poll() is None, 'target simulator exited unexpectedly'
            assert time.monotonic() < deadline, 'timed out waiting for target state'
            pump(0.02)

    log_path = tmp_path / 'target.log'
    with log_path.open('w') as log:
        process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT)
        try:
            wait(lambda: states and states[-1].get('state') == 'READY' and
                 states[-1].get('ready_latched') is True, process, timeout=6.0)

            local.x = 5.3
            local.vx = 1.0
            wait(lambda: states and states[-1].get('stable') is False and
                 states[-1].get('ready_latched') is True and
                 states[-1].get('state') == 'READY', process)

            schedule = node.get_clock().now().nanoseconds * 1e-9 + 1.0
            start_pub.publish(Float64(data=schedule))
            wait(lambda: states and states[-1].get('state') == 'WAITING_START', process)
            assert states[-1]['motion_start_ros_time'] == schedule
        finally:
            process.terminate()
            try:
                process.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=3.0)
            executor.shutdown()
            node.destroy_subscription(state_sub)
            node.destroy_node()
            context.shutdown()
            print(log_path.read_text())
