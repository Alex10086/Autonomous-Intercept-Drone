"""Run owned, real SITL trials with scheduled circle motion and visual takeover.

This is an experiment supervisor, not a guidance controller. It never publishes
PX4 setpoints or fabricated detections. Every flight uses the actual detector.
"""
import argparse
from collections import deque
from ament_index_python.packages import get_package_share_directory
from datetime import datetime
from functools import partial
import json
import math
import os
from pathlib import Path
import shutil
import statistics
import signal
import subprocess
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from px4_msgs.msg import SensorGps, VehicleLocalPosition, VehicleStatus
from std_msgs.msg import Bool, Float64, String
from uav_common_msg.msg import ExperimentTelemetry, RectMsg

from .recorder_node import gps_coordinates, gps_to_ned


def write_json(path, value):
    temporary = path.with_suffix(path.suffix + '.tmp')
    temporary.write_text(json.dumps(value, indent=2, allow_nan=False) + '\n', encoding='utf-8')
    temporary.replace(path)


class TrialMonitor(Node):
    def __init__(self):
        super().__init__('phase2_supervisor')
        self.latest = {}
        self.target_state = {}
        self.telemetry = None
        self.valid_detection_times = deque()
        self._owned_subscriptions = []
        for ident in (1, 2):
            for suffix, cls in [('vehicle_gps_position', SensorGps),
                                ('vehicle_local_position', VehicleLocalPosition),
                                ('vehicle_status_v1', VehicleStatus)]:
                key = f'{ident}:{suffix}'
                self._owned_subscriptions.append(self.create_subscription(
                    cls, f'/px4_{ident}/fmu/out/{suffix}',
                    partial(self._receive, key), qos_profile_sensor_data))
        self._owned_subscriptions.extend([
            self.create_subscription(String, '/target/experiment_state', self._target, 10),
            self.create_subscription(ExperimentTelemetry, '/experiment/telemetry', self._telemetry, 10),
            self.create_subscription(RectMsg, '/camera_detect_result', self._detect, 10),
        ])
        self.release = self.create_publisher(Bool, '/experiment/start', 10)
        self.target_start = self.create_publisher(Float64, '/experiment/target_start', 10)
        self.conditions = self.create_publisher(String, '/experiment/conditions', 10)

    def _receive(self, key, message):
        self.latest[key] = (message, time.monotonic())

    def _target(self, message):
        try:
            self.target_state = json.loads(message.data)
        except (ValueError, TypeError):
            self.target_state = {}
        self._receive('target_state', message)

    def _telemetry(self, message):
        self.telemetry = message
        self._receive('telemetry', message)

    def _detect(self, message):
        now = time.monotonic()
        if message.width > 0 and message.height > 0:
            self.valid_detection_times.append(now)
        while self.valid_detection_times and now-self.valid_detection_times[0] >= 0.3:
            self.valid_detection_times.popleft()
        self._receive('detection', message)

    def fresh(self, key, timeout=0.5):
        pair = self.latest.get(key)
        return pair[0] if pair and time.monotonic()-pair[1] <= timeout else None

    def gps_geometry(self):
        coordinates = [gps_coordinates(self.fresh(f'{i}:vehicle_gps_position'))
                       if self.fresh(f'{i}:vehicle_gps_position') is not None else None for i in (1,2)]
        if any(c is None for c in coordinates) or coordinates[0][3] != coordinates[1][3]:
            return None
        relative = gps_to_ned(*coordinates[1][:3], *coordinates[0][:3])
        return {'relative_position_ned':list(relative), 'distance':math.sqrt(sum(x*x for x in relative)),
                'height_difference':relative[2]}

    def self_ready(self, altitude):
        local = self.fresh('1:vehicle_local_position')
        status = self.fresh('1:vehicle_status_v1', 2.0)
        telemetry = self.fresh('telemetry')
        return (local is not None and status is not None and telemetry is not None
                and status.arming_state == VehicleStatus.ARMING_STATE_ARMED
                and status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD
                and local.v_xy_valid and local.v_z_valid and local.z_valid
                and abs(local.z-altitude) < 0.12
                and math.sqrt(local.vx**2+local.vy**2+local.vz**2) < 0.25
                and telemetry.state in ('TAKEOFF','SEARCHING'))

    def detector_ready(self):
        now = time.monotonic()
        while self.valid_detection_times and now-self.valid_detection_times[0] >= 0.3:
            self.valid_detection_times.popleft()
        return (len(self.valid_detection_times) >= 3
                and now-self.valid_detection_times[-1] < 0.15)

    def ground_z(self, ident):
        local = self.fresh(f'{ident}:vehicle_local_position')
        status = self.fresh(f'{ident}:vehicle_status_v1', 2.0)
        if (local is None or status is None or not local.z_valid
                or not math.isfinite(local.z)
                or status.arming_state != VehicleStatus.ARMING_STATE_DISARMED):
            return None
        return float(local.z)

    def ros_now(self):
        return self.get_clock().now().nanoseconds*1e-9


class OwnedProcesses:
    def __init__(self, directory):
        self.directory = directory
        self.processes = []

    def start(self, name, command, env=None, cwd=None):
        log = (self.directory / f'{name}.log').open('w', encoding='utf-8')
        process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                   env=env, cwd=cwd, start_new_session=True)
        self.processes.append((name,process,log))
        return process

    def check(self, allow_recorder_exit=False):
        for name,process,_ in self.processes:
            if process.poll() is not None and not (allow_recorder_exit and name=='recorder'):
                raise RuntimeError(f'{name} exited ({process.returncode}); inspect {name}.log')

    def close(self):
        # Recorder closes bag/tail before producers; simulator owns its own PX4 groups.
        for wanted in ('recorder','guidance','detector','target','simulation'):
            for name,process,log in self.processes:
                if name != wanted:
                    continue
                if process.poll() is None:
                    try:
                        if name=='simulation':
                            process.send_signal(signal.SIGTERM)
                        else:
                            os.killpg(process.pid, signal.SIGINT)
                        process.wait(timeout=25 if name=='recorder' else 15)
                    except subprocess.TimeoutExpired:
                        os.killpg(process.pid, signal.SIGTERM)
                        try:
                            process.wait(timeout=5)
                        except subprocess.TimeoutExpired:
                            os.killpg(process.pid, signal.SIGKILL)
                            process.wait()
                    except ProcessLookupError:
                        pass
                log.close()


def ros_command(package, executable, parameters, parameter_files=()):
    command = ['ros2','run',package,executable,'--ros-args']
    for parameter_file in parameter_files:
        command.extend(['--params-file',str(parameter_file)])
    for key,value in parameters.items():
        if isinstance(value,bool):
            value = str(value).lower()
        elif isinstance(value,str):
            value = json.dumps(value)
        command.extend(['-p',f'{key}:={value}'])
    return command


def run_trial(args, session, group, index):
    variant = f'_{args.vision_mode}' if args.algorithm=='vision_png' and args.vision_mode!='combined' else ''
    trial = session / f'{args.algorithm}{variant}_{group}_{index:02d}'
    trial.mkdir()
    requested_distance = 16.8 if group=='near' else 24.0
    center_e = args.near_center_e if group=='near' else args.far_center_e
    env = dict(os.environ, PX4_ROOT=str(args.px4_root), PX4_SELF_POSE='0,0',
               PX4_TARGET_POSE=f'{center_e},0', GZ_PARTITION=f'phase2_{session.name}_{group}_{index}',
               HEADLESS='1')
    requires_detector_entry = (args.algorithm=='vision_png' and args.vision_mode=='combined'
                               and not args.diagnostic_bypass_detector_entry)
    protocol = {
        'protocol':'phase2_fixed_circle_v2','group':group,'trial':index,'algorithm':args.algorithm,
        'vision_mode':args.vision_mode if args.algorithm=='vision_png' else None,
        'attitude_sync':args.attitude_sync if args.algorithm=='vision_png' else None,
        'pixel_estimator':args.pixel_estimator if args.algorithm=='vision_png' else None,
        'requested_distance':requested_distance,'distance_tolerance':0.8,
        'requested_height_difference':0.3,'height_tolerance':0.18,
        'circle_initial_phase':0.0,'circle_angular_velocity':0.5,'circle_radius':5.0,
        'circle_release_command_phase':args.release_phase,
        'motion_age_at_release':(2*math.pi+args.release_phase)/0.5,
        'self_height_above_ground':-args.self_altitude,'target_height_above_ground':5.0,
        'mode':'entry_smoke' if args.smoke else
               ('intercept' if requires_detector_entry else f'diagnostic_{args.vision_mode}'),
        'self_spawn_pose':'0,0','target_spawn_pose':f'{center_e},0',
        'standby_yaw':math.pi/2,'takeoff_altitude_tolerance':0.08,
        'takeoff_speed_tolerance':0.2,'takeoff_settle_time':2.0,
        'entry_condition':'scheduled epoch; latched target ready; target moving; fresh GPS; self ready; combined mode also requires a recent valid detection window',
        'detector_entry_gate':'recent valid window' if requires_detector_entry else
                              'not applied in diagnostic mode',
        'PNG_gains':'kv=kz=4, d_gain=2, yaw=.0008/.0002, k_ey=.0056',
        'speed_cmd':5.0,'hit_radius':0.8,'post_hit_duration':2.0,
    }
    write_json(trial/'protocol.json',protocol)
    monitor = TrialMonitor()
    owned = OwnedProcesses(trial)
    results = {'valid':False,'protocol':protocol}
    release_value = False
    schedule = None
    last_publish = 0.0
    recorder = None

    def step(check=True):
        nonlocal last_publish
        rclpy.spin_once(monitor,timeout_sec=0.01)
        now=time.monotonic()
        if now-last_publish >= 0.05:
            monitor.release.publish(Bool(data=release_value))
            monitor.conditions.publish(String(data=json.dumps(protocol)))
            if schedule is not None:
                monitor.target_start.publish(Float64(data=schedule))
            last_publish=now
        if check:
            owned.check()

    def wait_for(predicate, timeout, label):
        deadline=time.monotonic()+timeout
        while not predicate():
            if time.monotonic()>=deadline:
                raise RuntimeError(f'timeout waiting for {label}')
            step()
        print(f'{trial.name}: {label}',flush=True)

    try:
        owned.start('simulation',['bash',str(args.repo_root/'run_swarm.sh')],env=env)
        wait_for(lambda: all(monitor.fresh(f'{i}:vehicle_gps_position') is not None
                             and monitor.fresh(f'{i}:vehicle_status_v1',2.0) is not None for i in (1,2)),
                 100,'dual PX4 ready')
        # Capture each aircraft's own local datum before either controller can arm.
        wait_for(lambda: all(monitor.ground_z(i) is not None for i in (1,2)),
                 15,'disarmed ground positions ready')
        ground_samples = {1:deque(),2:deque()}
        ground_deadline = time.monotonic()+30.0
        while True:
            step()
            now = time.monotonic()
            for ident, samples in ground_samples.items():
                z = monitor.ground_z(ident)
                if z is None:
                    raise RuntimeError('ground position became invalid or aircraft armed during datum capture')
                samples.append((now,z))
                while len(samples)>1 and samples[1][0]<=now-2.0:
                    samples.popleft()
            if all(now-samples[0][0]>=2.0
                   and max(z for _,z in samples)-min(z for _,z in samples)<=0.05
                   for samples in ground_samples.values()):
                break
            if now>=ground_deadline:
                raise RuntimeError('timeout waiting for stable ground local z (2s range <=0.05m)')
        ground_samples = {i:[z for _,z in samples] for i,samples in ground_samples.items()}
        self_ground_z = statistics.median(ground_samples[1])
        target_ground_z = statistics.median(ground_samples[2])
        self_target_z = self_ground_z+args.self_altitude
        target_target_z = target_ground_z-5.0
        protocol.update(
            self_ground_z=self_ground_z,target_ground_z=target_ground_z,
            self_target_z=self_target_z,target_target_z=target_target_z,
            self_altitude_ned=self_target_z,target_altitude_ned=target_target_z,
            ground_capture={'method':'median disarmed local z after 2s range <=0.05m',
                            'ros_time':monitor.ros_now(),
                            'ranges':{str(i):[min(z),max(z)] for i,z in ground_samples.items()}})
        write_json(trial/'protocol.json',protocol)
        print(f'{trial.name}: ground z self={self_ground_z:.3f} target={target_ground_z:.3f}; '
              f'target z self={self_target_z:.3f} target={target_target_z:.3f}',flush=True)
        duration = 2.0 if args.smoke else args.duration
        recorder=owned.start('recorder',ros_command('uav_bc_recorder','experiment_recorder',{
            'out_dir':str(trial/'recording'),'algorithm':args.algorithm,'profile':'compare',
            'target_motion':'circle','experiment_id':f'{args.vision_mode}_{group}_{index:02d}',
            'max_duration':duration,'start_timeout':180.0,'post_hit_duration':2.0}))
        owned.start('target',ros_command('uav_target_sim','uav_target_sim',{
            'motion_mode':'circle','defer_motion':True,'circle_initial_phase':0.0,
            'circle_angular_velocity':0.5,'circle_radius':5.0,'target_altitude':target_target_z,
            'takeoff_position_tolerance':0.1,'takeoff_speed_tolerance':0.2,'stability_duration':2.0}))
        # Keep real inference running in every mode; truth_los ignores detector output.
        owned.start('detector',ros_command('uav_vision_dectect','uav_vision_dectect',{
            'show_debug_window':False,'use_cuda':True,
            'enable_tracking':args.vision_mode!='yolo_only'}))
        package='uav_vision_png' if args.algorithm=='vision_png' else 'uav_png_intercept'
        guidance_parameters = {
            'require_experiment_start':True,'standby_altitude':self_target_z,
            'standby_yaw':math.pi/2,'takeoff_altitude_tolerance':0.08,
            'takeoff_speed_tolerance':0.2,'takeoff_settle_time':2.0,
            'speed_cmd':5.0,'hit_radius':0.8}
        parameter_files = ()
        if args.algorithm=='vision_png':
            parameter_files = (Path(get_package_share_directory('uav_vision_png'))/'config'/'params.yaml',)
            guidance_parameters['guidance_source'] = (
                'truth_los' if args.vision_mode=='truth_los' else 'vision')
            guidance_parameters['attitude_sync'] = args.attitude_sync == 'image'
            guidance_parameters['pixel_estimator'] = args.pixel_estimator
        else:
            guidance_parameters['N_png'] = 4.0
        owned.start('guidance',ros_command(
            package,package,guidance_parameters,parameter_files=parameter_files))
        wait_for(lambda: monitor.target_state.get('state')=='READY'
                 and monitor.target_state.get('ready_latched') is True
                 and monitor.self_ready(self_target_z),100,'both stabilized')
        if args.smoke:
            # Entry smoke isolates startup prerequisites, not circle takeover range.
            def smoke_ready():
                geometry = monitor.gps_geometry()
                visual_entry_ready = (
                    not requires_detector_entry or monitor.detector_ready())
                return (monitor.self_ready(self_target_z)
                        and monitor.target_state.get('state')=='READY'
                        and monitor.target_state.get('ready_latched') is True
                        and geometry is not None and abs(geometry['height_difference']-0.3)<=0.18
                        and visual_entry_ready)
            wait_for(smoke_ready,15,'GPS height and entry prerequisites ready')
            release_epoch=monitor.ros_now()
            protocol.update(entry_condition='entry smoke: target ready latched; self ready; GPS height within tolerance; combined mode requires detector recent-valid window',
                            distance_gate_applied=False,target_motion_started=False,
                            scheduled_intercept_release_ros_time=release_epoch)
        else:
            schedule=monitor.ros_now()+2.0
            protocol['scheduled_target_start_ros_time']=schedule
            protocol['scheduled_intercept_release_ros_time']=schedule+protocol['motion_age_at_release']
            write_json(trial/'protocol.json',protocol)
            wait_for(lambda: monitor.target_state.get('motion_start_ros_time')==schedule,3,'circle schedule accepted')
            release_epoch=protocol['scheduled_intercept_release_ros_time']
            wait_for(lambda: monitor.ros_now()>=release_epoch,protocol['motion_age_at_release']+3,'release epoch reached')
        geometry=monitor.gps_geometry()
        if geometry is None:
            raise RuntimeError('invalid GPS geometry at scheduled release')
        entry={**geometry,'ros_time':monitor.ros_now(), 'target_state':monitor.target_state,
               'detector_ready':monitor.detector_ready(),'self_ready':monitor.self_ready(self_target_z)}
        protocol['actual_release']=entry
        target_local = monitor.fresh('2:vehicle_local_position')
        if target_local is not None:
            entry['actual_target_phase_rad'] = math.atan2(target_local.y, target_local.x)
            entry['actual_target_radius_m'] = math.hypot(target_local.x, target_local.y)
        write_json(trial/'protocol.json',protocol)
        expected_target_state='READY' if args.smoke else 'MOVING'
        if not entry['self_ready'] or monitor.target_state.get('state')!=expected_target_state:
            raise RuntimeError('takeover readiness violated')
        if not args.smoke and abs(geometry['distance']-requested_distance)>protocol['distance_tolerance']:
            raise RuntimeError(f"takeover distance outside protocol: {geometry['distance']}")
        if abs(geometry['height_difference']-0.3)>protocol['height_tolerance']:
            raise RuntimeError(f"takeover height difference outside protocol: {geometry['height_difference']}")
        if requires_detector_entry and not entry['detector_ready']:
            raise RuntimeError('no recent valid detection window at release; trial invalid')
        release_value=True
        wait_for(lambda: monitor.telemetry is not None and monitor.telemetry.state=='INTERCEPT',0.8,'INTERCEPT entered')
        protocol['observed_intercept_ros_time']=monitor.ros_now()
        protocol['observed_entry_delay']=protocol['observed_intercept_ros_time']-release_epoch
        protocol['entry_delay_limit']=0.8 if args.diagnostic_bypass_detector_entry else 0.3
        if protocol['observed_entry_delay']>protocol['entry_delay_limit']:
            raise RuntimeError('controller entered INTERCEPT too late for configured protocol')
        results['valid']=True
        write_json(trial/'protocol.json',protocol)
        # Smoke observes entry for 2s only; it is not an interception outcome trial.
        # Normal trials include the configured post-hit tail before finalization.
        deadline=time.monotonic()+duration+15
        while recorder.poll() is None:
            if time.monotonic()>deadline:
                raise RuntimeError('recorder did not finish within duration plus tail/finalization allowance')
            step(check=False)
            owned.check(allow_recorder_exit=True)
        if recorder.returncode!=0:
            raise RuntimeError(f'recorder failed ({recorder.returncode})')
        recordings=list((trial/'recording').iterdir())
        if len(recordings)!=1 or not (recordings[0]/'summary.csv').is_file():
            raise RuntimeError('missing unique finalized recording')
        results['recording']=str(recordings[0])
        results['completed']=True
        print(f'{trial.name}: completed {recordings[0]}',flush=True)
    except BaseException as exc:
        results['error']=f'{type(exc).__name__}: {exc}'
        print(f'{trial.name}: FAILED {results["error"]}',flush=True)
        raise
    finally:
        owned.close()
        for filename in ('drone1.log','drone2.log','agent.log','bridge.log'):
            source=args.px4_root/filename
            if source.is_file():
                shutil.copy2(source,trial/filename)
        results['protocol']=protocol
        write_json(trial/'result.json',results)
        monitor.destroy_node()
    return results


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--repo-root',type=Path,default=Path(__file__).resolve().parents[2])
    parser.add_argument('--px4-root',type=Path,default=Path.home()/'PX4-Autopilot')
    parser.add_argument('--out-root',type=Path,default=Path.home()/'intercept_ws/experiment_data/phase2')
    parser.add_argument('--groups',nargs='+',choices=['near','far'],default=['near','far'])
    parser.add_argument('--trials',type=int,default=3)
    parser.add_argument('--algorithm',choices=['vision_png','pure_png'],default='vision_png')
    parser.add_argument('--vision-mode',choices=['combined','yolo_only','truth_los'],
                        default='combined',
                        help='Vision frontend A/B; truth_los is diagnostic-only GPS geometry')
    parser.add_argument('--attitude-sync',choices=['image','receipt'],default='image',
                        help='P0 single-variable A/B: image-time SLERP or receipt-pose baseline')
    parser.add_argument('--pixel-estimator',choices=['raw','kf'],default='raw',
                        help='P2 single-variable A/B: image LOS or current-time pixel KF')
    parser.add_argument('--diagnostic-bypass-detector-entry',action='store_true',
                        help='Diagnostic only: release without the combined-mode detector window gate')
    parser.add_argument('--duration',type=float,default=60.0)
    parser.add_argument('--near-center-e',type=float,default=19.8)
    parser.add_argument('--far-center-e',type=float,default=27.0)
    parser.add_argument('--release-phase',type=float,default=4.4)
    parser.add_argument('--self-altitude',type=float,default=-5.3,
                        help='NED displacement above own measured ground datum (must be negative)')
    parser.add_argument('--smoke',action='store_true',
                        help='Check entry with target holding READY, record 2s; no circle range/phase or hit-rate experiment')
    args=parser.parse_args(argv)
    if args.pixel_estimator != 'raw' and (args.algorithm != 'vision_png' or args.attitude_sync != 'image'):
        parser.error('pixel KF requires vision_png and --attitude-sync image')
    if args.algorithm!='vision_png' and args.vision_mode!='combined':
        parser.error('--vision-mode applies only to --algorithm vision_png')
    if args.algorithm!='vision_png' and args.diagnostic_bypass_detector_entry:
        parser.error('--diagnostic-bypass-detector-entry requires vision_png')
    if args.trials<1 or args.duration<=0 or not math.isfinite(args.duration):
        parser.error('trials and finite duration must be positive')
    for name in ('near_center_e','far_center_e','release_phase','self_altitude'):
        if not math.isfinite(getattr(args,name)):
            parser.error(f'{name} must be finite')
    if args.self_altitude>=0:
        parser.error('self_altitude must be a negative displacement above ground')
    session=args.out_root/datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
    session.mkdir(parents=True)
    print(f'PHASE2_SESSION={session}',flush=True)
    rclpy.init(args=[])
    completed=[]
    try:
        for group in args.groups:
            for index in range(1,args.trials+1):
                completed.append(run_trial(args,session,group,index))
        write_json(session/'session.json',{'trials':completed})
    finally:
        rclpy.shutdown()
