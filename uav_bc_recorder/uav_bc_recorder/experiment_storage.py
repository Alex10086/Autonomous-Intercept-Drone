"""Durable per-run files and owned optional capture resources (no ROS imports)."""
import csv
from datetime import datetime
import math
import os
from pathlib import Path
import re
import shutil
import signal
import subprocess

import yaml

NAN = float("nan")
COLUMNS = (
    "timestamp elapsed_s self_x self_y self_z self_vx self_vy self_vz "
    "self_qw self_qx self_qy self_qz self_wx self_wy self_wz "
    "target_x target_y target_z target_vx target_vy target_vz distance "
    "detected bbox_x bbox_y bbox_w bbox_h confidence ex ey los_v los_z "
    "dlos_v dlos_z png_v png_z ref_vx ref_vy ref_vz cmd_thrust cmd_tx cmd_ty "
    "cmd_tz solver_time_ms cost solver_status state vision_latency_ms lost reacquired hit "
    "telemetry_age_s self_gps_age_s target_gps_age_s self_local_age_s "
    "target_local_age_s odom_age_s detection_age_s vpng_age_s trajectory_age_s "
    "thrust_age_s torque_age_s"
).split()
OBSERVATION_COLUMNS = (
    "image_stamp_s image_time_ros bbox_receive_time projection_time control_time "
    "attitude_before_time attitude_after_time image_to_bbox_ms image_to_control_ms "
    "clock_valid accepted attitude_synced status source confidence x y width height "
    "los_v los_z receive_pose_los_v receive_pose_los_z estimator estimate_time_ros "
    "estimated_u estimated_v estimated_du estimated_dv pixel_prediction_dt "
    "attitude_prediction_dt control_los_v control_los_z"
).split()
GEOMETRY_COLUMNS = "vehicle source_time receive_time x y z source_time_dds estimated_offset_us".split()
CLOCK_OFFSET_COLUMNS = (
    "vehicle receive_time timestamp_dds remote_time estimated_offset_us "
    "observed_offset_us round_trip_time_us"
).split()
SUMMARY_COLUMNS = (
    "status reason hit intercept_time elapsed_s min_distance samples detection_count "
    "lost_count reacquire_count latency_mean_ms latency_max_ms video_frames "
    "bag_status bag_exit_code bag_errors video_status video_errors recording_errors "
    "post_hit_duration_actual tail_truncated"
).split()
TOPICS = {
    "telemetry": "/experiment/telemetry",
    "vpng": "/vpng_data",
    "detection": "/camera_detect_result",
    "self_gps": "/px4_1/fmu/out/vehicle_gps_position",
    "target_gps": "/px4_2/fmu/out/vehicle_gps_position",
    "self_local": "/px4_1/fmu/out/vehicle_local_position",
    "target_local": "/px4_2/fmu/out/vehicle_local_position",
    "odom": "/px4_1/fmu/out/vehicle_odometry",
    "trajectory": "/px4_1/fmu/in/trajectory_setpoint",
    "thrust": "/px4_1/fmu/in/vehicle_thrust_setpoint",
    "torque": "/px4_1/fmu/in/vehicle_torque_setpoint",
    "conditions": "/experiment/conditions",
    "target_state": "/target/experiment_state",
    "experiment_start": "/experiment/start",
    "target_start": "/experiment/target_start",
    "clock_pairs": "/simulation/clock_pairs",
    "self_timesync": "/px4_1/fmu/out/timesync_status",
    "target_timesync": "/px4_2/fmu/out/timesync_status",
    "observations": "/vision/observations",
}
COMPARE_TOPICS = list(dict.fromkeys(TOPICS[key] for key in (
    "telemetry", "vpng", "detection", "self_gps", "target_gps",
    "self_local", "target_local", "trajectory", "conditions", "target_state",
    "experiment_start", "target_start", "clock_pairs", "observations", "odom",
    "self_timesync", "target_timesync")))
ARCHIVE_TOPICS = list(dict.fromkeys(
    COMPARE_TOPICS + [TOPICS[key] for key in ("thrust", "torque")] + ["/clock"]))


def safe_identifier(value, name):
    if not isinstance(value, str) or not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_.-]*", value) or value in (".", ".."):
        raise ValueError(f"{name} must be a nonempty safe identifier [A-Za-z0-9_.-]")
    return value


def sync_directory(path):
    descriptor = os.open(path, os.O_RDONLY | os.O_DIRECTORY)
    try:
        os.fsync(descriptor)
    finally:
        os.close(descriptor)


def atomic_yaml(path, value):
    temporary = path.with_suffix(path.suffix + ".tmp")
    with temporary.open("w", encoding="utf-8") as stream:
        yaml.safe_dump(value, stream, sort_keys=False)
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(temporary, path)
    sync_directory(path.parent)


class RunStorage:
    def __init__(self, config):
        root = Path(config["out_dir"]).expanduser().resolve()
        root.mkdir(parents=True, exist_ok=True)
        prefix = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        prefix += f"_{config['algorithm']}_{config['target_motion']}"
        if config["experiment_id"]:
            prefix += f"_{config['experiment_id']}"
        index = 1
        while True:
            self.path = root / f"{prefix}_{index:03d}"
            try:
                self.path.mkdir()
                break
            except FileExistsError:
                index += 1
        sync_directory(root)
        self.stream = None
        self.sidecar_streams = {}
        self.sidecar_writers = {}
        self.meta = {
            "algorithm": config["algorithm"], "target_motion": config["target_motion"],
            "experiment_id": config["experiment_id"] or f"{index:03d}",
            "scenario": config["scenario"], "profile": config["profile"],
            "parameters": {key: None for key in ("N_png", "speed_cmd", "hit_radius", "planner", "controller")},
            **{key: None for key in ("N_png", "speed_cmd", "hit_radius", "planner", "controller")},
            "git_commit": self._git_commit(), "recorder_config": dict(config),
            "git_dirty": self._git_dirty(),
            "source_path": str(Path(__file__).resolve()),
            "experiment_conditions": {"requested": None, "target": None},
            "initial_conditions": None,
            "first_hit_ros_time": None, "first_hit_monotonic": None,
            "topics": dict(TOPICS, camera=config["camera_topic"]),
            "provenance": {
                "timestamp": "ROS clock seconds; max_duration measured from eligible INTERCEPT start",
                "start_timeout": "monotonic wall seconds from node initialization; independent of ROS /clock",
                "elapsed_s": "ROS seconds since first eligible INTERCEPT observation",
                "intercept_time": "ROS seconds from eligible INTERCEPT to first measured distance <= hit_radius; excludes post-hit tail",
                "post_hit_duration": "monotonic wall seconds from first measured hit (or DONE without a hit); max_duration and DONE cannot truncate an active hit tail",
                "min_distance": "minimum finite measured GPS separation over all recorded samples, including post-hit tail",
                "initial_conditions": "single snapshot at first eligible INTERCEPT; positions in common GPS-origin NED, velocities in local NED; unavailable optional fields are null",
                "height_difference": "target_down - self_down in meters; positive means target below self",
                "target_heading_rad": "atan2(target_vE, target_vN); null when horizontal velocity is zero",
                "self_yaw_rad": "fresh self local heading, with NED odometry quaternion fallback; radians",
                "self_body_yaw_rate_rad_s": "fresh odometry body-FRD angular_velocity z, not Euler yaw derivative",
                "experiment_conditions": "latest valid requested protocol and target JSON stored separately from immutable measured initial_conditions",
                "freshness": "receipt ROS time; valid nonzero ROS header additionally gates telemetry. Detection receipt freshness does not guarantee source image age (camera stamp may use another clock)",
                "positions": "WGS84 ECEF to common NED at first valid self GPS; meters; ellipsoid altitude preferred. MSL fallback is approximate; mixed altitude datums are rejected",
                "velocities": "PX4 local NED, m/s; independent local positions never mixed",
                "quaternion": "PX4 VehicleOdometry q, wxyz, NED reference",
                "angular_velocity": "PX4 VehicleOdometry angular_velocity, body FRD, rad/s",
                "guidance": "NED radians; azimuth atan2(North,East); dlos radians/update",
                "cmd_thrust": "negative body-FRD thrust xyz[2], normalized (not Newtons)",
                "cmd_torque": "body FRD xyz, normalized (not SI torque)",
                "vision_latency_ms": "receipt ROS time minus original image ROS stamp only when nonzero and within topic_timeout; incompatible epochs/invalid/future stamps yield NaN",
                "detection_count": "valid fresh detection callbacks, not sampled timer snapshots",
                "missing_values": "NaN numeric values; false stale detection; empty unavailable strings",
                "observations.csv": "one diagnostic event per bbox, including rejected and never-consumed observations; not timer snapshots; absent publishers leave a header-only file",
                "observation_clock": "image_stamp_s retains camera simulation domain; image_time_ros maps to system epoch via /simulation/clock_pairs only when clock_valid; unmapped image and derived latency values are NaN; control_time is first velocity setpoint using the image, or NaN",
                "observation_attitude": "attitude_before_time/attitude_after_time bracket image_time_ros; attitude_synced distinguishes image-pose from receive-pose projection; LOS radians, azimuth atan2(North,East)",
                "observation_status": "0 RECEIVE_POSE, 1 IMAGE_POSE, 2 CLOCK_UNAVAILABLE, 3 ATTITUDE_UNAVAILABLE, 4 INVALID_BOX, 5 NOT_GUIDING, 6 OUT_OF_ORDER, 7 ESTIMATOR_UNAVAILABLE",
                "observation_source": "0 UNKNOWN, 1 YOLO, 2 LIGHTTRACK",
                "observation_prediction": "estimator 0 raw / 1 pixel CV KF; los_v/z remain raw image measurement LOS, control_los_v/z are actual guidance LOS; estimate_time_ros is estimate epoch; estimated_u/v pixels and estimated_du/dv pixels per system second; pixel_prediction_dt and attitude_prediction_dt seconds; current attitude uses at most 50ms body-rate propagation, never stale-pose fallback",
                "geometry.csv": "one row per valid GPS callback, including before INTERCEPT; self/target positions in the same GPS-origin NED as timeseries; callbacks before origin are retained until origin exists, or written with NaN positions at finish if no origin arrives; incompatible altitude datums omitted",
                "geometry_source_time": "gazebo_dds: signed integer SensorGps.timestamp + aircraft DDS estimated_offset recovers native Gazebo/PX4 HRT microseconds, then bounded /simulation/clock_pairs interpolation gives ROS/system epoch; ros: timestamp * 1e-6 only if positive and 0 <= receipt - source <= topic_timeout; unavailable mappings are NaN; timestamp_sample is never used",
                "geometry_receive_time": "original recorder ROS receipt epoch seconds, unchanged by clock recovery or pre-origin queuing",
                "geometry_clock_inputs": "source_time_dds retains SensorGps.timestamp seconds; estimated_offset_us is the latest accepted aircraft-specific DDS status offset used for recovery, or NaN if unavailable; accepted offset steps >20ms reset validation, but status loss and DDS topic boundaries still leave residual timing ambiguity, not a <=20ms error guarantee; raw status/sample inputs remain auditable; no receipt-derived offset or fallback",
                "clock_offsets.csv": "every DDS TimesyncStatus callback for both aircraft, including before INTERCEPT and rejected/unmapped statuses; timestamp_dds and unmodified Agent remote_time are seconds; estimated/observed offsets and RTT are microseconds; non-DDS protocols are ignored",
                "geometry_clock_validation": "status age <=2s; original recovered status and GPS sample must map without native-clock extrapolation; status native system time must agree with unmodified Agent remote_time within 50ms + RTT/2; bounded native history resets invalidate status validation",
            },
            "resources": {}, "recording_errors": [], "status": "starting",
        }
        try:
            self.save_meta()
            self.stream = (self.path / "timeseries.csv").open("w", newline="", encoding="utf-8")
            self.writer = csv.DictWriter(self.stream, fieldnames=COLUMNS)
            self.writer.writeheader()
            for name, columns in (("observations", OBSERVATION_COLUMNS),
                                  ("geometry", GEOMETRY_COLUMNS),
                                  ("clock_offsets", CLOCK_OFFSET_COLUMNS)):
                stream = (self.path / f"{name}.csv").open("w", newline="", encoding="utf-8")
                self.sidecar_streams[name] = stream
                writer = csv.DictWriter(stream, fieldnames=columns)
                self.sidecar_writers[name] = writer
                writer.writeheader()
            self.flush()
        except Exception:
            try:
                self.close()
            except Exception:
                pass  # Preserve the startup failure after attempting every close.
            raise

    @staticmethod
    def _git_commit():
        try:
            result = subprocess.run(
                ["git", "-C", str(Path(__file__).resolve().parent), "rev-parse", "HEAD"],
                capture_output=True, text=True, timeout=3, check=True)
            return result.stdout.strip() or None
        except (OSError, subprocess.SubprocessError):
            try:
                from ._build_info import GIT_COMMIT
            except ModuleNotFoundError:
                return None
            return GIT_COMMIT

    @staticmethod
    def _git_dirty():
        try:
            result = subprocess.run(
                ["git", "-C", str(Path(__file__).resolve().parent), "status", "--porcelain",
                 "--untracked-files=normal"],
                capture_output=True, text=True, timeout=3, check=True)
            return bool(result.stdout.strip())
        except (OSError, subprocess.SubprocessError):
            return None

    def save_meta(self):
        atomic_yaml(self.path / "meta.yaml", self.meta)

    def flush(self):
        error = None
        for stream in [self.stream, *self.sidecar_streams.values()]:
            if stream is not None and not stream.closed:
                try:
                    stream.flush()
                    os.fsync(stream.fileno())
                except Exception as exc:
                    if error is None:
                        error = exc
        if error is not None:
            raise error

    def close(self):
        error = None
        try:
            self.flush()
        except Exception as exc:
            error = exc
        finally:
            for stream in [self.stream, *self.sidecar_streams.values()]:
                if stream is not None and not stream.closed:
                    try:
                        stream.close()
                    except Exception as exc:
                        if error is None:
                            error = exc
        if error is not None:
            raise error

    def write_sidecar(self, name, row):
        self.sidecar_writers[name].writerow(row)
        self.sidecar_streams[name].flush()

    def write_row(self, row):
        self.writer.writerow(row)
        # Each sample remains available even if the process is interrupted.
        self.stream.flush()

    def finalize(self, summary):
        self.close()
        temporary = self.path / "summary.csv.tmp"
        with temporary.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=SUMMARY_COLUMNS)
            writer.writeheader()
            writer.writerow(summary)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, self.path / "summary.csv")
        self.save_meta()
        sync_directory(self.path)


class BagCapture:
    def __init__(self, run_path, topics, qos_path):
        self.path = run_path / "rosbag"
        self.process = None
        self.log = None
        self.status = {"requested": True, "status": "starting", "topics": topics,
                       "exit_code": None, "errors": [], "escalation": []}
        try:
            executable = shutil.which("ros2")
            if executable is None:
                raise RuntimeError("ros2 executable unavailable: requested bag cannot start")
            self.log = (run_path / "rosbag.log").open("wb")
            self.command = [executable, "bag", "record", "--output", str(self.path),
                            "--qos-profile-overrides-path", str(qos_path)] + topics
            self.status["command"] = self.command
            self.process = subprocess.Popen(self.command, stdout=self.log, stderr=subprocess.STDOUT,
                                            start_new_session=True)
            self.status["status"] = "recording"
        except Exception as exc:
            self.status["status"] = "error"
            self.status["errors"].append(str(exc))
            if self.log is not None:
                self.log.close()
            raise RuntimeError(str(exc)) from exc

    def poll(self):
        if self.process is not None and self.process.poll() is not None and self.status["status"] == "recording":
            self.status["exit_code"] = self.process.returncode
            self.status["status"] = "error"
            self.status["errors"].append(f"rosbag exited before recorder finish ({self.process.returncode}); see rosbag.log")
        return self.status["status"] != "error"

    def close(self):
        if self.process is not None:
            if self.process.poll() is None:
                for sig, wait_seconds in ((signal.SIGINT, 15), (signal.SIGTERM, 5), (signal.SIGKILL, 5)):
                    try:
                        os.killpg(self.process.pid, sig)
                    except ProcessLookupError:
                        break
                    if sig != signal.SIGINT:
                        self.status["escalation"].append(sig.name)
                    try:
                        self.process.wait(timeout=wait_seconds)
                        break
                    except subprocess.TimeoutExpired:
                        continue
            self.status["exit_code"] = self.process.poll()
            if self.status["exit_code"] != 0 or self.status["escalation"]:
                self.status["errors"].append(f"rosbag did not finalize cleanly (exit={self.status['exit_code']}, escalation={self.status['escalation']})")
            if not (self.path / "metadata.yaml").is_file():
                self.status["errors"].append("rosbag metadata.yaml missing; requested bag is incomplete")
            else:
                try:
                    with (self.path / "metadata.yaml").open(encoding="utf-8") as stream:
                        information = yaml.safe_load(stream)["rosbag2_bagfile_information"]
                    files = information["relative_file_paths"]
                    if not files or any(not (self.path / name).is_file() or (self.path / name).stat().st_size == 0 for name in files):
                        raise ValueError("bag storage files missing or empty")
                    counts = dict.fromkeys(self.status["topics"], 0)
                    for item in information.get("topics_with_message_count", []):
                        counts[item["topic_metadata"]["name"]] = item["message_count"]
                    self.status["topic_message_counts"] = counts
                    self.status["message_count"] = information.get("message_count")
                except (OSError, ValueError, TypeError, KeyError, yaml.YAMLError) as exc:
                    self.status["errors"].append(f"invalid rosbag metadata/storage: {exc}")
            self.status["status"] = "error" if self.status["errors"] else "finalized"
            if self.path.exists():
                for path in self.path.rglob("*"):
                    if path.is_file():
                        with path.open("rb") as stream:
                            os.fsync(stream.fileno())
                sync_directory(self.path)
        if self.log is not None and not self.log.closed:
            self.log.flush()
            os.fsync(self.log.fileno())
            self.log.close()


class VideoCapture:
    def __init__(self, run_path, config):
        # Debug/compare never import OpenCV/numpy or instantiate this resource.
        import cv2
        import numpy as np
        self.cv2, self.np = cv2, np
        self.path = run_path / "camera.mp4"
        self.width, self.height = config["image_width"], config["image_height"]
        self.fps = config["video_fps"]
        self.last_frame_time = None
        self.start_time = None
        self.last_frame_index = -1
        self.last_image = None
        self.max_gap_frames = max(1, math.ceil(config["max_duration"] * self.fps))
        self.status = {"requested": True, "status": "starting", "frames": 0, "errors": [],
                       "fps": self.fps, "width": self.width, "height": self.height,
                       "duplicate_frames": 0, "start_ros_time": None, "end_ros_time": None,
                       "timeline": "fixed ROS receipt-time grid; missing slots hold previous frame"}
        self.writer = cv2.VideoWriter(str(self.path), cv2.VideoWriter_fourcc(*"mp4v"),
                                      self.fps, (self.width, self.height))
        if not self.writer.isOpened():
            self.writer.release()
            raise RuntimeError("OpenCV cannot open requested MP4 writer")
        self.status["status"] = "recording"

    def add(self, message, now):
        if self.last_frame_time is not None and now < self.last_frame_time:
            raise ValueError("camera receipt clock moved backwards")
        frame_index = 0 if self.start_time is None else math.floor((now - self.start_time) * self.fps + 1e-5)
        if frame_index <= self.last_frame_index:
            return
        missing = frame_index - self.last_frame_index - 1
        if missing > self.max_gap_frames:
            raise ValueError("camera gap exceeds max_duration; refusing unbounded video backfill")
        channels = {"bgr8": 3, "rgb8": 3, "bgra8": 4, "rgba8": 4, "mono8": 1, "8uc1": 1}.get(message.encoding.lower())
        if channels is None:
            raise ValueError(f"unsupported image encoding {message.encoding!r}")
        if message.height <= 0 or message.width <= 0 or message.step < message.width * channels:
            raise ValueError("invalid image dimensions or row stride")
        data = self.np.frombuffer(message.data, dtype=self.np.uint8)
        if data.size < message.height * message.step:
            raise ValueError("image buffer shorter than height * step")
        image = data[:message.height * message.step].reshape(message.height, message.step)
        image = image[:, :message.width * channels].reshape(message.height, message.width, channels)
        encoding = message.encoding.lower()
        conversion = {"rgb8": self.cv2.COLOR_RGB2BGR, "rgba8": self.cv2.COLOR_RGBA2BGR,
                      "bgra8": self.cv2.COLOR_BGRA2BGR, "mono8": self.cv2.COLOR_GRAY2BGR,
                      "8uc1": self.cv2.COLOR_GRAY2BGR}.get(encoding)
        if conversion is not None:
            image = self.cv2.cvtColor(image, conversion)
        if (message.width, message.height) != (self.width, self.height):
            image = self.cv2.resize(image, (self.width, self.height))
        if self.start_time is None:
            self.start_time = now
            self.status["start_ros_time"] = now
        for _ in range(missing):
            self.writer.write(self.last_image)
        self.status["duplicate_frames"] += missing
        self.writer.write(image)
        self.last_frame_time = now
        self.last_frame_index = frame_index
        self.last_image = image
        self.status["end_ros_time"] = now
        self.status["frames"] += missing + 1

    def close(self):
        self.writer.release()
        if self.status["frames"] == 0:
            self.status["errors"].append("requested video received no encodable camera frames")
        if not self.path.exists() or self.path.stat().st_size == 0:
            self.status["errors"].append("requested MP4 missing or empty")
        elif self.status["frames"]:
            reader = self.cv2.VideoCapture(str(self.path))
            try:
                valid, _ = reader.read()
                if not valid:
                    self.status["errors"].append("finalized MP4 cannot be decoded")
                self.status["encoded_frames"] = int(reader.get(self.cv2.CAP_PROP_FRAME_COUNT))
                if self.status["encoded_frames"] != self.status["frames"]:
                    self.status["errors"].append(
                        f"MP4 contains {self.status['encoded_frames']} frames, expected {self.status['frames']}")
            finally:
                reader.release()
            with self.path.open("rb") as stream:
                os.fsync(stream.fileno())
        self.status["status"] = "error" if self.status["errors"] else "finalized"
