#!/usr/bin/env python3
"""ROS adapter for one explicitly labelled, durable intercept experiment."""
import math
from pathlib import Path
from time import monotonic

import rclpy
from ament_index_python.packages import get_package_share_directory
from px4_msgs.msg import (
    SensorGps, TrajectorySetpoint, VehicleLocalPosition, VehicleOdometry,
    VehicleThrustSetpoint, VehicleTorqueSetpoint,
)
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from uav_common_msg.msg import Data, ExperimentTelemetry, RectMsg
import yaml

from .experiment_storage import (
    ARCHIVE_TOPICS, COMPARE_TOPICS, COLUMNS, NAN, TOPICS,
    BagCapture, RunStorage, VideoCapture, safe_identifier,
)

DEFAULTS = {
    "out_dir": "experiment_data", "algorithm": "vision_png", "target_motion": "circle",
    "profile": "debug", "scenario": "intercept", "experiment_id": "",
    "start_timeout": 60.0, "max_duration": 120.0, "sample_rate": 20.0, "topic_timeout": 1.0,
    "video_fps": 20.0, "camera_topic": "/camera/image", "image_width": 1920,
    "image_height": 1080,
}


def stamp_seconds(message):
    header = getattr(message, "header", None)
    stamp = getattr(header, "stamp", None)
    if stamp is None or stamp.sec < 0 or not 0 <= stamp.nanosec < 1_000_000_000:
        return None
    value = stamp.sec + stamp.nanosec * 1e-9
    return value if value > 0 else None


def finite(values):
    return all(math.isfinite(float(value)) for value in values)


def gps_coordinates(message):
    """Prefer the WGS84 ellipsoid altitude needed by the ECEF conversion."""
    lat = float(message.latitude_deg)
    lon = float(message.longitude_deg)
    alt = float(getattr(message, "altitude_ellipsoid_m", NAN))
    altitude_source = "ellipsoid"
    if not math.isfinite(alt):
        alt = float(message.altitude_msl_m)
        altitude_source = "MSL fallback"
    if not finite((lat, lon, alt)) or not -90 <= lat <= 90 or not -180 <= lon <= 180:
        return None
    if hasattr(message, "fix_type") and message.fix_type < 3:
        return None
    return lat, lon, alt, altitude_source


def ecef(latitude, longitude, altitude):
    lat, lon = math.radians(latitude), math.radians(longitude)
    eccentricity_squared = 6.69437999014e-3
    radius = 6378137.0 / math.sqrt(1.0 - eccentricity_squared * math.sin(lat) ** 2)
    return ((radius + altitude) * math.cos(lat) * math.cos(lon),
            (radius + altitude) * math.cos(lat) * math.sin(lon),
            (radius * (1.0 - eccentricity_squared) + altitude) * math.sin(lat))


def gps_to_ned(latitude, longitude, altitude, origin_latitude, origin_longitude, origin_altitude):
    point, origin = ecef(latitude, longitude, altitude), ecef(origin_latitude, origin_longitude, origin_altitude)
    dx, dy, dz = (point[index] - origin[index] for index in range(3))
    lat, lon = math.radians(origin_latitude), math.radians(origin_longitude)
    return (-math.sin(lat) * math.cos(lon) * dx - math.sin(lat) * math.sin(lon) * dy + math.cos(lat) * dz,
            -math.sin(lon) * dx + math.cos(lon) * dy,
            -math.cos(lat) * math.cos(lon) * dx - math.cos(lat) * math.sin(lon) * dy - math.sin(lat) * dz)


class ExperimentRecorder(Node):
    def __init__(self):
        super().__init__("experiment_recorder")
        self.finished = False
        self.storage = None
        self.bag = None
        self.video = None
        self.cache = {}
        self.origin = None
        self.start_time = None
        self.start_wait_monotonic = monotonic()
        self.intercept_start = None
        self.last_time = None
        self.samples = 0
        self.minimum_distance = NAN
        self.hit = False
        self.detection_count = 0
        self.lost_count = 0
        self.reacquire_count = 0
        self.detected = False
        self.ever_detected = False
        self.pending_lost = False
        self.pending_reacquired = False
        self.latency_sum = 0.0
        self.latency_count = 0
        self.latency_max = NAN
        self.last_parameters_yaml = None
        self.timer = None
        self.errors = []
        self.resources = {
            "bag": {"requested": False, "status": "disabled", "exit_code": None, "errors": []},
            "video": {"requested": False, "status": "disabled", "frames": 0, "errors": []},
        }
        try:
            for name, value in DEFAULTS.items():
                self.declare_parameter(name, value)
            self.config = {name: self.get_parameter(name).value for name in DEFAULTS}
            self._validate_config()
            self.start_time = self._now()
            self.last_time = self.start_time
            self.storage = RunStorage(self.config)
            self.storage.meta["start_ros_time"] = self.start_time
            self.storage.meta["resources"] = self.resources
            self._start_resources()
            self._subscribe()
            self.timer = self.create_timer(1.0 / self.config["sample_rate"], self._tick)
            self.storage.meta["status"] = "waiting"
            self.storage.save_meta()
            self.get_logger().info(f"Experiment output: {self.storage.path}; waiting for fresh INTERCEPT prerequisites")
        except Exception as exc:
            if self.storage is None:
                super().destroy_node()
                raise
            self._error(f"startup failed: {exc}")
            self.finish("startup_error")

    def _validate_config(self):
        cfg = self.config
        if cfg["scenario"] != "intercept":
            raise ValueError("only scenario=intercept is supported")
        if cfg["algorithm"] not in ("pure_png", "vision_png"):
            raise ValueError("algorithm must be pure_png or vision_png")
        if cfg["profile"] not in ("debug", "compare", "archive"):
            raise ValueError("profile must be debug, compare, or archive")
        safe_identifier(cfg["target_motion"], "target_motion")
        if cfg["experiment_id"]:
            safe_identifier(cfg["experiment_id"], "experiment_id")
        for key in ("start_timeout", "max_duration", "sample_rate", "topic_timeout", "video_fps"):
            if not math.isfinite(cfg[key]) or cfg[key] <= 0:
                raise ValueError(f"{key} must be finite and positive")
        for key in ("image_width", "image_height"):
            if not isinstance(cfg[key], int) or cfg[key] <= 0:
                raise ValueError(f"{key} must be a positive integer")
        if not cfg["out_dir"] or not cfg["camera_topic"]:
            raise ValueError("out_dir and camera_topic must be nonempty")

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _start_resources(self):
        profile = self.config["profile"]
        self.resources["bag"]["requested"] = profile != "debug"
        self.resources["video"]["requested"] = profile == "archive"
        for status in self.resources.values():
            if status["requested"]:
                status["status"] = "pending"
        self.storage.save_meta()
        if profile != "debug":
            try:
                qos_path = Path(get_package_share_directory("uav_bc_recorder")) / "config" / "bag_qos.yaml"
                if not qos_path.is_file():
                    raise RuntimeError(f"installed rosbag QoS overrides missing: {qos_path}")
                topics = COMPARE_TOPICS if profile == "compare" else ARCHIVE_TOPICS
                self.bag = BagCapture(self.storage.path, topics, qos_path)
                self.resources["bag"] = self.bag.status
            except Exception as exc:
                self.resources["bag"].update(status="error", errors=[str(exc)])
                raise
        if profile == "archive":
            try:
                self.video = VideoCapture(self.storage.path, self.config)
                self.resources["video"] = self.video.status
            except Exception as exc:
                self.resources["video"].update(status="error", errors=[str(exc)])
                raise
        self.storage.save_meta()

    def _subscribe(self):
        self.subscriptions_owned = []
        for key, message_type, callback in (
            ("telemetry", ExperimentTelemetry, self._telemetry_cb),
            ("vpng", Data, lambda message: self._cache("vpng", message)),
            ("detection", RectMsg, self._detect_cb),
            ("self_gps", SensorGps, lambda message: self._gps_cb("self_gps", message)),
            ("target_gps", SensorGps, lambda message: self._gps_cb("target_gps", message)),
            ("self_local", VehicleLocalPosition, lambda message: self._cache("self_local", message)),
            ("target_local", VehicleLocalPosition, lambda message: self._cache("target_local", message)),
            ("odom", VehicleOdometry, lambda message: self._cache("odom", message)),
            ("trajectory", TrajectorySetpoint, lambda message: self._cache("trajectory", message)),
            ("thrust", VehicleThrustSetpoint, lambda message: self._cache("thrust", message)),
            ("torque", VehicleTorqueSetpoint, lambda message: self._cache("torque", message)),
        ):
            self.subscriptions_owned.append(self.create_subscription(message_type, TOPICS[key], callback, qos_profile_sensor_data))
        if self.config["profile"] == "archive":
            from sensor_msgs.msg import Image
            self.subscriptions_owned.append(self.create_subscription(Image, self.config["camera_topic"], self._image_cb, qos_profile_sensor_data))

    def _cache(self, key, message):
        if not self.finished:
            self.cache[key] = (message, self._now())

    def _age(self, key, now):
        cached = self.cache.get(key)
        if cached is None:
            return NAN
        age = now - cached[1]
        return age if age >= 0 else NAN

    def _fresh(self, key, now):
        age = self._age(key, now)
        if not math.isfinite(age) or age > self.config["topic_timeout"]:
            return None
        message = self.cache[key][0]
        # Camera headers can be Gazebo simulation time while this node uses wall
        # ROS time. Detection freshness is receipt-only, never an epoch guess.
        if key == "telemetry":
            source = stamp_seconds(message)
            if source is not None and not 0 <= now - source <= self.config["topic_timeout"]:
                return None
        return message

    def _telemetry_cb(self, message):
        if self.finished:
            return
        if message.algorithm != self.config["algorithm"]:
            self._error(f"telemetry algorithm {message.algorithm!r} does not match requested {self.config['algorithm']!r}")
            self.finish("algorithm_mismatch")
            return
        self._cache("telemetry", message)
        if message.parameters_yaml != self.last_parameters_yaml:
            try:
                parameters = yaml.safe_load(message.parameters_yaml)
                if not isinstance(parameters, dict):
                    raise ValueError("parameters_yaml is not a mapping")
                # Accept direct mappings and standard ROS node/ros__parameters YAML.
                if "ros__parameters" in parameters:
                    parameters = parameters["ros__parameters"]
                elif len(parameters) == 1:
                    nested = next(iter(parameters.values()))
                    if isinstance(nested, dict) and "ros__parameters" in nested:
                        parameters = nested["ros__parameters"]
                if not isinstance(parameters, dict):
                    raise ValueError("ros__parameters is not a mapping")
                self.storage.meta["parameters"].update(parameters)
                for key in ("N_png", "speed_cmd", "hit_radius", "planner", "controller"):
                    self.storage.meta[key] = self.storage.meta["parameters"].get(key)
                self.last_parameters_yaml = message.parameters_yaml
                self.storage.save_meta()
            except (ValueError, TypeError, yaml.YAMLError) as exc:
                self._error(f"invalid algorithm parameter telemetry: {exc}")
                self.finish("invalid_telemetry")
                return
        now = self._now()
        if self._fresh("telemetry", now) is not None:
            self._maybe_start(now)
            if message.state == "DONE":
                if self.intercept_start is not None:
                    self._sample(now)
                self.finish("done")

    def _gps_cb(self, key, message):
        if self.finished:
            return
        self._cache(key, message)
        coordinates = gps_coordinates(message)
        if coordinates is not None:
            altitude_sources = self.storage.meta.setdefault("gps_altitude_sources", {})
            if altitude_sources.get(key) != coordinates[3]:
                altitude_sources[key] = coordinates[3]
                self.storage.save_meta()
            if key == "self_gps" and self.origin is None:
                self.origin = coordinates[:3]
                self.storage.meta["gps_origin"] = {"latitude_deg": self.origin[0], "longitude_deg": self.origin[1], "altitude_m": self.origin[2], "altitude_source": coordinates[3]}
                self.storage.save_meta()
        now = self._now()
        self._maybe_start(now)
        if self.intercept_start is not None:
            distance = self._distance(now)
            if self._is_hit(distance):
                self._sample(now)
                self.finish("hit")

    def _set_detection(self, valid):
        if valid == self.detected:
            return
        if valid:
            if self.ever_detected:
                self.reacquire_count += 1
                self.pending_reacquired = True
            self.ever_detected = True
        elif self.ever_detected:
            self.lost_count += 1
            self.pending_lost = True
        self.detected = valid

    def _detect_cb(self, message):
        if self.finished:
            return
        now = self._now()
        if self.detected and self._fresh("detection", now) is None:
            self._set_detection(False)
        # Cache and evaluate with the same receipt time; a later cache timestamp
        # would make this callback's own freshness check see a negative age.
        self.cache["detection"] = (message, now)
        valid = self._fresh("detection", now) is not None and message.width > 0 and message.height > 0
        self._set_detection(valid)
        if valid:
            self.detection_count += 1
            latency = self._latency(message, now)
            if math.isfinite(latency):
                self.latency_sum += latency
                self.latency_count += 1
                self.latency_max = latency if not math.isfinite(self.latency_max) else max(self.latency_max, latency)
        self._maybe_start(now)

    def _latency(self, message, received):
        source = stamp_seconds(message)
        # Bound by the same freshness horizon to reject incompatible time epochs.
        if source is None or not 0 <= received - source <= self.config["topic_timeout"]:
            return NAN
        return (received - source) * 1000.0

    def _image_cb(self, message):
        if self.finished:
            return
        try:
            self.video.add(message, self._now())
        except Exception as exc:
            self.video.status["errors"].append(str(exc))
            self.video.status["status"] = "error"
            self._error(f"requested video capture failed: {exc}")
            self.finish("video_error")

    def _local_velocity(self, key, now):
        message = self._fresh(key, now)
        if message is None:
            return None
        if not getattr(message, "v_xy_valid", True) or not getattr(message, "v_z_valid", True):
            return None
        velocity = (message.vx, message.vy, message.vz)
        return velocity if finite(velocity) else None

    def _position(self, key, now):
        message = self._fresh(key, now)
        if message is None or self.origin is None:
            return None
        coordinates = gps_coordinates(message)
        if coordinates is None or coordinates[3] != self.storage.meta["gps_origin"]["altitude_source"]:
            return None
        return gps_to_ned(*coordinates[:3], *self.origin)

    def _distance(self, now):
        own, target = self._position("self_gps", now), self._position("target_gps", now)
        if own is None or target is None:
            return NAN
        return math.sqrt(sum((a - b) ** 2 for a, b in zip(own, target)))

    def _is_hit(self, distance):
        radius = self.storage.meta["parameters"].get("hit_radius")
        return (isinstance(radius, (int, float)) and not isinstance(radius, bool)
                and math.isfinite(radius) and radius >= 0 and math.isfinite(distance) and distance <= radius)

    def _check_start_timeout(self):
        if (not self.finished and self.intercept_start is None
                and monotonic() - self.start_wait_monotonic >= self.config["start_timeout"]):
            self.get_logger().warning(
                f"No eligible INTERCEPT within {self.config['start_timeout']}s; ending startup wait")
            self.finish("start_timeout")
        return self.finished

    def _maybe_start(self, now):
        if self.intercept_start is not None or self._check_start_timeout():
            return
        telemetry = self._fresh("telemetry", now)
        if telemetry is None or telemetry.state != "INTERCEPT":
            return
        if any(self._position(key, now) is None for key in ("self_gps", "target_gps")):
            return
        if any(self._local_velocity(key, now) is None for key in ("self_local", "target_local")):
            return
        detection = self._fresh("detection", now)
        if self.config["algorithm"] == "vision_png" and (detection is None or detection.width <= 0 or detection.height <= 0):
            return
        self.intercept_start = now
        self.storage.meta.update(status="recording", intercept_start_ros_time=now)
        self.storage.save_meta()
        self.get_logger().info("Fresh INTERCEPT prerequisites met; sampling started")

    def _tick(self):
        if self.finished:
            return
        try:
            now = self._now()
            if now < self.last_time:
                self._error("ROS clock moved backwards; refusing to mix experiment timelines")
                self.finish("clock_reset")
                return
            self.last_time = now
            if self.bag is not None and not self.bag.poll():
                self._error("requested rosbag exited unexpectedly; see rosbag.log")
                self.finish("bag_error")
                return
            detection = self._fresh("detection", now)
            if detection is None or detection.width <= 0 or detection.height <= 0:
                self._set_detection(False)
            self._maybe_start(now)
            if self.intercept_start is not None:
                self._sample(now)
                if self.hit:
                    self.finish("hit")
                    return
            if self.intercept_start is not None and now - self.intercept_start >= self.config["max_duration"]:
                self.finish("timeout")
        except Exception as exc:
            self._error(f"recording callback failed: {exc}")
            self.finish("recording_error")

    def _sample(self, now):
        row = dict.fromkeys(COLUMNS, NAN)
        row.update(timestamp=now, elapsed_s=max(0.0, now - self.intercept_start),
                   detected=False, state="", solver_status="", lost=self.pending_lost,
                   reacquired=self.pending_reacquired, hit=False)
        for key in self.cache:
            column = f"{key}_age_s"
            if column in row:
                row[column] = self._age(key, now)
        for key, prefix in (("self_gps", "self"), ("target_gps", "target")):
            values = self._position(key, now)
            if values is not None:
                row.update(zip((f"{prefix}_{axis}" for axis in "xyz"), values))
        for key, prefix in (("self_local", "self"), ("target_local", "target")):
            values = self._local_velocity(key, now)
            if values is not None:
                row.update(zip((f"{prefix}_v{axis}" for axis in "xyz"), values))
        row["distance"] = self._distance(now)
        if math.isfinite(row["distance"]):
            self.minimum_distance = row["distance"] if not math.isfinite(self.minimum_distance) else min(self.minimum_distance, row["distance"])
        self.hit = self.hit or self._is_hit(row["distance"])
        row["hit"] = self.hit
        odometry = self._fresh("odom", now)
        if odometry is not None:
            if (getattr(odometry, "pose_frame", 0) == getattr(VehicleOdometry, "POSE_FRAME_NED", 1)
                    and finite(odometry.q) and sum(value * value for value in odometry.q) > 0):
                row.update(zip(("self_qw", "self_qx", "self_qy", "self_qz"), odometry.q))
            if finite(odometry.angular_velocity):
                row.update(zip(("self_wx", "self_wy", "self_wz"), odometry.angular_velocity))
        detection = self._fresh("detection", now)
        if detection is not None and detection.width > 0 and detection.height > 0:
            row.update(detected=True, bbox_x=detection.x, bbox_y=detection.y,
                       bbox_w=detection.width, bbox_h=detection.height)
            confidence = float(getattr(detection, "confidence", NAN))
            row["confidence"] = confidence if math.isfinite(confidence) and 0 <= confidence <= 1 else NAN
            row["vision_latency_ms"] = self._latency(detection, self.cache["detection"][1])
        diagnostic = self._fresh("vpng", now)
        if diagnostic is not None and row["detected"]:
            row.update(ex=diagnostic.ex, ey=diagnostic.ey)
        telemetry = self._fresh("telemetry", now)
        if telemetry is not None:
            row["state"] = telemetry.state
            if telemetry.guidance_valid:
                for key in ("los_v", "los_z", "dlos_v", "dlos_z", "png_v", "png_z"):
                    row[key] = getattr(telemetry, key)
            row.update(solver_time_ms=telemetry.solver_time_ms, cost=telemetry.cost,
                       solver_status=telemetry.solver_status)
        trajectory = self._fresh("trajectory", now)
        if trajectory is not None:
            row.update(zip(("ref_vx", "ref_vy", "ref_vz"), trajectory.velocity))
        thrust = self._fresh("thrust", now)
        if thrust is not None:
            row["cmd_thrust"] = -float(thrust.xyz[2])
        torque = self._fresh("torque", now)
        if torque is not None:
            row.update(zip(("cmd_tx", "cmd_ty", "cmd_tz"), torque.xyz))
        self.storage.write_row(row)
        self.samples += 1
        self.pending_lost = self.pending_reacquired = False

    def _error(self, text):
        self.errors.append(text)
        self.get_logger().error(text)

    def finish(self, reason="shutdown"):
        if self.finished:
            return
        self.finished = True
        if self.timer is not None:
            self.timer.cancel()
        now = self._now()
        for name, resource in (("video", self.video), ("bag", self.bag)):
            if resource is not None:
                try:
                    resource.close()
                except Exception as exc:
                    resource.status["status"] = "error"
                    resource.status["errors"].append(f"finalization failed: {exc}")
                self.resources[name] = resource.status
            status = self.resources[name]
            if status["requested"] and status["status"] != "finalized":
                text = f"requested {name} incomplete: {'; '.join(status['errors']) or status['status']}"
                if text not in self.errors:
                    self._error(text)
        if self.storage is None:
            return
        elapsed = max(0.0, now - self.intercept_start) if self.intercept_start is not None else NAN
        self.storage.meta.update(
            status="error" if self.errors else ("complete" if self.samples else "no_intercept"), reason=reason,
            end_ros_time=now, resources=self.resources, recording_errors=list(self.errors),
        )
        self.storage.meta["topic_status"] = {
            key: {"received": key in self.cache, "age_s": self._age(key, now),
                  "fresh": self._fresh(key, now) is not None}
            for key in TOPICS
        }
        if self.samples == 0:
            self.get_logger().warning("No eligible INTERCEPT samples were recorded; inspect topic_status and telemetry state")
        summary = {
            "status": self.storage.meta["status"], "reason": reason, "hit": self.hit,
            "intercept_time": elapsed if self.hit else NAN,
            "elapsed_s": elapsed, "min_distance": self.minimum_distance, "samples": self.samples,
            "detection_count": self.detection_count, "lost_count": self.lost_count,
            "reacquire_count": self.reacquire_count,
            "latency_mean_ms": self.latency_sum / self.latency_count if self.latency_count else NAN,
            "latency_max_ms": self.latency_max, "video_frames": self.resources["video"]["frames"],
            "bag_status": self.resources["bag"]["status"],
            "bag_exit_code": self.resources["bag"]["exit_code"],
            "bag_errors": "; ".join(self.resources["bag"]["errors"]),
            "video_status": self.resources["video"]["status"],
            "video_errors": "; ".join(self.resources["video"]["errors"]),
            "recording_errors": "; ".join(self.errors),
        }
        try:
            self.storage.finalize(summary)
        except Exception as exc:
            self._error(f"cannot durably finalize experiment files: {exc}")
            raise
        self.get_logger().info(f"Experiment finalized: {reason}; samples={self.samples}; output={self.storage.path}")

    def destroy_node(self):
        try:
            if not self.finished:
                self.finish("shutdown")
        finally:
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    exit_code = 0
    try:
        node = ExperimentRecorder()
        while rclpy.ok() and not node.finished:
            # ROS timers do not fire while simulation time is stopped or /clock
            # is absent. Enforce the startup deadline from the wall-driven loop.
            if node._check_start_timeout():
                break
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception as exc:
        exit_code = 1
        if node is not None:
            node._error(f"recorder failed: {exc}")
        else:
            import sys
            print(f"experiment_recorder startup failed: {exc}", file=sys.stderr)
    finally:
        if node is not None:
            try:
                if not node.finished:
                    node.finish("shutdown" if exit_code == 0 else "recording_error")
                if node.errors:
                    exit_code = 1
            finally:
                node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if exit_code:
        raise SystemExit(exit_code)
