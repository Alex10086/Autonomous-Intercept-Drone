"""Consumer-visible recording boundaries, without a running ROS graph."""
import csv
from types import SimpleNamespace

import cv2
import numpy as np
import pytest
import yaml

from uav_bc_recorder.experiment_storage import RunStorage, VideoCapture
from uav_bc_recorder.recorder_node import DEFAULTS, gps_to_ned


def test_reused_experiment_id_never_overwrites_previous_run(tmp_path):
    config = dict(DEFAULTS, out_dir=str(tmp_path), experiment_id="trial-A")
    first = RunStorage(config)
    first.write_row({"timestamp": 12.0, "distance": 9.5})
    first.finalize({"status": "complete", "samples": 1})
    before = (first.path / "timeseries.csv").read_bytes()

    second = RunStorage(config)
    second.write_row({"timestamp": 13.0, "distance": 0.25})
    second.finalize({"status": "complete", "samples": 1})

    assert second.path != first.path
    assert (first.path / "timeseries.csv").read_bytes() == before
    assert yaml.safe_load((second.path / "meta.yaml").read_text())["experiment_id"] == "trial-A"
    assert list(csv.DictReader((second.path / "timeseries.csv").open()))[0]["distance"] == "0.25"


def test_common_ned_frame_preserves_distance_across_longitude_wrap():
    north, east, down = gps_to_ned(0.0, -179.99999, 100.0, 0.0, 179.99999, 100.0)
    assert abs(north) < 1e-6
    assert east == pytest.approx(2.2264, abs=0.001)
    assert abs(down) < 1e-5
    assert gps_to_ned(0.0, 0.0, 110.0, 0.0, 0.0, 100.0) == pytest.approx((0.0, 0.0, -10.0))


def test_padded_rgb_image_is_saved_with_correct_colors(tmp_path):
    capture = VideoCapture(tmp_path, dict(DEFAULTS, image_width=64, image_height=48, video_fps=10.0))
    pixels = np.zeros((48, 64 * 3 + 8), dtype=np.uint8)
    pixels[:, :64 * 3].reshape(48, 64, 3)[:] = (240, 10, 20)
    message = SimpleNamespace(encoding="rgb8", width=64, height=48,
                              step=64 * 3 + 8, data=pixels.tobytes())
    try:
        capture.add(message, 0.0)
        capture.add(message, 0.1)
    finally:
        capture.close()
    reader = cv2.VideoCapture(str(tmp_path / "camera.mp4"))
    try:
        ok, image = reader.read()
        assert ok
        blue, green, red = image.mean(axis=(0, 1))
        assert red > 200 and blue < 40 and green < 40
    finally:
        reader.release()


def test_truncated_image_is_reported_as_incomplete_video(tmp_path):
    capture = VideoCapture(tmp_path, dict(DEFAULTS, image_width=64, image_height=48))
    try:
        message = SimpleNamespace(encoding="bgr8", width=64, height=48, step=192, data=b"short")
        with pytest.raises(ValueError, match="buffer shorter"):
            capture.add(message, 1.0)
    finally:
        capture.close()
    assert capture.status["status"] == "error"
    assert any("no encodable" in error for error in capture.status["errors"])


def test_video_resampling_preserves_duration_and_holds_previous_frame(tmp_path):
    capture = VideoCapture(tmp_path, dict(DEFAULTS, image_width=64, image_height=48, video_fps=10.0))
    red = np.full((48, 64, 3), (240, 10, 20), dtype=np.uint8)
    blue = np.full((48, 64, 3), (10, 20, 240), dtype=np.uint8)
    message = SimpleNamespace(encoding="rgb8", width=64, height=48, step=192, data=red.tobytes())
    try:
        capture.add(message, 100.0)
        message.data = blue.tobytes()
        capture.add(message, 100.5)
    finally:
        capture.close()
    reader = cv2.VideoCapture(str(tmp_path / "camera.mp4"))
    try:
        frames = []
        while True:
            ok, image = reader.read()
            if not ok:
                break
            frames.append(image.mean(axis=(0, 1)))
        assert len(frames) == 6
        assert frames[4][2] > 200 and frames[4][0] < 40
        assert frames[5][0] > 200 and frames[5][2] < 40
    finally:
        reader.release()


def test_new_runs_do_not_inherit_previous_experiment_conditions(tmp_path):
    config = dict(DEFAULTS, out_dir=str(tmp_path), experiment_id="independent")
    first = RunStorage(config)
    first.meta["experiment_conditions"]["requested"] = {"trial": 1, "distance": 17.0}
    first.meta["initial_conditions"] = {"distance": 17.1}
    first.finalize({"status": "complete"})
    second = RunStorage(config)
    assert second.meta["experiment_conditions"] == {"requested": None, "target": None}
    assert second.meta["initial_conditions"] is None
    assert second.meta["first_hit_ros_time"] is None
    assert second.meta["first_hit_monotonic"] is None
    second.finalize({"status": "no_intercept"})
