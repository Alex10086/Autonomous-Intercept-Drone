"""First-pass selection and sampled angular-rate boundaries."""
import csv
import json
import math

import pytest
import yaml

from uav_bc_recorder import phase2_analysis
from uav_bc_recorder.phase2_analysis import (
    align_observations, diagnostic_rows, first_encounter, geometry_samples,
    interpolate_position, observation_metrics, source_metrics,
)


def test_first_encounter_ignores_jitter_and_does_not_select_later_best_pass():
    pairs = [(0,24), (1,15), (2,4.1), (2.1,4), (2.2,4.04),
             (2.3,3.98), (2.4,4.05), (3,5.2), (3.3,5.8), (5,2), (6,0.5)]
    rows = [{'elapsed_s':time, 'distance':distance} for time,distance in pairs]
    closest,end = first_encounter(rows)
    assert rows[closest]['distance'] == 3.98
    assert rows[closest]['elapsed_s'] == 2.3
    assert rows[end]['elapsed_s'] == 3.3


def angular_row(elapsed, angle, detected=True):
    return {
        'elapsed_s':elapsed, 'detected':str(detected), 'los_v':0, 'los_z':angle,
        'self_x':0, 'self_y':0, 'self_z':0, 'target_x':10, 'target_y':0, 'target_z':0,
        'self_vx':1, 'self_vy':0, 'self_vz':0,
        'target_vx':0, 'target_vy':0, 'target_vz':0,
        'ref_vx':1, 'ref_vy':0, 'ref_vz':0,
        'self_qw':1, 'self_qx':0, 'self_qy':0, 'self_qz':0,
        'self_wx':0, 'self_wy':0, 'self_wz':0.4,
    }


def test_visual_los_rate_wraps_pi_crossing():
    rows = [angular_row(1,math.radians(179)), angular_row(1.05,math.radians(-179))]
    output = diagnostic_rows(rows)
    assert output[-1]['los_rate_z'] == pytest.approx(math.radians(2)/0.05)
    assert output[-1]['yaw_rate'] == pytest.approx(0.4)
    assert output[-1]['closing_speed'] == pytest.approx(1.0)


def test_visual_rate_does_not_bridge_detection_loss():
    rows = [angular_row(1,0.1), angular_row(1.05,math.nan,False), angular_row(1.1,0.7)]
    output = diagnostic_rows(rows)
    assert math.isnan(output[-1]['los_rate_z'])


def test_first_encounter_keeps_early_turnaway_before_later_close_scrape():
    pairs = [(0,24), (1,20), (2.5,17), (3,18.3), (3.3,19), (12,0.6)]
    rows = [{'elapsed_s':time, 'distance':distance} for time,distance in pairs]
    closest,end = first_encounter(rows)
    assert rows[closest]['distance'] == 17
    assert rows[closest]['elapsed_s'] == 2.5
    assert rows[end]['elapsed_s'] == 3.3


def geometry_row(vehicle, time, x=0, y=0, z=0, receive_time=500):
    return dict(vehicle=vehicle, source_time=time, receive_time=receive_time, x=x, y=y, z=z)


def observation(time=100.05, **changes):
    row = dict(image_time_ros=time, control_time=math.nan, bbox_receive_time=time + 0.02,
               accepted=True, clock_valid=True, attitude_synced=True,
               los_v=0, los_z=0, receive_pose_los_v=0, receive_pose_los_z=0)
    row.update(changes)
    return row


def constant_geometry(distance=3):
    return geometry_samples([
        geometry_row(vehicle, time, y=distance if vehicle == 'target' else 0)
        for time in (100, 100.1) for vehicle in ('self', 'target')
    ])


def test_interpolation_exact_endpoints_gap_limit_and_no_extrapolation():
    samples = [(10, 0, 1, 2), (10.15, 3, 4, 5), (10.4, 8, 9, 10)]
    assert interpolate_position(samples, 10) == (0, 1, 2)
    assert interpolate_position(samples, 10.15) == (3, 4, 5)
    assert interpolate_position(samples, 10.075) == pytest.approx((1.5, 2.5, 3.5))
    for time in (9.99, 10.2, 10.41, math.nan):
        assert all(math.isnan(value) for value in interpolate_position(samples, time))
    assert all(math.isnan(value) for value in interpolate_position([], 10))


def test_interpolation_gap_boundary_at_ros_epoch():
    epoch = 1_800_000_000.0
    samples = [(epoch, 0, 0, 0), (epoch + 0.15, 1, 1, 1)]
    assert interpolate_position(samples, epoch + 0.075) == pytest.approx((0.5, 0.5, 0.5), abs=2e-6)
    samples[-1] = (epoch + 0.151, 1, 1, 1)
    assert all(math.isnan(value) for value in interpolate_position(samples, epoch + 0.075))


def test_geometry_keeps_only_monotonic_finite_source_samples_per_aircraft():
    rows = [
        geometry_row('self', 100), geometry_row('self', 100, x=999),
        geometry_row('self', 99, x=999), geometry_row('self', math.nan),
        geometry_row('target', 99.9, x=3), geometry_row('self', 100.1, x=1),
        geometry_row('self', 100.2, x=math.nan), geometry_row('unknown', 100),
    ]
    samples = geometry_samples(rows)
    assert samples['self'] == [(100, 0, 0, 0), (100.1, 1, 0, 0)]
    assert samples['target'] == [(99.9, 3, 0, 0)]
    assert all(math.isnan(value) for value in interpolate_position(samples['target'], 100))


def test_each_aircraft_and_each_epoch_are_interpolated_independently():
    samples = geometry_samples([
        geometry_row('self', 100), geometry_row('self', 100.1, x=0.1),
        geometry_row('target', 100.02, x=2, y=1),
        geometry_row('target', 100.12, x=2.2, y=1),
    ])
    row = align_observations([observation(control_time=100.09)], samples)[0]
    assert row['self_image_x'] == pytest.approx(0.05)
    assert row['target_image_x'] == pytest.approx(2.06)
    assert row['self_control_x'] == pytest.approx(0.09)
    assert row['target_control_x'] == pytest.approx(2.14)
    assert row['truth_los_image_z_rad'] == pytest.approx(math.atan2(2.01, 1))
    assert row['truth_los_control_z_rad'] == pytest.approx(math.atan2(2.05, 1))
    assert row['los_error_image_z_rad'] != row['los_error_control_z_rad']
    assert row['image_geometry_covered'] and row['control_geometry_covered']


def test_alignment_wraps_vision_and_counterfactual_errors_against_image_truth():
    angle = math.radians(-179)
    samples = geometry_samples([
        geometry_row(vehicle, time,
                     x=3 * math.sin(angle) if vehicle == 'target' else 0,
                     y=3 * math.cos(angle) if vehicle == 'target' else 0,
                     z=1 if vehicle == 'target' else 0)
        for time in (100, 100.1) for vehicle in ('self', 'target')
    ])
    rows = align_observations([
        observation(los_z=math.radians(179), receive_pose_los_z=math.radians(177))
    ], samples)
    assert rows[0]['los_error_image_z_rad'] == pytest.approx(math.radians(-2))
    assert rows[0]['receive_pose_error_image_z_rad'] == pytest.approx(math.radians(-4))
    assert rows[0]['truth_los_image_v_rad'] == pytest.approx(math.atan2(1, 3))
    metrics = observation_metrics(rows, 100, 0.1)
    assert metrics['first_approach_paired_z_count'] == 1
    assert metrics['first_approach_paired_z_image_abs_deg_median'] == pytest.approx(2)
    assert metrics['first_approach_paired_z_receive_pose_abs_deg_median'] == pytest.approx(4)
    assert metrics['first_approach_paired_z_improvement_deg_median'] == pytest.approx(2)


def test_clock_gate_missing_source_and_superseded_control_never_fabricate_truth():
    samples = constant_geometry()
    rows = align_observations([
        observation(clock_valid=False, control_time=100.08),
        observation(), observation(time=99.9, control_time=100.08),
    ], samples)
    assert not rows[0]['image_geometry_covered']
    assert not rows[0]['control_geometry_covered']
    assert rows[1]['image_geometry_covered'] and not rows[1]['control_geometry_covered']
    assert not rows[2]['image_geometry_covered'] and rows[2]['control_geometry_covered']
    assert math.isnan(rows[2]['los_error_image_z_rad'])
    missing = geometry_samples([geometry_row('self', 100), geometry_row('target', math.nan)])
    row = align_observations([observation(time=100)], missing)[0]
    assert not row['image_geometry_covered']
    assert math.isnan(row['image_distance_m'])
    assert math.isnan(row['truth_los_image_z_rad'])


def test_metrics_are_event_weighted_with_explicit_coverage_and_paired_counts():
    events = [
        observation(time=100, control_time=100.02, los_z=math.radians(2),
                    receive_pose_los_z=math.radians(10)),
        observation(time=100.1, control_time=100.2, los_z=math.radians(4),
                    receive_pose_los_z=math.radians(12)),
        observation(time=100.05, receive_pose_los_z=math.nan),
        observation(time=100.05, accepted=False), observation(time=100.05, clock_valid=False),
        observation(time=100.05, los_z=math.inf),
    ]
    rows = align_observations(events, constant_geometry())
    metrics = observation_metrics(rows, 100, 0.1)
    assert metrics['observation_events'] == 6
    assert metrics['observation_eligible_events'] == 3
    assert metrics['image_to_control_ms_count'] == 2
    assert metrics['image_to_control_ms_median'] == pytest.approx(60)
    assert metrics['image_to_control_ms_p95'] == pytest.approx(96)
    assert metrics['observations_image_geometry_count'] == 3
    assert metrics['observations_image_geometry_coverage'] == 1
    assert metrics['observations_control_geometry_count'] == 1
    assert metrics['observations_control_geometry_coverage'] == pytest.approx(1 / 3)
    assert metrics['first_approach_paired_z_count'] == 2
    assert metrics['first_approach_paired_z_image_abs_deg_median'] == pytest.approx(3)
    assert metrics['first_approach_paired_z_receive_pose_abs_deg_median'] == pytest.approx(11)
    assert metrics['first_approach_paired_z_improvement_deg_median'] == pytest.approx(8)
    assert metrics['first_approach_control_abs_z_deg_count'] == 1
    json.dumps(metrics, allow_nan=False)


def test_first_approach_distance_bins_boundaries_and_closest_epoch():
    times = [100, 100.1, 100.2, 100.3, 100.4]
    distances = [0.8, 2, 5, 17, 1]
    samples = geometry_samples([
        geometry_row(vehicle, time, y=distance if vehicle == 'target' else 0)
        for time, distance in zip(times, distances) for vehicle in ('self', 'target')
    ])
    events = [observation(time=time) for time in times]
    events.append(observation(time=99.9))
    metrics = observation_metrics(align_observations(events, samples), 100, 0.3)
    assert metrics['first_approach_all_distances_events'] == 4
    assert metrics['first_approach_events'] == 3
    assert metrics['first_approach_0p8_2m_events'] == 1
    assert metrics['first_approach_2_5m_events'] == 1
    assert metrics['first_approach_5_17m_events'] == 2


def test_missing_intercept_epoch_does_not_select_a_first_approach():
    rows = align_observations([observation()], constant_geometry())
    metrics = observation_metrics(rows, math.nan, 1)
    assert not metrics['first_approach_epoch_available']
    assert metrics['observations_image_geometry_count'] == 1
    assert metrics['first_approach_events'] == 0
    assert metrics['first_approach_image_abs_z_deg_median'] is None


def write_csv(path, rows, fields=None):
    with path.open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields or list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def make_recording(path, sidecars=False, empty=False):
    path.mkdir()
    (path / 'meta.yaml').write_text(yaml.safe_dump({
        'algorithm':'Combined', 'intercept_start_ros_time':100,
    }))
    write_csv(path / 'summary.csv', [
        dict(status='complete', reason='done', hit=False, intercept_time=0, post_hit_duration_actual=0)
    ])
    rows = [dict(angular_row(0, 0), distance=3), dict(angular_row(0.1, 0), distance=2)]
    write_csv(path / 'timeseries.csv', [] if empty else rows, list(rows[0]))
    if sidecars:
        write_csv(path / 'observations.csv', [observation()])
        write_csv(path / 'geometry.csv', [
            geometry_row(vehicle, time, y=3 if vehicle == 'target' else 0)
            for time in (100, 100.1) for vehicle in ('self', 'target')
        ])


def test_existing_entrypoint_mixes_legacy_and_sidecar_outputs(tmp_path, monkeypatch):
    make_recording(tmp_path / 'legacy')
    make_recording(tmp_path / 'with_sidecars', sidecars=True)
    monkeypatch.setattr(phase2_analysis, 'plot_observation_errors', lambda *args: None)
    output = tmp_path / 'analysis'
    phase2_analysis.main([str(tmp_path), '--output', str(output)])
    results = json.loads((output / 'results.json').read_text())
    legacy, modern = results
    assert 'observations_aligned' not in legacy
    assert legacy['first_min_distance'] == modern['first_min_distance'] == 2
    assert modern['observation_events'] == 1
    assert modern['observations_error_plot'] is None
    assert modern['observations_control_abs_z_deg_median'] is None
    with open(modern['observations_aligned']) as stream:
        aligned = list(csv.DictReader(stream))
    assert len(aligned) == 1
    assert float(aligned[0]['truth_los_image_z_rad']) == 0
    assert math.isnan(float(aligned[0]['truth_los_control_z_rad']))
    with (output / 'metrics.csv').open() as stream:
        metrics = list(csv.DictReader(stream))
    assert metrics[1]['observation_events'] == '1'
    assert metrics[0]['observation_events'] == ''


def test_sidecars_are_analyzed_even_without_intercept_timer_rows(tmp_path, monkeypatch):
    recording = tmp_path / 'no_intercept'
    make_recording(recording, sidecars=True, empty=True)
    monkeypatch.setattr(phase2_analysis, 'plot_observation_errors', lambda *args: None)
    result = phase2_analysis.analyze_recording(recording, tmp_path)
    assert not result['valid']
    assert result['observation_events'] == 1
    assert not result['first_approach_epoch_available']
    assert result['first_approach_events'] == 0


def test_empty_sidecar_writes_schema_and_null_statistics(tmp_path, monkeypatch):
    recording = tmp_path / 'empty_observations'
    make_recording(recording, sidecars=True)
    write_csv(recording / 'observations.csv', [], ['image_time_ros', 'accepted', 'clock_valid'])
    monkeypatch.setattr(phase2_analysis, 'plot_observation_errors', lambda *args: None)
    result = phase2_analysis.analyze_recording(recording, tmp_path)
    assert result['observation_events'] == 0
    assert result['image_to_bbox_ms_count'] == 0
    assert result['image_to_bbox_ms_p95'] is None
    with open(result['observations_aligned']) as stream:
        reader = csv.DictReader(stream)
        assert 'los_error_image_z_rad' in reader.fieldnames
        assert list(reader) == []


def test_source_refresh_step_wraps_and_does_not_bridge_rejections_or_gaps():
    events = [
        observation(time=100, accepted=False, source=1, width=20, height=10),
        observation(time=100.02, source=2, los_z=math.radians(179)),
        observation(time=100.04, source=1, width=20, height=10, los_z=math.radians(-179)),
        observation(time=100.05, accepted=False, source=0),
        observation(time=100.08, source=2),
        observation(time=100.3, source=1, width=20, height=10),
    ]
    rows = align_observations(events, constant_geometry())
    assert rows[1]['time_since_yolo_s'] == pytest.approx(0.02)
    assert rows[2]['is_yolo_refresh']
    assert rows[2]['error_step_z_rad'] == pytest.approx(math.radians(2))
    assert rows[4]['time_since_yolo_s'] == pytest.approx(0.04)
    assert math.isnan(rows[4]['error_step_z_rad'])
    assert not rows[5]['is_yolo_refresh']
    assert math.isnan(rows[5]['dt_image_s'])


def test_source_error_statistics_respect_near_boundary_and_event_weighting():
    times = [100 + index * 0.025 for index in range(5)]
    distances = [1, 1.5, 3, 5, 10]
    angles = [1, 5, 2, 8, 3]
    sources = [1, 2, 2, 1, 2]
    samples = geometry_samples([
        geometry_row(vehicle, time, y=distance if vehicle == 'target' else 0)
        for time, distance in zip(times, distances) for vehicle in ('self', 'target')
    ])
    rows = align_observations([
        observation(time=time, source=source, los_z=math.radians(angle))
        for time, source, angle in zip(times, sources, angles)], samples)
    groups = {(row['scope'], row['source']): row for row in source_metrics(rows, 100, 0.1)}
    assert groups['near', 'YOLO']['los_events'] == 2
    assert groups['near', 'YOLO']['los_image_abs_z_deg_median'] == pytest.approx(4.5)
    assert groups['near', 'LightTrack']['los_image_abs_z_deg_median'] == pytest.approx(3.5)
    assert groups['5_17m', 'YOLO']['los_events'] == 1
    assert groups['2_5m', 'LightTrack']['los_events'] == 1


def test_prediction_compares_actual_control_los_without_relabeling_image_measurement():
    samples = geometry_samples([
        geometry_row(vehicle, time, x=(time - 100) * 3 if vehicle == 'target' else 0,
                     y=3 if vehicle == 'target' else 0)
        for time in (100, 100.1) for vehicle in ('self', 'target')
    ])
    row = align_observations([
        observation(time=100.03, control_time=100.08, los_z=0.3,
                    control_los_v=0, control_los_z=0.1, estimator=1)], samples)[0]
    assert row['los_error_image_z_rad'] == pytest.approx(0.3 - math.atan2(0.09, 3))
    assert row['raw_los_error_control_z_rad'] == pytest.approx(0.3 - math.atan2(0.24, 3))
    assert row['los_error_control_z_rad'] == pytest.approx(0.1 - math.atan2(0.24, 3))
    good = observation_metrics([row], 100, 0.1)
    assert good['first_approach_prediction_z_improvement_deg_median'] == pytest.approx(math.degrees(0.2))
    bad = align_observations([
        observation(time=100.03, control_time=100.08, los_z=0.3,
                    control_los_v=0, control_los_z=-0.4, estimator=1)], samples)[0]
    assert observation_metrics([bad], 100, 0.1)['first_approach_prediction_z_improvement_deg_median'] < 0
