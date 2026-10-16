"""Offline phase-two metrics and first-encounter diagnostic windows.

Optional observations.csv/geometry.csv add event-weighted P0 diagnostics. The
aligned CSV retains every observation; missing geometry is NaN, never a held
position. Metrics use degrees, aligned residuals use radians (vision - truth).
First approach means intercept_start_ros_time through the first closest epoch.

Run ``phase2_analysis ROOT --output DIRECTORY`` as for legacy recordings. New
``*_observations_aligned.csv`` files distinguish ``los_error_image_{v,z}_rad``
from ``los_error_control_{v,z}_rad`` and ``receive_pose_error_image_{v,z}_rad``.
results.json and metrics.csv include ``image_to_{bbox,control}_ms_{count,median,p95}``,
``{scope}_{image,control}_abs_{v,z}_deg_{count,median,p95}``, geometry coverage,
and ``{scope}_paired_{v,z}_{image_abs_deg,receive_pose_abs_deg,improvement_deg}``
median/P95 with paired counts. Scopes are observations, first_approach (0.8..5m),
first_approach_all_distances, and first_approach_{0p8_2,2_5,5_17}m. No-coverage
statistics are JSON null. Optional ``*_observations_errors.png`` uses image-time
distance on the x-axis for all three residual types.
"""
import argparse
from bisect import bisect_left
import csv
import json
import math
from pathlib import Path

import yaml


def number(row, key):
    try:
        return float(row[key])
    except (KeyError, ValueError, TypeError):
        return math.nan


def truth(value):
    return str(value).lower() in ('true','1')


def norm(values):
    return math.sqrt(sum(value*value for value in values))


def wrapped(value):
    return math.atan2(math.sin(value),math.cos(value)) if math.isfinite(value) else math.nan


def first_encounter(rows):
    """First >=1m approach ends after separation grows 1m for >=0.2s.

    This rejects GPS jitter; it does not pick the best of later attempts.
    A hit tail is included and the first-hit timestamp is evaluated separately.
    """
    initial=number(rows[0],'distance')
    minimum=initial
    closest=0
    receding_since=None
    end=len(rows)-1
    for index,row in enumerate(rows):
        distance=number(row,'distance')
        elapsed=number(row,'elapsed_s')
        if not math.isfinite(distance):
            continue
        if distance<minimum:
            minimum=distance
            closest=index
            receding_since=None
        if elapsed<1.0 or initial-minimum<1.0:
            continue
        if distance>=minimum+1.0:
            if receding_since is None:
                receding_since=elapsed
            elif elapsed-receding_since>=0.2:
                end=index
                break
        else:
            receding_since=None
    return closest,end


def diagnostic_rows(rows):
    previous=None
    output=[]
    for row in rows:
        elapsed=number(row,'elapsed_s')
        relative=[number(row,'target_'+axis)-number(row,'self_'+axis) for axis in 'xyz']
        own=[number(row,'self_v'+axis) for axis in 'xyz']
        target=[number(row,'target_v'+axis) for axis in 'xyz']
        command=[number(row,'ref_v'+axis) for axis in 'xyz']
        distance=norm(relative)
        horizontal=math.hypot(*relative[:2])
        closing=-sum(r*(t-s) for r,t,s in zip(relative,target,own))/distance if distance>0 else math.nan
        quaternion=[number(row,'self_q'+axis) for axis in 'wxyz']
        w,x,y,z=quaternion
        qnorm=sum(v*v for v in quaternion)
        if qnorm>0:
            roll=math.atan2(2*(w*x+y*z),qnorm-2*(x*x+y*y))
            pitch=math.asin(max(-1.0,min(1.0,2*(w*y-z*x)/qnorm)))
            yaw=math.atan2(2*(w*z+x*y),qnorm-2*(y*y+z*z))
            yaw_rate=((math.sin(roll)*number(row,'self_wy')+math.cos(roll)*number(row,'self_wz'))
                      /math.cos(pitch)) if abs(math.cos(pitch))>1e-6 else math.nan
        else:
            yaw=yaw_rate=math.nan
        detected=truth(row.get('detected'))
        los=[number(row,'los_v'),number(row,'los_z')]
        rates=[math.nan,math.nan]
        if previous is not None:
            dt=elapsed-previous[0]
            if 0<dt<=0.15 and detected and previous[1]:
                rates=[wrapped(a-b)/dt if math.isfinite(a) and math.isfinite(b) else math.nan
                       for a,b in zip(los,previous[2])]
        previous=(elapsed,detected,los)
        own_speed,command_speed=norm(own),norm(command)
        cosine=sum(a*b for a,b in zip(own,command))/(own_speed*command_speed) if own_speed*command_speed>0.01 else math.nan
        angle=math.degrees(math.acos(max(-1,min(1,cosine)))) if math.isfinite(cosine) else math.nan
        output.append({
            'elapsed_s':elapsed,'distance':distance,'horizontal_distance':horizontal,
            'height_error':relative[2],'bbox_x':number(row,'bbox_x'),'bbox_y':number(row,'bbox_y'),
            'bbox_w':number(row,'bbox_w'),'bbox_h':number(row,'bbox_h'),'detection_valid':detected,
            'ex':number(row,'ex'),'ey':number(row,'ey'),'los_v':los[0],'los_z':los[1],
            'los_rate_v':rates[0],'los_rate_z':rates[1],
            'evaluation_los_v':math.atan2(relative[2],horizontal),
            'evaluation_los_z':math.atan2(relative[0],relative[1]),
            'yaw':yaw,'yaw_rate':yaw_rate,'cmd_vx':command[0],'cmd_vy':command[1],'cmd_vz':command[2],
            'controller_state':row.get('state',''),'self_speed':own_speed,'target_speed':norm(target),
            'closing_speed':closing,'command_speed':command_speed,'command_actual_angle_deg':angle,
            'hit':truth(row.get('hit')),
        })
    return output


MAX_GEOMETRY_GAP_S = 0.15
OBSERVATION_FIELDS = (
    'image_stamp_s', 'image_time_ros', 'bbox_receive_time', 'projection_time',
    'control_time', 'attitude_before_time', 'attitude_after_time', 'image_to_bbox_ms',
    'image_to_control_ms', 'clock_valid', 'accepted', 'attitude_synced', 'status',
    'source', 'confidence', 'x', 'y', 'width', 'height', 'los_v', 'los_z',
    'receive_pose_los_v', 'receive_pose_los_z',
    'estimator', 'estimate_time_ros', 'reference_time_ros', 'estimated_u', 'estimated_v', 'estimated_du', 'estimated_dv',
    'pixel_prediction_dt', 'attitude_prediction_dt', 'control_los_v', 'control_los_z',
)


def geometry_samples(rows):
    """Keep finite, strictly increasing source epochs per aircraft in event order.

    Never sort resets into a plausible trajectory, or substitute receipt epochs
    for incompatible/missing source epochs. Aircraft need not share timestamps.
    """
    samples = {'self': [], 'target': []}
    for row in rows:
        vehicle = row.get('vehicle')
        if vehicle not in samples:
            continue
        sample = tuple(number(row, key) for key in ('source_time', 'x', 'y', 'z'))
        if not all(math.isfinite(value) for value in sample):
            continue
        if samples[vehicle] and sample[0] <= samples[vehicle][-1][0]:
            continue
        samples[vehicle].append(sample)
    return samples


def interpolate_position(samples, epoch, max_gap=MAX_GEOMETRY_GAP_S):
    """Interpolate monotonic (source_time, N, E, D) tuples without extrapolation.

    Exact endpoints remain usable even beside a large gap. The small tolerance
    only accommodates subtraction roundoff at large ROS/system epochs.
    """
    missing = (math.nan, math.nan, math.nan)
    if not math.isfinite(epoch) or not samples:
        return missing
    index = bisect_left(samples, (epoch,))
    if index < len(samples) and samples[index][0] == epoch:
        return samples[index][1:]
    if index == 0 or index == len(samples):
        return missing
    before, after = samples[index - 1], samples[index]
    gap = after[0] - before[0]
    tolerance = max(math.ulp(before[0]), math.ulp(after[0]))
    if gap <= 0 or gap > max_gap + tolerance:
        return missing
    weight = (epoch - before[0]) / gap
    return tuple(a + weight * (b - a) for a, b in zip(before[1:], after[1:]))


def annotate_observations(rows):
    """Source history and wrapped step errors; never bridge rejected frames/gaps."""
    previous = None
    last_epoch = last_yolo = math.nan
    for row in rows:
        epoch = number(row, 'image_time_ros')
        source = number(row, 'source')
        valid_clock = row['clock_valid'] and math.isfinite(epoch)
        if not valid_clock or (math.isfinite(last_epoch) and epoch <= last_epoch):
            previous = None
            last_yolo = math.nan
        last_epoch = epoch if valid_clock else math.nan
        box_valid = number(row, 'width') > 0 and number(row, 'height') > 0
        if valid_clock and box_valid and source == 1:
            last_yolo = epoch
        row['bbox_center_u'] = number(row, 'x') + number(row, 'width') / 2 if box_valid else math.nan
        row['bbox_center_v'] = number(row, 'y') + number(row, 'height') / 2 if box_valid else math.nan
        row['time_since_yolo_s'] = epoch - last_yolo if valid_clock and math.isfinite(last_yolo) else math.nan
        row['source_name'] = {1: 'YOLO', 2: 'LightTrack'}.get(source, 'unknown')
        row['is_yolo_refresh'] = False
        row['dt_image_s'] = math.nan
        for axis in ('v', 'z'):
            row[f'los_step_{axis}_rad'] = row[f'error_step_{axis}_rad'] = math.nan
        eligible = valid_clock and row['accepted'] and all(
            math.isfinite(number(row, 'los_' + axis)) for axis in ('v', 'z'))
        if eligible and previous is not None:
            dt = epoch - number(previous, 'image_time_ros')
            if 0 < dt <= 0.15:
                row['dt_image_s'] = dt
                row['is_yolo_refresh'] = source == 1 and number(previous, 'source') == 2
                for axis in ('v', 'z'):
                    row[f'los_step_{axis}_rad'] = wrapped(number(row, 'los_' + axis) - number(previous, 'los_' + axis))
                    row[f'error_step_{axis}_rad'] = wrapped(
                        number(row, f'los_error_image_{axis}_rad') -
                        number(previous, f'los_error_image_{axis}_rad'))
        previous = row if eligible else None
    return rows


def align_observations(observations, samples):
    """Produce one row per bbox, distinguishing image and first-control truth.

    clock_valid gates both epochs. Accepted observations without control_time
    (superseded before a setpoint) still contribute image-time diagnostics.
    """
    aligned = []
    for event in observations:
        row = {key: event.get(key, '') for key in OBSERVATION_FIELDS}
        for key in ('accepted', 'clock_valid', 'attitude_synced'):
            row[key] = truth(event.get(key))
        for phase, time_key in (('image', 'image_time_ros'), ('control', 'control_time')):
            epoch = number(event, time_key) if row['clock_valid'] else math.nan
            positions = {vehicle: interpolate_position(samples[vehicle], epoch)
                         for vehicle in ('self', 'target')}
            covered = all(math.isfinite(value) for position in positions.values() for value in position)
            row[phase + '_geometry_covered'] = covered
            for vehicle, position in positions.items():
                for axis, value in zip('xyz', position):
                    row[f'{vehicle}_{phase}_{axis}'] = value
            relative = [t - s for t, s in zip(positions['target'], positions['self'])]
            distance = norm(relative)
            row[phase + '_distance_m'] = distance
            angles = (math.atan2(relative[2], math.hypot(*relative[:2])),
                      math.atan2(relative[0], relative[1])) if distance > 0 else (math.nan, math.nan)
            for axis, angle in zip(('v', 'z'), angles):
                row[f'truth_los_{phase}_{axis}_rad'] = angle
                row[f'los_error_{phase}_{axis}_rad'] = wrapped(number(event, 'los_' + axis) - angle)
                if phase == 'image':
                    row[f'receive_pose_error_image_{axis}_rad'] = wrapped(
                        number(event, 'receive_pose_los_' + axis) - angle)
                else:
                    # Legacy records used raw image LOS for control; new records
                    # explicitly preserve the actual predicted control LOS.
                    control_key = 'control_los_' + axis
                    control_los = number(event, control_key) if control_key in event else number(event, 'los_' + axis)
                    row[f'raw_los_error_control_{axis}_rad'] = row[f'los_error_control_{axis}_rad']
                    row[f'los_error_control_{axis}_rad'] = wrapped(control_los - angle)
        aligned.append(row)
    return annotate_observations(aligned)


def percentile(values, fraction):
    """Linearly interpolated percentile; None denotes no finite coverage."""
    values = sorted(value for value in values if math.isfinite(value))
    if not values:
        return None
    index = (len(values) - 1) * fraction
    lower = math.floor(index)
    upper = math.ceil(index)
    return values[lower] + (index - lower) * (values[upper] - values[lower])


def error_metrics(rows, prefix):
    """Compare paired image-pose/receipt-pose errors on identical observations."""
    metrics = {prefix + '_events': len(rows)}
    metrics[prefix + '_attitude_synced_events'] = sum(row['attitude_synced'] for row in rows)
    metrics[prefix + '_control_time_events'] = sum(math.isfinite(number(row, 'control_time')) for row in rows)
    for phase in ('image', 'control'):
        covered = sum(bool(row[phase + '_geometry_covered']) for row in rows)
        metrics[f'{prefix}_{phase}_geometry_count'] = covered
        metrics[f'{prefix}_{phase}_geometry_coverage'] = covered / len(rows) if rows else None
        for axis in ('v', 'z'):
            values = [abs(math.degrees(row[f'los_error_{phase}_{axis}_rad'])) for row in rows]
            key = f'{prefix}_{phase}_abs_{axis}_deg'
            metrics[key + '_count'] = sum(math.isfinite(value) for value in values)
            metrics[key + '_median'] = percentile(values, 0.5)
            metrics[key + '_p95'] = percentile(values, 0.95)
    for axis in ('v', 'z'):
        paired = [(abs(math.degrees(row[f'los_error_image_{axis}_rad'])),
                   abs(math.degrees(row[f'receive_pose_error_image_{axis}_rad']))) for row in rows]
        paired = [(vision, receipt) for vision, receipt in paired
                  if math.isfinite(vision) and math.isfinite(receipt)]
        key = f'{prefix}_paired_{axis}'
        metrics[key + '_count'] = len(paired)
        for label, values in (
                ('image_abs_deg', [vision for vision, _ in paired]),
                ('receive_pose_abs_deg', [receipt for _, receipt in paired]),
                ('improvement_deg', [receipt - vision for vision, receipt in paired])):
            metrics[f'{key}_{label}_median'] = percentile(values, 0.5)
            metrics[f'{key}_{label}_p95'] = percentile(values, 0.95)
        control_pairs = [(abs(math.degrees(row[f'raw_los_error_control_{axis}_rad'])),
                          abs(math.degrees(row[f'los_error_control_{axis}_rad']))) for row in rows]
        control_pairs = [(raw, estimated) for raw, estimated in control_pairs
                         if math.isfinite(raw) and math.isfinite(estimated)]
        key = f'{prefix}_prediction_{axis}'
        metrics[key + '_count'] = len(control_pairs)
        improvements = [raw - estimated for raw, estimated in control_pairs]
        metrics[key + '_improvement_deg_median'] = percentile(improvements, 0.5)
        metrics[key + '_improvement_deg_p95'] = percentile(improvements, 0.95)
    return metrics


def observation_metrics(aligned, intercept_start, closest_elapsed):
    """Flat JSON/CSV metrics, weighted once per accepted finite vision event.

    first_approach is 0.8..5m inclusive; distance bins are [0.8,2),
    [2,5), [5,17]m. Positive paired improvement means image-pose is better.
    Latencies use only finite nonnegative event timestamps, not timer rows.
    """
    eligible = [row for row in aligned if row['accepted'] and row['clock_valid']
                and all(math.isfinite(number(row, key))
                        for key in ('image_time_ros', 'los_v', 'los_z'))]
    metrics = {'observation_events': len(aligned), 'observation_eligible_events': len(eligible)}
    metrics.update(error_metrics(eligible, 'observations'))
    for label, key in (('image_to_bbox', 'bbox_receive_time'), ('image_to_control', 'control_time')):
        latencies = [(number(row, key) - number(row, 'image_time_ros')) * 1000 for row in eligible
                     if math.isfinite(number(row, key))
                     and number(row, key) >= number(row, 'image_time_ros')]
        metrics[label + '_ms_count'] = len(latencies)
        metrics[label + '_ms_median'] = percentile(latencies, 0.5)
        metrics[label + '_ms_p95'] = percentile(latencies, 0.95)
    window_valid = math.isfinite(intercept_start) and math.isfinite(closest_elapsed)
    metrics['first_approach_epoch_available'] = window_valid
    approach = [row for row in eligible if window_valid
                and intercept_start <= number(row, 'image_time_ros') <= intercept_start + closest_elapsed]
    metrics.update(error_metrics(approach, 'first_approach_all_distances'))
    near = [row for row in approach if 0.8 <= row['image_distance_m'] <= 5.0]
    metrics.update(error_metrics(near, 'first_approach'))
    for label, lower, upper in (('0p8_2', 0.8, 2.0), ('2_5', 2.0, 5.0), ('5_17', 5.0, 17.0)):
        subset = [row for row in approach if lower <= row['image_distance_m']
                  and (row['image_distance_m'] <= upper if upper == 17.0
                       else row['image_distance_m'] < upper)]
        metrics.update(error_metrics(subset, 'first_approach_' + label + 'm'))
    return metrics


def source_metrics(aligned, intercept_start, closest_elapsed):
    """Event-weighted first-approach source diagnostics, separate from hit rates."""
    approach = [row for row in aligned if row['accepted'] and row['clock_valid']
                and intercept_start <= number(row, 'image_time_ros') <= intercept_start + closest_elapsed
                and all(math.isfinite(number(row, key)) for key in ('los_v', 'los_z'))]
    output = []
    for scope, lower, upper in (
            ('all_first_approach', 0, math.inf), ('near', 0.8, 5),
            ('0p8_2m', 0.8, 2), ('2_5m', 2, 5), ('5_17m', 5, 17)):
        for source, label in ((1, 'YOLO'), (2, 'LightTrack')):
            selected = [row for row in approach if number(row, 'source') == source
                        and (scope == 'all_first_approach' or
                             lower <= row['image_distance_m'] < upper or
                             ((upper == 17 or scope == 'near') and row['image_distance_m'] == upper))]
            result = {'scope': scope, 'source': label}
            result.update(error_metrics(selected, 'los'))
            result['yolo_refresh_events'] = sum(row['is_yolo_refresh'] for row in selected)
            for label, key in (('image_to_bbox_ms', 'image_to_bbox_ms'),
                               ('image_to_control_ms', 'image_to_control_ms'),
                               ('time_since_yolo_s', 'time_since_yolo_s'),
                               ('confidence', 'confidence')):
                values = [number(row, key) for row in selected]
                result[label + '_median'] = percentile(values, 0.5)
                result[label + '_p95'] = percentile(values, 0.95)
            for axis in ('v', 'z'):
                steps = [abs(math.degrees(number(row, f'error_step_{axis}_rad'))) for row in selected]
                result[f'error_step_abs_{axis}_deg_median'] = percentile(steps, 0.5)
                result[f'error_step_abs_{axis}_deg_p95'] = percentile(steps, 0.95)
            output.append(result)
    return output


def plot_los_timeline(aligned, path, intercept_start, closest_elapsed):
    """Image-time LOS versus same-time GPS geometry around the first encounter."""
    if not math.isfinite(intercept_start) or not math.isfinite(closest_elapsed):
        return None
    try:
        from matplotlib.figure import Figure
        from matplotlib.backends.backend_agg import FigureCanvasAgg
    except ImportError:
        return None
    selected = [row for row in aligned if row['accepted'] and row['clock_valid']
                and closest_elapsed - 2 <= number(row, 'image_time_ros') - intercept_start <= closest_elapsed + 0.5]
    if not selected:
        return None
    figure = Figure(figsize=(10, 7))
    FigureCanvasAgg(figure)
    for index, axis in enumerate(('v', 'z'), start=1):
        panel = figure.add_subplot(2, 1, index)
        times, truth_angles, vision_angles = [], [], []
        previous_time = previous_truth = math.nan
        for row in selected:
            elapsed = number(row, 'image_time_ros') - intercept_start
            angle = number(row, f'truth_los_image_{axis}_rad')
            if not 0 < elapsed - previous_time <= 0.15:
                times.append(math.nan); truth_angles.append(math.nan); vision_angles.append(math.nan)
                previous_truth = math.nan
            unwrapped = previous_truth + wrapped(angle - previous_truth) if math.isfinite(previous_truth) else angle
            times.append(elapsed)
            truth_angles.append(math.degrees(unwrapped))
            vision_angles.append(math.degrees(unwrapped + number(row, f'los_error_image_{axis}_rad')))
            previous_time, previous_truth = elapsed, unwrapped
        panel.plot(times, truth_angles, color='black', linewidth=1, label='GPS geometry (image time)')
        for source, label, marker in ((1, 'YOLO LOS', 'o'), (2, 'LightTrack LOS', 'x')):
            points = [row for row in selected if number(row, 'source') == source]
            # Use the shared unwrapped angle basis; NaN separators have no event.
            values = [value for value, time in zip(vision_angles, times) if math.isfinite(time)]
            indices = [i for i, row in enumerate(selected) if number(row, 'source') == source]
            panel.scatter([number(row, 'image_time_ros') - intercept_start for row in points],
                          [values[i] for i in indices], s=12, marker=marker, label=label)
        panel.set_xlabel('Time since INTERCEPT (s)')
        panel.set_ylabel(f'LOS {axis} (deg)')
        panel.legend()
    figure.tight_layout()
    figure.savefig(path)
    return str(path)


def plot_observation_errors(aligned, path, intercept_start, closest_elapsed):
    """Optional diagnostic only; analysis does not require matplotlib."""
    try:
        from matplotlib.figure import Figure
        from matplotlib.backends.backend_agg import FigureCanvasAgg
    except ImportError:
        return None
    predicted = any(number(row, 'estimator') > 0 for row in aligned if row['accepted'])
    fields = (('raw_los_error_control', 'Raw image held to control time'),
              ('los_error_control', 'Predicted LOS at control time')) if predicted else (
                  ('los_error_image', 'Image-time vision'),
                  ('los_error_control', 'First-control-time vision'),
                  ('receive_pose_error_image', 'Receipt-pose at image time'))
    distance_field = 'control_distance_m' if predicted else 'image_distance_m'
    figure = Figure(figsize=(10, 7))
    FigureCanvasAgg(figure)
    for index, axis in enumerate(('v', 'z'), start=1):
        panel = figure.add_subplot(2, 1, index)
        for field, label in fields:
            points = [(row[distance_field], abs(math.degrees(row[f'{field}_{axis}_rad'])))
                      for row in aligned if row['accepted'] and row['clock_valid']
                      and all(math.isfinite(number(row, key)) for key in ('image_time_ros', 'los_v', 'los_z'))
                      and intercept_start <= number(row, 'image_time_ros') <= intercept_start + closest_elapsed
                      and 0.8 <= row[distance_field] <= 17.0
                      and math.isfinite(row[f'{field}_{axis}_rad'])]
            panel.scatter([point[0] for point in points], [point[1] for point in points],
                          s=8, alpha=0.6, label=label)
        panel.set_xlabel('Control-time separation (m)' if predicted else 'Image-time separation (m)')
        panel.set_ylabel(f'|{axis} residual| (deg)')
        panel.set_ylim(bottom=0)
        panel.legend()
    figure.tight_layout()
    figure.savefig(path)
    return str(path)


def analyze_observations(recording, output, name, meta, closest_elapsed):
    observations_path = recording / 'observations.csv'
    geometry_path = recording / 'geometry.csv'
    if not observations_path.exists() or not geometry_path.exists():
        return {}
    with observations_path.open() as stream:
        observations = list(csv.DictReader(stream))
    with geometry_path.open() as stream:
        samples = geometry_samples(csv.DictReader(stream))
    aligned = align_observations(observations, samples)
    path = output / (name + '_observations_aligned.csv')
    # Derive the schema even for a recording with no bbox events.
    fields = list((aligned or align_observations([{}], samples))[0])
    with path.open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(aligned)
    start = number(meta, 'intercept_start_ros_time')
    metrics = observation_metrics(aligned, start, closest_elapsed)
    metrics.update(observations_aligned=str(path), geometry_max_gap_s=MAX_GEOMETRY_GAP_S,
                   geometry_self_samples=len(samples['self']), geometry_target_samples=len(samples['target']))
    metrics['observations_error_plot'] = plot_observation_errors(
        aligned, output / (name + '_observations_errors.png'), start, closest_elapsed)
    source_path = output / (name + '_source_metrics.csv')
    sources = source_metrics(aligned, start, closest_elapsed)
    with source_path.open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(sources[0]))
        writer.writeheader()
        writer.writerows(sources)
    metrics['source_metrics'] = str(source_path)
    metrics['los_timeline_plot'] = plot_los_timeline(
        aligned, output / (name + '_los_timeline.png'), start, closest_elapsed)
    return metrics


def analyze_recording(recording, output):
    meta=yaml.safe_load((recording/'meta.yaml').read_text())
    with (recording/'summary.csv').open() as stream:
        summary=next(csv.DictReader(stream))
    with (recording/'timeseries.csv').open() as stream:
        rows=list(csv.DictReader(stream))
    if not rows:
        result = {'recording':str(recording),'status':summary['status'],'reason':summary['reason'],'valid':False}
        requested = meta.get('experiment_conditions', {}).get('requested') or {}
        name = f"{recording.name}_{meta.get('algorithm', 'unknown')}_{requested.get('group', 'unknown')}_{requested.get('trial', 'unknown')}"
        result.update(analyze_observations(recording, output, name, meta, math.nan))
        return result
    closest,end=first_encounter(rows)
    diagnostics=diagnostic_rows(rows)
    tclose=number(rows[closest],'elapsed_s')
    first_end=number(rows[end],'elapsed_s')
    initial=meta.get('initial_conditions') or {}
    requested=meta.get('experiment_conditions',{}).get('requested') or {}
    hit_time=number(summary,'intercept_time')
    first_hit=truth(summary['hit']) and math.isfinite(hit_time) and hit_time<=first_end
    selected=[r for r in diagnostics if tclose-2.0<=r['elapsed_s']<=tclose+3.0]
    name=f"{recording.name}_{meta['algorithm']}_{requested.get('group','unknown')}_{requested.get('trial','unknown')}"
    window=output/(name+'_first_encounter.csv')
    with window.open('w',newline='') as stream:
        writer=csv.DictWriter(stream,fieldnames=list(diagnostics[0]))
        writer.writeheader();writer.writerows(selected)
    observed_release=requested.get('actual_release',{})
    valid=(abs(initial.get('distance',math.inf)-requested.get('requested_distance',0))<=requested.get('distance_tolerance',0.8)
           and abs(initial.get('height_difference',math.inf)-requested.get('requested_height_difference',0.3))<=requested.get('height_tolerance',0.18))
    result={
        'recording':str(recording),'algorithm':meta['algorithm'],'group':requested.get('group'),
        'trial':requested.get('trial'),'vision_mode':requested.get('vision_mode'),'valid':valid,
        'attitude_sync':requested.get('attitude_sync'),
        'pixel_estimator':requested.get('pixel_estimator', 'raw'),
        'image_reliability':requested.get('image_reliability', 'best_effort'),
        'confidence_threshold':requested.get('confidence_threshold'),
        'acquisition_threshold':requested.get('acquisition_threshold'),
        'yolo_refresh_interval':requested.get('yolo_refresh_interval'),
        'tracking_validation_iou':requested.get('tracking_validation_iou'),
        'camera_visibility_policy':requested.get('camera_visibility_policy'),
        'initial_distance':initial.get('distance'),
        'initial_height_difference':initial.get('height_difference'),
        'initial_self_velocity':initial.get('self_velocity_ned'),
        'initial_target_velocity':initial.get('target_velocity_ned'),
        'target_phase_commanded':initial.get('target_phase_rad'),
        'target_phase_actual_at_release':observed_release.get('actual_target_phase_rad'),
        'entry_delay':requested.get('observed_entry_delay'),
        'first_hit':first_hit,'first_min_distance':number(rows[closest],'distance'),
        'first_min_time':tclose,'first_horizontal_distance':diagnostics[closest]['horizontal_distance'],
        'first_height_error':diagnostics[closest]['height_error'],
        'detection_at_first_min':diagnostics[closest]['detection_valid'],
        'lost_near_first_encounter':any(not r['detection_valid'] for r in diagnostics
                                       if tclose-0.5<=r['elapsed_s']<=tclose+3.0),
        'final_hit':truth(summary['hit']),'hit_time':hit_time,'status':summary['status'],'reason':summary['reason'],
        'post_hit_duration_actual':number(summary,'post_hit_duration_actual'),
        'tail_truncated':truth(summary.get('tail_truncated')),
        'detected_fraction':sum(r['detection_valid'] for r in diagnostics)/len(diagnostics),
        'track_lost_fraction':sum(r['controller_state']=='TRACK_LOST' for r in diagnostics)/len(diagnostics),
        'window':str(window),
    }
    result.update(analyze_observations(recording, output, name, meta, tclose))
    return result


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('root',type=Path)
    parser.add_argument('--output',type=Path)
    args=parser.parse_args(argv)
    output=args.output or args.root/'analysis'
    output.mkdir(parents=True,exist_ok=True)
    results=[]
    for summary in sorted(args.root.glob('**/summary.csv')):
        result=analyze_recording(summary.parent,output)
        result={key:(None if isinstance(value,float) and not math.isfinite(value) else value)
                for key,value in result.items()}
        results.append(result)
    (output/'results.json').write_text(json.dumps(results,indent=2)+'\n')
    if results:
        with (output/'metrics.csv').open('w',newline='') as stream:
            writer=csv.DictWriter(stream,fieldnames=list(dict.fromkeys(k for r in results for k in r)))
            writer.writeheader();writer.writerows(results)
    for result in results:
        print(json.dumps(result),flush=True)
    print(f'ANALYSIS={output}',flush=True)


if __name__=='__main__':
    main()
