"""First-pass selection and sampled angular-rate boundaries."""
import math

import pytest

from uav_bc_recorder.phase2_analysis import diagnostic_rows, first_encounter


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
