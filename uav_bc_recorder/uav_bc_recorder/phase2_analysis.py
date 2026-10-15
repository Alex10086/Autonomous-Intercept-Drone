"""Offline phase-two metrics and first-encounter diagnostic windows."""
import argparse
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
    return math.atan2(math.sin(value),math.cos(value))


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


def analyze_recording(recording, output):
    meta=yaml.safe_load((recording/'meta.yaml').read_text())
    with (recording/'summary.csv').open() as stream:
        summary=next(csv.DictReader(stream))
    with (recording/'timeseries.csv').open() as stream:
        rows=list(csv.DictReader(stream))
    if not rows:
        return {'recording':str(recording),'status':summary['status'],'reason':summary['reason'],'valid':False}
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
