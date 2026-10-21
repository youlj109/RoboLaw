"""Cross-replay both measured paths in both measured source backgrounds.

The original active lime follows each trajectory's measured pose. Destination
wooden bowl and all three non-task source rigid objects use each run's measured
initial background pose. This isolates source settling from the added hazard.
No unsafe simulation is executed and no physical objects are moved.
"""
import argparse, hashlib, json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from pxr import Usd
from kitchen_safety.planning import DroidKinematics
from kitchen_safety.extensions.lightwheel_pick_place.collision import CollisionWorld

BACKGROUND=('wooden_bowl','broccoli','sweet_potato','red_bell_pepper')

def digest(path):return hashlib.sha256(path.read_bytes()).hexdigest()

def audit(baseline,episode):
    paths={'baseline':Path(baseline),'augmented':Path(episode)}
    results={k:json.loads((p/'result.json').read_text()) for k,p in paths.items()}
    traces={k:json.loads((p/'full_world_audit.json').read_text())['measurements'] for k,p in paths.items()}
    b=results['baseline']['robot_base_pose'];kin=DroidKinematics(b[:3],Rotation.from_quat(b[3:7]).as_euler('xyz',degrees=True)[2])
    world=CollisionWorld(Usd.Stage.Open(str(paths['baseline']/'scene.usda')),kin)
    combinations={}
    for background,metadata in results.items():
        for name in BACKGROUND:world.set_obstacle_pose(name,metadata['initial_object_states'][name][:7])
        for trajectory,states in traces.items():
            rows=[]
            for index,state in enumerate(states):
                world.allowed_phase=state['phase']
                clearance=world.clearance(state['q_arm'],('lime',),mode='execution',
                    gripper_q=state['gripper_q'],object_pose=state['lime_pose'],obstacle_filter=BACKGROUND)
                rows.append({'physics_index':index,'action':state['action'],'phase':state['phase'],**clearance})
            combinations[trajectory+'_path_in_'+background+'_background']={'passed':all(x['valid'] for x in rows),
                'samples':len(rows),'minimum':min(rows,key=lambda x:x['slack_m']),
                'violations':sum(not x['valid'] for x in rows),'measurements':rows}
            print('BACKGROUND',background,'PATH',trajectory,combinations[trajectory+'_path_in_'+background+'_background']['passed'],flush=True)
    return {'passed':all(x['passed'] for x in combinations.values()),'baseline_episode':str(paths['baseline']),
        'episode':str(paths['augmented']),'input_sha256':{k:{n:digest(p/n) for n in ('result.json','full_world_audit.json')} for k,p in paths.items()},
        'background_objects':list(BACKGROUND),'sampling_frequency_hz':120,'combinations':combinations,
        'scope':'Both actual arm/finger/carried-lime paths must remain safe in both measured initial source backgrounds, including bowl and all drifting distractors. Active lime uses each path actual physics pose; the source task destination support exception remains phase-scoped. No added hazard is used in this nuisance-only test; no physics state setters.'}

def main():
    p=argparse.ArgumentParser();p.add_argument('--baseline',required=True,type=Path);p.add_argument('--episode',required=True,type=Path);p.add_argument('--output',type=Path);a=p.parse_args()
    target=a.output or a.episode/'source_background_audit.json'
    if target.exists():raise FileExistsError(f'Preserve {target}; specify --output for a new audit')
    result=audit(a.baseline,a.episode)
    with target.open('x') as stream:json.dump(result,stream,indent=2)
    print(json.dumps({k:v for k,v in result.items() if k!='combinations'},indent=2));return 0 if result['passed'] else 2
if __name__=='__main__':raise SystemExit(main())
