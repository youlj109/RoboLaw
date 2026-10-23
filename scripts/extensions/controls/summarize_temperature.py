"""Assemble original level-three oven evidence without upgrading missing gates."""
import argparse
import hashlib
import json
from pathlib import Path

from kitchen_safety.extensions.controls.task_contract import task_contract
from scripts.extensions.controls.summarize_acceptance import read, paired_gates, probe_videos, sha


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--baseline',type=Path,required=True)
    p.add_argument('--episodes',type=Path,nargs=3,required=True)
    p.add_argument('--neutral',type=Path,required=True)
    p.add_argument('--rack-mobility',type=Path,required=True)
    p.add_argument('--faucet-mobility',type=Path,required=True)
    p.add_argument('--paired',type=Path,required=True)
    p.add_argument('--config',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args()
    if a.output.exists():raise FileExistsError(a.output)
    contract=task_contract('temperature');config=read(a.config)
    source_sha=hashlib.sha256(contract['source'].read_bytes()).hexdigest()
    episodes=[];results=[]
    baseline=read(a.baseline/'result.json')
    for path in [a.baseline,*a.episodes]:
        result=read(path/'result.json');results.append(result)
        validation=read(path/'independent_temperature_validation.json')
        mechanisms=read(path/'mechanism_audit.json')
        physics=read(path/'audit.json')
        self_check=read(path/'self_collision_detailed_audit.json')
        gates={'collector':result.get('accepted') is True,
            'independent_original_level_contact_recording_release':validation.get('passed') is True,
            'measured_self_collision':self_check.get('passed') is True,
            'all_background_joints':mechanisms.get('passed') is True and len(mechanisms.get('joints',[]))==90,
            'physics_rate_clearance':physics.get('passed') is True,
            'source_task':result.get('task_id')==contract['task_id'] and result.get('source_sha256')==source_sha,
            'same_controls':result.get('control_parameters')==baseline.get('control_parameters')}
        if result.get('variant')!='base':
            for name,key in [('fixed_base_hazard_audit','passed'),('avoidance_audit','accepted')]:
                gates[name]=read(path/(name+'.json')).get(key) is True
            strict=read(path/'strict_clearance_audit.json')
            gates['independent_actual_hazard_clearance']=strict.get('executed_augmented_trajectory',{}).get('passed') is True
            comparison=read(path/'comparison_validation.json')
            gates['comparison_decoded']=comparison.get('frames',0)>0 and comparison.get('video')==comparison.get('frames')
            gates['comparison_matches_baseline']=Path(comparison.get('baseline_video','missing')).parent.resolve()==a.baseline.resolve()
            gates['final_candidate_matches_actual']=result.get('variant_spec')==config.get('variants',{}).get(result.get('variant'))
        files=[*path.glob('*.json'),*path.glob('*.mp4'),path/'trajectory.npz',path/'images.h5']
        episodes.append({'path':str(path),'variant':result.get('variant'),'passed':all(gates.values()),'gates':gates,
            'frames':validation.get('frames',0),'seconds':validation.get('frames',0)*result.get('dt',0),
            'final_level':validation.get('final_level'),'first_loaded_contact_sample':validation.get('first_loaded_contact_sample'),
            'first_target_motion_sample':validation.get('first_target_motion_sample'),
            'first_success_sample':validation.get('first_success_sample'),'first_success_time_s':validation.get('first_success_time_s'),
            'maximum_angle_deg':validation.get('maximum_angle_deg'),'retreat_target_drift_deg':validation.get('retreat_target_drift_deg'),
            'file_sha256':{str(f):sha(f) for f in sorted(set(files)) if f.is_file()}})
    neutral=read(a.neutral/'validation.json')
    background=read(a.neutral/'background_mechanism_audit.json')
    rack=read(a.rack_mobility/'validation.json')
    faucet=read(a.faucet_mobility/'validation.json')
    paired=read(a.paired/'paired_planning.json')
    videos=probe_videos(a.neutral,len(neutral.get('states',[])),neutral.get('dt',0))
    gates={'all_four_real_episodes':all(e['passed'] for e in episodes),
        'one_of_each_variant':sorted(e['variant'] or '' for e in episodes)==['base','human','property','robot'],
        'original_goal_three_reset_minus_one':config.get('original_target_level')==3 and config.get('original_reset_level')==-1,
        'neutral_matches_original_task':neutral.get('task_id')==contract['task_id'] and neutral.get('source_sha256')==source_sha,
        'neutral_stationary_all_90_dofs':neutral.get('accepted') is True and background.get('passed') is True and len(background.get('joints',[]))==90,
        'neutral_covers_longest_execution':neutral.get('neutral_seconds',0)>=max(e['seconds'] for e in episodes),
        'neutral_three_videos_decoded':videos['passed'],
        'rack_external_force_mobility':rack.get('accepted') is True and rack.get('rack_mobility',{}).get('passed') is True,
        'faucet_external_torque_mobility':faucet.get('accepted') is True,
        'paired_matches_final_config':paired.get('config_sha256')==sha(a.config),
        **paired_gates(paired)}
    for name in ('toaster_physics_provenance','faucet_physics_provenance','oven_physics_provenance','temperature_knob_physics_provenance'):
        model=neutral.get(name)
        gates['same_'+name]=bool(model) and all(r.get(name)==model for r in results)
    gates['same_rack_mobility_model']=rack.get('oven_physics_provenance')==neutral.get('oven_physics_provenance')
    gates['same_faucet_mobility_model']=faucet.get('faucet_physics_provenance')==neutral.get('faucet_physics_provenance')
    supporting=[a.config]
    for directory in (a.neutral,a.rack_mobility,a.faucet_mobility,a.paired):
        supporting.extend(directory.glob('*.json'))
    report={'accepted':all(gates.values()),'gates':gates,'task_id':contract['task_id'],'episodes':episodes,
        'neutral':str(a.neutral),'paired':str(a.paired),'rack_mobility':str(a.rack_mobility),'faucet_mobility':str(a.faucet_mobility),
        'supporting_sha256':{str(f):sha(f) for f in supporting},'probe_videos':videos,
        'planning_policy':config.get('prepress_planning',{'method':'curobo_pose'}),
        'scope':'Original fixed-layout oven level3, one baseline and three single physical demonstrations. Robot-only contact with release/retreat; 120 Hz external clearance/contact/background, action-rate independent self and source counterfactual. Paired seed42/43/44 plans use the declared planning_policy with the same original TCP and control parameters. No physical batch-success estimate or universal necessity claim.',
        'models':'Explicit idealized passive balanced knob/faucet/toaster and damped racks; no manufacturer calibration, hidden locks or runtime target setters. Whole fragile floor lamp protected conservatively; no fracture/injury simulation.',
        'data_upload':'Raw data and videos are local ignored files, not uploaded by Git.'}
    a.output.parent.mkdir(parents=True,exist_ok=True)
    a.output.write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({'accepted':report['accepted'],'gates':gates},indent=2))
    raise SystemExit(0 if report['accepted'] else 2)


if __name__=='__main__':main()
