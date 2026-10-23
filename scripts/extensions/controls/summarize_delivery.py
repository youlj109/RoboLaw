"""Index all three independently accepted D-group tasks and risk witnesses."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import shutil

from kitchen_safety.extensions.controls.task_contract import task_contract


def read(path):
    return json.loads(path.read_text()) if path.is_file() else {}


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest() if path.is_file() else None


def task_evidence(key,report_path):
    report=read(report_path);contract=task_contract(key)
    rows=report.get('episodes',[]);hazards=[]
    source_sha=sha(contract['source'])
    gates={'accepted_independent_report':report.get('accepted') is True,
        'correct_task':report.get('task_id')==contract['task_id'],
        'four_unique_physical_roles':len(rows)==4 and sorted(r.get('variant','') for r in rows)==['base','human','property','robot'],
        'all_episode_gates':bool(rows) and all(r.get('passed') is True for r in rows)}
    for row in rows:
        ep=Path(row['path']);role=row.get('variant')
        result=read(ep/'result.json')
        gates[role+'_source_bound']=result.get('source_sha256')==source_sha
        if role=='base':continue
        stem='first_violation' if key=='temperature' else 'first_violation_links'
        path=ep/(stem+'_evidence.json');evidence=read(path)
        witness=evidence.get('resolved_source_witness',{})
        picture=Path(evidence.get('picture','missing'))
        wrist=evidence.get('human_wrist_world_m')
        wrist_valid=isinstance(wrist,list) and len(wrist)==3 and all(isinstance(v,(int,float)) and math.isfinite(v) for v in wrist)
        valid=bool(evidence) and witness.get('valid') is False and witness.get('robot_part')==evidence.get('robot_part')
        valid=valid and evidence.get('picture_sha256')==sha(picture) and picture.is_file()
        valid=valid and evidence.get('audit_sha256')==sha(ep/'avoidance_audit.json')
        valid=valid and (role!='human' or wrist_valid)
        gates[role+'_risk_witness_bound']=bool(valid)
        hazards.append({'variant':role,'episode':str(ep),'evidence':str(path),'evidence_sha256':sha(path),
            'first_violation_time_s':evidence.get('time_from_recorded_start_s'),
            'robot_link':evidence.get('robot_part'),'human_wrist_world_m':wrist,
            'source_gap_m':witness.get('distance_m'),'required_m':witness.get('required_m'),
            'annotated_keyframe':str(picture),'picture_sha256':sha(picture),
            'comparison_video':str(ep/'comparison.mp4'),'three_camera_video':str(ep/'all_cameras.mp4')})
    return {'task':key,'task_id':contract['task_id'],'accepted':all(gates.values()),'gates':gates,
        'report':str(report_path),'report_sha256':sha(report_path),'hazards':hazards,
        'baseline':[r['path'] for r in rows if r.get('variant')=='base']}


def main():
    p=argparse.ArgumentParser(description=__doc__)
    for key in ('left','right','temperature'):p.add_argument('--'+key,type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--publish-previews',type=Path)
    a=p.parse_args()
    if a.output.exists():raise FileExistsError(a.output)
    tasks=[task_evidence(key,getattr(a,key)) for key in ('left','right','temperature')]
    report={'accepted':all(t['accepted'] for t in tasks),'author':'zxw','tasks':tasks,
        'scope':'Three original fixed-layout tasks; three physical baselines and nine single successful hazard episodes. Three paired planner seeds per task, not ten physical seeds or a batch-success estimate. Unsafe baseline checks are offline geometry only. Data/video remain local, not Git-uploaded.',
        'models':'Explicit idealized passive mechanism overlays, source robot collision hulls and authored obstacle primitives/meshes; not manufacturer calibration or injury/fracture certification.'}
    if report['accepted'] and a.publish_previews:
        a.publish_previews.mkdir(parents=True,exist_ok=False)
        for task in tasks:
            for hazard in task['hazards']:
                prefix=task['task']+'_'+hazard['variant']
                for source,key,suffix in ((hazard['annotated_keyframe'],'published_keyframe','.png'),
                                          (hazard['evidence'],'published_evidence','.json')):
                    target=a.publish_previews/(prefix+suffix)
                    shutil.copyfile(source,target)
                    if sha(target)!=sha(Path(source)):raise ValueError('Published preview hash mismatch')
                    hazard[key]=str(target)
    a.output.parent.mkdir(parents=True,exist_ok=True)
    a.output.write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({'accepted':report['accepted'],'tasks':[(t['task'],t['accepted']) for t in tasks]}))
    raise SystemExit(0 if report['accepted'] else 2)


if __name__=='__main__':main()
