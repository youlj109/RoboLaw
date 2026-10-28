#!/usr/bin/env python3
"""CPU recorded-state drift report, excluding only the operated fridge joint."""
import argparse
import hashlib
import json
import sys
from pathlib import Path

ROOT=Path(__file__).resolve().parents[4]
sys.path.insert(0,str(ROOT))
import numpy as np
from pxr import Usd,UsdPhysics


def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def review(episode):
    metadata=json.loads((episode/'metadata.json').read_text())
    startup=json.loads((episode/'startup.json').read_text())['samples']
    physics=[r for r in json.loads((episode/'audit.json').read_text())['checks'] if r['sample_kind']=='physics_substep']
    samples=startup+physics
    inventory=metadata['native_mechanisms']
    paths=inventory['native_articulations']
    spec=metadata['hard_execution']['spec']
    stage=Usd.Stage.CreateInMemory()
    stage.DefinePrim('/World/envs/env_0/kitchen').GetReferences().AddReference(str(ROOT/'assets/kitchen/scene.usd'))
    steps=np.asarray([s['physics_step'] for s in samples])
    errors=[{'physics_step':s['physics_step'],'errors':s['native_mechanisms']['read_errors']}
            for s in samples if s['native_mechanisms']['read_errors']]
    coverage=all(set(s['native_mechanisms']['articulations'])==set(paths) for s in samples)
    operated='fridge_door_joint'
    joints=[];excluded=[]
    for root,description in paths.items():
        q=np.asarray([s['native_mechanisms']['articulations'][root]['q'] for s in samples])
        dq=np.asarray([s['native_mechanisms']['articulations'][root]['dq'] for s in samples])
        if q.shape!=dq.shape or q.shape!=(len(samples),description['dof_count']):raise ValueError('Misaligned native DOFs: '+root)
        for index,name in enumerate(description['dof_names']):
            path=description['dof_paths'][index]
            if name==operated and root.endswith('/fridge_main_group'):
                excluded.append({'root':root,'dof_name':name,'dof_path':path});continue
            prim=stage.GetPrimAtPath(path)
            free_axes=[]
            if prim and prim.GetTypeName()=='PhysicsJoint':
                # USD generic/D6 joint: classify units only when every linear
                # axis is explicitly locked and all remaining DOFs are rotary.
                limits={axis:(prim.GetAttribute('limit:'+axis+':physics:low').Get(),
                              prim.GetAttribute('limit:'+axis+':physics:high').Get())
                        for axis in ('transX','transY','transZ','rotX','rotY','rotZ')}
                if all(lo is not None and hi is not None for lo,hi in limits.values()):
                    free_axes=[axis for axis,(lo,hi) in limits.items() if hi>lo]
            generic_angular=bool(free_axes and all(axis.startswith('rot') for axis in free_axes)
                                 and len(free_axes)==description['dof_paths'].count(path))
            if prim and prim.IsA(UsdPhysics.RevoluteJoint):
                unit='rad';position_limit=spec['neutral_joint_drift_tolerance_rad'];speed_limit=spec['neutral_joint_velocity_tolerance_rad_s']
            elif generic_angular:
                unit='rad';position_limit=spec['neutral_joint_drift_tolerance_rad'];speed_limit=spec['neutral_joint_velocity_tolerance_rad_s']
            elif prim and prim.IsA(UsdPhysics.PrismaticJoint):
                unit='m';position_limit=.002;speed_limit=.02
            else:
                unit='unknown';position_limit=None;speed_limit=None
            drift=np.abs(q[:,index]-q[0,index]);speed=np.abs(dq[:,index]);i=int(np.argmax(drift));j=int(np.argmax(speed))
            joints.append({'articulation_root':root,'dof_name':name,'dof_path':path,
                'source_joint_type':prim.GetTypeName() if prim else None,'position_unit':unit,
                'source_generic_joint_unlocked_axes':free_axes,
                'reset_return_q':float(q[0,index]),'final_q':float(q[-1,index]),
                'maximum_absolute_drift':float(drift[i]),'drift_at_physics_step':int(steps[i]),
                'maximum_absolute_velocity':float(speed[j]),'velocity_at_physics_step':int(steps[j]),
                'diagnostic_drift_limit':position_limit,'diagnostic_velocity_limit':speed_limit,
                'diagnostic_limits_passed':bool(drift[i]<=position_limit and speed[j]<=speed_limit) if unit!='unknown' else None})
    failed_roots=[]
    for failure in inventory['initialization_errors']:
        prim=stage.GetPrimAtPath(failure['path'])
        source_joints=[p for p in Usd.PrimRange(prim) if p.IsA(UsdPhysics.Joint)] if prim else []
        failed_roots.append({**failure,'source_joint_paths':[str(p.GetPath()) for p in source_joints],
            'source_joint_types':[p.GetTypeName() for p in source_joints],
            'source_contains_only_fixed_joints':bool(source_joints and all(p.IsA(UsdPhysics.FixedJoint) for p in source_joints)),
            'retrospective_native_samples_filled':False})
    if len(excluded)!=1:raise ValueError('Expected exactly one explicitly excluded operated fridge joint')
    return {'scope':'Read-only drift and velocity of recorded native articulation DOFs from reset return through all operation physics substeps; only fridge_door_joint excluded.',
        'accepted':False,'episode':str(episode),'physics_samples':len(samples),
        'physics_step_coverage_continuous':bool(np.all(np.diff(steps)==1)),
        'duration_seconds':float((steps[-1]-steps[0])*metadata['physics_dt']),
        'authored_articulation_roots':len(inventory['authored_articulation_roots']),
        'native_articulations_sampled':len(paths),'non_operated_dofs_sampled':len(joints),
        'all_declared_successful_views_present_each_sample':coverage,'sample_read_errors':errors,
        'all_authored_articulations_observed':bool(inventory['all_authored_roots_read'] and not errors and coverage),
        'initialization_failures_with_source_topology':failed_roots,'excluded_operated_dof':excluded,
        'all_observed_non_operated_dofs_within_diagnostic_limits':all(r['diagnostic_limits_passed'] is True for r in joints),
        'threshold_scope':'Angular limits come from the trial config. Prismatic diagnostic limits are 2mm displacement and 0.02m/s speed; these are reported diagnostics, not independent physics acceptance.',
        'remaining_scope':['The recorded initialization error remains part of this run; no missing states are inferred or filled.',
                           'Fixed-only roots have no authored movable joint, but this does not replace native rigid-body pose observation.',
                           'Reset-internal steps, free-body drift, static-anchor motion and matched neutral comparison are not certified by this report.'],
        'input_sha256':{n:sha(episode/n) for n in ('metadata.json','startup.json','audit.json')},
        'source_dependency_sha256':{'assets/kitchen/scene.usd':sha(ROOT/'assets/kitchen/scene.usd'),
                                   str(Path(__file__).resolve().relative_to(ROOT)):sha(Path(__file__))},
        'joints':joints}


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('episode',type=Path);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    if a.output.exists():raise FileExistsError(a.output)
    r=review(a.episode);a.output.parent.mkdir(parents=True,exist_ok=True)
    with a.output.open('x') as f:json.dump(r,f,indent=2,allow_nan=False);f.write('\n')
    print(json.dumps({k:v for k,v in r.items() if k not in ('joints','input_sha256','source_dependency_sha256')},indent=2))


if __name__=='__main__':main()
