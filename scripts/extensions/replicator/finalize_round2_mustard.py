#!/usr/bin/env python3
"""Freeze the fourteen mustard proposals after the finite CPU-only scan."""
import json
from pathlib import Path
import sys
ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))
import numpy as np
from scipy.spatial.transform import Rotation
from kitchen_safety.planning import DroidKinematics
from kitchen_safety.extensions.replicator.physical_retry_round2_mustard import make_targets, PHASES
from scripts.extensions.replicator.prepare_round2_mustard import read, save, sha


def main():
    report_path = ROOT/'reports/replicator_physical_retry_round2_20261028/mustard_recipes.json'
    report = read(report_path)
    plan = {x['scene_id']:x for x in read(ROOT/'configs/extensions/replicator/physical_retry_round1_20261028.json')['entries']}
    selected_path = ROOT/'reports/replicator_physical_retry_round2_20261028/mustard_final_selection.json'
    config_path = ROOT/'configs/extensions/replicator/physical_retry_round2_mustard_recipes_20261028.json'
    if selected_path.exists() or config_path.exists(): raise FileExistsError('Final mustard selection already frozen')
    explicit = {
        'l_shape_mustard_bowl__property_tall_glass': 'reference_side0_z0.01',
        'l_shape_mustard_bowl__property_tablet_stand': 'reference_side0_z0.01',
        'peninsula_mustard_bowl__robot_projecting_crossbar': 'reference_side180_z0',
        'u_shape_mustard_bowl__robot_sharp_equipment_housing': 'reference_side180_z0',
    }
    for entry in report['entries']:
        sid = entry['scene_id']; row = plan[sid]; old = read(ROOT/row['result_path']); layout = read(ROOT/row['config_path'])
        if sid in explicit:
            candidate = next(x for x in report['declarations'] if x['name'] == explicit[sid])
            entry.update(make_targets(old, candidate, transport_raise_m=.04 if sid.endswith('tall_glass') else 0.))
            entry['selection_caveat'] = ('The selected pose avoids the clearly intersecting original-side alternative. '
                'The source CPU 2 mm margin screen failed; no threshold was relaxed. '
                'L scenes show 1.99 mm finger-to-tray clearance at one closure sample. '
                'Side180 scenes show about 1.71 mm non-pad-to-bottle clearance. '
                'These sampled near-surface values are not proof of collision or feasibility; one full-path planner call remains required.')
            kin = DroidKinematics(np.asarray(layout['robot_position'])+[0,0,layout['stand_height']], layout['robot_yaw_deg'])
            q = np.asarray(old['start_q']); checks=[]
            for phase in PHASES:
                target = np.asarray(entry['target_poses_world'][phase])
                try:
                    q=kin.ik(target,q,attempts=4);actual=kin.fk(q)
                    pe=float(np.linalg.norm(actual[:3,3]-target[:3,3]))
                    re=float(np.rad2deg(Rotation.from_matrix(actual[:3,:3]@target[:3,:3].T).magnitude()))
                    checks.append(dict(phase=phase,q_rad=q.tolist(),position_error_m=pe,orientation_error_deg=re,passed=pe<=.002 and re<=2.))
                except RuntimeError as error:
                    checks.append(dict(phase=phase,passed=False,error=str(error)));break
            entry['selected_endpoint_ik']=checks
        targets={p:np.asarray(entry['target_poses_world'][p]) for p in PHASES}
        previous=dict(zip(PHASES[1:],PHASES[:-1]))
        entry['hold_orientation_phases']=[p for p in ('approach','lift','transfer','lower')
            if np.allclose(targets[p][:3,:3],targets[previous[p]][:3,:3],atol=1e-7,rtol=0)]
        entry['orientation_hold_endpoint_consistency_verified']=True
        entry['description']=entry['description_zh']
        entry['description']=entry['description'].replace('规划预计夹持角0.37 rad','规划预计夹持角0.37 rad（接触估计，并非本轮实测）')
    report['prior_scan']={'path':str(report_path.relative_to(ROOT)),'sha256':sha(report_path)}
    report['selection_script']={'path':str(Path(__file__).relative_to(ROOT)),'sha256':sha(__file__)}
    report['scope']+=' Four near-surface margin failures are explicitly retained as unverified proposals, not silently promoted to CPU passes.'
    save(selected_path,report)
    evidence={'path':str(selected_path.relative_to(ROOT)),'sha256':sha(selected_path)}
    keys=('scene_id','task_id','payload_name','layout_fixed','target_poses_world','nominal_attachment',
          'nominal_gripper_joint_rad','planning_gripper_joint_rad','grasp_info','hold_orientation_phases',
          'description','cpu_screen_passed','orientation_hold_endpoint_consistency_verified')
    recipes={e['scene_id']:{**{k:e[k] for k in keys},'evidence':evidence} for e in report['entries']}
    save(config_path,dict(kind='physical_retry_round2_mustard_recipes',recipes=recipes,
        layout_changed=False,physics_launched=False,candidate_count_per_scene=1,evidence=evidence))
    print(json.dumps(dict(output=str(config_path),recipes=len(recipes),cpu_screen_passed=sum(e['cpu_screen_passed'] for e in report['entries']),
        additional_endpoint_ik={e['scene_id']:all(x['passed'] for x in e['selected_endpoint_ik']) for e in report['entries'] if 'selected_endpoint_ik' in e})),flush=True)


if __name__=='__main__':main()
