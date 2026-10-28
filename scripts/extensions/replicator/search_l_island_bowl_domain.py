#!/usr/bin/env python3
"""CPU original-validator domain, source stand witnesses, and reach distances.

Derives the complete continuous source On/NextTo position intervals for fixed
source yaw. Samples that entire domain, retaining all failures. A finite grid
is never treated as a continuous infeasibility proof. No simulation or assets
are modified, and all saved witnesses remain execution_ready=false.
"""
import argparse
import copy
import json
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT))
import numpy as np
from pxr import Usd
from scipy.spatial.transform import Rotation
from isaaclab_arena.relations.relations import On,NextTo
from kitchen_safety.planning import DroidKinematics
from kitchen_safety.extensions.replicator.collision_planner import CollisionWorld
from scripts.extensions.replicator.variant_geometry import SourceRobotGeometry,code_evidence
from scripts.extensions.replicator.audit_fixed_base import fixed_colliders
from scripts.extensions.replicator.audit_initial_robot_geometry import source_initial_q,near_failures
from scripts.extensions.replicator.audit_source_layout_geometry import REQUIRED_M,minimum_distance,sha,diagnose
from scripts.extensions.replicator.audit_task_source_relation import check_layout
from scripts.extensions.replicator.search_source_layout_candidates import CandidateSearch,TranslatedShapes


def source_position_domain(scene,subject):
    """Direct algebra of the unmodified validators; original bbox precision."""
    asset=scene.assets[subject]
    low=np.full(3,-np.inf); high=np.full(3,np.inf);clauses=[]
    child=scene.bounds[asset]
    clo=child.min_point.numpy()[0].astype(float);chi=child.max_point.numpy()[0].astype(float)
    for relation in asset.relations:
        if not isinstance(relation,(On,NextTo)):continue
        parent=scene.bounds[relation.parent]
        plo=parent.min_point.numpy()[0].astype(float);phi=parent.max_point.numpy()[0].astype(float)
        row=dict(type=type(relation).__name__,parent=relation.parent.name,child_local_bounds=[clo.tolist(),chi.tolist()],parent_world_bounds=[plo.tolist(),phi.tolist()])
        if isinstance(relation,On):
            if relation.overlap:raise ValueError('This source task must retain containment On')
            a=plo[:2]+relation.edge_margin_m-clo[:2];b=phi[:2]-relation.edge_margin_m-chi[:2]
            low[:2]=np.maximum(low[:2],a);high[:2]=np.minimum(high[:2],b)
            zlo=phi[2]-.005-clo[2];zhi=phi[2]+relation.clearance_m+.005-clo[2]
            low[2]=max(low[2],zlo);high[2]=min(high[2],zhi)
            row.update(xy_lower=a.tolist(),xy_upper=b.tolist(),z_lower_exclusive=float(zlo),z_upper_inclusive=float(zhi),
                original_tolerance_m=.005,original_clearance_m=relation.clearance_m,original_edge_margin_m=relation.edge_margin_m,
                original_bottom_gap_m=dict(lower_exclusive=-.005,upper_inclusive=relation.clearance_m+.005))
        else:
            side=str(relation.side.value);axis=0 if side.endswith('x') else 1
            positive=side.startswith('positive')
            edge=phi[axis] if positive else plo[axis]
            target=edge+(relation.distance_m if positive else -relation.distance_m)-(clo[axis] if positive else chi[axis])
            a,b=target-relation.tolerance_m,target+relation.tolerance_m
            if positive:a=max(a,edge-relation.tolerance_m)
            else:b=min(b,edge+relation.tolerance_m)
            low[axis]=max(low[axis],a);high[axis]=min(high[axis],b)
            row.update(side=side,axis=axis,target_origin=float(target),lower=float(a),upper=float(b),
                original_distance_m=relation.distance_m,original_tolerance_m=relation.tolerance_m,
                cross_axis='No validator constraint; cross_position_ratio is a source solver soft preference only')
        clauses.append(row)
    if not np.isfinite([low,high]).all() or np.any(high<=low):raise ValueError('Incomplete or contradictory source domain')
    return dict(subject=subject,origin_xyz_lower=low.tolist(),origin_xyz_upper=high.tolist(),
        lower_inclusive=[True,True,False],upper_inclusive=[True,True,True],clauses=clauses,
        scope='Complete analytical position domain for these original fixed-yaw anchor relations, prior to collision/physics/IK. Actual floating-point validators are additionally run for every grid point.')


def reach_ball(kin):
    segments=[float(np.linalg.norm((kin.chain[i][1]@kin.chain[i+1][0])[:3,3])) for i in range(6)]
    tail=kin.chain[6][1].copy()
    for a,b,_ in kin.chain[7:]:tail=tail@a@b
    tcp=np.eye(4);tcp[:3,3]=[.131,0.,0.];tail=tail@tcp
    return dict(shoulder_local_xyz=kin.chain[0][0][:3,3].tolist(),
        joint_center_segment_lengths_m=segments,last_revolute_joint_to_tcp_m=float(np.linalg.norm(tail[:3,3])),
        radius_upper_bound_m=sum(segments)+float(np.linalg.norm(tail[:3,3])),
        method='Triangle inequality on source joint-frame translations; rotations preserve each segment length. Joint limits and obstacles can only reduce this TCP position set. This is not IK feasibility or a bound on all finger contact points.')


def point_box_distance(point,low,high):
    nearest=np.clip(point,low,high)
    return dict(distance_m=float(np.linalg.norm(point-nearest)),nearest_box_point=nearest.tolist())


def inclusive_grid(low,high,maximum_step,extra):
    # Stay a micrometre inside analytic endpoints: avoids float32 validation
    # rounding, never relaxes the domain. Original endpoint values are declared.
    a,b=low+1e-6,high-1e-6
    values=np.linspace(a,b,int(np.ceil((b-a)/maximum_step))+1)
    return sorted(set(float(x) for x in np.r_[values,extra] if low<x<high))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir',type=Path,required=True)
    args=parser.parse_args()
    if args.output_dir.exists():raise FileExistsError(args.output_dir)
    args.output_dir.mkdir(parents=True)
    config=ROOT/'configs/extensions/replicator/l_island_bowl_sink_source_layout.json'
    original=json.loads(config.read_text());scene=CandidateSearch(original)
    base0=np.asarray(original['robot_position'])+[0,0,original['stand_height']]
    base_domain=source_position_domain(scene,'droid');bowl_domain=source_position_domain(scene,'bowl')
    lo,hi=np.asarray(base_domain['origin_xyz_lower']),np.asarray(base_domain['origin_xyz_upper'])
    xs=inclusive_grid(lo[0],hi[0],.025,[base0[0]])
    ys=inclusive_grid(lo[1],hi[1],.025,[base0[1]])
    zs=[float(lo[2]+1e-6),float(base0[2]),float(hi[2]-1e-6)]
    kin=DroidKinematics(base0,original['robot_yaw_deg']);reach=reach_ball(kin)
    sink=scene.bounds[scene.assets['sink']]
    sinklo,sinkhi=sink.min_point.numpy()[0].astype(float),sink.max_point.numpy()[0].astype(float)
    bowl=scene.bounds[scene.assets['bowl']]
    blo,bhi=np.array(bowl_domain['origin_xyz_lower']),np.array(bowl_domain['origin_xyz_upper'])
    bowlenvlo=blo+bowl.min_point.numpy()[0];bowlenvhi=bhi+bowl.max_point.numpy()[0]
    evidence=code_evidence([Path(__file__),ROOT/'scripts/extensions/replicator/audit_initial_robot_geometry.py',
        ROOT/'scripts/extensions/replicator/search_source_layout_candidates.py',ROOT/'scripts/extensions/replicator/audit_source_layout_geometry.py',
        ROOT/'scripts/extensions/replicator/audit_task_source_relation.py',ROOT/'scripts/extensions/replicator/variant_geometry.py',
        ROOT/'scripts/extensions/replicator/audit_fixed_base.py',ROOT/'kitchen_safety/geometry_audit.py',ROOT/'kitchen_safety/planning.py',
        ROOT/'kitchen_safety/extensions/replicator/collision_planner.py',
        ROOT/'third_party/IsaacLab-Arena/isaaclab_arena/relations/relations.py',
        ROOT/'third_party/IsaacLab-Arena/isaaclab_arena/relations/validation/pre_physics.py',
        ROOT/'third_party/IsaacLab-Arena/isaaclab_arena/relations/relation_loss_strategies.py'])
    declaration=dict(layout_path=str(config),layout_sha256=sha(config),base_domain=base_domain,bowl_domain=bowl_domain,
        search=dict(x_values=xs,y_values=ys,z_values=zs,xy_maximum_step_m=.025,
            z_scope='Lower source On endpoint +1um, original authored floor height, upper source On endpoint -1um. No continuous geometric proof between samples.',
            full_robot_selection='For each left/right counter-end region, up to first20 stand-valid points ranked by max(bowl-envelope distance,sink-box distance), then nominal-Z preference, then original XY displacement.',
            candidate_selection='Preserve first source+conservative initial-robot passing witness per region; independently retain its reach distances without calling it executable.',
            final_object_adjustment='Bowl only +1mm Z for strict initial2mm surface clearance; unchanged XY/yaw/source/relations. No target changes.'),
        source_tcp_reach_ball=reach,required_clearance_m=REQUIRED_M,execution_ready=False,**evidence)
    (args.output_dir/'search_declaration.json').write_text(json.dumps(declaration,indent=2,allow_nan=False)+'\n')
    rows=[];valid=[]
    for z in zs:
        for y in ys:
            for x in xs:
                base=np.array([x,y,z]);delta=base-base0
                row=scene.evaluate('droid',delta)
                row.update(grid_index=len(rows),base_origin_xyz=base.tolist())
                if row['passed']:
                    kin.base[:3,3]=base
                    shoulder=(kin.base@kin.chain[0][0])[:3,3]
                    distances=dict(sink_reference_box=point_box_distance(shoulder,sinklo,sinkhi),
                        all_legal_bowl_center_domain=point_box_distance(shoulder,blo,bhi),
                        conservative_union_of_all_legal_bowl_bounds=point_box_distance(shoulder,bowlenvlo,bowlenvhi))
                    for distance in distances.values():distance['minus_tcp_reach_upper_bound_m']=distance['distance_m']-reach['radius_upper_bound_m']
                    row.update(shoulder_world_xyz=shoulder.tolist(),geometric_distance_lower_bounds=distances)
                    valid.append(row)
                rows.append(row)
        print(json.dumps(dict(z=z,evaluated=len(rows),stand_valid=len(valid))),flush=True)
    # Never discard the failed grid even if later full-robot/geometry code fails.
    (args.output_dir/'stand_grid.json').write_text(json.dumps(dict(declaration_sha256=sha(args.output_dir/'search_declaration.json'),rows=rows,
        sampled=len(rows),stand_valid=len(valid),execution_ready=False),indent=2,allow_nan=False)+'\n')
    values,initial=source_initial_q();q=np.array([values[f'panda_joint{i}'] for i in range(1,8)])
    fingers={name:values[name] for name in CollisionWorld.GRIPPER_JOINT_NAMES}
    world=CollisionWorld(Usd.Stage.CreateInMemory(),DroidKinematics(base0,original['robot_yaw_deg']))
    robot=SourceRobotGeometry(world)
    fixed=fixed_colliders(np.r_[base0,Rotation.from_euler('z',original['robot_yaw_deg'],degrees=True).as_quat()])
    fixed_shapes=TranslatedShapes([(info['path'],shape,*np.asarray(info['bounds'])) for shape,info in fixed],np.zeros((0,3)))
    scene.offsets['bowl']=np.array([0.,0.,.001])
    obstacles=scene.world+scene.shapes['bowl'].at(scene.offsets['bowl'])
    full_rows=[];witnesses=[]
    for side in ('left','right'):
        subset=[row for row in valid if (row['base_origin_xyz'][0]<base0[0])==(side=='left')]
        def rank(row):
            d=row['geometric_distance_lower_bounds'];base=np.array(row['base_origin_xyz'])
            return (max(d['sink_reference_box']['distance_m'],d['conservative_union_of_all_legal_bowl_bounds']['distance_m']),
                    abs(base[2]-base0[2]),float(np.linalg.norm(base[:2]-base0[:2])))
        for candidate in sorted(subset,key=rank)[:20]:
            base=np.array(candidate['base_origin_xyz']);delta=base-base0
            world.kin.base[:3,3]=base
            source_rows=robot.shapes(q,fingers)+scene.shapes['droid'].at(delta)
            proxy=world.shapes(q,gripper_q=fingers)+fixed_shapes.at(delta)
            source_min=minimum_distance(source_rows,obstacles,REQUIRED_M)
            proxy_min=minimum_distance(proxy,obstacles,REQUIRED_M)
            passed=source_min['valid'] and proxy_min['valid']
            row=dict(side=side,grid_index=candidate['grid_index'],base_origin_xyz=base.tolist(),
                source_initial_robot=source_min,conservative_initial_robot=proxy_min,passed=bool(passed),
                source_failures=near_failures(source_rows,obstacles),conservative_failures=near_failures(proxy,obstacles),
                geometric_distance_lower_bounds=candidate['geometric_distance_lower_bounds'])
            full_rows.append(row)
            if passed and not any(w['side']==side for w in witnesses):
                layout=copy.deepcopy(original);layout['robot_position']=(base-[0,0,original['stand_height']]).tolist()
                layout['objects']['bowl']['position'][2]+=.001;layout['execution_ready']=False
                out=args.output_dir/(side+'_initial_geometry_witness.json')
                out.write_text(json.dumps(layout,indent=2)+'\n')
                final=diagnose(out);finalout=args.output_dir/(side+'_initial_geometry_audit.json')
                finalout.write_text(json.dumps(final,indent=2,allow_nan=False)+'\n')
                witnesses.append(dict(side=side,layout_path=str(out),layout_sha256=sha(out),
                    geometry_path=str(finalout),geometry_sha256=sha(finalout),initial_geometry_passed=final['initial_geometry_passed'],
                    full_robot_initial_passed=True,grid_index=candidate['grid_index'],
                    geometric_distance_lower_bounds=candidate['geometric_distance_lower_bounds'],execution_ready=False))
    code_unchanged=all(sha(ROOT/path)==digest for path,digest in evidence['code_sha256'].items())
    report=dict(declaration_path=str(args.output_dir/'search_declaration.json'),declaration_sha256=sha(args.output_dir/'search_declaration.json'),
        stand_grid_path=str(args.output_dir/'stand_grid.json'),stand_grid_sha256=sha(args.output_dir/'stand_grid.json'),
        grid_points=len(rows),source_relation_valid=sum(row['original_relations_passed'] for row in rows),stand_valid=len(valid),
        full_robot_checked=len(full_rows),full_robot_rows=full_rows,witnesses=witnesses,
        initial_joint_source=initial,source_tcp_reach_ball=reach,base_domain=base_domain,bowl_domain=bowl_domain,
        original_source_yaml=scene.entry,source_robot=robot.provenance,
        geometry_layer_sha256={str(Path(layer.realPath)):sha(layer.realPath) for stage in scene.stages for layer in stage.GetUsedLayers() if Path(layer.realPath).is_file()},
        code_unchanged=code_unchanged,source_task_global_impossible=False,
        passed=bool(code_unchanged and any(w['initial_geometry_passed'] for w in witnesses)),execution_ready=False,
        scope='Complete continuous source-relation interval derivation; finite full-domain source-stand sample grid; at most20 closest positions per counter end checked with full source/conservative initial robot. Source mesh clearance witness proves existence only at saved pose. Reach bounds are necessary geometric distances, not IK/path/contact/task success. No global infeasibility conclusion from this finite grid.',**evidence)
    (args.output_dir/'report.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    print(json.dumps(dict(stand_valid=len(valid),full_robot_checked=len(full_rows),witnesses=witnesses,passed=report['passed'],execution_ready=False)),flush=True)
    return int(not report['passed'])


if __name__=='__main__':raise SystemExit(main())
