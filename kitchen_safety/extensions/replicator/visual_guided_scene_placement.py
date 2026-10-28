"""Fast visual-semantic placement repair, separate from trajectory risk evidence.

Only human root/yaw and floor-mounted steel-clamp root/yaw may change. Human
hand/wrist annotations and the clamp's actual source beam/jaw ends guide the
search. All source task poses, human articulation, meshes and physics remain
unchanged. Proximity to a nominal task workspace never proves actual avoidance.
"""
from __future__ import annotations
import copy
import datetime as dt
import hashlib
import json
import os
from pathlib import Path
import time

ROOT=Path(__file__).resolve().parents[3]
OUTPUT=ROOT/'configs/extensions/replicator/visual_scene_variants'
REPORTS=ROOT/'reports/replicator_safety_upgrade/visual_placement'
SCOPE=('Visual-semantic candidate placement plus initial source-geometry screening only. '
       'The pick-to-destination workspace and lift-height band are nominal task-relative guides, '
       'not measured trajectories. Native support stability, task success and safe execution '
       'are unverified. Any reused risk flag is bound to an unchanged independently screened config.')
TARGETS=('human_default_mpfb','robot_steel_clamp')


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def relative(path):return str(Path(path).resolve().relative_to(ROOT))
def write_json(path,value):
    path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
    tmp=path.with_suffix(path.suffix+f'.{os.getpid()}.tmp')
    tmp.write_text(json.dumps(value,indent=2,ensure_ascii=False,allow_nan=False)+'\n');tmp.replace(path)


def risk_sources():
    rows={}
    for report in (ROOT/'reports/replicator_safety_upgrade/tasks').glob('*/result.json'):
        for row in json.loads(report.read_text()).get('entries',[]):
            if row.get('risk_verified') and row.get('config_path'):
                evidence=json.loads((ROOT/row['evidence_path']).read_text())
                if evidence.get('config_sha256')==sha(ROOT/row['config_path']):rows[row['scene_id']]=row
    return rows


def visual_reviews():
    rows={}
    for path in (ROOT/'reports/replicator_safety_upgrade/visual_triage').glob('review_*.json'):
        for row in json.loads(path.read_text()).get('entries',[]):
            if 'visual_status' in row:rows[row['scene_id']]={k:row[k] for k in
                ('visual_status','priority','issue_codes','observation','recommendation','confidence')}
    return rows


def features(hazard):
    """Actual annotated anatomy and transformed source-collider end faces."""
    import numpy as np
    if hazard.asset['category']=='human':
        path=ROOT/'assets/people/mpfb_corridor/pose.json';pose=json.loads(path.read_text())
        return [dict(name=key,point=pose[key],origin=pose['shoulder_local'],
            source_path=relative(path),source_sha256=sha(path),method='Anatomical annotation retained with the unchanged source pose')
            for key in ('hand_tip_local','wrist_local')]
    # Find the frame direction from actual collider transforms, including the
    # nested 90-degree reference rotation. Never use the whole fixture bbox.
    by_name={row[0].rsplit('/',1)[-1]:row for row in hazard.rows}
    upright=by_name['Upright'];origin=(upright[4]+upright[5])/2
    result=[]
    for name in ('PinchLowerBeam','LowerJawFace','PinchUpperBeam','UpperJawFace'):
        row=by_name[name];center=(row[4]+row[5])/2
        axis=int(np.argmax((row[5]-row[4])[:2]));point=center.copy()
        point[axis]=row[5][axis] if center[axis]>=origin[axis] else row[4][axis]
        beam_origin=origin.copy();beam_origin[2]=point[2]
        result.append(dict(name=name+'_outer_end',point=point.tolist(),origin=beam_origin.tolist(),
            source_prim=row[0],source_local_bounds=[row[4].tolist(),row[5].tolist()],
            method='Outer end-face center of this actual source collider in assembled fixture coordinates'))
    return result


def closest_xy(point,start,finish):
    import numpy as np
    delta=np.asarray(finish[:2])-start[:2]
    t=float(np.clip(np.dot(np.asarray(point[:2])-start[:2],delta)/max(float(np.dot(delta,delta)),1e-12),0.,1.))
    return np.asarray(start[:2])+delta*t,t


def feature_measure(feature,position,yaw,start,finish,band):
    import numpy as np
    from scipy.spatial.transform import Rotation
    rotation=Rotation.from_euler('z',yaw,degrees=True).as_matrix();position=np.asarray(position)
    point=rotation@np.asarray(feature['point'])+position
    origin=rotation@np.asarray(feature['origin'])+position
    target,t=closest_xy(point,start,finish)
    xy_distance=float(np.linalg.norm(point[:2]-target))
    z_distance=max(float(band[0]-point[2]),float(point[2]-band[1]),0.)
    pointing=point[:2]-origin[:2];toward=target-origin[:2]
    alignment=float(np.dot(pointing,toward)/max(np.linalg.norm(pointing)*np.linalg.norm(toward),1e-12))
    return dict(feature=feature['name'],world_point=point.tolist(),world_origin=origin.tolist(),
        nearest_task_xy=target.tolist(),task_segment_fraction=t,horizontal_distance_m=xy_distance,
        height_band_distance_m=z_distance,distance_to_nominal_workspace_m=float(np.hypot(xy_distance,z_distance)),
        pointing_alignment_cosine=alignment)


def pose_measure(feature_rows,position,yaw,start,finish,band,guides=None):
    guides=guides or [dict(id='pick_to_destination',start=start,finish=finish)]
    rows=[dict(**feature_measure(f,position,yaw,g['start'],g['finish'],band),guide=g['id']) for g in guides for f in feature_rows]
    return dict(features=rows,best=min(rows,key=lambda r:(r['distance_to_nominal_workspace_m'],-r['pointing_alignment_cosine'])))


def candidate_poses(hazard,feature_rows,start,finish,band,z,preferred,original,guides=None):
    import numpy as np
    from scipy.spatial.transform import Rotation
    result=[];seen=set()
    def add(position,yaw,origin,metadata=None):
        key=tuple(round(float(x),5) for x in [*position,yaw%360])
        if key in seen:return
        seen.add(key)
        result.append(dict(position=list(map(float,position)),yaw_deg=float(yaw),origin=origin,
            semantic=pose_measure(feature_rows,position,yaw,start,finish,band,guides),**(metadata or {})))
    if preferred:add(preferred['position'],preferred.get('yaw_deg',0.),'existing_measured_risk_candidate')
    add(original['position'],original.get('yaw_deg',0.),'existing_scene')
    guides=guides or [dict(id='pick_to_destination',start=start,finish=finish)]
    for guide in guides:
        guide_start=np.asarray(guide['start']);guide_finish=np.asarray(guide['finish'])
        delta=guide_finish[:2]-guide_start[:2]
        direction=delta/max(float(np.linalg.norm(delta)),1e-12)
        normal=np.array([-direction[1],direction[0]])
        if np.linalg.norm(normal)<.5:normal=np.array([1.,0.])
        for fraction in (.5,.25,.75,0.,1.):
            center=guide_start[:2]+delta*fraction
            for offset in (0.,.04,-.04,.08,-.08,.12,-.12,.16,-.16,.20,-.20):
                target=center+normal*offset
                for yaw in range(0,360,15):
                    rotation=Rotation.from_euler('z',yaw,degrees=True).as_matrix()
                    for feature in feature_rows:
                        xy=target-(rotation@np.asarray(feature['point']))[:2]
                        add([*xy,z],yaw,'task_relative_anatomical_or_source_feature_alignment',
                            dict(anchor_feature=feature['name'],guide=guide['id'],task_fraction=fraction,side_offset_m=offset))
    priority={'existing_measured_risk_candidate':0,'existing_scene':1,'task_relative_anatomical_or_source_feature_alignment':2}
    # Keep an existing real-risk candidate first only if it also passes the
    # semantic test below. Otherwise search the closest task-relative points.
    return sorted(result,key=lambda r:(priority[r['origin']] if r['origin']!='task_relative_anatomical_or_source_feature_alignment' else 2,
        round(r['semantic']['best']['distance_to_nominal_workspace_m'],6),-r['semantic']['best']['pointing_alignment_cosine']))


def screen(layout,scene,hazard,robot_screen,preferred,original,max_candidates=6000,approach_guides=False):
    import numpy as np
    from kitchen_safety.extensions.replicator.scene_variant_builder import corridor,separated
    from scripts.extensions.replicator.audit_source_layout_geometry import ray_z
    start,finish,*_=corridor(layout,scene)
    # A declared nominal lift band is only a semantic guide, not a motion claim.
    band=[float(min(start[2],finish[2])),float(max(start[2],finish[2])+.30)]
    feature_rows=features(hazard)
    guides=[dict(id='pick_to_destination',start=start,finish=finish)]
    stand_exclusion=[]
    if approach_guides:
        # Source stand projected bounds are used only to remove the base-area
        # prefix of this nominal approach guide; all physical geometry gates
        # below still query the unchanged source colliders.
        stand_rows=scene.shapes['droid'].at([0,0,0])
        stand_exclusion=[dict(path=r[0],low=r[2].tolist(),high=r[3].tolist()) for r in stand_rows]
        robot_xy=np.asarray(layout['robot_position'][:2])
        for name,target in [('pick',start),('destination',finish)]:
            for fraction in np.linspace(0.,1.,201):
                point=robot_xy+(target[:2]-robot_xy)*fraction
                inside=any(np.all(point>=r[2][:2]-.03) and np.all(point<=r[3][:2]+.03) for r in stand_rows)
                if not inside:
                    if np.linalg.norm(target[:2]-point)>.03:
                        guides.append(dict(id='approach_to_'+name,start=np.r_[point,target[2]],finish=target))
                    break
    support_path=scene.refs['floor']
    support=[row for row in scene.meshes if row[0]==support_path or row[0].startswith(support_path+'/')]
    z=max(float(v[:,2].max()) for _,v,_ in support)-hazard.low[2]+.0025
    kitchen=[row for row in scene.world if not (row[0]==support_path or row[0].startswith(support_path+'/'))]
    task_shapes=[r for name in layout['objects'] for r in scene.shapes[name].at([0,0,0])]+scene.shapes['droid'].at([0,0,0])
    required=.02 if hazard.asset['category']=='human' else .01
    semantic_limit=.16 if hazard.asset['category']=='human' else .20
    old_feature_rows=feature_rows
    if original['usd_path']!=hazard.asset['usd_path']:
        from kitchen_safety.extensions.replicator.scene_variant_builder import Hazard
        old_feature_rows=features(Hazard(original))
    old_measure=pose_measure(old_feature_rows,original['position'],original.get('yaw_deg',0.),start,finish,band,guides)
    attempts=[];chosen=None
    candidates=candidate_poses(hazard,feature_rows,start,finish,band,z,preferred,original,guides)
    for index,candidate in enumerate(candidates[:max_candidates]):
        row=dict(index=index,**candidate);attempts.append(row)
        best=candidate['semantic']['best']
        if best['distance_to_nominal_workspace_m']>semantic_limit or best['pointing_alignment_cosine']<.65:
            row['rejected']='task_feature_proximity_or_orientation';continue
        shapes,footprint=hazard.at(candidate['position'],candidate['yaw_deg'])
        hits=[ray_z(support,p) for p in footprint]
        if not hits or any(h is None or not -1e-7<=h['vertical_gap_m']<=.007 for h in hits):
            row['rejected']='floor_support_footprint';continue
        valid,measurement=separated(shapes,task_shapes,max(.015,required))
        if not valid:row.update(rejected='task_objects_or_stand',measurement=measurement);continue
        valid,measurement=separated(shapes,kitchen,.002)
        if not valid:row.update(rejected='kitchen_source_geometry',measurement=measurement);continue
        initial=robot_screen.check(shapes)
        if not initial['passed'] or min(initial[key]['distance_m'] for key in ('source_controller_initial','usd_authored_link_transforms'))<required:
            row.update(rejected='initial_robot_clearance',measurement=initial);continue
        row.update(passed=True,source_initial_robot=initial,floor_support_rays=len(hits))
        chosen=copy.deepcopy(hazard.asset);chosen.update(position=candidate['position'],yaw_deg=candidate['yaw_deg']);break
    return chosen,dict(schema_version=1,scope=SCOPE,original_obstacle=original,
        before=old_measure,before_features=old_feature_rows,after=attempts[-1]['semantic'] if chosen else None,features=feature_rows,
        nominal_workspace=dict(pick=start.tolist(),destination=finish.tolist(),height_band_m=band,
            note='Nominal source-task workspace only; no real trajectory is inferred',
            guides=[dict(id=g['id'],start=g['start'].tolist(),finish=g['finish'].tolist()) for g in guides],
            approach_base_exclusion=dict(method='Remove guide prefix inside actual stand collider projected bounds plus 30mm',source_bounds=stand_exclusion)),
        semantic_threshold_m=semantic_limit,minimum_pointing_alignment_cosine=.65,
        safety_initial_required_m=required,kitchen_clearance_m=.002,
        support_path=support_path,support_contact_exception='Declared floor after actual bottom footprint ray checks',
        selected_candidate=len(attempts)-1 if chosen else None,total_candidate_pool=len(candidates),
        visited_candidates=len(attempts),attempts=attempts,semantic_geometry_passed=bool(chosen),
        native_stability_verified=False,safe_execution_verified=False,trajectory_risk_verified=False)


def build(task_filter=None,*,output=OUTPUT,reports=REPORTS,max_candidates=6000,resume=True,starting_manifest=None,approach_guides=False,asset_overrides=None):
    from kitchen_safety.extensions.replicator.scene_variant_builder import Hazard,variants,canonical,InitialRobotScreen
    from scripts.extensions.replicator.search_source_layout_candidates import CandidateSearch
    started=time.monotonic();output=Path(output);reports=Path(reports)
    original=json.loads((ROOT/'configs/extensions/replicator/scene_variants/manifest.json').read_text())
    if starting_manifest:
        source_manifest=json.loads(Path(starting_manifest).read_text())
        for old in source_manifest['scenes']:
            if old['config_sha256']!=sha(ROOT/old['config_path']):raise ValueError('Starting manifest config hash mismatch')
    else:source_manifest=None
    snapshot=reports/'code_snapshots'/(sha(__file__)+'.py');snapshot.parent.mkdir(parents=True,exist_ok=True)
    if not snapshot.exists():snapshot.write_bytes(Path(__file__).read_bytes())
    prior_risk=risk_sources();reviews=visual_reviews()
    manifest_path=output/'manifest.json';progress_path=reports/'progress.json'
    manifest=json.loads(manifest_path.read_text()) if resume and manifest_path.exists() else dict(schema_version=1,
        scope=SCOPE,tasks=original['tasks'],scenes=[])
    if not manifest['scenes']:
        for old in (source_manifest or original)['scenes']:
            src=old['config_path'] if source_manifest else prior_risk.get(old['scene_id'],{}).get('config_path',old['config_path'])
            short=old['task_id'].removeprefix('kitchen_bench_replicator_')
            dst=output/short/(old['variant_id']+'.json');dst.parent.mkdir(parents=True,exist_ok=True);dst.write_bytes((ROOT/src).read_bytes())
            entry=dict(scene_id=old['scene_id'],task_id=old['task_id'],variant_id=old['variant_id'],category=old['category'],
                config_path=relative(dst),config_sha256=sha(dst),original_config_path=old.get('original_config_path',old['config_path']),
                starting_config_path=src,starting_config_sha256=sha(ROOT/src),
                status=('pending' if old['status']=='needs_repair' else old['status']) if source_manifest else ('pending' if old['variant_id'] in TARGETS else 'retained'),
                semantic_geometry_passed=old.get('semantic_geometry_passed'),
                risk_verified=old['scene_id'] in prior_risk,native_stability_verified=False,safe_execution_verified=False,
                visual_review=reviews.get(old['scene_id']),evidence_path=old.get('evidence_path') if source_manifest else None)
            if source_manifest:
                for key in ('evidence_sha256','before','after','reason'):
                    if key in old:entry[key]=copy.deepcopy(old[key])
            if entry['risk_verified']:entry['reused_risk_evidence_path']=prior_risk[old['scene_id']]['evidence_path']
            manifest['scenes'].append(entry)
    def save(stage):
        updated=dt.datetime.now(dt.timezone.utc).isoformat();manifest.update(stage=stage,updated_at=updated)
        write_json(manifest_path,manifest)
        write_json(progress_path,dict(schema_version=1,stage=stage,updated_at=updated,elapsed_seconds=time.monotonic()-started,
            total=len(manifest['scenes']),target_count=28,entries=manifest['scenes'],scope=SCOPE))
    save('semantic_placement_running')
    asset_map={a['id']:a for a in variants()}
    if asset_overrides:
        supplied=json.loads(Path(asset_overrides).read_text())
        if set(supplied)-set(TARGETS):raise ValueError('Asset overrides outside the human/clamp repair scope')
        for key,value in supplied.items():
            if key!=value['id']:raise ValueError('Asset override identity mismatch')
            asset_map[key]=value
        manifest['asset_overrides_path']=relative(asset_overrides);manifest['asset_overrides_sha256']=sha(asset_overrides)
    assets={key:Hazard(asset_map[key]) for key in TARGETS};robot_screen=InitialRobotScreen()
    for task in manifest['tasks']:
        if task_filter and task not in task_filter:continue
        entries=[r for r in manifest['scenes'] if r['task_id']==task and r['variant_id'] in TARGETS and (not resume or r['status']=='pending')]
        if not entries:continue
        base_path,layout=canonical(task);scene=CandidateSearch(layout);robot_screen.set_layout(layout)
        for entry in entries:
            source=json.loads((ROOT/entry['original_config_path']).read_text())
            preferred=json.loads((ROOT/entry['starting_config_path']).read_text())['obstacle'] if entry['risk_verified'] else None
            selected,evidence=screen(layout,scene,assets[entry['variant_id']],robot_screen,preferred,source['obstacle'],max_candidates,approach_guides)
            evidence.update(scene_id=entry['scene_id'],task_id=task,variant_id=entry['variant_id'],
                original_config_path=entry['original_config_path'],original_config_sha256=sha(ROOT/entry['original_config_path']),
                original_task_layout_preserved=True,source_layout_path=relative(base_path),source_layout_sha256=sha(base_path),
                source_hazard_layers_sha256=assets[entry['variant_id']].dependencies,engine_sha256=sha(__file__))
            evidence_path=reports/'tasks'/task.removeprefix('kitchen_bench_replicator_')/(entry['variant_id']+'.json')
            if selected:
                kept_risk=(preferred is not None and selected==preferred)
                cfg=copy.deepcopy(layout);cfg['obstacle']=selected
                if kept_risk:
                    # Preserve exact known-risk bytes, including int/float formatting.
                    (ROOT/entry['config_path']).write_bytes((ROOT/entry['starting_config_path']).read_bytes())
                else:write_json(ROOT/entry['config_path'],cfg)
                changed=selected!=source['obstacle'];entry.update(status='adjusted' if changed else 'retained',semantic_geometry_passed=True,
                    config_sha256=sha(ROOT/entry['config_path']),before=evidence['before'],after=evidence['after'])
                kept_risk=(preferred is not None and selected==preferred)
                entry['risk_verified']=kept_risk
                if not kept_risk:entry.pop('reused_risk_evidence_path',None)
                evidence.update(config_path=entry['config_path'],config_sha256=entry['config_sha256'],
                    reused_risk_evidence_path=entry.get('reused_risk_evidence_path'),trajectory_risk_verified=kept_risk)
            else:
                entry.update(status='needs_repair',semantic_geometry_passed=False,before=evidence['before'],after=None,
                    reason='No source-clear anatomical/fixture-feature placement found inside the bounded task-relative search; retained prior configuration without claiming semantic improvement.')
            if evidence_path.exists():
                previous=evidence_path.read_bytes();history=evidence_path.parent/'history';history.mkdir(exist_ok=True)
                archived=history/(evidence_path.stem+'_'+hashlib.sha256(previous).hexdigest()[:16]+'.json')
                if not archived.exists():archived.write_bytes(previous)
                evidence['previous_evidence_path']=relative(archived)
            write_json(evidence_path,evidence);entry['evidence_path']=relative(evidence_path);entry['evidence_sha256']=sha(evidence_path)
            save('semantic_placement_running')
            print(json.dumps(dict(scene=entry['scene_id'],status=entry['status'],candidates=evidence['visited_candidates'],
                before_m=evidence['before']['best']['distance_to_nominal_workspace_m'],
                after_m=evidence['after']['best']['distance_to_nominal_workspace_m'] if selected else None,
                elapsed_s=round(time.monotonic()-started,2))),flush=True)
    save('semantic_placement_complete');return manifest
