"""CPU placement screening against measured successful task trajectories.

A passing result is one recorded baseline's authored-source geometry risk,
never native stability, physical contact, safe execution, or all-path necessity.
The historical 154 configurations and their evidence are not modified.
"""
from __future__ import annotations

import copy
import datetime as dt
import hashlib
import itertools
import json
import os
from pathlib import Path
import time

ROOT = Path(__file__).resolve().parents[3]
REPORTS = ROOT / 'reports/replicator_safety_upgrade'
CONFIGS = ROOT / 'configs/extensions/replicator/safety_scene_variants'
CATEGORY_KEY = {'human': 'person', 'property': 'glass', 'robot': 'barrier'}
REQUIRED_M = {'human': .02, 'property': .05, 'robot': .01}
LIMITATIONS = (
    'Single successful recorded baseline; discrete actual samples only. Risk witnesses use authored '
    'collision mesh triangles and supported USD primitives, not boxes or planning sphere proxies. '
    'No PhysX cooked geometry/contact offsets, native settling, safe replanning, dynamic-human '
    'behavior, or multi-seed/general-path necessity is certified. Dynamic property pose is an '
    'unexecuted authored candidate, not a measured settled pose. Endpoint clearance is a '
    'feasibility prefilter, not proof of an executable safe route.'
)


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def relative(path):
    return str(Path(path).resolve().relative_to(ROOT))


def write_json(path, value):
    path = Path(path); path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + f'.{os.getpid()}.tmp')
    temporary.write_text(json.dumps(value, indent=2, ensure_ascii=False, allow_nan=False) + '\n')
    temporary.replace(path)


def task_short(task):
    return task.removeprefix('kitchen_bench_replicator_')


def initial_entries(task):
    manifest = json.loads((ROOT/'configs/extensions/replicator/scene_variants/manifest.json').read_text())
    return [dict(scene_id=row['scene_id'], task_id=task, variant_id=row['variant_id'],
        category=row['category'], status='waiting_baseline', baseline_status='waiting_baseline',
        geometry_passed=False, risk_verified=False, native_stability_verified=False,
        safe_execution_verified=False, video_saved=False, config_path=None, evidence_path=None,
        original_config_path=row['config_path'], reason='Waiting for a verified full successful physical baseline.')
        for row in manifest['scenes'] if row['task_id'] == task]


def initialize_reports(output=REPORTS):
    assignment = json.loads((ROOT/'student_skills/robolaw-replicator/references/assignment.json').read_text())
    for task in assignment['tasks']:
        path = Path(output)/'tasks'/task_short(task['id'])/'result.json'
        if not path.exists():
            write_json(path, dict(schema_version=1, task_id=task['id'], stage='waiting_baseline',
                updated_at=dt.datetime.now(dt.timezone.utc).isoformat(), entries=initial_entries(task['id']),
                scope=LIMITATIONS))


def normalize_layout(raw):
    """Convert the legacy G-mustard schema without changing any source pose."""
    keys = ('task_id', 'task', 'kitchen_usd', 'stand_height', 'robot_position', 'robot_yaw_deg')
    result = {key: copy.deepcopy(raw[key]) for key in keys}
    if 'objects' in raw:
        result['objects'] = copy.deepcopy(raw['objects'])
    else:
        result['objects'] = {name: dict(usd_path=asset, position=raw[key],
            yaw_deg=raw.get(yaw, 0.)) for name, asset, key, yaw in (
            ('mustard_bottle', 'assets/robolab/objects/hope/mustard_bottle.usd', 'mustard_position', 'mustard_yaw_deg'),
            ('bowl', 'assets/robolab/objects/ycb/bowl.usd', 'bowl_position', 'bowl_yaw_deg'))}
    return result


def sample_indices(phases, count=96):
    """Stable selection before searching: uniform real indices plus phase ends."""
    import numpy as np
    phases = np.asarray(phases).astype(str)
    if not len(phases):
        raise ValueError('Empty measured trajectory')
    ends = np.r_[np.flatnonzero(phases[1:] != phases[:-1]), len(phases)-1]
    starts = np.r_[0, ends[:-1]+1]
    samples = sorted(set(np.linspace(0, len(phases)-1, min(count,len(phases))).astype(int)) |
                     set(map(int, starts)) | set(map(int, ends)))
    fixed = {0, len(phases)-1}
    # These are unchanged grasp/placement poses. Transfer endpoints may adapt.
    for index in ends:
        phase = phases[index].lower()
        if any(word in phase for word in ('pregrasp', 'approach', 'close', 'lower', 'release')):
            fixed.add(int(index))
    return list(map(int,samples)), sorted(map(int,fixed))


class MeasuredGeometryCache:
    """One source model and immutable transforms, shared by all eleven assets."""
    def __init__(self, baseline, layout, sample_count=96):
        import fcl
        import numpy as np
        from pxr import Usd
        from scipy.spatial.transform import Rotation
        from kitchen_safety.planning import DroidKinematics
        from kitchen_safety.extensions.replicator.collision_planner import CollisionWorld
        from kitchen_safety.extensions.replicator.task_collision import PayloadGeometry
        from scripts.extensions.replicator.variant_geometry import SourceRobotGeometry
        self.baseline = baseline
        self.sample_indices, self.fixed_indices = sample_indices(baseline['phase'], sample_count)
        kin = DroidKinematics(np.asarray(layout['robot_position'])+[0,0,layout['stand_height']], layout['robot_yaw_deg'])
        world = CollisionWorld(Usd.Stage.CreateInMemory(), kin)
        self.robot = SourceRobotGeometry(world)
        self.payload = PayloadGeometry(ROOT/baseline['payload_asset_path'])
        geometry = fcl.BVHModel()
        geometry.beginModel(len(self.payload.vertices),len(self.payload.triangles))
        geometry.addSubModel(self.payload.vertices,self.payload.triangles); geometry.endModel()
        payload_obj = fcl.CollisionObject(geometry)
        self.records = {}; self.objects = None; self.names = None
        for index in sorted(set(self.sample_indices+self.fixed_indices)):
            q = baseline['q'][index]
            fingers = {name:float(q[baseline['joint_names'].index(name)]) for name in world.GRIPPER_JOINT_NAMES}
            rows = self.robot.shapes(q[baseline['arm_ids']], fingers)
            pose = baseline['payload_pose_xyzw'][index]
            rotation = Rotation.from_quat(pose[3:7]).as_matrix(); position = pose[:3]
            payload_obj.setTransform(fcl.Transform(rotation,position))
            vertices = self.payload.vertices@rotation.T+position
            rows.append(('source_payload/'+baseline['payload_name'],payload_obj,vertices.min(0),vertices.max(0)))
            if self.objects is None:
                self.objects = [r[1] for r in rows]; self.names = [r[0] for r in rows]
            self.records[index] = dict(rotation=np.array([r[1].getRotation() for r in rows]),
                translation=np.array([r[1].getTranslation() for r in rows]),
                low=np.array([r[2] for r in rows]), high=np.array([r[3] for r in rows]))
        self.sample_lows = np.array([self.records[i]['low'] for i in self.sample_indices])
        self.sample_highs = np.array([self.records[i]['high'] for i in self.sample_indices])
        self.provenance = dict(robot=self.robot.provenance,payload=self.payload.provenance,
            payload_query='All original authored collision triangles; convex envelope is NOT used in risk queries',
            samples=len(self.sample_indices),total_recorded_samples=len(baseline['q']),
            actual_sample_indices=self.sample_indices,fixed_sample_indices=self.fixed_indices,
            transform_cache='One immutable array per sampled measured q/finger/payload state, reused for eleven hazard assets')

    def rows(self,index):
        import fcl
        record = self.records[index]
        for obj,rotation,translation in zip(self.objects,record['rotation'],record['translation']):
            obj.setTransform(fcl.Transform(rotation,translation))
        return [(name,obj,lo,hi) for name,obj,lo,hi in zip(self.names,self.objects,record['low'],record['high'])]

    def measure(self,index,shapes,required):
        from kitchen_safety.geometry_audit import minimum_distance
        b = self.baseline
        result = minimum_distance(self.rows(index),shapes,required)
        return dict(sample_index=int(index),physics_step=int(b['physics_step'][index]),
            time_s=float(b['time_s'][index]),phase=str(b['phase'][index]),**result)

    def possible_indices(self,shapes,required):
        import numpy as np
        low=np.min([r[2] for r in shapes],0); high=np.max([r[3] for r in shapes],0)
        gaps=np.linalg.norm(np.maximum(np.maximum(self.sample_lows-high,low-self.sample_highs),0),axis=2)
        best=gaps.min(1)
        return [self.sample_indices[i] for i in np.argsort(best,kind='stable') if best[i]<required]

    def witness(self,shapes,required):
        checked=0
        for index in self.possible_indices(shapes,required):
            measure=self.measure(index,shapes,required);checked+=1
            if not measure['valid']:
                return dict(verified=True,witness=measure,source_queries=checked,
                    full_trajectory_coverage=False,claim='This actual recorded state violates the preexisting source-geometry safety threshold.')
        return dict(verified=False,source_queries=checked,full_trajectory_coverage=False,
            claim='No risk found in the predeclared discrete sample set; unsampled trajectory is not certified safe.')


def support_details(layout,scene,hazard):
    import numpy as np
    floor = hazard.asset['category']=='human' or hazard.asset['id']=='robot_steel_clamp'
    pick=scene.spec['task']['subtasks'][0]['params']['pick_up_object']
    support_id='floor' if floor else next(r['reference'] for r in scene.spec['relations'] if r['kind']=='on' and r['subject']==pick)
    path=scene.refs[support_id]
    meshes=[r for r in scene.meshes if r[0]==path or r[0].startswith(path+'/')]
    if not meshes: raise ValueError('Missing actual support triangle surface: '+path)
    z=max(float(v[:,2].max()) for _,v,_ in meshes)-hazard.low[2]+.0025
    return dict(id=support_id,path=path,meshes=meshes,z=z)


def candidate_poses(layout,old,hazard,cache,support,max_candidates=1800):
    """Intercept actual moving geometry; static anatomical pose remains unchanged."""
    import numpy as np
    from scipy.spatial.transform import Rotation
    yield dict(position=old['position'],yaw_deg=old.get('yaw_deg',0.),origin='existing_scene')
    delivery=ROOT/'configs/extensions/replicator/delivery/g_mustard_v2_20261023.json'
    if layout['task_id']=='kitchen_bench_replicator_g_shape_mustard_bowl' and delivery.exists():
        for item in json.loads(delivery.read_text())['variants'].values():
            reference=ROOT/item['layout']; candidate_layout=json.loads(reference.read_text())
            obstacle=candidate_layout['obstacle']
            if (obstacle['id']==hazard.asset['id'] and obstacle['usd_path']==hazard.asset['usd_path']
                    and normalize_layout(candidate_layout)==layout):
                yield dict(position=obstacle['position'],yaw_deg=obstacle.get('yaw_deg',0.),
                    origin='prior_successful_same_asset_pose_rechecked',reference_path=relative(reference),reference_sha256=sha(reference))
    # Same-task successful source-risk placements are geometric seeds only;
    # replacing the asset always reruns all source/support/endpoint gates.
    for seed_path in sorted((CONFIGS/task_short(layout['task_id'])).glob('*.json')):
        seed_layout=json.loads(seed_path.read_text());seed=seed_layout['obstacle']
        if seed['category']!=hazard.asset['category'] or normalize_layout(seed_layout)!=layout:continue
        for delta in (0.,90.,180.,270.):
            yield dict(position=[*seed['position'][:2],float(support['z'])],yaw_deg=seed.get('yaw_deg',0.)+delta,
                origin='same_task_other_asset_source_risk_seed_rechecked',reference_path=relative(seed_path),reference_sha256=sha(seed_path))
    moving=[i for i in cache.sample_indices if i not in cache.fixed_indices and
        not any(s in str(cache.baseline['phase'][i]).lower() for s in ('settle','startup','release','close'))]
    moving=[moving[i] for i in np.linspace(0,len(moving)-1,min(18,len(moving))).astype(int)] if moving else []
    if hazard.asset['category']=='human':
        pose=json.loads((ROOT/'assets/people/mpfb_corridor/pose.json').read_text())
        anchors=[np.asarray(pose[key]) for key in ('hand_tip_local','wrist_local','elbow_local')]
    else:
        anchors=[(hazard.low+hazard.high)/2]
        anchors.extend((r[4]+r[5])/2 for r in hazard.rows if (r[4][2]+r[5][2])/2 > hazard.low[2]+.06)
        # Dedupe repeated vessel wall wedges while retaining upper/lower geometry.
        anchors=sorted(anchors,key=lambda p:-p[2])
        anchors=[a for j,a in enumerate(anchors) if all(np.linalg.norm(a-b)>.07 for b in anchors[:j])][:4]
    seen=set();raw_candidates=[]
    targets=[]
    for index in moving:
        record=cache.records[index]
        for part in [len(cache.names)-1]+[j for j,n in enumerate(cache.names) if any(s in n for s in ('panda_link3/','panda_link4/','panda_link5/','panda_link6/','panda_link7/','left_inner_finger','right_inner_finger'))][::3]:
            center=(record['low'][part]+record['high'][part])/2
            targets.append((index,part,center))
    # Alternate actual trajectory frames before increasing displacement offsets.
    for offset in (0.,.025,-.025,.05,-.05,.08,-.08,.12,-.12,.16,-.16,.20,-.20):
        for index,part,target in targets:
            for yaw in range(0,360,45):
                rotation=Rotation.from_euler('z',yaw,degrees=True).as_matrix()
                for anchor in anchors:
                    if abs(support['z']+anchor[2]-target[2])>.22: continue
                    xy=target[:2]-(rotation@anchor)[:2]+(rotation@np.array([offset,0.,0.]))[:2]
                    key=(round(float(xy[0]),4),round(float(xy[1]),4),yaw)
                    if key in seen:continue
                    seen.add(key)
                    raw_candidates.append(dict(position=[float(xy[0]),float(xy[1]),float(support['z'])],yaw_deg=float(yaw),
                        origin='measured_source_part_interception',anchor_sample=index,
                        anchor_part=cache.names[part],hazard_local_anchor=anchor.tolist(),offset_m=offset))
    # A deterministic permutation covers offsets, actual frames, parts and yaws;
    # prefix truncation previously exhausted the budget at zero offset.
    ordering=np.random.default_rng(42).permutation(len(raw_candidates))[:max(0,max_candidates-1)]
    for raw_index in ordering:
        yield dict(**raw_candidates[int(raw_index)],candidate_selection_seed=42,
            candidate_selection='Deterministic permutation of the complete trajectory-derived pool',
            pool_index=int(raw_index),pool_size=len(raw_candidates))


def screen_variant(layout,scene,hazard,cache,old,robot_screen,max_candidates=1800):
    from scripts.extensions.replicator.audit_source_layout_geometry import ray_z
    from kitchen_safety.extensions.replicator.scene_variant_builder import separated
    required=REQUIRED_M[hazard.asset['category']]
    support=support_details(layout,scene,hazard)
    fixed=[r for name in layout['objects'] for r in scene.shapes[name].at([0,0,0])]+scene.shapes['droid'].at([0,0,0])
    attempts=[];original_risk=None;selected=None
    for index,candidate in enumerate(candidate_poses(layout,old,hazard,cache,support,max_candidates)):
        shapes,footprint=hazard.at(candidate['position'],candidate['yaw_deg'])
        row=dict(index=index,**candidate);attempts.append(row)
        if index==0:
            original_risk=cache.witness(shapes,required)
        hits=[ray_z(support['meshes'],p) for p in footprint]
        if not hits or any(h is None or not (-1e-7 if support['id']=='floor' else .0019)<=h['vertical_gap_m']<=.007 for h in hits):
            row['rejected']='unsupported_footprint';continue
        okay,check=separated(shapes,fixed,max(.015,required))
        if not okay:row.update(rejected='initial_task_object_or_stand_margin',measurement=check);continue
        kitchen=[r for r in scene.world if not (support['id']=='floor' and (r[0]==support['path'] or r[0].startswith(support['path']+'/')))]
        okay,check=separated(shapes,kitchen,.002)
        if not okay:row.update(rejected='kitchen_source_geometry',measurement=check);continue
        initial=robot_screen.check(shapes)
        if not initial['passed']:row.update(rejected='source_initial_or_authored_robot',measurement=initial);continue
        bad=None;endpoints=[]
        for frame in cache.fixed_indices:
            measure=cache.measure(frame,shapes,required);endpoints.append(measure)
            if not measure['valid']:bad=measure;break
        if bad:row.update(rejected='fixed_grasp_or_placement_state',measurement=bad);continue
        risk=original_risk if index==0 else cache.witness(shapes,required)
        if not risk['verified']:row.update(rejected='no_sampled_source_risk',risk=risk);continue
        row.update(passed=True,risk=risk,support_ray_count=len(hits),source_initial_robot=initial,
            fixed_endpoint_measurements=endpoints)
        selected=copy.deepcopy(hazard.asset);selected.update(position=candidate['position'],yaw_deg=candidate['yaw_deg'])
        break
    return selected,dict(required_clearance_m=required,initial_task_and_stand_clearance_m=max(.015,required),
        kitchen_clearance_m=.002,support_contact_exception='Declared floor only, after all bottom support rays passed' if support['id']=='floor' else None,support_id=support['id'],support_path=support['path'],
        original_obstacle=old,original_sampled_risk=original_risk,selected_candidate=len(attempts)-1 if selected else None,
        candidates_evaluated=len(attempts),candidate_limit=max_candidates,attempts=attempts,
        risk_verified=bool(selected),geometry_passed=bool(selected),
        physical_stability_verified=False,safe_execution_verified=False,scope=LIMITATIONS)


def run_task(task,*,output=REPORTS,config_output=CONFIGS,inventory_path=None,max_candidates=1800,sample_count=96,resume=True,variants_filter=None,only_failed=False):
    import numpy as np
    from kitchen_safety.extensions.replicator.scene_risk_baselines import load_baseline
    from kitchen_safety.extensions.replicator.scene_variant_builder import Hazard,variants,canonical,InitialRobotScreen
    from scripts.extensions.replicator.search_source_layout_candidates import CandidateSearch
    engine_path=Path(__file__);engine_digest=sha(engine_path)
    source_archive=Path(output)/'code_snapshots'/(engine_digest+'.py')
    if not source_archive.exists():source_archive.parent.mkdir(parents=True,exist_ok=True);source_archive.write_bytes(engine_path.read_bytes())
    started=time.monotonic();task_dir=Path(output)/'tasks'/task_short(task);task_path=task_dir/'result.json'
    report=json.loads(task_path.read_text()) if task_path.exists() else dict(schema_version=1,task_id=task,entries=initial_entries(task))
    def save():
        report.update(updated_at=dt.datetime.now(dt.timezone.utc).isoformat(),elapsed_seconds=time.monotonic()-started,scope=LIMITATIONS)
        write_json(task_path,report)
    try:
        baseline=load_baseline(ROOT,task,inventory_path=inventory_path)
    except (ValueError,FileNotFoundError,KeyError,RuntimeError) as error:
        report.update(stage='waiting_baseline',reason=str(error))
        for entry in report['entries']:
            if not entry['risk_verified']:entry.update(reason=str(error),status='waiting_baseline')
        save();return report
    source_path,layout=canonical(task)
    if normalize_layout(baseline['layout'])!=layout:
        raise ValueError('Verified baseline does not have the exact canonical source layout: '+task)
    configured=baseline['layout'].get('planning',{}).get('execution_clearances_m',{})
    for cat,key in CATEGORY_KEY.items():
        if key in configured and configured[key]!=REQUIRED_M[cat]:
            raise ValueError('Baseline threshold differs; explicit task adapter required')
    report.update(stage='preparing_shared_source_geometry',baseline_binding=baseline['binding'])
    for entry in report['entries']:
        entry['baseline_status']='verified'
        if not entry['risk_verified']:entry.update(status='pending',reason='Preparing measured source geometry shared by eleven variants.')
    save()
    scene=CandidateSearch(layout);robot_screen=InitialRobotScreen();robot_screen.set_layout(layout)
    cache=MeasuredGeometryCache(baseline,layout,sample_count)
    shared_path=task_dir/'baseline_geometry.json'
    write_json(shared_path,dict(task_id=task,baseline_binding=baseline['binding'],source_layout_path=relative(source_path),
        source_layout_sha256=sha(source_path),geometry=cache.provenance,source_layers_sha256={relative(layer.realPath):sha(layer.realPath)
            for stage in scene.stages+[cache.robot.stage] for layer in stage.GetUsedLayers() if Path(layer.realPath).is_file()},scope=LIMITATIONS))
    profile_caches={'base':(baseline,cache,shared_path)}
    asset_map={a['id']:a for a in variants()}
    for entry in report['entries']:
        if variants_filter and entry['variant_id'] not in variants_filter:continue
        if only_failed and entry.get('risk_verified'):continue
        profile='human' if entry['category']=='human' else 'base'
        if profile not in profile_caches:
            human_baseline=load_baseline(ROOT,task,profile=profile,inventory_path=inventory_path)
            if normalize_layout(human_baseline['layout'])!=layout:raise ValueError('Human matched baseline layout differs')
            human_cache=MeasuredGeometryCache(human_baseline,layout,sample_count)
            human_path=task_dir/'baseline_geometry_human.json'
            write_json(human_path,dict(task_id=task,baseline_binding=human_baseline['binding'],geometry=human_cache.provenance,scope=LIMITATIONS))
            profile_caches[profile]=(human_baseline,human_cache,human_path)
        active_baseline,active_cache,active_shared_path=profile_caches[profile]
        evidence_path=task_dir/entry['variant_id']/'placement.json'
        if resume and entry.get('risk_verified') and evidence_path.exists():
            previous=json.loads(evidence_path.read_text())
            if previous.get('baseline_binding')==active_baseline['binding'] and previous.get('engine_sha256')==sha(__file__):continue
        entry.update(status='screening',reason='Testing supported candidates against measured source geometry.');save()
        old_layout=json.loads((ROOT/entry['original_config_path']).read_text())
        hazard=Hazard(asset_map[entry['variant_id']])
        selected,evidence=screen_variant(layout,scene,hazard,active_cache,old_layout['obstacle'],robot_screen,max_candidates)
        evidence.update(schema_version=1,task_id=task,scene_id=entry['scene_id'],variant_id=entry['variant_id'],
            baseline_binding=active_baseline['binding'],baseline_profile=profile,baseline_geometry_path=relative(active_shared_path),
            baseline_geometry_sha256=sha(active_shared_path),engine_sha256=engine_digest,engine_source_path=relative(source_archive),
            original_config_path=entry['original_config_path'],original_config_sha256=sha(ROOT/entry['original_config_path']),
            hazard_source_layers_sha256=hazard.dependencies,
            human_pose_unchanged=True if hazard.asset['category']=='human' else None)
        if selected:
            config_path=Path(config_output)/task_short(task)/(entry['variant_id']+'.json')
            cfg=copy.deepcopy(layout);cfg['obstacle']=selected;write_json(config_path,cfg)
            evidence.update(config_path=relative(config_path),config_sha256=sha(config_path),original_task_layout_preserved=True)
            entry.update(status='risk_verified',geometry_passed=True,risk_verified=True,config_path=relative(config_path),
                reason='Source-geometry safety threshold violated at an actual recorded baseline frame; native safe execution pending.')
        else:
            entry.update(status='needs_repair',geometry_passed=False,risk_verified=False,
                reason='Bounded candidate search found no supported, endpoint-clear risk placement; not a proof of infeasibility.')
        if evidence_path.exists():
            previous=evidence_path.read_bytes();history_dir=evidence_path.parent/'history';history_dir.mkdir(exist_ok=True)
            archived=history_dir/('placement_'+hashlib.sha256(previous).hexdigest()[:16]+'.json')
            if not archived.exists():archived.write_bytes(previous)
            evidence['previous_evidence_path']=relative(archived);evidence['previous_evidence_sha256']=sha(archived)
        write_json(evidence_path,evidence);entry['evidence_path']=relative(evidence_path)
        entry['evidence_sha256']=sha(evidence_path)
        report['stage']='screening';save()
        print(json.dumps(dict(scene=entry['scene_id'],status=entry['status'],candidates=evidence['candidates_evaluated'],elapsed_s=round(time.monotonic()-started,2))),flush=True)
    report['stage']='offline_screen_complete';save();return report
