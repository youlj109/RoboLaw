"""Offline HS/PT/RS reconstruction from immutable raw capture; no simulator.

Geometry uses frozen full meshes and source-time poses. Temporal contact,
grip, fault and drop evidence is replayed at the original physics cadence.
Processing timestamps are explicitly separate from simulated decision times.
"""
import argparse
from collections import Counter, deque
from copy import deepcopy
import csv
import hashlib
import json
import os
import sys
from pathlib import Path
from types import SimpleNamespace
import time

EXECUTED_SOURCE_SHA256 = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
START = time.perf_counter()
import h5py
import numpy as np
from kitchen_safety.risk_binding import ArenaRiskRuntime
from kitchen_safety.risk_observer import RiskObserver, finite_json, write_json
from kitchen_safety.risk_profile_loader import create_evaluator, digest
from kitchen_safety.grasp_evidence import GraspEvidence
from kitchen_safety.rs_evidence import FaultEvidence
from kitchen_safety.frozen_geometry import load_geometry, run_frame
from kitchen_safety.geometry_backend import get_backend,configured_name,descriptor,migrate_frozen
from scripts.export_offline_risk_features import FIELDS, project


def aggregate_contacts(packet, dt):
    pairs = {}
    for record in packet['contacts']:
        key = tuple(sorted((record['actor0'], record['actor1'])))
        value = pairs.setdefault(key, dict(bodyA=key[0], bodyB=key[1],
            impulse_vector_ns=np.zeros(3), contacts=[]))
        sign = 1 if record['actor0'] == key[0] else -1
        value['impulse_vector_ns'] += sign * np.sum(
            [p['impulse_vector_ns'] for p in record['points']], axis=0)
        value['contacts'].append(record)
    for value in pairs.values():
        impulse = float(np.linalg.norm(value['impulse_vector_ns']))
        value.update(impulse_vector_ns=value['impulse_vector_ns'].tolist(),
                     impulse_ns=impulse, force_n=impulse/dt)
    return dict(coverage=packet['contact_coverage'], pairs=list(pairs.values()),
        errors=packet['contact_errors'], source='PhysX contact report impulse vectors',
        statistic='body_pair_step_vector_norm', dt_s=dt)


class OfflineObserver(RiskObserver):
    """Reuse the evaluator without constructing fictitious online audit streams."""
    def __init__(self, output, frozen):
        self.directory = output
        self.settings = dict(config=frozen['resolved_config'])
        self.binding = frozen['binding']
        self.identity = frozen['identity']
        self.profile = frozen['resolved_profile']
        if digest(self.profile) != frozen['profile_sha256']:
            raise ValueError('Frozen profile hash mismatch')
        self.dt = frozen['physics_dt_s']
        self.stride = frozen['decision_stride']
        self.disabled_categories = frozen['resolved_config'].get('disabled_categories', [])
        self.evaluator = create_evaluator(self.profile)
        self.records, self.steps, self.errors = [], 0, []
        self.timings = Counter()
        self.decisions = (output/'safety_decisions.jsonl').open('x', buffering=1024*1024)
        self.counts = {c: Counter() for c in ('HS', 'PT', 'RS')}
        self.complete = Counter()
        self.missing = {c: Counter() for c in self.counts}
        self.geometry_complete = Counter()
        self.geometry_not_applicable = Counter()
        self.csvfile = (output/'risk_features.csv').open('x', newline='')
        self.csv = csv.DictWriter(self.csvfile, fieldnames=FIELDS)
        self.csv.writeheader()

    def _line(self, stream, record):
        # Base evaluator is reused for rule identity. Its wall clocks describe
        # OFFLINE processing here, never a live before-submission evaluation.
        for old, new in (
            ('decision_wall_time_ns', 'processing_start_wall_ns'),
            ('evaluation_complete_time_ns', 'processing_complete_wall_ns'),
            ('decision_monotonic_ns', 'processing_start_monotonic_ns'),
            ('evaluation_complete_monotonic_ns', 'processing_complete_monotonic_ns')):
            record[new] = record.pop(old)
        record.update(evaluation_mode='offline_replay',
            source_capture_receive_wall_ns=self.source_packet['receive_wall_ns'],
            source_state_semantics='Initial state or last completed physics step; no future dynamic evidence',
            source_raw_dataset=str(self.source_directory))
        record['input']['metadata']['evaluation_mode'] = 'offline_replay'
        record['input']['metadata']['mesh_distance_backend'] = configured_name(self.settings['config'])
        record['input_sha256'] = digest(record['input'])
        f, report = record['input']['features'], record['report']
        h, p, r = (f[x] for x in ('hs', 'pt', 'rs'))
        for cat in self.counts:
            self.counts[cat][str(report['risk_levels'].get(cat))] += 1
            self.complete[cat] += report['categories'][cat].get('data_complete') is True
            self.missing[cat].update(report['categories'][cat].get('missing', []))
        if self.binding['human_inventory'].get('verified_absent') is True:
            self.geometry_not_applicable['HS'] += 1
        else:
            self.geometry_complete['HS'] += h.get('distance_evidence',{}).get('complete') is True
        self.geometry_complete['PT'] += all(x.get('distance_coverage_complete') is True for x in p['protected_pairs'])
        self.geometry_complete['RS'] += r['self_collision_evidence'].get('coverage_complete') is True
        self.csv.writerow(project(record))
        super()._line(stream, record)
        # Keep the observation count without retaining a gigabyte of reports.
        self.records[-1] = None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--episode', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--geometry-backend',choices=['frozen','gdist','warp'],default='frozen',help='Default preserves frozen backend; explicit override writes a derived binding/profile without modifying source')
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    original_frozen = json.loads((args.episode/'safety_observer_binding.json').read_text())
    if digest(original_frozen['resolved_profile'])!=original_frozen['profile_sha256']:raise ValueError('Source frozen profile hash mismatch')
    frozen = original_frozen if args.geometry_backend=='frozen' else migrate_frozen(original_frozen,args.geometry_backend)
    write_json(args.output/'safety_observer_binding.json',frozen)
    times = Counter(imports_s=time.perf_counter()-START)
    tick = time.perf_counter()
    b, hs, pt, rs = load_geometry(args.episode, args.episode/'raw_meshes',frozen=frozen)
    if hasattr(pt.fcl,'ensure_available'):pt.fcl.ensure_available()
    observer = OfflineObserver(args.output, frozen)
    observer.source_directory = args.episode.resolve()
    runtime = ArenaRiskRuntime.__new__(ArenaRiskRuntime)
    runtime.dt = frozen['physics_dt_s']
    runtime.settings = dict(config=frozen['resolved_config'])
    runtime.robot = SimpleNamespace(body_names=b['body_names'], joint_names=b['joint_names'])
    runtime.latest = [{}]; runtime.distances = [{}]; runtime.observers = [observer]
    runtime.windows = [deque(maxlen=observer.stride)]
    runtime.grasps = [GraspEvidence(frozen['resolved_config']['grasp_evidence'])]
    runtime.faults = [FaultEvidence(frozen['resolved_config']['rs_evidence']['fault_motion'])]
    runtime.sample_steps = [-1]; runtime.highest_bottle_z = [None]
    runtime.approach_speed = [None]; runtime.motion_approach_speed = [None]
    runtime.pt_inventories = [b['pt_asset_inventory']]
    runtime.geometries = [SimpleNamespace(root=pt.root)]; runtime.hs_geometries = [hs]
    # HDF5 is part of this immutable raw dataset. It contains measured kitchen
    # link poses, not simulator replay/prediction. Use source time, never future.
    links = json.loads((args.episode/'render_state_metadata.json').read_text())['kitchen']['link_paths']
    with h5py.File(args.episode/'trajectory.h5', 'r') as f:
        kitchen = np.asarray(f['obs/kitchen_link_pose'])
        final_kitchen = np.asarray(f['next_obs/kitchen_link_pose'][-1])
        legacy_hazard = np.asarray(f['obs/hazard_state']) if 'obs/hazard_state' in f else None
        final_hazard = np.asarray(f['next_obs/hazard_state'][-1]) if 'next_obs/hazard_state' in f else None
    kitchen[..., :3] += np.asarray(b['environment_origin_world_m'])
    final_kitchen[..., :3] += np.asarray(b['environment_origin_world_m'])
    times['initialization_s'] += time.perf_counter()-tick
    previous = motion_previous = None
    last_geometry_step = -1
    counts = Counter()
    source_hash = hashlib.sha256()
    started_loop = time.perf_counter()

    def geometry(row, step, initial=False):
        nonlocal previous, motion_previous, last_geometry_step
        t = time.perf_counter()
        frame = (step+1)//observer.stride
        kp = kitchen[frame] if frame < len(kitchen) else final_kitchen
        poses = dict(zip(b['body_names'], np.asarray(row['body_pose'])))
        objects = dict(zip(links, kp))
        objects.update(mustard_bottle=np.asarray(row['mustard']), bowl=np.asarray(row['bowl']))
        # New archives have every measured rigid actor at the physical step.
        # Older archives can supply the source-time dynamic hazard observation.
        actors=row.get('rigid_actor_states_world',{})
        for path,state in actors.items():
            if path.startswith(pt.root+'/'):objects[path[len(pt.root)+1:]]=np.asarray(state)
        if not actors and legacy_hazard is not None:
            hazard=legacy_hazard[frame].copy() if frame<len(legacy_hazard) else final_hazard.copy()
            hazard[:3]+=np.asarray(b['environment_origin_world_m'])
            objects['glass']=hazard
        h, p, r = run_frame(hs, pt, rs, poses, objects)
        sample_dt = runtime.dt if initial else (step-last_geometry_step)*runtime.dt
        if h is not None:
            values = list(h['features'].values())
            current = min(values) if all(v is not None for v in values) else None
            mv = [h['scoped_features'].get('d_moving_h_min_gt_m'),
                  h['features'].get('d_ee_h_min_gt_m'), h['features'].get('d_obj_h_min_gt_m')]
            motion = min(mv) if all(v is not None for v in mv) else None
            runtime.approach_speed[0] = max(0., (previous-current)/sample_dt) if current is not None and previous is not None else None
            runtime.motion_approach_speed[0] = max(0., (motion_previous-motion)/sample_dt) if motion is not None and motion_previous is not None else None
            previous, motion_previous, last_geometry_step = current, motion, step
        else:
            last_geometry_step=step
        runtime.distances[0] = dict(rs_fault=runtime.distances[0].get('rs_fault'),
            geometry_sample_physics_step=step, geometry_fresh=True,
            geometry_hz=frozen['resolved_config']['decision_hz'], pt_v4=p,
            payload_geometry_center_world_m=p.get('payload_geometry_center_world_m'),
            rs_self_geometry=r)
        if h is not None:runtime.distances[0]['hs_geometry']=h
        times['geometry_s'] += time.perf_counter()-t
        counts['geometry_samples'] += 1

    with (args.episode/'raw_evidence.jsonl').open('rb') as source:
        for line in source:
            tick = time.perf_counter()
            source_hash.update(line)
            packet = json.loads(line)
            times['raw_decode_hash_s'] += time.perf_counter()-tick
            kind = packet['kind']; counts[kind] += 1
            if kind in ('initial_state', 'physics'):
                row, step = packet['state'], packet['physics_step']
                if step != runtime.sample_steps[0]+(kind == 'physics'):
                    raise ValueError('Non-contiguous source physics clock')
                runtime.sample_steps[0] = step; runtime.latest[0] = row
                tick = time.perf_counter()
                runtime.distances[0]['rs_fault'] = runtime.faults[0].sample(row, step)
                runtime.highest_bottle_z[0] = max(row['mustard'][2], runtime.highest_bottle_z[0]) if runtime.highest_bottle_z[0] is not None else row['mustard'][2]
                times['fault_height_s'] += time.perf_counter()-tick
                fresh = kind == 'initial_state' or (step+1) % observer.stride == 0
                runtime.distances[0]['geometry_fresh'] = fresh
                if fresh: geometry(row, step, kind == 'initial_state')
                if kind == 'physics':
                    tick = time.perf_counter()
                    contact = aggregate_contacts(packet, runtime.dt)
                    runtime.windows[0].append(contact)
                    runtime.distances[0]['grasp_evidence'] = runtime.measure_grasp(0, packet['phase'], contact)
                    times['contact_grasp_s'] += time.perf_counter()-tick
            elif kind == 'observation':
                if packet['source_physics_step'] != runtime.sample_steps[0]:
                    raise ValueError('Observation uses a mismatched physical state')
                observer.steps = packet['physics_step']; observer.source_packet = packet
                tick = time.perf_counter()
                features = runtime.features(0, packet['phase'])
                times['feature_projection_s'] += time.perf_counter()-tick
                observer.evaluate_before_action(features, packet['phase'], packet['action_index'])
                if counts[kind] % 100 == 0:
                    print(json.dumps(dict(decisions=counts[kind], elapsed_s=time.perf_counter()-START,
                        geometry_s=times['geometry_s'], evaluation_s=observer.timings['evaluation_s'])), flush=True)
            elif kind == 'fault_event':
                event=packet['payload']
                if event['kind']!='command_write_exception' or event['source_physics_step']>runtime.sample_steps[0]:
                    raise ValueError('Unsupported or future fault evidence')
                runtime.faults[0].event(event['source_physics_step'],event['kind'],event['method']+': '+event['error'])
            else:
                raise ValueError('Unsupported raw record kind; do not ignore evidence: '+kind)
    times['replay_loop_s'] = time.perf_counter()-started_loop
    tick = time.perf_counter()
    for stream in (observer.decisions, observer.csvfile):
        stream.flush(); os.fsync(stream.fileno()); stream.close()
    times['flush_fsync_s'] = time.perf_counter()-tick
    writer = json.loads((args.episode/'raw_writer_result.json').read_text())
    expected = writer.get('sha256')
    if expected is not None and source_hash.hexdigest() != expected:
        raise ValueError('Raw source SHA256 does not match capture writer')
    times['evaluation_including_write_s'] = observer.timings['evaluation_s']
    times['serialization_write_subset_s'] = observer.timings['write_s']
    times['total_process_to_persisted_s'] = time.perf_counter()-START
    summary = dict(schema_version='arena-offline-risk-1', evaluation_mode='offline_replay',
        source_dataset=str(args.episode.resolve()), output=str(args.output.resolve()),
        simulator_rerun=False, geometry_backend=descriptor(configured_name(frozen['resolved_config']))['label']+' full authored triangle meshes',
        geometry_backend_statistics=pt.fcl.batch_stats(),warp_runtime_loaded='kitchen_safety.warp_mesh_backend' in sys.modules,
        mesh_backend_runtime=pt.fcl.runtime_info() if hasattr(pt.fcl,'runtime_info') else descriptor('warp'),
        source_profile_sha256=original_frozen['profile_sha256'],backend_migration=frozen.get('geometry_backend_migration'),
        raw_sha256=source_hash.hexdigest(), source_hash_verified=expected is not None,
        counts=dict(counts), risk_counts={k:dict(v) for k,v in observer.counts.items()},
        complete_counts=dict(observer.complete), geometry_complete_counts=dict(observer.geometry_complete),
        geometry_not_applicable_counts=dict(observer.geometry_not_applicable),
        missing={k:dict(v) for k,v in observer.missing.items()}, timings_s=dict(times),
        profile_sha256=frozen['profile_sha256'], errors=observer.errors,
        timing_semantics='Total includes Python imports, frozen mesh/GPU initialization, raw read/hash/decode, 120Hz temporal replay, 30Hz geometry/features/evaluation, JSON/CSV serialization and fsync; excludes later reference comparison',
        known_results_only=True, disabled_categories=observer.disabled_categories,
        source_simulated_duration_s=counts['physics']*runtime.dt,
        implementation_sha256={str(Path(__file__).name):EXECUTED_SOURCE_SHA256})
    write_json(args.output/'offline_risk_summary.json', summary)
    print(json.dumps(summary, ensure_ascii=False), flush=True)

if __name__ == '__main__': main()
