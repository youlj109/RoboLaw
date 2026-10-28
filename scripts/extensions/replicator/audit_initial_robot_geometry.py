#!/usr/bin/env python3
"""CPU full DROID initial-source geometry for source-preserving A candidates.

Reads the actual pinned DroidSceneCfg initial-joint expressions without
importing Kit. No sampled/zero arm configuration substitutes for source q.
This is authored initialization, not a claim of measured physics for new tasks.
"""
import argparse
import ast
import json
import math
from pathlib import Path
import re
import sys

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))
import fcl
import numpy as np
from pxr import Usd
from scipy.spatial.transform import Rotation
from kitchen_safety.planning import DroidKinematics, ROBOT_USD
from kitchen_safety.extensions.replicator.collision_planner import CollisionWorld
from scripts.extensions.replicator.variant_geometry import SourceRobotGeometry, code_evidence
from scripts.extensions.replicator.audit_fixed_base import fixed_colliders, ASSEMBLY
from scripts.extensions.replicator.search_source_layout_candidates import CandidateSearch
from scripts.extensions.replicator.audit_source_layout_geometry import REQUIRED_M, minimum_distance, sha

DROID_SOURCE = ROOT/'third_party/IsaacLab-Arena/isaaclab_arena/embodiments/droid/droid.py'


def source_initial_q():
    tree = ast.parse(DROID_SOURCE.read_text())
    names = ast.literal_eval(next(n.value for n in tree.body if isinstance(n, ast.Assign)
        and any(isinstance(t, ast.Name) and t.id == '_DROID_JOINT_NAMES' for t in n.targets)))
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'DroidSceneCfg')
    robot = next(n.value for n in cls.body if isinstance(n, ast.AnnAssign) and n.target.id == 'robot')
    initial = next(k.value for k in robot.keywords if k.arg == 'init_state')
    joints = next(k.value for k in initial.keywords if k.arg == 'joint_pos')
    def number(node):
        if isinstance(node, ast.Constant) and isinstance(node.value, (int,float)): return float(node.value)
        if isinstance(node, ast.Attribute) and isinstance(node.value, ast.Name) and node.value.id == 'torch' and node.attr == 'pi': return math.pi
        if isinstance(node, ast.UnaryOp) and isinstance(node.op, ast.USub): return -number(node.operand)
        if isinstance(node, ast.BinOp):
            a,b=number(node.left),number(node.right)
            if isinstance(node.op, ast.Div):return a/b
            if isinstance(node.op, ast.Mult):return a*b
            if isinstance(node.op, ast.Add):return a+b
            if isinstance(node.op, ast.Sub):return a-b
        raise ValueError('Unsupported source joint expression; never silently substitute it: '+ast.dump(node))
    patterns={ast.literal_eval(k):number(v) for k,v in zip(joints.keys,joints.values)}
    values={}
    for name in names:
        matches=[v for pattern,v in patterns.items() if re.fullmatch(pattern,name)]
        if len(matches)!=1:raise ValueError('Missing or ambiguous source initial joint: '+name)
        values[name]=matches[0]
    return values,dict(source=str(DROID_SOURCE.relative_to(ROOT)),sha256=sha(DROID_SOURCE),
        authored_patterns=patterns,full_joint_positions_rad=values,
        semantics='Pinned DroidSceneCfg authored initial state, matching local task adapter; not a measured reset of this unexecuted candidate')


def near_failures(left, right):
    low=np.array([row[2] for row in right]);high=np.array([row[3] for row in right])
    rows=[]
    for item in left:
        bound=np.linalg.norm(np.maximum(np.maximum(low-item[3],item[2]-high),0.),axis=1)
        for i in np.flatnonzero(bound <= REQUIRED_M):
            measure=minimum_distance([item],[right[i]],REQUIRED_M)
            if not measure['valid']:rows.append(measure)
    return rows


def audit_layout(path, robot_world, source_robot, q, fingers):
    layout=json.loads(Path(path).read_text())
    scene=CandidateSearch(layout)
    base=np.asarray(layout['robot_position'])+[0,0,layout['stand_height']]
    robot_world.kin=DroidKinematics(base,layout['robot_yaw_deg'])
    moving=source_robot.shapes(q,fingers)
    stand=scene.shapes['droid'].at([0,0,0])
    objects=[row for name in layout['objects'] for row in scene.shapes[name].at([0,0,0])]
    obstacles=scene.world+objects
    source_rows=moving+stand
    source_minimum=minimum_distance(source_rows,obstacles,REQUIRED_M)
    fixed=fixed_colliders(np.r_[base,Rotation.from_euler('z',layout['robot_yaw_deg'],degrees=True).as_quat()])
    fixed_rows=[(info['path'],shape,*np.asarray(info['bounds'])) for shape,info in fixed]
    proxy=robot_world.shapes(q,gripper_q=fingers)+fixed_rows
    proxy_minimum=minimum_distance(proxy,obstacles,REQUIRED_M)
    source_failures=near_failures(source_rows,obstacles)
    proxy_failures=near_failures(proxy,obstacles)
    return dict(task_id=layout['task_id'],layout_path=str(path),layout_sha256=sha(path),
        passed=bool(source_minimum['valid'] and proxy_minimum['valid']),execution_ready=False,
        source_robot_and_stand=dict(passed=source_minimum['valid'],minimum=source_minimum,failures=source_failures,
            moving_geometry_parts=len(moving),stand_parts=len(stand)),
        conservative_robot_and_fixed_solids=dict(passed=proxy_minimum['valid'],minimum=proxy_minimum,
            failures=proxy_failures,parts=len(proxy),fixed_geometry=[info for _,info in fixed]),
        source_per_body_minima={body:minimum_distance([row for row in moving if row[0] in paths],obstacles,REQUIRED_M)
            for body,paths in {body:['source_robot'+part[0] for part in source_robot.parts if part[1]==body]
                               for body in {part[1] for part in source_robot.parts}}.items()},
        initial_tcp_world=robot_world.kin.fk(q).tolist(),
        geometry_layer_sha256={str(Path(layer.realPath)):sha(layer.realPath) for stage in scene.stages
            for layer in stage.GetUsedLayers() if Path(layer.realPath).is_file()},
        scope='All source arm/finger/link0 collision surfaces plus source stand against every original kitchen/floor and task-object collider. Separately all existing conservative arm capsules/link hulls/measured-angle gripper hulls plus solid authored fixed base. No object masks or support exceptions. No PhysX cooked geometry, self-collision, path, stability, IK or task-success certification.')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--layout',type=Path,action='append',required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    if args.output.exists():raise FileExistsError(args.output)
    values,initial=source_initial_q()
    q=np.array([values[f'panda_joint{i}'] for i in range(1,8)])
    fingers={name:values[name] for name in CollisionWorld.GRIPPER_JOINT_NAMES}
    stage=Usd.Stage.CreateInMemory()
    world=CollisionWorld(stage,DroidKinematics([0,0,.8],0.))
    robot=SourceRobotGeometry(world)
    files=[Path(__file__),ROOT/'scripts/extensions/replicator/variant_geometry.py',
        ROOT/'scripts/extensions/replicator/search_source_layout_candidates.py',ROOT/'scripts/extensions/replicator/audit_source_layout_geometry.py',
        ROOT/'scripts/extensions/replicator/audit_task_source_relation.py',ROOT/'scripts/extensions/replicator/audit_fixed_base.py',
        ROOT/'kitchen_safety/geometry_audit.py',ROOT/'kitchen_safety/planning.py',ROOT/'kitchen_safety/extensions/replicator/collision_planner.py']
    evidence=code_evidence(files)
    rows=[]
    for path in args.layout:
        row=audit_layout(path,world,robot,q,fingers);rows.append(row)
        print(json.dumps(dict(task=row['task_id'],passed=row['passed'],source=row['source_robot_and_stand']['minimum'],
                              conservative=row['conservative_robot_and_fixed_solids']['minimum'])),flush=True)
    code_unchanged=all(sha(ROOT/path)==digest for path,digest in evidence['code_sha256'].items())
    report=dict(passed=all(row['passed'] for row in rows) and code_unchanged,execution_ready=False,
        required_clearance_m=REQUIRED_M,initial_joint_source=initial,source_robot=robot.provenance,
        source_robot_sha256=sha(ROBOT_USD),source_assembly_sha256=sha(ASSEMBLY),results=rows,
        code_unchanged=code_unchanged,**evidence)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    return int(not report['passed'])


if __name__=='__main__':raise SystemExit(main())
