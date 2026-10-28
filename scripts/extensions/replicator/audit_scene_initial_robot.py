#!/usr/bin/env python3
"""Read-only 154-scene hazard checks against two source robot pose scopes."""
import argparse
import json
from pathlib import Path
import hashlib

import fcl
import numpy as np
from pxr import Usd, UsdGeom

from kitchen_safety.extensions.replicator.scene_variant_builder import Hazard, variants, write_json
from kitchen_safety.extensions.replicator.collision_planner import CollisionWorld
from kitchen_safety.planning import DroidKinematics
from kitchen_safety.geometry_audit import minimum_distance
from scripts.extensions.replicator.variant_geometry import SourceRobotGeometry
from scripts.extensions.replicator.audit_initial_robot_geometry import source_initial_q

ROOT = Path(__file__).resolve().parents[3]


def authored_rows(robot, base):
    # The scene renderer poses a wrapper and keeps the referenced USD intact.
    # Preserve even the source link0's tiny authored translation here.
    delta = base
    r, t = delta[:3,:3], delta[:3,3]
    rows = []
    for name, body, obj, inverse, rotation, position, lo, hi in robot.parts:
        obj.setTransform(fcl.Transform(r@rotation, r@position+t))
        center = r@((lo+hi)/2)+t
        half = np.abs(r)@((hi-lo)/2)
        rows.append(('source_authored_robot'+name,obj,center-half,center+half))
    return rows


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    manifest=json.loads((ROOT/'configs/extensions/replicator/scene_variants/manifest.json').read_text())
    hazards={entry['id']:Hazard(entry) for entry in variants()}
    values,initial=source_initial_q()
    q=np.array([values[f'panda_joint{i}'] for i in range(1,8)])
    fingers={name:values[name] for name in CollisionWorld.GRIPPER_JOINT_NAMES}
    world=CollisionWorld(Usd.Stage.CreateInMemory(),DroidKinematics([0,0,.8],0))
    robot=SourceRobotGeometry(world)
    rows=[];task=None
    for item in manifest['scenes']:
        path=ROOT/item['config_path'];layout=json.loads(path.read_text())
        if task!=item['task_id']:
            task=item['task_id']
            world.kin=DroidKinematics(np.asarray(layout['robot_position'])+[0,0,layout['stand_height']],layout['robot_yaw_deg'])
        obstacle=layout['obstacle']
        shapes,_=hazards[item['variant_id']].at(obstacle['position'],obstacle['yaw_deg'])
        initial_measure=minimum_distance(robot.shapes(q,fingers),shapes,.002)
        authored_measure=minimum_distance(authored_rows(robot,world.kin.base),shapes,.002)
        row=dict(scene_id=item['scene_id'],task_id=task,variant_id=item['variant_id'],config_path=item['config_path'],
            config_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
            source_controller_initial=initial_measure,usd_authored_link_transforms=authored_measure,
            passed=bool(initial_measure['valid'] and authored_measure['valid']))
        rows.append(row)
        if not row['passed']:print(json.dumps(row),flush=True)
    report=dict(schema_version=1,scenes_checked=len(rows),passed=all(r['passed'] for r in rows),
        source_controller_initial_joints=initial,source_robot=robot.provenance,
        failed_scenes=[r['scene_id'] for r in rows if not r['passed']],results=rows,
        scope='Read-only original collision surfaces for full source arm, gripper and link0 versus added hazard. Authored USD transforms and original controller q are separate. No native simulation pose, self-collision, trajectory or task acceptance claim.')
    write_json(args.output,report)
    print(json.dumps(dict(checked=len(rows),failed=report['failed_scenes'],output=str(args.output))),flush=True)
    return int(not report['passed'])


if __name__=='__main__':raise SystemExit(main())
