#!/usr/bin/env python3
"""Finite, source-bound CPU grasp/layout search; never a physical success claim.

Uses the real G reference first, then a written finite bottle-local grasp grid.
Every rejected evaluated candidate is retained. All original source geometry,
stand/yaw/relations and 2 mm pair margins remain unchanged.
"""
import argparse
import copy
import json
from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT))
import fcl
import numpy as np
from scipy.spatial.transform import Rotation
from kitchen_safety.planning import DroidKinematics,ROBOT_USD
from kitchen_safety.extensions.replicator.mustard_family_preparation import load_preparation,sha
from kitchen_safety.extensions.replicator.mustard_family_collision import MustardFamilyCollisionWorld
from kitchen_safety.extensions.replicator.task_plan_diagnostics import local_cartesian_candidate
from scripts.extensions.replicator.audit_mustard_family_approach import source_stage
from scripts.extensions.replicator.variant_geometry import SourceRobotGeometry,code_evidence
from scripts.extensions.replicator.audit_initial_robot_geometry import source_initial_q
from scripts.extensions.replicator.audit_fixed_base import fixed_colliders,ASSEMBLY
from scripts.extensions.replicator.audit_source_layout_geometry import minimum_distance,REQUIRED_M,diagnose
from scripts.extensions.replicator.search_source_layout_candidates import CandidateSearch,nearest_grid


def save(path,value):Path(path).write_text(json.dumps(value,indent=2,allow_nan=False)+'\n')


class RigidRows:
    """Move cached gripper shapes with one TCP rigid transform, without IK."""
    def __init__(self,rows,tcp):
        self.inverse=np.linalg.inv(tcp)
        self.rows=[(name,obj,np.array(obj.getRotation()).copy(),np.array(obj.getTranslation()).copy(),lo.copy(),hi.copy())
                   for name,obj,lo,hi in rows]

    def at(self,tcp):
        delta=tcp@self.inverse;r,t=delta[:3,:3],delta[:3,3];out=[]
        for name,obj,r0,t0,lo,hi in self.rows:
            obj.setTransform(fcl.Transform(r@r0,r@t0+t))
            center=r@((lo+hi)/2)+t;extent=np.abs(r)@((hi-lo)/2)
            out.append((name,obj,center-extent,center+extent))
        return out


def declarations(prepared,reference):
    candidates=copy.deepcopy(reference['ordered_transfer_candidates'])
    fixed=np.array([[0,-1,0],[0,0,1],[-1,0,0]])
    center=np.asarray(prepared['profile']['source_bottle_geometry']['default_tcp_point_bottle_local_m'])
    for tilt in (35.,45.,55.,65.,75.,85.):
        for yaw in (0.,90.,180.,270.):
            rz=Rotation.from_euler('z',yaw,degrees=True).as_matrix()
            for offset in (.01,.02,.03):
                for z in (0.,.01,.019):
                    local=np.eye(4);local[:3,:3]=rz@Rotation.from_euler('x',tilt,degrees=True).as_matrix()@fixed
                    local[:3,3]=center+rz@np.array([0.,-offset,z])
                    candidates.append(dict(name=f'grid_tilt{tilt:g}_yaw{yaw:g}_offset{offset:g}_z{z:g}',
                        bottle_to_tcp_local=local.tolist(),derivation='Predeclared body-local finite grid; no physical contact claim',
                        tilt_deg=tilt,yaw_deg=yaw,lateral_offset_m=offset,local_z_m=z))
    return candidates


class Search:
    def __init__(self,prepared):
        self.prepared=prepared;self.layout=prepared['layout'];self.scene=CandidateSearch(self.layout)
        base=np.asarray(self.layout['robot_position'])+[0,0,self.layout['stand_height']]
        self.kin=DroidKinematics(base,self.layout['robot_yaw_deg'])
        self.world=MustardFamilyCollisionWorld(source_stage(self.layout),self.kin,preparation=prepared,env_root='/World/envs/env_0')
        self.robot=SourceRobotGeometry(self.world);values,self.initial=source_initial_q()
        self.q=np.array([values[f'panda_joint{i}'] for i in range(1,8)])
        self.fingers={name:values[name] for name in self.world.GRIPPER_JOINT_NAMES}
        self.world.gripper_joint_positions=self.fingers;self.world.gripper_joint_position=0.
        bottle=self.layout['objects']['mustard_bottle'];self.bp=np.eye(4)
        self.bp[:3,:3]=Rotation.from_euler('z',bottle['yaw_deg'],degrees=True).as_matrix();self.bp[:3,3]=bottle['position']
        self.world.payload_pose=self.bp;self.world.set_phase('approach')
        self.environment=self.scene.world+self.scene.shapes['bowl'].at([0,0,0])
        self.stand=self.scene.shapes['droid'].at([0,0,0])
        fixed=fixed_colliders(np.r_[base,Rotation.from_euler('z',self.layout['robot_yaw_deg'],degrees=True).as_quat()])
        self.fixed=[(info['path'],obj,*np.asarray(info['bounds'])) for obj,info in fixed]
        self.intended={'source_robot'+part[0] for part in self.robot.parts if part[1].rsplit('/',1)[-1] in ('left_inner_finger','right_inner_finger')}
        self.source_gripper_names={'source_robot'+part[0] for part in self.robot.parts if '/Gripper/' in part[1]}
        allsource=self.robot.shapes(self.q,self.fingers)
        self.gripper_source=RigidRows([row for row in allsource if row[0] in self.source_gripper_names],self.kin.fk(self.q))
        proxy=self.world.shapes(self.q,gripper_q=self.fingers)
        self.gripper_proxy=RigidRows([row for row in proxy if row[0].startswith('gripper') or row[0] in ('finger','robot_hull/whole_gripper')],self.kin.fk(self.q))

    def own_gripper(self,target):
        source=[row for row in self.gripper_source.at(target) if row[0] not in self.intended]
        policy=self.world.contact_policy
        proxy=[row for row in self.gripper_proxy.at(target) if not(policy.finger_pair(row[0],'approach') or policy.redundant_finger_envelope(row[0],'approach'))]
        bottle=self.scene.shapes['mustard_bottle'].at([0,0,0])
        own=[row for row in self.world.objects if self.world._matches(row[0],'mustard_bottle')]
        a=minimum_distance(source,bottle,REQUIRED_M);b=minimum_distance(proxy,own,REQUIRED_M)
        return dict(source_noncontact_gripper=a,conservative_noncontact_gripper=b,passed=bool(a['valid'] and b['valid']))

    def envelope_scan(self,candidate):
        target=self.bp@np.asarray(candidate['bottle_to_tcp_local']);rows=[]
        # End first rejects impossible grasp geometry quickly, then all 64
        # additional 1.953125 mm vertical samples, without skipping the palm.
        heights=[0.]+list(np.linspace(.125,0.,65)[:-1])
        for rise in heights:
            tcp=target.copy();tcp[2,3]+=rise;row=dict(rise_m=float(rise),**self.own_gripper(tcp));rows.append(row)
            if not row['passed']:break
        return dict(name=candidate['name'],checked_samples=len(rows),declared_samples=len(heights),rows=rows,
                    passed=bool(len(rows)==len(heights) and all(x['passed'] for x in rows)))

    def gripper_environment(self,target):
        a=minimum_distance(self.gripper_source.at(target),self.environment,REQUIRED_M)
        b=minimum_distance(self.gripper_proxy.at(target),self.environment,REQUIRED_M)
        return dict(source=a,conservative=b,passed=bool(a['valid'] and b['valid']))

    def check_q(self,q,bottle_pose,phase):
        self.world.set_phase(phase);source=self.robot.shapes(q,self.fingers)
        env=minimum_distance(source+self.stand,self.environment,REQUIRED_M)
        # Caller sets the authored bottle shape delta before these queries.
        bottle=self.scene.shapes['mustard_bottle'].at(bottle_pose[:3,3]-self.bp[:3,3])
        own=minimum_distance([row for row in source if phase=='pregrasp' or row[0] not in self.intended],bottle,REQUIRED_M)
        proxy=self.world.valid(q,mode='planning',gripper_q=self.fingers,object_pose=bottle_pose)
        fixed=minimum_distance(self.fixed,self.environment+bottle,REQUIRED_M)
        return dict(source_environment=env,source_noncontact_bottle=own,conservative_valid=proxy,
            conservative_failure=None if proxy else self.world.last_clearance,conservative_fixed=fixed,
            passed=bool(env['valid'] and own['valid'] and proxy and fixed['valid']))

    def trajectory(self,target,bottle_pose):
        pregrasp=target.copy();pregrasp[2,3]+=.125
        try:q=self.kin.ik(pregrasp,self.q,attempts=12)
        except RuntimeError as error:return dict(passed=False,reason=str(error),stage='pregrasp_ik')
        actual=self.kin.fk(q);position_error=float(np.linalg.norm(actual[:3,3]-pregrasp[:3,3]))
        rotation_error=float(np.rad2deg(Rotation.from_matrix(actual[:3,:3]@pregrasp[:3,:3].T).magnitude()))
        if position_error>.002 or rotation_error>2 or np.any(q<self.kin.limits[:,0]) or np.any(q>self.kin.limits[:,1]):
            return dict(passed=False,stage='pregrasp_ik',position_error_m=position_error,rotation_error_deg=rotation_error)
        rows=[]
        count=max(2,int(np.ceil(np.max(np.abs(q-self.q))/.02))+1)
        for index,fraction in enumerate(np.linspace(0,1,count)):
            value=self.q+(q-self.q)*fraction;row=dict(phase='pregrasp',sample=index,q_rad=value.tolist(),**self.check_q(value,bottle_pose,'pregrasp'));rows.append(row)
            if not row['passed']:return dict(passed=False,stage='initial_to_pregrasp_joint_line',rows=rows,declared_samples=count)
        try:plan=local_cartesian_candidate(self.kin,q,target,'approach',1/15,.4)
        except RuntimeError as error:return dict(passed=False,reason=str(error),stage='approach_ik',rows=rows)
        dense=[plan['positions'][0]]
        for a,b in zip(plan['positions'][:-1],plan['positions'][1:]):
            count=max(1,int(np.ceil(np.max(np.abs(b-a))/.02)))
            dense.extend(a+(b-a)*u for u in np.linspace(0,1,count+1)[1:])
        for index,value in enumerate(dense):
            row=dict(phase='approach',sample=index,q_rad=value.tolist(),**self.check_q(value,bottle_pose,'approach'));rows.append(row)
            if not row['passed']:return dict(passed=False,stage='approach_geometry',rows=rows,approach_declared_samples=len(dense))
        return dict(passed=True,rows=rows,pregrasp_q_rad=q.tolist(),approach_q_rad=dense[-1].tolist(),
                    pregrasp_target_world=pregrasp.tolist(),approach_target_world=target.tolist(),approach_metadata=plan['metadata'])


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--preparation',type=Path,required=True)
    p.add_argument('--reference',type=Path,required=True);p.add_argument('--output-dir',type=Path,required=True)
    p.add_argument('--envelope-only',action='store_true')
    args=p.parse_args()
    if args.output_dir.exists():raise FileExistsError(args.output_dir)
    prepared=load_preparation(args.preparation);reference=json.loads(args.reference.read_text())
    if sha(ROBOT_USD)!=reference['source_robot_sha256'] or sha(ROOT/prepared['layout']['objects']['mustard_bottle']['usd_path'])!=reference['source_bottle_sha256']:
        raise ValueError('Transfer requires exactly the same source robot and bottle')
    for name,digest in reference['reference_input_sha256'].items():
        if sha(ROOT/reference['reference_episode']/name)!=digest:raise ValueError('Changed reference episode')
    candidates=declarations(prepared,reference);grid=nearest_grid(.1,.005,[0.,.0001])
    evidence=code_evidence([Path(__file__),ROOT/'scripts/extensions/replicator/extract_mustard_grasp_reference.py',
        ROOT/'scripts/extensions/replicator/audit_mustard_family_approach.py',ROOT/'scripts/extensions/replicator/search_source_layout_candidates.py',
        ROOT/'scripts/extensions/replicator/variant_geometry.py',ROOT/'scripts/extensions/replicator/audit_source_layout_geometry.py',
        ROOT/'scripts/extensions/replicator/audit_task_source_relation.py',ROOT/'scripts/extensions/replicator/audit_fixed_base.py',
        ROOT/'scripts/extensions/replicator/audit_initial_robot_geometry.py',ROOT/'kitchen_safety/planning.py',ROOT/'kitchen_safety/geometry_audit.py',
        ROOT/'kitchen_safety/extensions/replicator/collision_planner.py',ROOT/'kitchen_safety/extensions/replicator/task_collision.py',
        ROOT/'kitchen_safety/extensions/replicator/task_contact_policy.py',ROOT/'kitchen_safety/extensions/replicator/task_plan_diagnostics.py',
        ROOT/'kitchen_safety/extensions/replicator/mustard_family_collision.py',ROOT/'kitchen_safety/extensions/replicator/mustard_family_preparation.py'])
    declaration=dict(preparation=str(args.preparation),preparation_sha256=sha(args.preparation),reference=str(args.reference),reference_sha256=sha(args.reference),
        candidates=candidates,layout_grid_xyz_m=grid,required_clearance_m=.002,pregrasp_rise_m=.125,
        selection='First passing layout in exact declared nearest-displacement order, then first passing grasp in declared order (3 G-derived first). Own-bottle rejected candidates are translation invariant at unchanged source yaw. Stop only on a fully sampled CPU candidate; preserve every visited rejection and unvisited count.',
        fixed=['source assets/background poses','source robot initial q','robot and stand pose/yaw/height','bottle/bowl yaw','bowl pose','source On/NextTo validators','2mm margins and all noncontact palm/arm checks'],
        limitations='Finite 5mm XY / 0 or +0.1mm Z grid; not a global optimum. Nominal source geometry and sampled CPU IK only; no self-collision, continuous proof, PhysX, bilateral contact, cuRobo, release, neutral or task-success claim.',**evidence)
    args.output_dir.mkdir(parents=True);save(args.output_dir/'declaration.json',declaration)
    search=Search(prepared);envelopes=[]
    for index,candidate in enumerate(candidates):
        row=search.envelope_scan(candidate);row['candidate_index']=index;envelopes.append(row)
        print(json.dumps(dict(stage='own_gripper',index=index,name=row['name'],passed=row['passed'])),flush=True)
    save(args.output_dir/'gripper_bottle_scan.json',dict(results=envelopes,source_robot=search.robot.provenance,policy=search.world.contact_policy.report(),execution_ready=False))
    survivors=[index for index,row in enumerate(envelopes) if row['passed']]
    attempts=[];selected=None
    if not args.envelope_only:
        for grid_index,delta in enumerate(grid):
            initial=search.scene.evaluate('mustard_bottle',delta);row=dict(grid_index=grid_index,initial=initial,grasps=[]);attempts.append(row)
            if not initial['passed']:continue
            bp=search.bp.copy();bp[:3,3]+=delta
            # Original initial full robot plus authored fixed solids remains required.
            reset=search.check_q(search.q,bp,'pregrasp');row['initial_full_robot']=reset
            if not reset['passed']:continue
            for index in survivors:
                target=bp@np.asarray(candidates[index]['bottle_to_tcp_local'])
                endpoint=search.gripper_environment(target);item=dict(candidate_index=index,endpoint_environment=endpoint);row['grasps'].append(item)
                if not endpoint['passed']:continue
                trajectory=search.trajectory(target,bp);item['trajectory']=trajectory
                if trajectory['passed']:
                    selected=dict(grid_index=grid_index,candidate_index=index,delta_xyz_m=list(delta),grasp=candidates[index]);break
            if grid_index%10==0 or selected is not None:
                print(json.dumps(dict(stage='layout',index=grid_index,delta=delta,selected=selected is not None)),flush=True)
                save(args.output_dir/'attempts_partial.json',dict(attempts=attempts,selected=selected,execution_ready=False))
            if selected is not None:break
    unchanged=all(sha(ROOT/path)==digest for path,digest in evidence['code_sha256'].items())
    report=dict(selected=selected,passed=bool(selected and unchanged),own_gripper_survivors=survivors,visited_layouts=len(attempts),
        unvisited_layouts=len(grid)-len(attempts),attempts=attempts,code_unchanged=unchanged,execution_ready=False,
        physics_launched=False,curobo_validated=False,actual_grasp_validated=False,source_initial_joint_evidence=search.initial,
        source_robot_sha256=sha(ROBOT_USD),source_stand_sha256=sha(ASSEMBLY),
        geometry_layer_sha256={str(Path(layer.realPath)):sha(layer.realPath) for stage in search.scene.stages for layer in stage.GetUsedLayers() if Path(layer.realPath).is_file()},
        declaration_sha256=sha(args.output_dir/'declaration.json'),scope=declaration['limitations'])
    if selected:
        layout=copy.deepcopy(prepared['layout']);layout['objects']['mustard_bottle']['position']=(np.asarray(layout['objects']['mustard_bottle']['position'])+selected['delta_xyz_m']).tolist()
        layout['execution_ready']=False;save(args.output_dir/'candidate_layout.json',layout)
        geometry=diagnose(args.output_dir/'candidate_layout.json');save(args.output_dir/'candidate_geometry.json',geometry)
        report['selected_source_initial_geometry_passed']=geometry['initial_geometry_passed'];report['passed'] &= geometry['initial_geometry_passed']
    save(args.output_dir/'report.json',report)
    print(json.dumps(dict(output=str(args.output_dir),survivors=len(survivors),selected=selected,passed=report['passed'],execution_ready=False)),flush=True)
    return 0 if args.envelope_only and unchanged else int(not report['passed'])


if __name__=='__main__':raise SystemExit(main())
