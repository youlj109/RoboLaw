"""Selective original mustard contacts; no whole-object FCL exclusions."""
from dataclasses import dataclass
from .collision_planner import CollisionWorld
from .task_collision import PayloadCollisionWorld,PayloadGeometry
from .task_contact_policy import BananaContactPolicy


@dataclass(frozen=True)
class MustardContactPolicy(BananaContactPolicy):
    payload_name: str='mustard_bottle'
    destination_name: str='bowl'
    source_support: str=''

    def __post_init__(self):
        if not self.source_support.startswith('/World/envs/env_0/kitchen/') or self.source_support.endswith('/'):
            raise ValueError('Require this task exact source support collider path')
        if self.support_band_m!=.006:raise ValueError('Original support band must remain unchanged')

    def report(self):
        report=super().report();report.pop('measured_midbody_contacts_required')
        report.update(measured_source_broad_bottle_body_contacts_required=True,
            carried_bottle_max_tilt_deg=28.,task='Original mustard bottle into original bowl',
            contact_region='bottle_body_geometry safe_contact_z_range_m; every recorded loaded/near inner-finger contact is audited in bottle-local coordinates')
        return report


class MustardFamilyCollisionWorld(PayloadCollisionWorld):
    def __init__(self,stage,kin,*,preparation,**kwargs):
        from pathlib import Path
        from .mustard_family_preparation import ROOT,TASKS
        if preparation['task_id'] not in TASKS or preparation['variant']!='base':
            raise ValueError('Only the four source-prepared mustard baselines are implemented')
        policy=preparation['profile']['clearances']
        for name,expected in (('default_clearance',policy['default_m']),('clearances',policy['planning']),
                              ('execution_clearances',policy['execution'])):
            if name in kwargs and kwargs[name]!=expected:raise ValueError('Prepared original safety policy must not change: '+name)
            kwargs[name]=expected
        self.payload_name='mustard_bottle'
        self.payload_geometry=PayloadGeometry(ROOT/Path(preparation['layout']['objects'][self.payload_name]['usd_path']))
        CollisionWorld.__init__(self,stage,kin,**kwargs)
        self.payload_model,self.payload_hull_vertices=self._convex(self.payload_geometry.vertices)
        self.contact_policy=MustardContactPolicy(source_support=preparation['profile']['source_support_collider_paths'][self.payload_name])
        supports=[row for row in self.objects if row[0]==self.contact_policy.source_support]
        if len(supports)!=1:raise ValueError('Exact original support collider missing or duplicated')
        parts={name.rsplit('/',1)[-1] for name,_,_ in self.gripper_models}
        if not {'base_link','left_inner_finger','right_inner_finger'}<=parts or len(parts)<=3:
            raise ValueError('Every source gripper part hull is required')
        self.phase,self.payload_pose,self.gripper_joint_positions='reset',None,None
        model=self.fcl.BVHModel()
        model.beginModel(len(self.payload_geometry.vertices),len(self.payload_geometry.triangles))
        model.addSubModel(self.payload_geometry.vertices,self.payload_geometry.triangles);model.endModel()
        self.payload_triangle_model=model

    def valid(self,q,ignore=(),*,mode='planning',gripper_q=None,object_pose=None,obstacle_filter=None):
        # Unlike banana, the original carried bottle 28-degree upright gate
        # remains enforced. Base method dynamically calls our full-pair policy.
        return CollisionWorld.valid(self,q,ignore,mode=mode,gripper_q=gripper_q,
            object_pose=object_pose,obstacle_filter=obstacle_filter)


def audit_bottle_body_contacts(records,geometry):
    """Reject unintended phase/body-region contacts across the entire trace."""
    import numpy as np
    from .task_contact_policy import FINGER_PHASES
    low,high=map(np.array,geometry['mesh_bounds_m'])
    low[2],high[2]=geometry['safe_contact_z_range_m']
    failures=[];loaded_count=near_count=0
    for row in records:
        if row['object_name']!='mustard_bottle':raise ValueError('Incorrect task payload in contact evidence')
        for name in ('left_inner_finger','right_inner_finger'):
            entry=row['fingers'][name]
            points=np.asarray(entry['points_object_local_m']).reshape(-1,3)
            force=np.asarray(entry['normal_forces_n']);gap=np.asarray(entry['separations_m'])
            if points.shape!=(entry['count'],3) or force.shape!=(entry['count'],) or gap.shape!=force.shape:
                raise ValueError('Incomplete raw finger contact arrays')
            if not all(np.isfinite(value).all() for value in (points,force,gap)):raise ValueError('Nonfinite finger contact')
            loaded=force>.01;near=gap<=.00025
            loaded_count+=int(loaded.sum());near_count+=int(near.sum())
            inside=np.all((points>=low)&(points<=high),axis=1)
            bad=(loaded|near)&(~inside | (row['phase'] not in FINGER_PHASES))
            if bad.any():failures.append(dict(physics_step=row['physics_step'],phase=row['phase'],finger=name,
                offending_contact_indices=np.flatnonzero(bad).tolist(),points_object_local_m=points[bad].tolist()))
    return dict(passed=bool(records and not failures),samples=len(records),failures=failures,
        loaded_contacts=loaded_count,near_contacts=near_count,region_object_local_m=[low.tolist(),high.tolist()],
        minimum_force_n=.01,max_separation_m=.00025,
        scope='All actual inner-finger/bottle contact rows; every loaded or near contact must be in a permitted phase and source broad body. Bilateral sustained grasp/lift and all-other-pair safety remain separate mandatory gates.')
