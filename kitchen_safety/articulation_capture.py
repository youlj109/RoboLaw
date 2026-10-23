"""Actual articulated link/COM/joint telemetry for every physical step.

No mesh queries, risk features, pose differencing or level computation here.
Views are persistent and can come from any task, not only a kitchen. Full paths
and backend row order are retained. External joints are never robot RS joints.
"""
import numpy as np


def array(value):
    value = value.torch if hasattr(value, 'torch') else value
    return value.detach().cpu().numpy().copy() if hasattr(value, 'detach') else np.asarray(
        value.numpy() if hasattr(value, 'numpy') else value).copy()


def com_world(poses, coms):
    """Rotate body-local COM xyz with measured xyzw, then add actor origin."""
    q = poses[..., 3:7]
    norms = np.linalg.norm(q, axis=-1)
    if not np.isfinite(poses).all() or not np.isfinite(coms).all() or np.any(abs(norms-1) > 1e-3):
        raise ValueError('Invalid measured pose/COM')
    xyz, w = q[..., :3], q[..., 3:4]
    local = coms[..., :3]
    rotated = local + 2*np.cross(xyz, np.cross(xyz, local) + w*local)
    return poses[..., :3] + rotated


class ArticulationCapture:
    def __init__(self, views):
        self.views = list(views)
        self.metadata = []
        seen = set();seen_owners={}
        for view in self.views:
            if not view.is_homogeneous:
                raise ValueError('Split heterogeneous articulation views before capture')
            roots = [str(x) for x in view.prim_paths]
            links = [[str(p) for p in row] for row in view.link_paths]
            dofs = [[str(p) for p in row] for row in view.dof_paths]
            if len(roots) != int(view.count) or len(links) != len(roots) or len(dofs) != len(roots):
                raise ValueError('Articulation backend row/path identity mismatch')
            for row in links:
                if len(row) != int(view.max_links) or any(p in seen for p in row):
                    raise ValueError('Missing/overlapping articulated links: '+str(dict(roots=roots,actual_links=len(row),expected_links=int(view.max_links),overlap={p:seen_owners[p] for p in sorted(set(row)&seen)},row=row)))
                seen.update(row);seen_owners.update({p:roots for p in row})
            for row in dofs:
                if len(row) != int(view.max_dofs):
                    raise ValueError('Articulation DOF path count mismatch')
            mt=view.shared_metatype
            # This pinned PhysX API omits the root from link_parent_indices.
            # Non-root entries still carry their measured parent-link index.
            parents=dict(mt.link_parent_indices)
            roots_by_index=[i for i,name in enumerate(mt.link_names) if name not in parents or parents[name]<0]
            if len(roots_by_index)!=1:raise ValueError('PhysX root link identity is ambiguous')
            self.metadata.append(dict(roots=roots, link_paths=links, dof_paths=dofs,
                fixed_base=bool(mt.fixed_base),root_link_index=roots_by_index[0],
                link_parent_indices=parents,link_names=list(mt.link_names),
                fixed_actor_paths=[row[roots_by_index[0]] for row in links] if mt.fixed_base else [],
                joint_types=[str(x) for x in mt.joint_types],dof_types=[str(x) for x in mt.dof_types],
                dof_identity='root + backend DOF index; paths may repeat',
                units='physx SI: m, rad, m/s, rad/s, N or Nm; use frozen joint types',
                external_joint_state=True))

    def capture(self):
        actors, motion, articulations, errors = {}, {}, {}, []
        for view, meta in zip(self.views, self.metadata):
            count, links, dofs = int(view.count), int(view.max_links), int(view.max_dofs)
            try:
                poses = array(view.get_link_transforms()).reshape(count, links, 7)
                velocities = array(view.get_link_velocities()).reshape(count, links, 6)
                if not np.isfinite(poses).all() or not np.isfinite(velocities).all():
                    raise ValueError('Nonfinite measured articulation state')
            except Exception as exc:
                errors.append(dict(roots=meta['roots'], channel='pose_velocity', error=repr(exc)))
                continue
            try:
                coms = array(view.get_coms()).reshape(count, links, 7)
                centers, com_error = com_world(poses, coms), None
            except Exception as exc:
                centers, com_error = None, repr(exc)
                errors.append(dict(roots=meta['roots'], channel='COM', error=com_error))
            for i, paths in enumerate(meta['link_paths']):
                for j, path in enumerate(paths):
                    actors[path] = np.r_[poses[i,j], velocities[i,j]].tolist()
                    motion[path] = dict(com_position_world_m=centers[i,j].tolist() if centers is not None else None,
                        com_linear_velocity_mps=velocities[i,j,:3].tolist(),
                        angular_velocity_radps=velocities[i,j,3:].tolist(),
                        source='PhysX articulation get_link_transforms/get_link_velocities/get_coms', error=com_error)
            channels = {}
            root_states=None
            try:
                root_states=np.concatenate([array(view.get_root_transforms()).reshape(count,7),
                    array(view.get_root_velocities()).reshape(count,6)],axis=1).tolist()
            except Exception as exc:errors.append(dict(roots=meta['roots'],channel='root_state',error=repr(exc)))
            for name, method in [('q','get_dof_positions'), ('dq','get_dof_velocities'),
                    ('position_targets','get_dof_position_targets'), ('velocity_targets','get_dof_velocity_targets'),
                    ('actuation_forces','get_dof_actuation_forces')]:
                try:
                    values = array(getattr(view, method)()).reshape(count, dofs) if dofs else np.empty((count, 0))
                    if not np.isfinite(values).all():
                        raise ValueError('Nonfinite '+name)
                    channels[name] = values.tolist()
                except Exception as exc:
                    channels[name] = None
                    errors.append(dict(roots=meta['roots'], channel=name, error=repr(exc)))
            for i, root in enumerate(meta['roots']):
                articulations[root] = dict(dof_paths=meta['dof_paths'][i],
                    root_state_world=root_states[i] if root_states is not None else None,
                    **{k: v[i] if v is not None else None for k, v in channels.items()})
        return dict(actor_states_world=actors, actor_motion_world=motion,
            articulations=articulations, errors=errors, complete=not errors,
            source='measured_current_physics_step', frequency_hz=120)


class SceneCapture:
    """Discover every PhysX articulation and remaining rigid actor in one env.

    Native Arena scene dictionaries only contain task-registered assets. They
    are not a closed-world inventory of physically enabled kitchen bodies.
    Never substitute authored USD poses for missing measured dynamic states.
    """
    def __init__(self, physics_sim_view, inventory):
        self.required={e['actor'] for e in inventory['colliders'] if e['dynamic']}
        root=inventory['env_root']
        discovery=physics_sim_view.create_articulation_view(root+'/**')
        discovered=[str(p) for p in discovery.prim_paths]
        paths=sorted(set(discovered))
        if any(not p.startswith(root+'/') for p in paths):
            raise ValueError('Articulation discovery escaped environment')
        # PhysX may enumerate the same articulation root more than once in
        # scenes with repeated instance references. Repeated discovery is not
        # a second physical actor. Only exact identity duplicates are removed;
        # differing/overlapping link sets still fail ArticulationCapture below.
        from collections import Counter
        discovery_duplicates={p:n for p,n in Counter(discovered).items() if n>1}
        views=[];identities={};view_duplicates=[]
        for path in paths:
            view=physics_sim_view.create_articulation_view(path)
            identity=(tuple(map(str,view.prim_paths)),
                tuple(tuple(map(str,row)) for row in view.link_paths),
                tuple(tuple(map(str,row)) for row in view.dof_paths),
                int(view.count),int(view.max_links),int(view.max_dofs))
            if identity in identities:
                view_duplicates.append(dict(query_path=path,identical_to=identities[identity]))
            else:
                identities[identity]=path;views.append(view)
        self.articulation=ArticulationCapture(views)
        covered={p for m in self.articulation.metadata for row in m['link_paths'] for p in row}
        remaining=sorted(self.required-covered)
        self.rigid=None;self.rigid_paths=[]
        if remaining:
            self.rigid=physics_sim_view.create_rigid_body_view(remaining)
            self.rigid_paths=[str(p) for p in self.rigid.prim_paths]
            if set(self.rigid_paths)!=set(remaining) or len(self.rigid_paths)!=len(remaining):
                raise ValueError('PhysX cannot bind required free actors: '+str(sorted(set(remaining)-set(self.rigid_paths))))
        self.metadata=dict(articulations=self.articulation.metadata,rigid_paths=self.rigid_paths,
            required_actor_paths=sorted(self.required), discovery='PhysX full environment subtree',
            duplicate_discovery_roots=discovery_duplicates,identical_discovery_views=view_duplicates,
            discovery_duplicate_policy='exact root/link/DOF identity only; remaining overlapping actors are errors')

    def capture(self):
        result=self.articulation.capture()
        if self.rigid is not None:
            try:
                poses=array(self.rigid.get_transforms()).reshape(-1,7)
                velocity=array(self.rigid.get_velocities()).reshape(-1,6)
                centers=com_world(poses,array(self.rigid.get_coms()).reshape(-1,7))
                if not np.isfinite(velocity).all():raise ValueError('Nonfinite free-body velocity')
                for i,path in enumerate(self.rigid_paths):
                    result['actor_states_world'][path]=np.r_[poses[i],velocity[i]].tolist()
                    result['actor_motion_world'][path]=dict(com_position_world_m=centers[i].tolist(),
                        com_linear_velocity_mps=velocity[i,:3].tolist(),angular_velocity_radps=velocity[i,3:].tolist(),
                        source='PhysX rigid-body measured transforms/velocities/COM')
            except Exception as exc:
                result['errors'].append(dict(channel='free_rigid_body_state',error=repr(exc)))
        result['missing_actor_paths']=sorted(self.required-set(result['actor_states_world']))
        result['complete']=not result['errors'] and not result['missing_actor_paths']
        return result
