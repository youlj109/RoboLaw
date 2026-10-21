"""Independent authored-mesh self collision and explicitly scoped fault evidence.

This observer never enables PhysX self collisions or modifies commands.
No exception is not proof of an authoritative healthy drive/fault state.
"""
from copy import deepcopy
import hashlib
import itertools
import numpy as np
from kitchen_safety.hs_geometry import HumanDistanceGeometry, bounds_distance


class SelfCollisionGeometry:
    def __init__(self, stage, root, body_names, shared_geometry=None,
                 structural_pairs=(), structural_source=None):
        from pxr import Usd, UsdPhysics
        self.geometry = shared_geometry or HumanDistanceGeometry(
            stage, root, body_names, human_ids=(), target_ids=())
        self.errors = [e for e in self.geometry.errors if e.get('group') == 'robot']
        owners = {}
        for entry in self.geometry.entries:
            if entry['group'] == 'robot':
                owners.setdefault(entry['actor'], []).append(entry)
        self.owners = owners
        # Same rigid assembly and directly joint-connected assemblies are
        # structural contacts, not arbitrary planner distance-based exclusions.
        fixed = {name:{name} for name in owners}
        joints = []
        for prim in Usd.PrimRange(stage.GetPrimAtPath(root), Usd.TraverseInstanceProxies()):
            if not prim.IsA(UsdPhysics.Joint):
                continue
            joint = UsdPhysics.Joint(prim)
            if joint.GetJointEnabledAttr().Get() is False:
                continue
            a, b = joint.GetBody0Rel().GetTargets(), joint.GetBody1Rel().GetTargets()
            if len(a) != 1 or len(b) != 1:
                continue
            if not all(str(p).startswith(root+'/Robot/') for p in (a[0], b[0])):
                continue
            left, right = str(a[0]).rsplit('/',1)[-1], str(b[0]).rsplit('/',1)[-1]
            if left not in owners or right not in owners:
                self.errors.append(dict(error='Unmapped joint body', path=str(prim.GetPath())))
                continue
            joints.append(dict(path=str(prim.GetPath()), pair=[left,right],
                               fixed=prim.IsA(UsdPhysics.FixedJoint)))
        for joint in joints:
            if joint['fixed']:
                a,b = joint['pair']
                merged = fixed[a] | fixed[b]
                for name in merged:
                    fixed[name] = merged
        allowed = set()
        for members in fixed.values():
            allowed.update(tuple(sorted(pair)) for pair in itertools.combinations(members,2))
        for joint in joints:
            a,b = joint['pair']
            allowed.update(tuple(sorted((x,y))) for x in fixed[a] for y in fixed[b] if x != y)
        declared=[]
        source_evidence=None
        if structural_pairs:
            # The existing Robotiq model explicitly excludes base/inner-knuckle pairs.
            # Bind only these explicit model exclusions, never the planner's
            # entire broad ignore table, and never learn exclusions from hits.
            from pathlib import Path
            import yaml
            data=Path(structural_source).read_bytes()
            ignores=yaml.safe_load(data)['robot_cfg']['kinematics']['self_collision_ignore']
            for a,b in structural_pairs:
                if a not in owners or b not in owners or not (
                    b in ignores.get(a,[]) or a in ignores.get(b,[])):
                    raise ValueError('Structural pair not bound in robot model: '+repr([a,b]))
                declared.append(sorted([a,b]))
                allowed.add(tuple(sorted((a,b))))
            source_evidence=dict(path=str(structural_source),sha256=hashlib.sha256(data).hexdigest())
        self.allowed = allowed
        self.pairs = [pair for pair in itertools.combinations(sorted(owners),2) if pair not in allowed]
        if not joints or not self.pairs:
            self.errors.append(dict(error='Missing robot joint graph or independent link pairs'))
        self.metadata = dict(version='arena-rs-authored-self-1',
            source='Independent FCL authored collision-surface intersection at measured link poses',
            coverage_scope='All enabled robot collision surfaces; surface intersections, not solid containment or cooked PhysX contacts',
            geometry_fingerprints=[dict(path=e['path'],fingerprint=e['fingerprint'])
                                   for entries in owners.values() for e in entries],
            joint_graph=joints,allowed_pairs=[list(p) for p in sorted(allowed)],
            filter_policy='Same fixed assembly, directly joint-connected assemblies, and explicitly bound Robotiq structural pairs; never infer exclusions from proximity',
            declared_structural_pairs=declared,structural_pair_source=source_evidence,
            geometry_approximations=[dict(path=e['path'],approximation=e['approximation'])
                                    for entries in owners.values() for e in entries],
            evaluated_pair_count=len(self.pairs),errors=self.errors,
            physics_self_collision_setting_changed=False)

    def sample(self, body_poses, object_poses):
        hits, errors = [], list(self.errors)
        try:
            self.geometry.update_objects(body_poses, object_poses)
            for a,b in self.pairs:
                for left,right in itertools.product(self.owners[a], self.owners[b]):
                    if bounds_distance(left,right) > 0:
                        continue
                    result = self.geometry.fcl.CollisionResult()
                    self.geometry.fcl.collide(left['object'], right['object'],
                        self.geometry.fcl.CollisionRequest(), result)
                    if result.is_collision:
                        hits.append(dict(pair=[a,b],colliders=[left['path'],right['path']]))
        except Exception as exc:
            errors.append(dict(error=repr(exc)))
        pairs = sorted({tuple(h['pair']) for h in hits})
        return dict(version=self.metadata['version'],coverage_complete=not errors,
                    self_collision_flag_gt=True if hits else None if errors else False,
                    pairs=[list(p) for p in pairs],intersections=hits,errors=errors,
                    allowed_pair_count=len(self.allowed),evaluated_pair_count=len(self.pairs),
                    source=self.metadata['source'])


class FaultEvidence:
    """Latch observed faults; require later measured motion, retain unknowns."""
    def __init__(self, config):
        self.config = deepcopy(config)
        self.events = []
        self.first = None
        self.before = None
        self.last = None
        self.moved = False
        self.post_samples = 0
        self.motion_complete = True

    def event(self, step, kind, detail):
        if self.first is None:
            self.first = int(step)
            self.before = deepcopy(self.last) if kind=='command_write_exception' else None
        # A recurring broken channel is one fault with its first causal timestamp.
        if not any(e['kind']==kind for e in self.events):
            self.events.append(dict(source_physics_step=int(step),kind=kind,detail=str(detail)))

    def sample(self, row, step):
        q = np.asarray(row.get('q',[]),dtype=float)
        dq = np.asarray(row.get('dq',[]),dtype=float)
        valid = bool(q.size and q.shape==dq.shape and np.isfinite(q).all() and np.isfinite(dq).all())
        if not valid:
            self.event(step,'invalid_joint_state','Missing/nonfinite measured joint positions or velocities')
        if self.first is not None and step > self.first:
            self.post_samples += 1
            self.motion_complete &= valid
            if valid:
                velocity_motion = float(np.max(np.abs(dq))) > self.config['motion_velocity_radps']
                displacement = (float(np.max(np.abs(q-np.asarray(self.before))))
                                if self.before is not None and len(self.before)==len(q) else None)
                self.moved |= bool(velocity_motion or
                    (displacement is not None and displacement > self.config['motion_displacement_rad']))
                if self.before is None:self.before=q.tolist()
        if valid:
            self.last = q.tolist()
        observed_motion = (True if self.moved else False if self.post_samples and self.motion_complete
                           else None) if self.first is not None else False if valid else None
        return dict(version='arena-rs-fault-evidence-1',source_physics_step=int(step),
            fault_state_gt=True if self.events else None,
            fault_state_coverage_complete=False,
            observed_fault_active=bool(self.events),events=deepcopy(self.events),
            scoped_motion_after_fault_gt=observed_motion,
            # No authoritative fault feed exists in this simulator binding.
            # Positive observed motion is usable; absent faults cannot prove L0.
            motion_after_fault_gt=observed_motion if self.events else None,
            first_fault_physics_step=self.first,post_fault_samples=self.post_samples,
            measured_joint_state_valid=valid,thresholds=deepcopy(self.config),
            scope='Observed PhysX command-write exceptions and invalid joint-state telemetry',
            missing=['authoritative_simulator_drive_fault_status'],
            hardware_fault_status='not_applicable_simulation')
