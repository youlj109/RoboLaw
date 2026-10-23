"""Read-only PhysX joint monitoring for every kitchen articulation root."""
import numpy as np


class MechanismMonitor:
    def __init__(self, stage):
        from isaaclab_physx.physics import PhysxManager
        from pxr import Usd, UsdPhysics
        from kitchen_safety.grasp_contacts import _numpy as as_numpy
        self.simulation_view = PhysxManager.get_physics_sim_view()
        self.views = []
        self.metadata = []
        self.samples = []
        self.missing = []
        self.types = []
        for prim in Usd.PrimRange(stage.GetPrimAtPath('/World/envs/env_0/kitchen'),
                                 Usd.TraverseInstanceProxies()):
            if not prim.HasAPI(UsdPhysics.ArticulationRootAPI):
                continue
            path = str(prim.GetPath())
            try:
                view = self.simulation_view.create_articulation_view(path)
                if view.count != 1:
                    raise ValueError(f'Expected one articulation, got {view.count}')
                paths = list(view.dof_paths[0])
                # D6 joints can have multiple rotational/translational DOFs.
                # PhysX declares each DOF's units: 0=rotation, 1=translation.
                types = as_numpy(view.get_dof_types()).reshape(-1).tolist()
                if len(types) != len(paths) or any(t not in (0, 1) for t in types):
                    raise ValueError(f'Unsupported runtime DOF types: {types}')
                rotational = [t == 0 for t in types]
                names = list(view.shared_metatype.dof_names)
                self.views.append(view)
                self.types.extend(rotational)
                self.metadata.extend({'path': p, 'dof_name': n, 'rotational': r}
                                     for p, n, r in zip(paths, names, rotational))
            except Exception as exc:
                self.missing.append({'root': path, 'error': str(exc)})
        self.initial = self.state()['q']

    def state(self):
        from kitchen_safety.grasp_contacts import _numpy as as_numpy
        return {key: np.concatenate([as_numpy(getattr(view, method)()).reshape(-1).copy()
                                     for view in self.views]).tolist() if self.views else []
                for key, method in [('q', 'get_dof_positions'), ('dq', 'get_dof_velocities')]}

    def sample(self, phase, action_index):
        self.samples.append({'phase': phase, 'action_index': action_index, **self.state()})

    def report(self, allowed_joint_names=()):
        limits = {'rotation_excursion_rad': float(np.deg2rad(1.)),
                  'translation_excursion_m': .0001}
        rows = []
        q = np.array([self.initial] + [s['q'] for s in self.samples])
        for index, joint in enumerate(self.metadata):
            excursion = float(np.max(np.abs(q[:, index] - q[0, index])))
            allowed = joint['path'] in allowed_joint_names or joint['path'].rsplit('/', 1)[-1] in allowed_joint_names
            threshold = limits['rotation_excursion_rad' if joint['rotational'] else 'translation_excursion_m']
            rows.append({**joint, 'maximum_excursion': excursion, 'threshold': threshold,
                         'allowed_task_motion': allowed, 'passed': allowed or excursion < threshold})
        return {'passed': bool(rows and self.samples and not self.missing
                               and all(row['passed'] for row in rows)),
                'thresholds': limits, 'missing_roots': self.missing, 'joints': rows,
                'samples': self.samples, 'scope': 'Read-only measured q/dq at each monitored physics update; '
                '1 degree rotation / 0.1 mm translation predeclared static limits. '
                'Task joint excluded only from excursion rejection, not from recording.'}
