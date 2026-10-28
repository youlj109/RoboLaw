"""Read-only native PhysX joint observation beyond scene.articulations.

Prepared separately so it can be integrated after the current execution ends.
An authored articulation whose native view fails is retained as a named error.
"""
import numpy as np


def _array(value):
    value = value.torch if hasattr(value, 'torch') else value
    if hasattr(value, 'detach'):
        return value.detach().cpu().numpy().copy()
    # Warp's zero-DOF array has a null data pointer. Its explicit numpy()
    # conversion preserves shape (1, 0); direct np.asarray raises ValueError.
    return np.asarray(value.numpy() if hasattr(value, 'numpy') else value).copy()


class NativeMechanisms:
    def __init__(self, stage, sim_view, kitchen_root='/World/envs/env_0/kitchen'):
        from pxr import Usd, UsdPhysics
        root = stage.GetPrimAtPath(kitchen_root)
        if not root:
            raise ValueError('No native kitchen root: ' + kitchen_root)
        self.views = {}
        self.metadata = {
            'scope': 'USD articulation-root inventory and native PhysX q/dq reads; observer performs no state writes.',
            'kitchen_root': kitchen_root,
            'authored_articulation_roots': [], 'native_articulations': {},
            'initialization_errors': [], 'source_joint_paths': [],
        }
        for prim in Usd.PrimRange(root, Usd.TraverseInstanceProxies()):
            path = str(prim.GetPath())
            if prim.IsA(UsdPhysics.Joint):
                self.metadata['source_joint_paths'].append(path)
            if not prim.HasAPI(UsdPhysics.ArticulationRootAPI):
                continue
            self.metadata['authored_articulation_roots'].append(path)
            try:
                view = sim_view.create_articulation_view(path)
                if view.count != 1:
                    raise RuntimeError(f'Expected exactly one native articulation, got {view.count}')
                q = _array(view.get_dof_positions())
                dq = _array(view.get_dof_velocities())
                meta = view.get_metatype(0)
                if q.shape != dq.shape or q.shape != (1, int(meta.dof_count)):
                    raise RuntimeError(f'Unexpected q/dq dimensions: {q.shape}, {dq.shape}')
                if not np.isfinite(q).all() or not np.isfinite(dq).all():
                    raise RuntimeError('Nonfinite initial native q/dq')
                self.views[path] = view
                self.metadata['native_articulations'][path] = {
                    'native_paths': list(view.prim_paths),
                    'dof_names': list(meta.dof_names), 'dof_paths': list(view.dof_paths[0]),
                    'link_names': list(meta.link_names), 'link_paths': list(view.link_paths[0]),
                    'dof_count': int(meta.dof_count),
                }
            except Exception as error:
                self.metadata['initialization_errors'].append({'path': path, 'error': repr(error)})
        self.metadata['all_authored_roots_read'] = (
            bool(self.metadata['authored_articulation_roots']) and not self.metadata['initialization_errors'])

    def sample(self):
        observations, errors = {}, []
        for path, view in self.views.items():
            try:
                q, dq = _array(view.get_dof_positions()), _array(view.get_dof_velocities())
                expected = (1, self.metadata['native_articulations'][path]['dof_count'])
                if q.shape != expected or dq.shape != expected or not np.isfinite(q).all() or not np.isfinite(dq).all():
                    raise RuntimeError('Invalid native q/dq sample')
                observations[path] = {'q': q[0].tolist(), 'dq': dq[0].tolist()}
            except Exception as error:
                errors.append({'path': path, 'error': repr(error)})
        return {'articulations': observations, 'read_errors': errors}
