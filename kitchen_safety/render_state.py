"""Nested kitchen PhysX state inventory, capture and explicit restoration.

Call after reset with the existing Arena SimulationView (subspace root ``/``).
The inventory function is read-only. KitchenPhysicsState.restore explicitly
writes measured root/joint states; neither API advances physics or calls FK.
Actual view counts and missing paths are separate from authored USD schemas.
"""
import numpy as np


def _array(value):
    value = value.torch if hasattr(value, 'torch') else value
    if hasattr(value, 'detach'):
        return value.detach().cpu().numpy().copy()
    if hasattr(value, 'numpy'):
        return np.asarray(value.numpy()).copy()
    return np.asarray(value).copy()


def _under(path, root):
    return path == root or path.startswith(root.rstrip('/') + '/')


class KitchenPhysicsState:
    """Persist one homogeneous articulation view per kitchen-relative path.

    The existing Arena Warp SimulationView must use subspace root ``/``.
    Array rows follow environment indices, independently of backend view order.
    Metadata contains relative paths and native DOF indices, so a multi-env
    source and a single-env renderer can compare it directly. Duplicate DOF
    paths are retained: e.g. the sink handle has two DOFs on the same joint.

    This implementation supports articulated kitchens. Any enabled kitchen
    rigid body outside the articulation links raises an ordinary ValueError,
    rather than silently dropping a free body's state. No physical properties
    or camera settings are modified.
    """

    def __init__(self, stage, physics_sim_view, env_origins, device):
        from pxr import Usd, UsdPhysics

        self.origins = _array(env_origins).astype(np.float32, copy=False)
        if self.origins.ndim != 2 or self.origins.shape[1] != 3 or not len(self.origins) or not np.isfinite(self.origins).all():
            raise ValueError('env_origins must be finite [N,3] positions')
        self.num_envs = len(self.origins)
        self.device = str(device)
        self._groups = []
        self._write_indices = None
        env_paths = [f'/World/envs/env_{i}' for i in range(self.num_envs)]
        kitchen_paths = [path + '/kitchen' for path in env_paths]
        enabled_bodies = []
        for root, env_path in zip(kitchen_paths, env_paths):
            prim = stage.GetPrimAtPath(root)
            if not prim:
                raise ValueError(f'Kitchen root missing: {root}')
            bodies = set()
            for child in Usd.PrimRange(prim, Usd.TraverseInstanceProxies()):
                if child.HasAPI(UsdPhysics.RigidBodyAPI) and UsdPhysics.RigidBodyAPI(child).GetRigidBodyEnabledAttr().Get():
                    bodies.add(str(child.GetPath())[len(env_path) + 1:])
            enabled_bodies.append(bodies)
        if any(paths != enabled_bodies[0] for paths in enabled_bodies[1:]):
            raise ValueError('Environment kitchens have different enabled rigid-body paths')

        discovery = physics_sim_view.create_articulation_view(kitchen_paths[0] + '/**')
        first_paths = list(discovery.prim_paths)
        if not first_paths or any(not _under(path, kitchen_paths[0]) for path in first_paths):
            raise ValueError('Kitchen articulation discovery is empty or outside its requested subtree')
        relatives = sorted(path[len(env_paths[0]) + 1:] for path in first_paths)
        if len(set(relatives)) != len(relatives):
            raise ValueError('Duplicate kitchen articulation paths')
        group_metadata, all_links, all_dofs = [], [], []
        joint_start = link_start = 0
        for relative in relatives:
            requested = [path + '/' + relative for path in env_paths]
            view = physics_sim_view.create_articulation_view(requested)
            actual = list(view.prim_paths)
            if int(view.count) != self.num_envs or len(set(actual)) != self.num_envs or set(actual) != set(requested):
                raise ValueError(f'{relative}: backend view does not select exactly one articulation per environment')
            if not view.is_homogeneous:
                raise ValueError(f'{relative}: backend articulation view is not homogeneous')
            # Capture reorders backend rows into env order; restore applies its
            # inverse. Explicit path requests alone do not guarantee SDK order.
            env_to_view = np.asarray([actual.index(path) for path in requested], dtype=np.int64)
            view_to_env = np.argsort(env_to_view)
            links_by_row, dofs_by_row = list(view.link_paths), list(view.dof_paths)
            link_count, joint_count = int(view.max_links), int(view.max_dofs)
            relative_links, relative_dofs = [], []
            for env_index, row in enumerate(env_to_view):
                prefix = env_paths[env_index] + '/'
                links, dofs = list(links_by_row[row]), list(dofs_by_row[row])
                if len(links) != link_count or len(dofs) != joint_count or any(
                        not path.startswith(prefix) for path in links + dofs):
                    raise ValueError(f'{relative}: invalid link/DOF paths or counts')
                relative_links.append([path[len(prefix):] for path in links])
                relative_dofs.append([path[len(prefix):] for path in dofs])
            if any(row != relative_links[0] for row in relative_links[1:]) or any(
                    row != relative_dofs[0] for row in relative_dofs[1:]):
                raise ValueError(f'{relative}: link/DOF order differs between environments')
            metadata = {'relative_path': relative, 'link_start': link_start, 'link_count': link_count,
                        'joint_start': joint_start, 'joint_count': joint_count,
                        'link_paths': relative_links[0], 'joint_paths': relative_dofs[0]}
            group_metadata.append(metadata)
            self._groups.append({'view': view, 'env_to_view': env_to_view, 'view_to_env': view_to_env,
                                 'links': slice(link_start, link_start + link_count),
                                 'joints': slice(joint_start, joint_start + joint_count),
                                 'joint_count': joint_count})
            all_links.extend(relative_links[0])
            all_dofs.extend(relative_dofs[0])
            link_start += link_count
            joint_start += joint_count
        if len(set(all_links)) != len(all_links):
            raise ValueError('Kitchen articulation views overlap in body links')
        missing = enabled_bodies[0] - set(all_links)
        if missing:
            raise ValueError('Enabled kitchen bodies outside supported articulation state coverage: ' + ', '.join(sorted(missing)))
        self._shapes = {'kitchen_root_state': (len(self._groups), 13),
                        'kitchen_joint_pos': (joint_start,), 'kitchen_joint_vel': (joint_start,),
                        'kitchen_link_pose': (link_start, 7), 'kitchen_link_velocity': (link_start, 6)}
        self.metadata = {
            'coordinate_frame': 'environment_local', 'quaternion_order': 'xyzw',
            'velocity_frame': 'world axes; linear velocity at body center of mass',
            'path_reference': 'Relative to environment root; starts with kitchen/',
            'joint_units': 'radians for angular DOFs, metres for linear DOFs; corresponding units per second',
            'joint_identity': 'articulation group plus native DOF vector index; paths may repeat',
            'articulation_count': len(self._groups), 'joint_count': joint_start, 'link_count': link_start,
            'enabled_usd_rigid_body_count': len(enabled_bodies[0]),
            'support_scope': 'All enabled kitchen rigid bodies must be covered by articulation links; free rigid bodies are unsupported and raise an error',
            'groups': group_metadata, 'joint_paths': all_dofs, 'link_paths': all_links,
            'array_shapes_after_environment_axis': {name: list(shape) for name, shape in self._shapes.items()},
            'restore_semantics': 'Writes root pose/velocity and DOF position/velocity. Link states are measured comparison targets; caller must forward/update/render without stepping physics.'}

    def capture(self):
        """Return copied measured arrays in environment order and local xyz."""
        result = {name: np.empty((self.num_envs,) + shape, dtype=np.float32) for name, shape in self._shapes.items()}
        for index, group in enumerate(self._groups):
            view, order = group['view'], group['env_to_view']
            result['kitchen_root_state'][:, index, :7] = _array(view.get_root_transforms())[order]
            result['kitchen_root_state'][:, index, 7:] = _array(view.get_root_velocities())[order]
            result['kitchen_link_pose'][:, group['links']] = _array(view.get_link_transforms())[order]
            result['kitchen_link_velocity'][:, group['links']] = _array(view.get_link_velocities())[order]
            if group['joint_count']:
                result['kitchen_joint_pos'][:, group['joints']] = _array(view.get_dof_positions())[order]
                result['kitchen_joint_vel'][:, group['joints']] = _array(view.get_dof_velocities())[order]
        result['kitchen_root_state'][..., :3] -= self.origins[:, None, :]
        result['kitchen_link_pose'][..., :3] -= self.origins[:, None, :]
        for name, value in result.items():
            if not np.isfinite(value).all():
                raise ValueError(f'Nonfinite measured kitchen state: {name}')
        return result

    def _to_backend(self, value, *, indices=False):
        # The local Arena PhysicsManager creates a Warp SimulationView. Its
        # frontend accepts Warp arrays, not plain numpy/torch in set_* calls.
        import warp as wp
        return wp.array(np.ascontiguousarray(value), dtype=wp.int32 if indices else wp.float32,
                        device=self.device)

    def restore(self, states):
        """Write measured root/joint state only; caller performs FK and render."""
        arrays = {}
        for name, shape in self._shapes.items():
            value = np.asarray(states[name], dtype=np.float32)
            if value.shape != (self.num_envs,) + shape or not np.isfinite(value).all():
                raise ValueError(f'{name}: expected finite {(self.num_envs,) + shape}, got {value.shape}')
            arrays[name] = value
        if self._write_indices is None:
            self._write_indices = self._to_backend(np.arange(self.num_envs, dtype=np.int32), indices=True)
        # Retain setter input buffers until the next restore; asynchronous
        # backend copies must not outlive their temporary storage.
        self._restore_buffers = []
        for index, group in enumerate(self._groups):
            view, order = group['view'], group['view_to_env']
            root = arrays['kitchen_root_state'][:, index].copy()
            root[:, :3] += self.origins
            writes = [(view.set_root_transforms, root[order, :7]),
                      (view.set_root_velocities, root[order, 7:])]
            if group['joint_count']:
                writes.extend([(view.set_dof_positions, arrays['kitchen_joint_pos'][order, group['joints']]),
                               (view.set_dof_velocities, arrays['kitchen_joint_vel'][order, group['joints']])])
            for setter, values in writes:
                buffer = self._to_backend(values)
                self._restore_buffers.append(buffer)
                setter(buffer, self._write_indices)


def read_kitchen_physics_inventory(stage, physics_sim_view, *, kitchen_roots=None, read_state=True):
    """Return JSON-ready USD/PhysX coverage and optionally one measured snapshot.

    ``physics_sim_view`` is ``env.scene['robot']._physics_sim_view`` in the
    checked-out Arena/Isaac Lab version. Its subspace root must remain ``/``;
    this helper never changes it. Getter output is thus world xyz/xyzw, with
    angular DOFs in radians and linear DOFs in metres. The returned state is
    diagnostic; sustained capture should retain the discovered views and store
    arrays with their path ordering, instead of rebuilding views every frame.

    The discovery articulation view may be heterogeneous. State reads use one
    view per discovered root, avoiding the SDK's homogeneous-view restriction.
    Query failures remain visible and do not turn missing bodies into static
    bodies. No forward-kinematics recomputation is requested by this function.
    """
    from pxr import Usd, UsdPhysics

    if kitchen_roots is None:
        kitchen_roots = [str(prim.GetPath()) for prim in stage.Traverse() if prim.GetName() == 'kitchen']
    roots = list(dict.fromkeys(str(path).rstrip('/') for path in kitchen_roots))
    if any(not root.startswith('/') for root in roots):
        raise ValueError('Kitchen roots must be absolute USD prim paths')
    if any(_under(a, b) for i, a in enumerate(roots) for j, b in enumerate(roots) if i != j):
        raise ValueError('Kitchen roots must not overlap')
    report = {'read_only': True, 'coordinate_frame': 'world', 'quaternion_order': 'xyzw',
              'simulation_view_subspace_requirement': '/', 'kitchen_roots': roots,
              'authored_rigid_bodies': [], 'authored_articulation_roots': [],
              'backend_rigid_views': [], 'backend_articulations': [], 'query_errors': [],
              'state_read_requested': bool(read_state),
              'state_semantics': 'Current backend getter state; not historical reconstruction and not a claim of full visual coverage'}
    registered_rigid, registered_links, registered_articulations = set(), set(), set()

    def failure(operation, path, error):
        report['query_errors'].append({'operation': operation, 'path': path, 'error': repr(error)})

    def read_values(view, methods, row, path):
        row['state'] = {}
        for key, method in methods.items():
            try:
                array = _array(getattr(view, method)())
                if not np.isfinite(array).all():
                    raise ValueError(f'{method} returned nonfinite values')
                row['state'][key] = array.tolist()
            except Exception as error:
                failure(method, path, error)

    for root in roots:
        prim = stage.GetPrimAtPath(root)
        if not prim:
            failure('USD traversal', root, ValueError('Kitchen root missing'))
            continue
        for child in Usd.PrimRange(prim, Usd.TraverseInstanceProxies()):
            path = str(child.GetPath())
            if child.HasAPI(UsdPhysics.RigidBodyAPI):
                rigid = UsdPhysics.RigidBodyAPI(child)
                report['authored_rigid_bodies'].append({
                    'path': path, 'enabled': bool(rigid.GetRigidBodyEnabledAttr().Get()),
                    'kinematic': bool(rigid.GetKinematicEnabledAttr().Get()),
                    'instance_proxy': bool(child.IsInstanceProxy())})
            if child.HasAPI(UsdPhysics.ArticulationRootAPI):
                report['authored_articulation_roots'].append({
                    'path': path, 'instance_proxy': bool(child.IsInstanceProxy())})
        pattern = root + '/**'
        try:
            rigid_view = physics_sim_view.create_rigid_body_view(pattern)
            paths = list(rigid_view.prim_paths)
            row = {'pattern': pattern, 'count': int(rigid_view.count), 'prim_paths': paths}
            registered_rigid.update(paths)
            report['backend_rigid_views'].append(row)
            if read_state and row['count']:
                read_values(rigid_view, {'pose_xyz_xyzw': 'get_transforms',
                                        'linear_angular_velocity': 'get_velocities'}, row, pattern)
        except Exception as error:
            failure('create_rigid_body_view/read metadata', pattern, error)
        try:
            discovery = physics_sim_view.create_articulation_view(pattern)
            paths = list(discovery.prim_paths)
            registered_articulations.update(paths)
        except Exception as error:
            failure('create_articulation_view/discover roots', pattern, error)
            paths = []
        for path in paths:
            try:
                view = physics_sim_view.create_articulation_view([path])
                actual_paths = list(view.prim_paths)
                links = [list(items) for items in view.link_paths]
                dofs = [list(items) for items in view.dof_paths]
                row = {'path': path, 'count': int(view.count), 'prim_paths': actual_paths,
                       'link_paths': links, 'dof_paths': dofs,
                       'max_links': int(view.max_links), 'max_dofs': int(view.max_dofs),
                       'homogeneous': bool(view.is_homogeneous)}
                report['backend_articulations'].append(row)
                for items in links:
                    registered_links.update(item for item in items if item)
                # Exact backend paths should select one articulation; keep a
                # mismatched selection visible instead of trusting padded data.
                if actual_paths != [path] or row['count'] != 1 or not row['homogeneous']:
                    raise ValueError('Single-articulation path did not select exactly that homogeneous object')
                if read_state:
                    methods = {'root_pose_xyz_xyzw': 'get_root_transforms',
                               'root_linear_angular_velocity': 'get_root_velocities',
                               'link_pose_xyz_xyzw': 'get_link_transforms',
                               'link_linear_angular_velocity': 'get_link_velocities'}
                    if row['max_dofs']:
                        methods.update(joint_position='get_dof_positions', joint_velocity='get_dof_velocities')
                    read_values(view, methods, row, path)
            except Exception as error:
                failure('single articulation metadata/state', path, error)
    actual_bodies = registered_rigid | registered_links
    authored = {row['path'] for row in report['authored_rigid_bodies']}
    enabled_dynamic = {row['path'] for row in report['authored_rigid_bodies'] if row['enabled'] and not row['kinematic']}
    report.update(
        authored_rigid_body_count=len(authored),
        authored_enabled_nonkinematic_body_count=len(enabled_dynamic),
        authored_articulation_root_count=len(report['authored_articulation_roots']),
        backend_rigid_view_unique_body_count=len(registered_rigid),
        backend_articulation_unique_count=len(registered_articulations),
        backend_articulation_unique_link_count=len(registered_links),
        backend_combined_unique_body_path_count=len(actual_bodies),
        backend_combined_path_count_note='This counts unique path identifiers, not independent physical bodies. '
                                        'RigidBodyView may name an articulation root body by its articulation group '
                                        'path while ArticulationView.link_paths names the corresponding link prim. '
                                        'These root-path aliases must not be added as extra physical bodies.',
        authored_body_paths_absent_from_backend=sorted(authored - actual_bodies),
        authored_dynamic_body_paths_absent_from_backend=sorted(enabled_dynamic - actual_bodies),
        backend_body_paths_absent_from_usd_inventory=sorted(actual_bodies - authored),
        backend_articulation_paths=sorted(registered_articulations),
        authored_articulation_roots_without_backend_descendant=[
            row['path'] for row in report['authored_articulation_roots']
            if not any(_under(path, row['path']) for path in registered_articulations)],
        articulation_root_matching_note='USD ArticulationRootAPI may be on an ancestor while PhysX prim_paths names its root link; descendant matching is diagnostic only.',
        restoration_api_note='Free rigid bodies: set_transforms + set_velocities. Articulations: '
                             'set_root_transforms + set_root_velocities + set_dof_positions + set_dof_velocities, '
                             'then sim.forward without sim.step. Preserve link poses for readback comparison; '
                             'do not independently set articulation links through the rigid-body view.',
        complete_visual_state_restoration_tested=False)
    return report
