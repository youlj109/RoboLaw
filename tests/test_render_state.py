"""CPU-only inventory checks: instance proxies and honest backend omissions."""
import json
from types import SimpleNamespace
import unittest

import numpy as np
from pxr import Usd, UsdGeom, UsdPhysics

from kitchen_safety.render_state import KitchenPhysicsState, read_kitchen_physics_inventory


class RenderStateInventoryTest(unittest.TestCase):
    def stage(self):
        stage = Usd.Stage.CreateInMemory()
        asset = UsdGeom.Xform.Define(stage, '/Cabinet').GetPrim()
        UsdPhysics.ArticulationRootAPI.Apply(asset)
        for name in ('base', 'door'):
            prim = UsdGeom.Xform.Define(stage, '/Cabinet/' + name).GetPrim()
            UsdPhysics.RigidBodyAPI.Apply(prim).CreateRigidBodyEnabledAttr(True)
        root = '/World/envs/env_0/kitchen'
        for name in ('cab1', 'cab2'):
            prim = UsdGeom.Xform.Define(stage, root + '/' + name).GetPrim()
            prim.GetReferences().AddInternalReference('/Cabinet')
            prim.SetInstanceable(True)
        return stage, root

    def test_instance_proxy_scan_and_registered_union_do_not_hide_missing_bodies(self):
        stage, root = self.stage()
        base, door = root + '/cab1/base', root + '/cab1/door'
        group = root + '/cab1'
        identity = np.array([0., 0., 0., 0., 0., 0., 1.])
        # PhysX can name the root through its group while the articulation
        # link list names the child body. Count identifiers, not three bodies.
        rigid = SimpleNamespace(count=1, prim_paths=[group],
                                get_transforms=lambda: identity[None, :],
                                get_velocities=lambda: np.zeros((1, 6)))
        articulation = SimpleNamespace(
            count=1, prim_paths=[group], link_paths=[[base, door]], dof_paths=[[root + '/cab1/hinge']],
            max_links=2, max_dofs=1, is_homogeneous=True,
            get_root_transforms=lambda: identity[None, :], get_root_velocities=lambda: np.zeros((1, 6)),
            get_link_transforms=lambda: np.tile(identity, (1, 2, 1)),
            get_link_velocities=lambda: np.zeros((1, 2, 6)),
            get_dof_positions=lambda: np.array([[.25]]), get_dof_velocities=lambda: np.zeros((1, 1)))
        calls = []
        def create_articulation(pattern):
            calls.append(pattern)
            return articulation
        sim_view = SimpleNamespace(create_rigid_body_view=lambda pattern: rigid,
                                   create_articulation_view=create_articulation)
        report = read_kitchen_physics_inventory(stage, sim_view)
        self.assertEqual(calls, [root + '/**', [group]])
        self.assertEqual(report['authored_rigid_body_count'], 4)
        self.assertTrue(all(row['instance_proxy'] for row in report['authored_rigid_bodies']))
        self.assertEqual(report['authored_articulation_root_count'], 2)
        self.assertEqual(report['backend_rigid_view_unique_body_count'], 1)
        self.assertEqual(report['backend_articulation_unique_link_count'], 2)
        self.assertEqual(report['backend_combined_unique_body_path_count'], 3)
        self.assertNotIn('backend_combined_unique_body_count', report)
        self.assertEqual(report['backend_body_paths_absent_from_usd_inventory'], [group])
        self.assertEqual(report['authored_dynamic_body_paths_absent_from_backend'],
                         [root + '/cab2/base', root + '/cab2/door'])
        self.assertEqual(report['backend_articulations'][0]['state']['joint_position'], [[.25]])
        self.assertEqual(report['query_errors'], [])
        self.assertFalse(report['complete_visual_state_restoration_tested'])

    def test_query_failures_are_preserved_and_never_reported_as_static(self):
        stage, root = self.stage()
        def unsupported(pattern):
            raise RuntimeError('Backend view unavailable')
        view = SimpleNamespace(create_rigid_body_view=unsupported, create_articulation_view=unsupported)
        report = read_kitchen_physics_inventory(stage, view, kitchen_roots=[root], read_state=False)
        self.assertEqual(len(report['query_errors']), 2)
        self.assertEqual(report['authored_enabled_nonkinematic_body_count'], 4)
        self.assertEqual(len(report['authored_dynamic_body_paths_absent_from_backend']), 4)
        self.assertEqual(report['backend_combined_unique_body_path_count'], 0)


class FakeBatchedArticulation:
    def __init__(self, paths, origins, dofs):
        self.prim_paths = list(reversed(paths))
        self.count, self.max_links, self.max_dofs = len(paths), 2 if dofs else 1, dofs
        self.is_homogeneous = True
        self.link_paths = [[path + '/' + name for name in (['base', 'door'] if dofs else ['panel'])]
                           for path in self.prim_paths]
        # Two distinct DOF vector entries deliberately share one joint path.
        self.dof_paths = [[path + '/hinge'] * dofs for path in self.prim_paths]
        self.root = np.tile([0., 0., 0., 0., 0., 0., 1.], (self.count, 1)).astype(np.float32)
        self.root_vel = np.empty((self.count, 6), dtype=np.float32)
        self.q = np.empty((self.count, dofs), dtype=np.float32)
        for row, path in enumerate(self.prim_paths):
            env_id = int(path.split('/env_')[1].split('/')[0])
            self.root[row, :3] = origins[env_id] + [1. + env_id, 2., 3.]
            self.root_vel[row] = env_id + .25
            self.q[row] = np.arange(dofs) + 10 * env_id
        self.dq = self.q + .125
        self.links = np.tile(self.root[:, None, :], (1, self.max_links, 1))
        self.links[:, -1, 2] += .5
        self.link_vel = np.tile(self.root_vel[:, None, :], (1, self.max_links, 1))
        self.writes = []

    def get_root_transforms(self): return self.root
    def get_root_velocities(self): return self.root_vel
    def get_link_transforms(self): return self.links
    def get_link_velocities(self): return self.link_vel
    def get_dof_positions(self): return self.q
    def get_dof_velocities(self): return self.dq

    def write(self, name, data, indices):
        np.testing.assert_array_equal(indices, np.arange(self.count))
        self.writes.append(name)
        getattr(self, name)[:] = data

    def set_root_transforms(self, data, indices): self.write('root', data, indices)
    def set_root_velocities(self, data, indices): self.write('root_vel', data, indices)
    def set_dof_positions(self, data, indices): self.write('q', data, indices)
    def set_dof_velocities(self, data, indices): self.write('dq', data, indices)


class KitchenPhysicsStateTest(unittest.TestCase):
    def make_reader(self, count=2, *, free_body=False, heterogeneous=False):
        stage = Usd.Stage.CreateInMemory()
        origins = np.array([[10. * i, -5. * i, 2. * i] for i in range(count)], dtype=np.float32)
        for i in range(count):
            root = f'/World/envs/env_{i}/kitchen'
            for group, names in [('cabinet', ['base', 'door']), ('panel_group', ['panel'])]:
                parent = UsdGeom.Xform.Define(stage, root + '/' + group).GetPrim()
                UsdPhysics.ArticulationRootAPI.Apply(parent)
                for name in names:
                    child = UsdGeom.Xform.Define(stage, root + '/' + group + '/' + name).GetPrim()
                    UsdPhysics.RigidBodyAPI.Apply(child).CreateRigidBodyEnabledAttr(True)
            if free_body:
                UsdPhysics.RigidBodyAPI.Apply(UsdGeom.Xform.Define(stage, root + '/loose').GetPrim())
        views = {}
        def create(paths):
            if isinstance(paths, str):
                return SimpleNamespace(prim_paths=['/World/envs/env_0/kitchen/panel_group',
                                                   '/World/envs/env_0/kitchen/cabinet'])
            group = paths[0].rsplit('/', 1)[1]
            view = FakeBatchedArticulation(paths, origins, 2 if group == 'cabinet' else 0)
            view.is_homogeneous = not heterogeneous
            views[group] = view
            return view
        sim_view = SimpleNamespace(create_articulation_view=create)
        reader = KitchenPhysicsState(stage, sim_view, origins, 'cpu')
        reader._to_backend = lambda value, **kwargs: np.ascontiguousarray(value)
        return reader, views

    def test_capture_orders_env_rows_localizes_xyz_and_keeps_duplicate_dofs(self):
        reader, views = self.make_reader()
        state = reader.capture()
        self.assertEqual(state['kitchen_root_state'].shape, (2, 2, 13))
        self.assertEqual(state['kitchen_link_pose'].shape, (2, 3, 7))
        np.testing.assert_array_equal(state['kitchen_root_state'][:, 0, :3], [[1., 2., 3.], [2., 2., 3.]])
        np.testing.assert_array_equal(state['kitchen_joint_pos'], [[0., 1.], [10., 11.]])
        np.testing.assert_array_equal(state['kitchen_root_state'][:, 0, 7:], [[.25] * 6, [1.25] * 6])
        self.assertEqual(reader.metadata['joint_paths'], ['kitchen/cabinet/hinge'] * 2)
        self.assertEqual(reader.metadata['groups'][1]['joint_count'], 0)
        views['cabinet'].root[:] = 0
        self.assertEqual(state['kitchen_root_state'][0, 0, 0], 1.)
        single, _ = self.make_reader(1)
        self.assertEqual(single.metadata, reader.metadata)
        json.dumps(reader.metadata)

    def test_restore_writes_inverse_env_order_and_does_not_mutate_input_or_links(self):
        reader, views = self.make_reader()
        state = reader.capture()
        preserved = {key: value.copy() for key, value in state.items()}
        expected = {name: (view.root.copy(), view.root_vel.copy(), view.q.copy(), view.dq.copy())
                    for name, view in views.items()}
        for view in views.values():
            view.root[:] = 0
            view.root_vel[:] = 0
            view.q[:] = 0
            view.dq[:] = 0
        reader.restore(state)
        for name, view in views.items():
            for actual, wanted in zip((view.root, view.root_vel, view.q, view.dq), expected[name]):
                np.testing.assert_array_equal(actual, wanted)
        self.assertEqual(views['cabinet'].writes, ['root', 'root_vel', 'q', 'dq'])
        self.assertEqual(views['panel_group'].writes, ['root', 'root_vel'])
        for key in state:
            np.testing.assert_array_equal(state[key], preserved[key])

    def test_unsupported_free_body_and_heterogeneous_batch_are_not_dropped(self):
        with self.assertRaisesRegex(ValueError, 'outside supported articulation state coverage'):
            self.make_reader(free_body=True)
        with self.assertRaisesRegex(ValueError, 'not homogeneous'):
            self.make_reader(heterogeneous=True)


if __name__ == '__main__':
    unittest.main()
