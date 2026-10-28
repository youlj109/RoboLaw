"""CPU invariants for measured-pose targets and honest neutral gating."""
import json,tempfile,unittest
from pathlib import Path
from types import SimpleNamespace
import numpy as np
from scipy.spatial.transform import Rotation
from kitchen_safety.extensions.replicator.mustard_family_controller import grasp_target,transport_target,require_neutral


class MustardControllerTests(unittest.TestCase):
    def test_grasp_uses_actual_bottle_frame(self):
        local=np.eye(4);local[:3,3]=[.001,.031,0.]
        profile=dict(bottle_to_tcp_local=local.tolist(),selection='fixture')
        prepared=dict(profile=dict(source_bottle_geometry=dict(mesh_bounds_m=[[-.03,-.04,-.08],[.03,.04,.08]],safe_contact_z_range_m=[-.06,.02])))
        origin=np.array([.5,-1.,.9]);quat=Rotation.from_euler('xyz',[2,-1,90],degrees=True).as_quat()
        target,info=grasp_target(np.r_[origin,quat],profile,prepared)
        np.testing.assert_allclose(target[:3,3],origin+Rotation.from_quat(quat).apply(local[:3,3]))
        self.assertEqual(info['contact_region_local_m'][1][2],.02)
        self.assertFalse(info['physical_contact_verified'])

    def test_transport_height_follows_source_bowl_rim_and_carried_bottom(self):
        bottle=np.r_[[0.,0.,1.],[0,0,0,1]];bowl=np.r_[[.3,.4,1.5],[0,0,0,1]]
        state=dict(task_object_states=np.array([np.r_[bottle,np.zeros(6)],np.r_[bowl,np.zeros(6)]]))
        payload=SimpleNamespace(vertices=np.array([[0,0,-.08],[0,0,.08]]))
        destination=SimpleNamespace(vertices=np.array([[0,0,-.01],[0,0,.04]]))
        target,info=transport_target(state,np.eye(4),0,1,payload,destination,.05)
        np.testing.assert_allclose(target[:3,3],[.3,.4,1.67]);self.assertAlmostEqual(info['source_bowl_rim_world_z_m'],1.54)
        with self.assertRaises(ValueError):transport_target(state,np.eye(4),0,1,payload,destination,.01)

    def test_cpu_success_or_short_neutral_does_not_authorize_task_run(self):
        from kitchen_safety.extensions.replicator.mustard_family_preparation import sha
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory);profile=path/'profile.json';profile.write_text('{}')
            record=dict(task_id='task',controller_profile_sha256=sha(profile),neutral_probe_passed=False,neutral_hold_seconds=45.)
            (path/'result.json').write_text(json.dumps(record))
            with self.assertRaises(ValueError):require_neutral(path,profile,'task')
            record.update(neutral_probe_passed=True,neutral_hold_seconds=44.99);(path/'result.json').write_text(json.dumps(record))
            with self.assertRaises(ValueError):require_neutral(path,profile,'task')


if __name__=='__main__':unittest.main()
