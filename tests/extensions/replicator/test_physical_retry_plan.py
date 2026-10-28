import unittest
import numpy as np

from kitchen_safety.extensions.replicator.physical_retry_plan import (
    PHASES, DT, ENDPOINT_HOLDS, retime_candidate, validate_candidate, fk_batch)


def fixture():
    # Non-collinear corners and abrupt source speed changes expose accidental
    # shortcuts and acceleration-only bugs that a straight uniform path misses.
    q=np.zeros((12,7),dtype=np.float32)
    q[:,0]=[0,.11,.11,-.21,-.2,-.01,.3,.3,.29,.1,.1,-.2]
    q[:,1]=[0,0,.1,.11,.2,.2,.22,.1,.1,-.13,.2,.19]
    phase=np.repeat(PHASES,2)
    grip=np.where(np.isin(phase,['lift','transfer','lower']),.57,0.)
    return dict(q=q,phase=phase,gripper=grip,dt=np.asarray(DT),
        tcp_world=np.broadcast_to(np.eye(4),(12,4,4)).copy())


class RetimingTests(unittest.TestCase):
    def test_bounds_original_vertices_and_no_corner_cutting(self):
        original=fixture(); source_q=original['q'].copy()
        candidate,check=retime_candidate(original)
        self.assertTrue(check['passed'])
        self.assertLessEqual(check['float32_command_bounds']['peak_joint_speed_rad_s'],.4)
        self.assertLessEqual(check['float32_command_bounds']['peak_joint_acceleration_rad_s2'],.8)
        np.testing.assert_array_equal(candidate['q'][candidate['source_vertex_output_index']],source_q)
        np.testing.assert_array_equal(original['q'],source_q)
        self.assertGreater(len(candidate['q']),len(source_q))
        np.testing.assert_array_equal(candidate['gripper'],original['gripper'][candidate['source_edge_end_index']])

    def test_deterministic_single_candidate_and_exact_holds(self):
        source=fixture(); a,_=retime_candidate(source); b,_=retime_candidate(source)
        for key in a: np.testing.assert_array_equal(a[key],b[key])
        for phase,seconds in ENDPOINT_HOLDS.items():
            old_end=np.flatnonzero(source['phase']==phase)[-1]
            end=a['source_vertex_output_index'][old_end]
            count=int(np.ceil(seconds/DT))
            np.testing.assert_array_equal(a['q'][end+1:end+count+1],np.repeat(source['q'][old_end][None],count,axis=0))
            self.assertTrue(np.all(a['phase'][end+1:end+count+1]==phase))

    def test_mutated_geometry_and_grip_rejected(self):
        source=fixture(); candidate,_=retime_candidate(source)
        candidate['q'][10,2]+=.001
        with self.assertRaisesRegex(ValueError,'Geometric'): validate_candidate(source,candidate)
        candidate,_=retime_candidate(source); candidate['gripper'][10]=.8
        with self.assertRaisesRegex(ValueError,'gripper'): validate_candidate(source,candidate)

    def test_bad_dt_phase_alignment_and_nonfinite_rejected(self):
        for mutate in [lambda s:s.update(dt=np.asarray(1/15)),
                       lambda s:s.update(gripper=s['gripper'][:-1]),
                       lambda s:s['q'].__setitem__((0,0),np.nan),
                       lambda s:s['phase'].__setitem__(0,'retreat')]:
            source=fixture(); mutate(source)
            with self.assertRaises(ValueError): retime_candidate(source)

    def test_duplicate_vertices_and_zero_length_edges_retained(self):
        source=fixture(); source['q'][:]=source['q'][0]
        candidate,check=retime_candidate(source)
        self.assertEqual(check['peak_joint_speed_rad_s'],0.)
        self.assertEqual(len(candidate['source_vertex_output_index']),len(source['q']))
        np.testing.assert_array_equal(candidate['q'][candidate['source_vertex_output_index']],source['q'])

    def test_batch_fk_uses_rotations_not_pose_interpolation(self):
        class Kin: pass
        kin=Kin(); kin.base=np.eye(4); kin.chain=[]
        for i in range(9):
            a,b=np.eye(4),np.eye(4); a[1,3]=.05
            kin.chain.append((a,b,str(i)))
        q=fixture()['q']; actual=fk_batch(kin,q)
        from scipy.spatial.transform import Rotation
        for row,tcp in zip(q,actual):
            expected=kin.base.copy()
            for i,(a,b,_) in enumerate(kin.chain):
                r=np.eye(4)
                if i<7:r[:3,:3]=Rotation.from_euler('x',float(row[i])).as_matrix()
                expected=expected@a@r@b
            expected[:3,3]+=expected[:3,:3]@np.array([.131,0,0])
            np.testing.assert_allclose(tcp,expected,atol=1e-12,rtol=0)


if __name__=='__main__':unittest.main()
