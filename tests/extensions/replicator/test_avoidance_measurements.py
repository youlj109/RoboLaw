"""Causal gates must include early physics and rotation, and actual reset state."""
import sys,tempfile,unittest
from pathlib import Path
import h5py,numpy as np
from scipy.spatial.transform import Rotation
sys.path.insert(0,str(Path(__file__).resolve().parents[3]/'scripts/extensions/replicator'))
from audit_avoidance import hazard_stationarity,initial_state_comparison

class AvoidanceMeasurementTest(unittest.TestCase):
 def test_early_hazard_displacement_and_rotation_cannot_hide_after_settling(self):
  with tempfile.TemporaryDirectory() as tmp:
   root=Path(tmp);poses=np.zeros((50,7));poses[:,6]=1
   def run():
    np.savez(root/'physics_states.npz',hazard_pose=poses,has_hazard_pose=np.ones(50,dtype=bool))
    return hazard_stationarity(root,{},None,None)
   self.assertTrue(run()['passed'])
   poses[5,0]=.003
   self.assertFalse(run()['passed'])
   poses[5,0]=0;poses[5,3:7]=Rotation.from_euler('x',3,degrees=True).as_quat()
   result=run();self.assertFalse(result['passed']);self.assertGreater(result['max_upright_deviation_deg'],2)
 def test_identical_layout_cannot_hide_actual_robot_or_task_object_shift(self):
  with tempfile.TemporaryDirectory() as tmp:
   root=Path(tmp);base=root/'base';out=root/'augmented';base.mkdir();out.mkdir()
   for folder in [base,out]:
    with h5py.File(folder/'trajectory.h5','w') as f:
     f.create_dataset('obs/q',data=np.zeros((31,13)))
     pose=np.zeros((31,7));pose[:,6]=1
     for name in ['mustard','bowl']:f.create_dataset('obs/'+name,data=pose)
     for name in ['robot_body_pose_w','background_rigid_poses']:f.create_dataset('obs/'+name,data=pose[:,None,:])
   self.assertTrue(initial_state_comparison(base,out)['passed'])
   with h5py.File(out/'trajectory.h5','r+') as f:f['obs/q'][0,0]=.01
   self.assertFalse(initial_state_comparison(base,out)['passed'])
   with h5py.File(out/'trajectory.h5','r+') as f:f['obs/q'][0,0]=0;f['obs/bowl'][30,0]=.001
   self.assertFalse(initial_state_comparison(base,out)['passed'])
if __name__=='__main__':unittest.main()
