from pathlib import Path
import unittest
from safety_risk.iso_traceable import Evaluator,load_json
from safety_risk.observation_cadence import build_index,physics_stride


class ObservationCadenceTests(unittest.TestCase):
    def fixture(self):
        rows=[]
        for i in range(9):
            rows.append({"acquisition_sequence":i//4+1,"acquisition_epoch":0,"capture_clock_consistent":True,
                "capture_sim_time_s":i//4/30,"age_at_read_s":i%4/120})
        raw={"metadata":{"num_steps":9,"physics_frame_clocks":[{"before_sim_time_s":i/120,"after_sim_time_s":(i+1)/120} for i in range(9)]},
            "episode_meta":{"physics_config":{"physics_dt_s":1/120}},
            "camera_timeline":{"franka_head":rows,"franka_hand":rows},
            "collision_gt":{"collision_pair_gt":[[] for i in range(9)]}}
        evaluator=Evaluator(load_json(Path(__file__).resolve().parents[1]/"configs/iso_profile.experimental.json"))
        return raw,evaluator

    def test_labels_only_at_observation_rate_and_never_use_future_state(self):
        raw,e=self.fixture();calls=[];original=e.evaluate
        def spy(x):calls.append(x["frame_index"]);return original(x)
        e.evaluate=spy
        result=build_index(raw,e)
        self.assertEqual(calls,[None,3,7]);self.assertEqual(result["observation_frames"],3)
        samples=result["samples"]
        self.assertEqual([s["decision_physics_step_index"] for s in samples],[0,4,8])
        self.assertEqual([i for s in samples for i in s["action_physics_step_indices"]],list(range(9)))
        self.assertTrue(all(v is None for v in samples[0]["risk_report"]["risk_levels"].values()))

    def test_short_contact_between_camera_frames_is_retained_as_consequence(self):
        raw,e=self.fixture();raw["collision_gt"]["collision_pair_gt"][2]=[{"bodyA":"robot/link","bodyB":"human/hand"}]
        result=build_index(raw,e)
        sample=result["samples"][0]
        self.assertEqual(sample["interval_event_evidence"]["human_contact_frames"],[2])
        self.assertEqual(sample["risk_input"]["features"],{})

    def test_stale_camera_does_not_request_duplicate_visual_inference(self):
        raw,e=self.fixture();raw["camera_timeline"]["franka_head"][4]["acquisition_sequence"]=1
        sample=build_index(raw,e)["samples"][1]
        self.assertFalse(sample["visual_inference_required"])
        self.assertEqual(sample["policy_views"]["franka_head"]["status"],"stale")

    def test_nonintegral_rate_and_missing_axes_fail_explicitly(self):
        with self.assertRaises(ValueError):physics_stride(1/120,31)
        raw,e=self.fixture();raw["collision_gt"]["collision_pair_gt"].pop()
        with self.assertRaises(ValueError):build_index(raw,e)
