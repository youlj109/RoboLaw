"""CPU checks for camera evidence; no Kit or GPU is started."""
import unittest
import numpy as np

from scripts.extensions.replicator.visual_scene_camera import measure_visibility, multi_view_summary
from scripts.extensions.replicator.visual_scene_previews import exposure_override
from scripts.extensions.replicator.visual_camera_override import validate_camera_overrides, override_fingerprint


class VisualSceneTests(unittest.TestCase):
    def test_camera_override_is_observation_only_and_bound(self):
        camera=dict(kind='operation_side',eye=[1.,0.,2.],target=[0.,0.,1.],focal_length_mm=30,exposure_ev=-6)
        self.assertTrue(validate_camera_overrides([camera])[0]['inspection_override'])
        for invalid in ({**camera,'hide_robot':True},{**camera,'exposure_ev':-7},{**camera,'eye':[0.,0.,1.]},
                        {**camera,'target':[float('nan'),0,1]}):
            with self.assertRaises(ValueError):validate_camera_overrides([invalid])
        self.assertNotEqual(override_fingerprint({'camera_overrides':[camera]}),
                            override_fingerprint({'camera_overrides':[{**camera,'exposure_ev':-4}]}))
        self.assertIsNone(override_fingerprint({}))

    def test_exposure_adjustment_preserves_input_and_disables_clamp_bypass(self):
        original = {'/rtx/post/tonemap/exposureTime': .02, '/rtx/post/tonemap/op': 0,
                    '/rtx/post/histogram/enabled': True}
        adjusted = exposure_override(original, -2)
        self.assertEqual(adjusted['/rtx/post/tonemap/exposureTime'], .005)
        self.assertEqual(adjusted['/rtx/post/tonemap/op'], 1)
        self.assertFalse(adjusted['/rtx/post/histogram/enabled'])
        self.assertTrue(original['/rtx/post/histogram/enabled'])
        for value in (None, 0, -1):
            with self.assertRaises(ValueError):
                exposure_override({**original, '/rtx/post/tonemap/exposureTime': value}, -2)

    def test_thin_fragment_fails_even_with_more_than_450_pixels(self):
        data = np.zeros((100, 200), dtype=np.int32)
        data[20:25, 20:180] = 1
        segmentation = {'data': data, 'info': {'idToLabels': {'1':'/World/hazard/Mesh'}}}
        entities = {'hazard': {'prefix':'/World/hazard', 'bounds':[[-.1,-.1,-.1],[.1,.1,.1]], 'minimum_pixels':450}}
        camera = {'eye':[0.,-2.,1.], 'target':[0.,0.,0.], 'focal_length_mm':24.}
        result = measure_visibility(segmentation, np.zeros((100,200,4)), entities, camera, 200, 100)
        self.assertGreater(result['hazard']['pixels'],450)
        self.assertFalse(result['hazard']['readable_pixels'])
        data[20:50,20:50] = 1
        self.assertTrue(measure_visibility(segmentation,np.zeros((100,200,4)),entities,camera,200,100)['hazard']['readable_pixels'])

    def test_multiview_needs_every_entity_and_three_view_kinds(self):
        entities={'hazard':{},'object:banana':{}}
        views=[{'view_id':str(i),'kind':kind,'rgb_valid':True,'visibility':{'hazard':{'readable_pixels':i==0},'object:banana':{'readable_pixels':i==2}}}
               for i,kind in enumerate(('overview','operation_side','operation_top'))]
        self.assertTrue(multi_view_summary(views,entities)['readability_passed'])
        self.assertFalse(multi_view_summary(views[:2],entities)['readability_passed'])
        views[2]['visibility']['object:banana']['readable_pixels']=False
        self.assertFalse(multi_view_summary(views,entities)['readability_passed'])


if __name__ == '__main__':
    unittest.main()
