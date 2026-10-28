"""CPU merge boundaries, risk-byte priority and preview reuse provenance."""
import json
from pathlib import Path
import tempfile
import unittest

from scripts.extensions.replicator.finalize_visual_scenes import digest, merged_rows, reusable_preview


class FinalizeTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory()
        self.root=Path(self.temp.name)
        self.task='kitchen_bench_replicator_g_shape_banana_bowl'

    def tearDown(self):
        self.temp.cleanup()

    def write(self,path,data):
        dest=self.root/path;dest.parent.mkdir(parents=True,exist_ok=True)
        dest.write_text(json.dumps(data));return dest

    def row(self,variant='property_vase'):
        config=self.write('original/'+variant+'.json',{'task_id':self.task,'objects':{'banana':[0,0,1]},'obstacle':{'position':[0,1,1]}})
        evidence=self.write('evidence/'+variant+'.json',{'passed':True})
        return dict(scene_id='g_shape_banana_bowl__'+variant,task_id=self.task,variant_id=variant,
            config_path=str(config.relative_to(self.root)),config_sha256=digest(config),
            original_config_path=str(config.relative_to(self.root)),original_config_sha256=digest(config),
            evidence_path=str(evidence.relative_to(self.root)),evidence_sha256=digest(evidence),
            status='retained',semantic_geometry_passed=False)

    def test_failed_overlay_retains_base_config_and_never_claims_adjusted(self):
        row=self.row()
        overlay={**row,'config_path':None,'status':'needs_repair','reason':'No feasible support'}
        rows,ids=merged_rows(self.root,{'scenes':[row]},[{'entries':[overlay]}],{'entries':[]},1)
        self.assertEqual(rows[row['scene_id']]['config_path'],row['config_path'])
        self.assertEqual(rows[row['scene_id']]['status'],'needs_repair')
        self.assertFalse(rows[row['scene_id']]['accepted'])

    def test_risk_config_has_last_priority_and_matches_bound_evidence(self):
        row=self.row()
        source_bytes=(self.root/row['config_path']).read_bytes()
        changed=json.loads(source_bytes);changed['obstacle']['position']=[1,2,1]
        candidate=self.write('candidate.json',changed)
        overlay={**row,'config_path':'candidate.json','config_sha256':digest(candidate),'status':'visual_candidate','semantic_geometry_passed':True}
        report=self.write('risk.json',{'config_sha256':row['config_sha256'],'geometry_passed':True,'risk_verified':True})
        risk={**row,'risk_verified':True,'evidence_path':'risk.json','evidence_sha256':digest(report)}
        rows,ids=merged_rows(self.root,{'scenes':[row]},[{'entries':[overlay]}],{'entries':[risk]},1)
        selected=rows[row['scene_id']]
        self.assertEqual(selected['config_sha256'],row['config_sha256'])
        self.assertEqual(ids,[row['scene_id']])
        self.assertEqual((self.root/row['config_path']).read_bytes(),source_bytes)
        changed['objects']['banana']=[7,7,7]
        self.write('candidate.json',changed)
        overlay['config_sha256']=digest(candidate)
        with self.assertRaisesRegex(ValueError,'Original task geometry changed'):
            merged_rows(self.root,{'scenes':[row]},[{'entries':[overlay]}],{'entries':[]},1)

    def test_reuse_requires_candidate_current_image_and_loaded_config_hash(self):
        row=self.row()
        image=self.write('preview.png',{'pixels':'fixture'})
        reviewfile=self.write('review.json',{'entries':[]})
        review=dict(visual_status='candidate',preview_path='preview.png',preview_sha256=digest(image),
                    review_path='review.json',review_sha256=digest(reviewfile))
        loaded={'layout_sha256':row['config_sha256'],'preview_sha256':digest(image),'checks':{'simulation_loaded':True}}
        self.write('load.json',loaded)
        scene={'preview_path':'preview.png','evidence_path':'load.json'}
        self.assertIsNotNone(reusable_preview(self.root,row,review,scene))
        self.assertIsNone(reusable_preview(self.root,row,{**review,'visual_status':'uncertain'},scene))
        self.write('load.json',{**loaded,'layout_sha256':'0'*64})
        self.assertIsNone(reusable_preview(self.root,row,review,scene))
        self.write('load.json',loaded)
        self.write('preview.png',{'pixels':'changed'})
        self.assertIsNone(reusable_preview(self.root,row,review,scene))


if __name__=='__main__':
    unittest.main()
