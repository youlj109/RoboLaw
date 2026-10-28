"""A scene-change authorization cannot silently change task geometry or repeat a pose change."""
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import tempfile
import unittest

from kitchen_safety.extensions.replicator.scene_retry_once import validate_adjustment


class SceneAdjustmentTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        self.old = {'robot_position':[0,0,0], 'objects':{'bowl':{'position':[1,0,1]}},
                    'obstacle':{'position':[1,1,1], 'yaw_deg':0, 'usd_path':'fixed.usd'}}
        self.new = deepcopy(self.old)
        self.new['obstacle']['position'][0] += .1
        self.recipe = {'layout_fixed':False, 'action':'plan', 'scene_adjustment':{'adjustments':1}}
        self.bind()

    def bind(self):
        for name, config in [('source_config',self.old), ('config',self.new)]:
            path = self.root/(name+'.json');path.write_text(json.dumps(config))
            self.recipe['scene_adjustment'][name+'_path'] = path.name
            self.recipe['scene_adjustment'][name+'_sha256'] = hashlib.sha256(path.read_bytes()).hexdigest()

    def test_one_horizontal_hazard_move_is_allowed(self):
        validate_adjustment(self.root,self.recipe,'source_config.json',self.recipe['scene_adjustment']['source_config_sha256'])

    def test_task_robot_asset_and_support_height_changes_rejected(self):
        for group,key,value in [('robot_position',None,[1,0,0]),('objects','bowl',{'position':[2,0,1]}),
                                ('obstacle','usd_path','different.usd'),('obstacle','position',[1.1,1,2])]:
            with self.subTest(group=group,key=key):
                self.new=deepcopy(self.old);self.new['obstacle']['position'][0]+=.1
                if key is None:self.new[group]=value
                else:self.new[group][key]=value
                self.bind()
                with self.assertRaises(ValueError):validate_adjustment(self.root,self.recipe)

    def test_second_change_or_stale_config_rejected(self):
        self.recipe['scene_adjustment']['adjustments']=2
        with self.assertRaises(ValueError):validate_adjustment(self.root,self.recipe)
        self.recipe['scene_adjustment']['adjustments']=1
        (self.root/'config.json').write_text('{}')
        with self.assertRaises(ValueError):validate_adjustment(self.root,self.recipe)

    def test_noop_or_wrong_previous_binding_rejected(self):
        with self.assertRaises(ValueError):validate_adjustment(self.root,self.recipe,'other.json')
        self.new=deepcopy(self.old);self.bind()
        with self.assertRaises(ValueError):validate_adjustment(self.root,self.recipe)


if __name__=='__main__':unittest.main()
