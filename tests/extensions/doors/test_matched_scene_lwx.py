import hashlib
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest

spec = importlib.util.spec_from_file_location('matched_scene_lwx', Path('scripts/extensions/doors/matched_scene_lwx.py'))
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class MatchedSceneTests(unittest.TestCase):
    def test_scene_must_be_inherited_and_recorded_config_unchanged(self):
        with tempfile.TemporaryDirectory() as directory:
            base = Path(directory) / 'base.json'
            candidate = Path(directory) / 'candidate.json'
            scene = {'_kitchen_asset': 'declared_background.usda', '_fixed_source_layout': 'source_layout.json'}
            base.write_text(json.dumps(scene))
            metadata = {'variant_config': str(base), 'implementation_sha256': {str(base): hashlib.sha256(base.read_bytes()).hexdigest()}}
            candidate.write_text(json.dumps({**scene, 'human': {'position': [1, 2, 0]}}))
            module.require_matching_scene(metadata, candidate)
            candidate.write_text(json.dumps({'_fixed_source_layout': scene['_fixed_source_layout']}))
            with self.assertRaisesRegex(RuntimeError, '_kitchen_asset'):
                module.require_matching_scene(metadata, candidate)
            candidate.write_text(json.dumps(scene))
            base.write_text(json.dumps({**scene, '_kitchen_asset': 'changed.usda'}))
            with self.assertRaisesRegex(RuntimeError, 'configuration changed'):
                module.require_matching_scene(metadata, candidate)


if __name__ == '__main__':
    unittest.main()
