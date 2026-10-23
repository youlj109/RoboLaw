"""Exercise actual USD materialization of a support with an open space below its tray."""
import itertools
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
import uuid

import numpy as np
from pxr import Usd, UsdGeom, UsdPhysics


class OffsetSupportTests(unittest.TestCase):
    def test_parts_materialize_at_the_declared_world_positions(self):
        name = 'test_support_' + uuid.uuid4().hex
        generated = Path('assets/extensions/doors') / (name + '_property_support.usda')
        parts = [{'low': [2.95, -1.05, .002], 'high': [3.05, -.95, 1.378]},
                 {'low': [2.95, -1.05, 1.378], 'high': [3.05, -.65, 1.398]}]
        candidate = {'name': 'glass', 'asset': 'assets/obstacle_library/property/tablet_stand.usda',
                     'position': [3., -.7, 1.4], 'yaw_deg': 0., 'static': False,
                     'proposed_support': {'low': [2.95, -1.05, .002], 'high': [3.05, -.65, 1.398],
                                          'parts': parts, 'rests_on': '/source/floor', 'design': 'offset tray'}}
        try:
            with tempfile.TemporaryDirectory() as directory:
                root = Path(directory)
                config, search, output = root / 'base.json', root / 'search.json', root / (name + '.json')
                config.write_text('{}')
                search.write_text(json.dumps({'candidates': [candidate]}))
                subprocess.run([sys.executable, 'scripts/extensions/doors/prepare_task_candidate_lwx.py',
                                '--config', str(config), '--search', str(search), '--variant', 'property',
                                '--index', '0', '--output', str(output)], check=True, capture_output=True)
                spec = json.loads(output.read_text())['property']['support']
                stage = Usd.Stage.Open(spec['asset'])
                shapes = [p for p in stage.Traverse() if p.HasAPI(UsdPhysics.CollisionAPI)]
                self.assertEqual(len(shapes), 2)
                cache = UsdGeom.XformCache()
                for shape, expected in zip(shapes, parts):
                    local = np.asarray(list(itertools.product([-.5, .5], repeat=3)))
                    transform = np.asarray(cache.GetLocalToWorldTransform(shape)).T
                    world = local @ transform[:3, :3].T + transform[:3, 3] + spec['position']
                    low, high = np.array(expected['low']), np.array(expected['high'])
                    low[2] -= .002
                    high[2] += .002
                    np.testing.assert_allclose(world.min(0), low, atol=1e-6)
                    np.testing.assert_allclose(world.max(0), high, atol=1e-6)
                self.assertLess(parts[0]['high'][1], candidate['position'][1] - .2)
        finally:
            if generated.exists():
                generated.unlink()


if __name__ == '__main__':
    unittest.main()
