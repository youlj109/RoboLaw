import hashlib
import json
import sys

import pytest
from scripts.extensions.controls import complete_temperature


def test_rejected_baseline_cannot_launch_collection(tmp_path,monkeypatch):
    config=tmp_path/'config.json';config.write_text('{}')
    baseline=tmp_path/'base';baseline.mkdir()
    (baseline/'result.json').write_text(json.dumps({'accepted':False,'config_sha256':hashlib.sha256(config.read_bytes()).hexdigest()}))
    reports=tmp_path/'reports'
    args=['complete_temperature','--config',str(config),'--baseline',str(baseline),
        '--episodes-root',str(tmp_path/'episodes'),'--reports-root',str(reports),
        '--stage','missing','--neutral','missing','--rack-mobility','missing','--faucet-mobility','missing',
        '--output',str(tmp_path/'result.json')]
    monkeypatch.setattr(sys,'argv',args)
    def forbidden(*args,**kwargs):raise AssertionError('Must not launch any subprocess')
    monkeypatch.setattr(complete_temperature.subprocess,'run',forbidden)
    with pytest.raises(ValueError,match='Physical baseline rejected'):
        complete_temperature.main()
    progress=json.loads((reports/'verification_progress.json').read_text())
    assert progress['status']=='failed'
    assert progress['accepted'] is False
    assert not progress['steps']
    assert not (tmp_path/'result.json').exists()
