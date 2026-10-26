import json
import sys
from types import SimpleNamespace

import pytest
from scripts.extensions.controls import complete_button_hand_revision as pipeline


def test_rejected_layout_cannot_launch_planning_or_collection(tmp_path, monkeypatch):
    docs=tmp_path/'docs/extensions/controls'
    docs.mkdir(parents=True)
    (docs/'ZXW_LEFT_BUTTON_REVIEWED_RESULTS.json').write_text(json.dumps({
        'episodes':[{'variant':'base','path':'unused-baseline'}]}))
    monkeypatch.setattr(pipeline,'ROOT',tmp_path)
    output=tmp_path/'verification'
    monkeypatch.setattr(sys,'argv',['pipeline','--output-root',str(output),'--sides','left'])
    commands=[]

    def screen_only(command, **kwargs):
        commands.append(command)
        assert command[2].endswith('screen_button_layout.py')
        destination=command[command.index('--output')+1]
        from pathlib import Path
        Path(destination).write_text(json.dumps({'candidates':{'human':{'potential_candidate':False}}}))
        return SimpleNamespace(returncode=0)

    monkeypatch.setattr(pipeline.subprocess,'run',screen_only)
    with pytest.raises(ValueError,match='endpoint/layout screen rejected'):
        pipeline.main()
    assert len(commands)==1
    progress=json.loads((output/'progress.json').read_text())
    assert progress['status']=='failed' and progress['accepted'] is False
