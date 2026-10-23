import json
from pathlib import Path
import subprocess
import sys


def test_missing_evidence_cannot_be_upgraded_to_accepted(tmp_path):
    root=Path(__file__).resolve().parents[3]
    report=tmp_path/'result.json'
    command=[sys.executable,str(root/'scripts/extensions/controls/summarize_temperature.py'),
        '--baseline',str(tmp_path/'base'),'--episodes',*[str(tmp_path/k) for k in ('human','property','robot')],
        '--neutral',str(tmp_path/'neutral'),'--rack-mobility',str(tmp_path/'rack'),
        '--faucet-mobility',str(tmp_path/'faucet'),'--paired',str(tmp_path/'paired'),
        '--config',str(root/'configs/extensions/controls/temperature_trial03.json'),'--output',str(report)]
    result=subprocess.run(command,cwd=root,capture_output=True,text=True)
    assert result.returncode==2,result.stderr
    data=json.loads(report.read_text())
    assert data['accepted'] is False
    assert data['gates']['all_four_real_episodes'] is False
    assert data['gates']['complete_12_unique_pairs'] is False
    assert data['gates']['neutral_stationary_all_90_dofs'] is False
    assert data['gates']['original_goal_three_reset_minus_one'] is True
