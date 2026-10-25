"""Jupyter-readable snapshot derived only from real B02 reports and logs."""
from pathlib import Path
from datetime import datetime
from zoneinfo import ZoneInfo
import json,re
root=Path('/root/autodl-tmp/RoboLaw-lgd');logs=Path('/root/autodl-tmp/lgd-b02-logs');out=Path('/root/autodl-tmp/B02-review/status.txt')
def read(p):return json.loads(p.read_text()) if p.exists() else {}
rows=['B02 progress at '+datetime.now(ZoneInfo('Asia/Shanghai')).isoformat(timespec='seconds'),'Branch: lgd; B03 base layout preview approved by user; preparing dual-pick validation.','']
for v in ('human','property','robot'):
 candidates=sorted(logs.glob(v+'_review_20261025T*.log'));log=candidates[-1] if candidates else None
 text=log.read_text(errors='replace') if log else '';steps=re.findall(r'^STEP .*$',text,re.M);matches=re.findall(r'^COLLECT_RESULT (.*)$',text,re.M)
 state='not started'
 if matches:
  r=json.loads(matches[-1]);state='physical SUCCESS' if r.get('success') else 'FAILED: '+str(r.get('error'))
 elif steps:state=steps[-1]
 elif log:state='initializing or waiting for GPU lock'
 episodes=sorted((root/'datasets/skill_trials/lightwheel_pick_place').glob(v+'_lgd_b02_review_*'))
 audit=read(episodes[-1]/'avoidance_audit.json') if episodes else {}
 rows.append(v+': '+state+'; independent avoidance '+('PASS' if audit.get('accepted') else 'pending'))
 if log:rows.append('  '+str(log))
neutral=root/'reports/skill_trials/lightwheel_pick_place/b02_neutral_hold_69s_20261025/neutral_hold.json';nr=read(neutral)
rows+=['','Task scope: background fixture investigation stopped per user instruction.','Task-object neutral hold: '+('PASS' if nr.get('task_objects_passed') and nr.get('robot_hold_passed') else 'pending'),'Global kitchen-joint neutral gate excluded; not claimed passed.']
manifest=read(out.parent/'acceptance_manifest.json');rows+=['Overall B02 acceptance: '+('PASS (fixed_only)' if manifest.get('passed') else 'PENDING'),'','Review images are in this folder. Data root: '+str(root/'datasets/skill_trials/lightwheel_pick_place')]
out.write_text('\n'.join(rows)+'\n',encoding='utf-8');print('\n'.join(rows))
