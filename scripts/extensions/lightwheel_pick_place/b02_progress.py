"""Write a small Jupyter-readable snapshot from real B02 logs and results."""
from pathlib import Path
from datetime import datetime
import json,re
root=Path('/root/autodl-tmp/RoboLaw-lgd');logs=Path('/root/autodl-tmp/lgd-b02-logs');out=Path('/root/autodl-tmp/B02-review/status.txt')
rows=['B02 progress at '+datetime.now().isoformat(timespec='seconds'),'Branch: lgd; B03 remains frozen.','Human/property new transfer paired seeds 42/43/44: all 9 runs valid; both hazards require changes to all 3 hazard-off paths.','']
for p in sorted(logs.glob('*review_20261025T*.log')):
 text=p.read_text(errors='replace');steps=re.findall(r'^STEP .*$',text,re.M);matches=re.findall(r'^COLLECT_RESULT (.*)$',text,re.M)
 if matches:
  r=json.loads(matches[-1]);state='SUCCESS' if r.get('success') else 'FAILED: '+str(r.get('error'))
 elif 'unrecognized arguments' in text:state='CLI error before simulation; corrected in v2 runner'
 else:state=steps[-1] if steps else 'initializing or waiting for GPU lock'
 rows.append(p.name+': '+state)
rows+=['','Layout images: property.jpg, property-overview.jpg, human.jpg, human-side.jpg','Data root: '+str(root/'datasets/skill_trials/lightwheel_pick_place'),'Main collection log: '+str(logs/'codex_reviewed_collect_v2.log'),'No accepted claim until independent audits and complete video verification pass.']
out.write_text('\n'.join(rows)+'\n',encoding='utf-8');print('\n'.join(rows[-10:]))
