#!/usr/bin/env bash
set -euo pipefail
trap 'printf "AUDIT_EXIT=%s\n" "$?"' EXIT
cd /root/autodl-tmp/RoboLaw-lgd
episode=datasets/skill_trials/lightwheel_pick_place/robot_lgd_b03_standard_stage_property_20261028T074115Z
py=/root/autodl-tmp/conda_envs/robolaw_data/bin/python
"$py" - "$episode" <<'PY'
import json,sys,pathlib,hashlib
p=pathlib.Path(sys.argv[1]);backups={}
for n in ['validation.json','source_support_audit.json']:
 src=p/n;dst=p/(src.stem+'.before_source_runtime_parity.json')
 if dst.exists():raise FileExistsError(dst)
 data=src.read_bytes();dst.write_bytes(data);backups[n]={'backup':str(dst),'sha256':hashlib.sha256(data).hexdigest()}
(p/'source_audit_revision.json').write_text(json.dumps({'reason':'Offline audit wrongly converted shallow-contact floor recalibration permission into an unconditional contact rejection. Match original collect.py floor-retention policy, no physical change or margin change.','original_false_reports':backups,'runtime_source_support_sha256':hashlib.sha256(pathlib.Path('kitchen_safety/extensions/lightwheel_pick_place/source_support.py').read_bytes()).hexdigest(),'corrected_auditor_sha256':hashlib.sha256(pathlib.Path('scripts/extensions/lightwheel_pick_place/audit_multi_source.py').read_bytes()).hexdigest(),'original_floor_jitter_m':.0005,'original_xy_m':.05,'original_detach_distance_m':.002},indent=2))
sidecar=pathlib.Path('/root/autodl-tmp/B03-review/robot074115_source_runtime_parity_20261028.json');report=json.loads(sidecar.read_text());assert report['passed'] and len(report['measurements'])==9384
(p/'source_support_audit.json').write_bytes(sidecar.read_bytes())
revision=json.loads((p/'source_audit_revision.json').read_text());revision['corrected_source_report']={'source_sidecar':str(sidecar),'sha256':hashlib.sha256(sidecar.read_bytes()).hexdigest()};(p/'source_audit_revision.json').write_text(json.dumps(revision,indent=2))
PY
env PYTHONPATH=. "$py" scripts/extensions/lightwheel_pick_place/validate.py "$episode"
env PYTHONPATH=. "$py" scripts/extensions/lightwheel_pick_place/audit_fixed_base.py "$episode"
"$py" - "$episode" <<'PY'
import json,sys,pathlib
p=pathlib.Path(sys.argv[1]);g={n:json.loads((p/n).read_text())['passed'] for n in ['validation.json','full_world_audit.json','self_collision_audit.json','source_support_audit.json','fixed_base_hazard_audit.json']};print(json.dumps(g),flush=True);assert all(g.values())
PY
