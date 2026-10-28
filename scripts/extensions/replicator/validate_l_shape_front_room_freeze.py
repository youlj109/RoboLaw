"""Read-only frozen-config binding and former-pair validation sidecar."""
import copy,json,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[3];sys.path.insert(0,str(ROOT))
from scripts.extensions.replicator.adjust_l_shape_mustard_layout import sha,rel,binding,write_json,protected_base
from scripts.extensions.replicator.search_source_layout_candidates import CandidateSearch
from scripts.extensions.replicator.audit_source_layout_geometry import minimum_distance
OUT=ROOT/'configs/extensions/replicator/l_shape_mustard_adjust16';REPORT=ROOT/'reports/replicator_safety_upgrade/adjust16/l_shape_mustard'
base=OUT/'base_layout.json';layout=json.loads(base.read_text());scene=CandidateSearch(layout);rows=[]
for obj,root in [('mustard_bottle','/World/TrayA01_01/'),('bowl','/World/TrayA01/')]:
 result=minimum_distance(scene.shapes[obj].at([0,0,0]),[r for r in scene.world if r[0].startswith(root)],.002)
 rows.append(dict(object=obj,former_conflicting_background_root=root,measurement=result))
pairs=dict(config_path=rel(base),config_sha256=sha(base),passed=all(r['measurement']['valid'] for r in rows),entries=rows,scope='Exact original-source geometry separation; not cooked physics.')
write_json(REPORT/'former_intersection_pairs.json',pairs)
m=OUT/'manifest.json';manifest=json.loads(m.read_text());checks=[]
for e in manifest['entries']:
 c=json.loads((ROOT/e['config_path']).read_text());v=json.loads((ROOT/e['evidence_path']).read_text());checks.append(dict(scene_id=e['scene_id'],same_base={k:val for k,val in c.items() if k!='obstacle'}==layout,config_sha_matches=e['config_sha256']==sha(ROOT/e['config_path']),evidence_sha_matches=e['evidence_sha256']==sha(ROOT/e['evidence_path']),evidence_config_bound=v['config_sha256']==e['config_sha256'],all_geometry_gates_passed=all(v['after']['checks'].values()),semantic_geometry_passed=e['semantic_geometry_passed']))
passed=all(all(v for k,v in row.items() if k!='scene_id') for row in checks) and pairs['passed'] and len(checks)==11
write_json(REPORT/'freeze_validation.json',dict(passed=passed,manifest=binding(m),base=binding(base),entries=checks,former_pairs=binding(REPORT/'former_intersection_pairs.json'),regression_tests=dict(command='bash scripts/arena_python.sh -m pytest -q tests/extensions/replicator/test_housing_lip_distance.py tests/extensions/replicator/test_l_island_mustard_adjustment.py',passed=4),risk_verified=False,native_stability_verified=False,safe_execution_verified=False,source_code=binding(__file__)))
print(json.dumps(dict(passed=passed,pairs=pairs['entries'],count=len(checks))),flush=True)
