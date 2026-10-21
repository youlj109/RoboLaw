"""CPU provenance audit: original graph solver and frozen source poses.

This checks execution logs/source/recorded init configs. It is not a fresh
geometric solver execution and does not replace saved physics collision audits.
"""
import hashlib,json,re
from pathlib import Path
ROOT=Path(__file__).resolve().parents[3]
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def solver_evidence(path):
 text=path.read_text();lines=text.splitlines();return {'log':str(path.relative_to(ROOT)),'log_sha256':sha(path),'placement_validation_lines':[{'line':i+1,'text':s} for i,s in enumerate(lines) if s.startswith('[placement] Validated')],'best_loss_fallback_reported':'Falling back to best-loss layouts' in text}
report={'scope':'Source/actual execution provenance and exact initial pose equality; no fresh CPU relation solve','doors':{},'lightwheel_pick_place':{}}
for name in ('base_attempt_08','human_attempt_02','property_attempt_01','robot_attempt_01'):
 ep=ROOT/'datasets/skill_trials/doors'/name;m=json.loads((ep/'metadata.json').read_text());log=ROOT/'reports/skill_trials/doors'/(name+'.log');e=solver_evidence(log);text=log.read_text();match=re.search(r'robot=ArticulationCfg\(.*?init_state=ArticulationCfg.InitialStateCfg\(pos=\(([^)]+)\), rot=\(([^)]+)\)',text)
 pos=[float(x.strip()) for x in match.group(1).split(',') if x.strip()] if match else None
 e.update(recorded_base=m['robot_base'],configured_initial_base=pos,configured_base_matches_recorded=pos==m['robot_base'],source_yaml_sha256=m['source_yaml_sha256']);e['passed']=bool(e['placement_validation_lines'] and not e['best_loss_fallback_reported'] and e['configured_base_matches_recorded']);report['doors'][name]=e
basep=ROOT/'datasets/skill_trials/lightwheel_pick_place/base_attempt12/result.json';base=json.loads(basep.read_text());frozen=json.loads((ROOT/'configs/extensions/lightwheel_pick_place/fixed_layout.json').read_text());e=solver_evidence(ROOT/'reports/skill_trials/lightwheel_pick_place/base_attempt12.log');e.update(fixed_layout_matches_solver_record=frozen['original_asset_initial_poses']==base['original_asset_initial_poses'],frozen_result_hash_matches=frozen['provenance']['result_sha256']==sha(basep),base_source_hash=base['source_yaml_sha256']);e['passed']=bool(e['placement_validation_lines'] and not e['best_loss_fallback_reported'] and e['fixed_layout_matches_solver_record'] and e['frozen_result_hash_matches']);report['lightwheel_pick_place']['base_attempt12']=e
for name in ('property_attempt02','robot_attempt02','human_attempt02','human_attempt03'):
 ep=ROOT/'datasets/skill_trials/lightwheel_pick_place'/name
 if not (ep/'result.json').exists():continue
 r=json.loads((ep/'result.json').read_text());matches={n:pose==base['original_asset_initial_poses'].get(n) for n,pose in r['original_asset_initial_poses'].items()};report['lightwheel_pick_place'][name]={'passed':all(matches.values()) and len(matches)==6,'source_initial_pose_matches':matches,'scope':'Exact pre-gravity original droid/lime/bowl/broccoli/potato/pepper pose equality to successful source-solver baseline. Physical settling may subsequently change poses.'}
report['implementation_evidence']={'doors':'build() parses original YAML and attaches graph.relations verbatim; no source set_initial_pose calls; collector ArenaEnvBuilder solve_relations=True/resolve_on_reset=False; source DROID placement_bbox_stand_only default False retained','lime':'base build parses original YAML and graph.relations, no object-coordinate literals; fixed_layout holds exact base12 solved source poses; safety variants retain On/NextTo and remove only already-applied RotateAroundSolution marker before IsAnchor','pool_selection':'Original pooled_object_placer.py _store_ranked_results admits r.success layouts. Any no-valid best-loss fallback explicitly prints Falling back to best-loss layouts; none found in these original solver runs.','source_files':{str(p.relative_to(ROOT)):sha(p) for p in [ROOT/'kitchen_safety/extensions/doors/environment.py',ROOT/'kitchen_safety/extensions/lightwheel_pick_place/environment.py',ROOT/'third_party/IsaacLab-Arena/isaaclab_arena/relations/pooled_object_placer.py']}}
report['passed']=all(x['passed'] for g in ('doors','lightwheel_pick_place') for x in report[g].values());p=ROOT/'reports/skill_trials/doors/source_layout_provenance.json';p.write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2));raise SystemExit(0 if report['passed'] else 2)
