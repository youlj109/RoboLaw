#!/usr/bin/env python3
"""Select among the existing twelve CPU trials; no extra IK or candidates."""
from copy import deepcopy
from pathlib import Path
import json,sys
import numpy as np
ROOT=Path(__file__).resolve().parents[3];sys.path.insert(0,str(ROOT))
from scripts.extensions.replicator.prepare_round4_other_cpu import read,save,ref,geo
from kitchen_safety.extensions.replicator.scene_build_runtime import compose
from kitchen_safety.extensions.replicator.physical_retry_round2 import validate_recipe
from scripts.extensions.replicator.prepare_round8_human_cpu import OUT,CONFIG,RECIPES,SID

def rank(trial):
 checks=trial['endpoint_checks'];initial=trial['initial_robot']
 start_clear=not initial['unique_world_pairs'] and not initial['self_sphere_overlaps']
 unavoidable_payload_hits=sum(h['link']=='attached_object' for c in checks for h in c.get('collision',{}).get('unique_world_pairs',[]))
 release=trial['source_release']
 release_clear=release['fully_open_fingers_clear'] and release['fully_open_gripper_hulls_clear'] and release['all_sampled_gripper_world_clear']
 return (start_clear,not unavoidable_payload_hits,release_clear,sum(c['passed'] for c in checks),-sum(c['score'] for c in checks),trial['source_geometry_passed'])

def main():
 report=read(OUT/'candidates.json')
 backup=OUT/'initial_ranking_report.json'
 if backup.exists():raise FileExistsError('Final selection already retained')
 backup.write_bytes((OUT/'candidates.json').read_bytes())
 save(OUT/'initial_automatic_recipe.json',read(RECIPES));save(OUT/'initial_automatic_layout.json',read(CONFIG))
 assert len(report['trials'])<=12
 selected=max(report['trials'],key=rank);assert selected['index']==2
 layout=read(geo.bound(selected['new_config']));save(CONFIG,layout)
 scene=OUT/'selected_source_scene.usda';scene.unlink();composition=compose(CONFIG,scene)
 # Correct a display-only declaration scalar from the actual frozen matrices.
 # No pose, layout, IK result, or checked geometry sample changes here.
 for tr in report['trials']:
  d=tr['declaration'];d.pop('transfer_from_goal_delta_m',None)
  tcp=np.array(tr['recipe']['target_poses_world']['transfer']);attachment=np.array(tr['recipe']['nominal_attachment'])
  goal=np.array(tr['recipe']['final_payload_pose_world'])
  d['transfer_from_goal_delta_xyz_m']=((tcp@attachment)[:3,3]-goal[:3,3]).tolist()
 selected=report['trials'][selected['index']]
 region=deepcopy(selected['nominal_destination']);region['new_config']=ref(CONFIG);region['new_source_scene']=ref(scene);save(OUT/'selected_nominal_destination.json',region)
 oldlayout=read(geo.bound(report['original_config']));check=deepcopy(layout);check['obstacle']['position']=oldlayout['obstacle']['position'];check['obstacle']['yaw_deg']=oldlayout['obstacle']['yaw_deg'];assert check==oldlayout
 report.update(selected_index=selected['index'],cpu_screen_passed=False,new_config=ref(CONFIG),new_source_scene=ref(scene),new_composition=composition,
  final_selection=dict(initial_ranking_report=ref(backup),source_finalizer=ref(Path(__file__)),new_candidates=0,new_cpu_ik_calls=0,
   reason='Prioritize clear authored robot start, fixed nominal payload endpoints and open-release endpoint before the ideal straight retreat sweep. Free-path planning need not follow that diagnostic sweep. The lower robot branch warnings remain, so the overall CPU screen is not passed.'),
  layout_revision=dict(only_changed_json_fields=['obstacle.position','obstacle.yaw_deg'],old_position=oldlayout['obstacle']['position'],new_position=layout['obstacle']['position'],position_delta_m=(np.array(layout['obstacle']['position'])-oldlayout['obstacle']['position']).tolist(),old_yaw_deg=oldlayout['obstacle']['yaw_deg'],new_yaw_deg=layout['obstacle']['yaw_deg'],all_other_json_equal=True,source_asset_paths_sha256=composition['source_paths_sha256']))
 save(OUT/'candidates.json',report)
 r=deepcopy(selected['recipe']);r.update(config_path=str(CONFIG.relative_to(ROOT)),config_sha256=geo.sha(CONFIG),evidence=ref(OUT/'candidates.json'),previous_candidate_planning_result=report['source_previous_candidate'],cpu_screen_passed=False,selected_round8_candidate=selected['declaration'],
  destination_region_check=dict(passed=region['passed'],evidence_path=str((OUT/'selected_nominal_destination.json').relative_to(ROOT)),sha256=geo.sha(OUT/'selected_nominal_destination.json')),
  description='人体相对r7向世界+X平移120mm，yaw由-10°回到0°；保留已通过规划的竖直碗缘抓姿，搬运端点改到槽口初始侧100mm的高位。碗直立偏心释放，源碗底距槽底25mm使保守球包避开排水口。CPU初态和pregrasp/approach/lift/transfer/retreat端点通过；lower仍有有限IK分支的手臂/人体警告，直线退回扫掠也有警告，不能宣称全通过。真实cuRobo允许碰撞检查自由绕行，成功仅由一次原生物理判定。',
  remaining_cpu_warnings=[dict(phase=c['phase'],score=c['score'],collision=c.get('collision'),error=c.get('error')) for c in selected['endpoint_checks'] if not c['passed']],
  ideal_straight_retreat_has_source_intersections=True)
 validate_recipe(r,read(geo.bound(report['source_original_planning'])),layout)
 save(RECIPES,dict(kind='physical_retry_round8_human_recipes',recipes={SID:r},evidence=ref(OUT/'candidates.json'),layout_changed=True))
 print(json.dumps(dict(config=ref(CONFIG),recipes=ref(RECIPES),evidence=ref(OUT/'candidates.json'),selected_index=2),ensure_ascii=False))

if __name__=='__main__':main()
