"""Resolve only hash/command-backed replay provenance; never rewrite native results."""
import json,hashlib,h5py,numpy as np
from pathlib import Path
root=Path('datasets/skill_trials/lightwheel_pick_place')
keys=('selected_grasp','pregrasp_offset_m','grasp_lateral_x_m','grasp_finger_bias_m','grasp_local_approach','pregrasp_planner_mode','grasp_search_policy','transfer_route_policy')
reports={}
for name in ('property_lgd_b03_checkpoint_replay_20261027T064923Z','robot_lgd_b03_checkpoint_replay_20261027T033440Z'):
 episode=root/name;current=episode;chain=[];seen=set();outer=json.loads((episode/'result.json').read_text())
 while True:
  if str(current.resolve()) in seen:raise RuntimeError('Replay provenance cycle')
  seen.add(str(current.resolve()));r=json.loads((current/'result.json').read_text());prefix=r.get('prefix_replay')
  if not prefix:break
  source=Path(prefix['source_episode']);count=int(prefix['actions'])
  with h5py.File(current/'trajectory.h5') as a,h5py.File(source/'trajectory.h5') as b:
   matched=bool(np.array_equal(a['action'][:count],b['action'][:count]) and np.array_equal(a['phase'].asstr()[:count],b['phase'].asstr()[:count]))
  if not matched:raise RuntimeError('Recorded prefix mismatch: '+str(current))
  chain.append({'episode':str(current.resolve()),'result_sha256':hashlib.sha256((current/'result.json').read_bytes()).hexdigest(),'source_episode':str(source.resolve()),'actions':count,'actual_recorded_actions_and_phases_identical':matched})
  current=source
 original=json.loads((current/'result.json').read_text());missing={k:original[k] for k in keys if outer.get(k) is None and original.get(k) is not None}
 second=outer.get('second_grasp_replay',{});second_proof=None
 if second:
  checkpoint=Path(outer['prefix_replay']['checkpoint']);file=checkpoint.parent/second['file'];data=np.load(file,allow_pickle=False);start=int(second['start']);count=len(data['actions'])
  with h5py.File(episode/'trajectory.h5') as h:matched=np.array_equal(h['action'][start:start+count],data['actions'])
  second_proof={'file':str(file),'sha256_matches':hashlib.sha256(file.read_bytes()).hexdigest()==second['sha256'],'actual_commands_match':bool(matched),'source_pick_grasp_solution':outer.get('grasp_solutions',{}).get('banana_2')}
 reports[name]={'original_result_unchanged':True,'scope':'Supplementary provenance, not a replacement result or acceptance upgrade. First prefix and later placement controllers are separate effective stages. Missing logged metadata does not itself prove a different executed grasp, but real placement/release differences still require matched evidence.','first_pick_source':str(current.resolve()),'verified_replay_chain':chain,'missing_logged_grasp_fields_resolved_from_identical_commands':missing,'second_grasp_prefix':second_proof,'first_pick_executed_motion_parameters':original.get('task_motion_parameters'),'second_placement_logged_motion_parameters':outer.get('task_motion_parameters'),'strict_whole_episode_controls_match_proven':False}
 print(name,'inherited_missing_fields',list(missing),'chain_links',len(chain),'second_commands_verified',bool(second_proof and second_proof['actual_commands_match']),flush=True)
out=Path('/root/autodl-tmp/B03-review/replay_control_provenance_review_20261028.json');out.write_text(json.dumps(reports,indent=2))
