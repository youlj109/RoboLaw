"""Build an evidence-based fixed-trial handoff; no simulator or GPU is started."""
import json
from pathlib import Path
import h5py,numpy as np
from scipy.spatial.transform import Rotation
from trial_evidence import acceptance
ROOT=Path(__file__).resolve().parents[3]
DATA=ROOT/'datasets/skill_trials/replicator';REPORTS=ROOT/'reports/skill_trials/replicator'
baseline=json.loads((DATA/'human/attempt_04/result.json').read_text())
reference_layout={k:v for k,v in baseline['layout'].items() if k!='obstacle'}
rows=[]
for variant,attempt in [('human','attempt_04'),('property','attempt_05'),('robot','attempt_03')]:
 out=DATA/variant/attempt;reports=REPORTS/variant/attempt
 result=json.loads((out/'result.json').read_text());summary=json.loads((reports/'summary.json').read_text())
 accepted=acceptance(out,reports,summary['exit_codes'],require_rgb=True)
 layout={k:v for k,v in result['layout'].items() if k!='obstacle'}
 layout_matches=layout==reference_layout
 assert accepted['passed'] and layout_matches, (variant,accepted,layout_matches)
 audit=json.loads((reports/'audit.json').read_text())['episodes'][0]
 row=dict(variant=variant,attempt=attempt,accepted=True,authored_original_layout_matches_baseline=layout_matches,
          frames=result['steps'],physics_substeps=result['deferred_safety']['checks'],
          lift_height_m=result['lift_height'],max_nonfinger_robot_contact_n=result['max_arm_contact_force_n'],
          minimum_hazard_clearance_m=audit['minimum_clearance_m'],clearance_sampling='Action-rate observations plus final next_obs; full physics substeps checked separately',
          output=str(out.relative_to(ROOT)),reports=str(reports.relative_to(ROOT)),
          videos=[str((out/(name+'.mp4')).relative_to(ROOT)) for name in ['external_camera','external_camera_2','wrist_camera','all_cameras']],
          exit_codes=summary['exit_codes'],acceptance_gates=accepted['acceptance_gates'])
 if result.get('overview_camera'):
  row['overview_video']=str((out/'overview_camera.mp4').relative_to(ROOT))
  row['overview_validation']=json.loads((out/'overview_validation.json').read_text())
 if variant=='property':
  with h5py.File(out/'trajectory.h5') as f:
   states=np.concatenate([f['obs/hazard_pose'][30:],f['next_obs/hazard_pose'][-1:]])
  row['vase_after_settling']=dict(max_translation_m=float(np.linalg.norm(states[:,:3]-states[0,:3],axis=1).max()),
    max_orientation_change_deg=float((Rotation.from_quat(states[0,3:]).inv()*Rotation.from_quat(states[:,3:])).magnitude().max()*180/np.pi),
    scope='Recorded states after 30 settling actions; no fracture model')
 rows.append(row)
report=dict(task_id=baseline['task_id'],accepted_episodes=3,variants=rows,fixed_only=True,
            source_yaml_sha256=baseline['source_task']['yaml_sha256'],software_versions=baseline['software_versions'],
            baseline='Shared legal reset layout, identical across these three variants; CPU keyframe IK/FCL preflight in reports/skill_trials/replicator/legal_layout_cpu_preflight.json. Unaugmented diagnostic_05 uses the rejected old layout and is historical only.',
            rejected_prior_layout=['human/attempt_01','property/attempt_03','robot/attempt_02','base/diagnostic_05'],
            source_relation_verification='Per-episode source_relation_audit.json calls unchanged upstream validators for all original YAML relations; measured base pose is cross-checked.',
            fixed_base_verification='Per-episode fixed_base_hazard_audit.json independently checks actual fixed stand/base collision hulls and visible mesh surfaces.',
            visual_review='Sampled start/transfer/final RGB frames reviewed; every JPEG and MP4 frame decoded by validation. Geometry/contact evidence is separate.',
            limitations=['Only this one of 14 source tasks is verified.','One fixed successful episode per variant; no randomized or 200-episode claim.','Asset restore is required on another machine.','Historical counterfactual probes against diagnostic_05 do not establish behavior in this corrected layout.'])
(REPORTS/'summary.json').write_text(json.dumps(report,indent=2)+'\n')
assignment=json.loads((ROOT/'student_skills/robolaw-replicator/references/assignment.json').read_text())
coverage=dict(verified_source_tasks=1,total_source_tasks=14,tasks=[dict(id=t['id'],status='fixed_three_variants_verified' if t['id']==baseline['task_id'] else 'not_verified',successful_episodes=3 if t['id']==baseline['task_id'] else 0) for t in assignment['tasks']])
(ROOT/'docs/extensions/replicator/coverage.json').write_text(json.dumps(coverage,indent=2)+'\n')
print(json.dumps(report,indent=2))
