"""Apply existing support-contact rule to the actual approved native island tabletop."""
import json,hashlib
from pathlib import Path
import numpy as np,h5py
from pxr import Usd
from scipy.spatial.transform import Rotation
from kitchen_safety.planning import DroidKinematics
from kitchen_safety.extensions.lightwheel_pick_place.collision import CollisionWorld
from kitchen_safety.extensions.lightwheel_pick_place.task_config import episode_spec
from scan_avoidance import initial_intersections,fixed_shapes
p=Path('datasets/skill_trials/lightwheel_pick_place/property_lgd_b03_checkpoint_replay_20261027T064923Z');r=json.loads((p/'result.json').read_text());spec=episode_spec(r);b=r['robot_base_pose'];kin=DroidKinematics(b[:3],Rotation.from_quat(b[3:7]).as_euler('xyz',degrees=True)[2]);stage=Usd.Stage.Open(str(p/'scene.usda'));w=CollisionWorld(stage,kin,pick_object=spec.pick_object,destination_object=spec.destination_object,source_surface_suffix=spec.source_surface_suffix,movable_objects=tuple(n for n in spec.rigid_objects if n!=spec.pick_object)+('glass',))
with h5py.File(p/'trajectory.h5') as h:pose=h['obs/hazard_pose'][0]
w.set_obstacle_pose('glass',pose);hazards=[v for v in w.objects if w._matches(v[0],'glass')];source=[v for v in w.objects if not w._matches(v[0],'glass')]+fixed_shapes(stage)
support='/World/envs/env_0/kitchen/island_island_group/top_geometry_right';assert stage.GetPrimAtPath(support),'Expected original native support not found'
roots=(spec.source_surface_abs or '/World/envs/env_0/kitchen'+spec.source_surface_suffix,support)
bad,contacts=initial_intersections(source,hazards,intended_support_roots=roots)
measurements=[]
for hn,obj,lo,hi in hazards:
 for sn,other,slo,shi in source:
  if sn==support:measurements.append({'hazard':hn,'source':sn,'hazard_min_z_m':float(lo[2]),'support_max_z_m':float(shi[2]),'vertical_overlap_bound_m':float(shi[2]-lo[2]),'same_existing_support_rule_threshold_m':.006})
report={'passed':not bad,'scope':'Initial scene support-context review only. Original helper already permits native countertop support when hazard bottom is within6mm of top. Apply identical rule to exact original island tabletop supporting approved vase, rather than only task source or names containing counter_. No geometry/poses/thresholds changed; prior false audit retained. Not a PhysX penetration-depth certificate.','episode':str(p),'scene_sha256':hashlib.sha256((p/'scene.usda').read_bytes()).hexdigest(),'native_support_root':support,'measurements':measurements,'allowed_support_contacts':contacts,'remaining_non_support_intersections':bad}
out=Path('/root/autodl-tmp/B03-review/property_native_support_context_review.json');out.write_text(json.dumps(report,indent=2));print('NATIVE_ISLAND_SUPPORT_CONTEXT',report['passed'],[x for x in measurements if x['hazard'].endswith('/Base')],flush=True)
