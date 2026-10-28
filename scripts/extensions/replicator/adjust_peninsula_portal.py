"""Second bounded portal search; retains the first failed search and crossbar."""
from pathlib import Path
import importlib.util
import itertools
import json
import sys
import numpy as np
from scipy.spatial.transform import Rotation

ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT))
spec=importlib.util.spec_from_file_location('peninsula_features',Path(__file__).with_name('adjust_peninsula_features.py'))
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)


def portal_pool(hazard,start,finish,z):
    parts={r[0].rsplit('/',1)[-1]:r for r in hazard.rows}
    local=np.mean([(parts[n][4]+parts[n][5])/2 for n in ['InsideWearPlate0','InsideWearPlate1']],axis=0)
    layout=json.loads((ROOT/'configs/extensions/replicator/visual_scene_variants_final01/peninsula_mustard_bowl/robot_narrow_opening.json').read_text())
    base=np.asarray(layout['robot_position'],dtype=float)
    guides=[('pick_to_destination',start,finish,0.),('robot_to_pick_approach',base,start,.08),('robot_to_goal_approach',base,finish,.08)]
    result=[]
    for name,a,b,penalty in guides:
        delta=b[:2]-a[:2];direction=delta/np.linalg.norm(delta);normal=np.array([-direction[1],direction[0]])
        heading=float(np.degrees(np.arctan2(direction[1],direction[0])))
        fractions=np.linspace(.25,.75,21) if name=='pick_to_destination' else np.linspace(.35,.85,21)
        for fraction,offset,angle in itertools.product(fractions,(0.,-.015,.015,-.03,.03,-.045,.045),range(-80,81,5)):
            yaw=heading+angle;rot=Rotation.from_euler('z',yaw,degrees=True).as_matrix()
            center=a[:2]+fraction*delta+offset*normal
            denominator=float(np.dot(delta,rot[:2,0]))
            if abs(denominator)<1e-8:continue
            cross_t=float(np.dot(center-a[:2],rot[:2,0])/denominator)
            cross=a[:2]+cross_t*delta;cross_local_y=float(np.dot(cross-center,rot[:2,1]))
            # The actual 146 mm aperture must contain the guide crossing with
            # 18 mm lateral margin. No widening of hazard-risk thresholds.
            if not .2<=cross_t<=.9 or abs(cross_local_y)>.055:continue
            position=np.r_[center-(rot@local)[:2],z]
            result.append(dict(position=position.tolist(),yaw_deg=yaw,feature='actual_aperture_center',
                local_feature=local.tolist(),feature_world=(position+rot@local).tolist(),
                task_fraction=float(fraction),guide=name,guide_start=a.tolist(),guide_finish=b.tolist(),
                aperture_plane_guide_crossing_xy=cross.tolist(),aperture_crossing_local_y_m=cross_local_y,
                aperture_half_width_m=.073,aperture_crossing_margin_m=.073-abs(cross_local_y),
                feature_to_guide_xy_m=abs(offset),semantic_score=float(abs(offset)+.1*abs(fraction-.5)+penalty)))
    return sorted(result,key=lambda r:(r['semantic_score'],abs(r['yaw_deg'])))


if __name__=='__main__':
    OriginalScreen=m.InitialRobotScreen
    class InitialMarginScreen(OriginalScreen):
        def check(self,shapes):
            out=super().check(shapes)
            out['additional_initial_margin_m']=.01
            out['passed']=out['passed'] and min(out[k]['distance_m'] for k in ('source_controller_initial','usd_authored_link_transforms')) >= .01
            return out
    m.InitialRobotScreen=InitialMarginScreen
    m.IDS=['robot_narrow_opening']
    m.OUT=ROOT/'configs/extensions/replicator/peninsula_mustard_adjust02/portal03'
    m.REPORT=ROOT/'reports/replicator_safety_upgrade/adjust02/peninsula_mustard/portal03'
    m.pool=portal_pool
    m.build()
    # Bind the extra candidate generator as well as the shared checking script.
    ep=m.REPORT/'robot_narrow_opening.json';e=json.loads(ep.read_text())
    e['code_sha256'][m.relative(__file__)]=m.sha(__file__)
    e['additional_initial_robot_margin_m']=.01
    e['nominal_workspace']['approach_guides_scope']='XY guide from robot base toward task goal; aperture height is from its source geometry. This is not a 3D execution path.'
    m.write_json(ep,e)
    for p in [m.OUT/'manifest.json',m.REPORT/'progress.json']:
        d=json.loads(p.read_text());d['total']=len(d['entries']);d['entries'][0]['evidence_sha256']=m.sha(ep);d['scenes']=d['entries'];m.write_json(p,d)
