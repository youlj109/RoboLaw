"""CPU pinned-sphere diagnostic for exact, already viewed property proposals."""
import argparse,json
from pathlib import Path
import numpy as np
from kitchen_safety.extensions.doors.task_reconstruction import reconstruct
from kitchen_safety.extensions.doors.task_spheres import initial_spheres

p=argparse.ArgumentParser(description=__doc__)
p.add_argument('--manifest',type=Path,required=True)
p.add_argument('--output',type=Path,required=True)
a=p.parse_args()
if a.output.exists():raise FileExistsError(a.output)
report=dict(scope='CPU approximation of pinned cuRobo spheres against original padded source collider AABBs. Not a GPU query or safety acceptance.',candidates=[])
for item in json.loads(a.manifest.read_text())['candidates']:
    if item['variant']!='property':continue
    m=json.loads((Path(item['baseline'])/'metadata.json').read_text())
    cfg=json.loads(Path(item['config']).read_text());spec=cfg['property']
    stage,kin,w,g=reconstruct(m,cfg,spec)
    spheres=initial_spheres(m,kin);centers=np.array([s[2] for s in spheres]);radii=np.array([s[3] for s in spheres])
    scores=[]
    for n,o,lo,hi in w.objects:
        hazard=w._matches(n,spec['name'])
        support=spec.get('support') and w._matches(n,spec['support']['name'])
        if not (hazard or support):continue
        margin=w.clearances[spec['name']]+m['controller_parameters'].get('planner_hazard_extra_padding_m',0.) if hazard else 0.
        low,high=lo-margin,hi+margin
        distances=np.linalg.norm(np.maximum(np.maximum(low-centers,centers-high),0.),axis=1)-radii
        j=int(np.argmin(distances))
        scores.append(dict(slack_m=float(distances[j]),link=spheres[j][0],sphere=spheres[j][1],collider=n,margin_m=margin))
    worst=min(scores,key=lambda s:s['slack_m'])
    report['candidates'].append(dict(id=item['id'],minimum=worst,positive_cpu_slack=worst['slack_m']>0.))
    print('INITIAL_SPHERE',item['id'],worst,flush=True)
a.output.write_text(json.dumps(report,indent=2)+'\n')
