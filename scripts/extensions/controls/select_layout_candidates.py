"""Materialize candidate configs without overwriting historical trials."""
import argparse
import hashlib
import json
from pathlib import Path


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--select',nargs=3,action='append',metavar=('ROLE','SEARCH_JSON','INDEX'),default=[])
    p.add_argument('--pose',nargs=5,action='append',default=[],metavar=('ROLE','X','Y','Z','YAW'))
    p.add_argument('--asset',nargs=2,action='append',default=[],metavar=('ROLE','ASSET'))
    p.add_argument('--fixed-ik-from',type=Path)
    a=p.parse_args()
    if a.output.exists():raise FileExistsError(a.output)
    config=json.loads(a.config.read_text())
    provenance=config.setdefault('placement_candidate_selection',{})
    for role,path,index in a.select:
        source=Path(path);report=json.loads(source.read_text())
        config['variants'][role]=report['candidates'][role][int(index)]['spec']
        provenance[role]={'search':str(source),'sha256':hashlib.sha256(source.read_bytes()).hexdigest(),'candidate_index':int(index),
            'rule':'Ranked source-geometry screen; physical and paired acceptance still required.'}
    for role,x,y,z,yaw in a.pose:
        config['variants'][role].update(position=[float(x),float(y),float(z)],yaw_deg=float(yaw))
        provenance[role]={'rule':'Explicit diagnostic pose, not an accepted layout.'}
    for role,asset in a.asset:
        config['variants'][role]['asset']=asset
    if a.fixed_ik_from:
        result=json.loads((a.fixed_ik_from/'result.json').read_text())
        if not result['accepted'] or result['task_id']!=config['task_id']:
            raise ValueError('Branch seed requires an accepted same-task baseline')
        source=a.fixed_ik_from/'records.json'
        rows=json.loads(source.read_text())
        state=[r for r in rows if r['phase']=='pregrasp'][-1]['after']
        names=result['robot_joint_names']
        config['prepress_planning']={'method':'curobo_joint_goal',
            'ik_branch_seed':[state['q'][names.index(f'panda_joint{i}')] for i in range(1,8)],
            'paired_seeds':[42,43,44],
            'provenance':'Predeclared lawful original-TCP branch from '+str(source)+' sha256='+hashlib.sha256(source.read_bytes()).hexdigest()+'. Re-solve the same TCP without hazard-dependent waypoints. Requires new physical baseline and all three variants; do not mix old pose-policy episodes.'}
    a.output.parent.mkdir(parents=True,exist_ok=True)
    a.output.write_text(json.dumps(config,indent=2)+'\n')


if __name__=='__main__':main()
