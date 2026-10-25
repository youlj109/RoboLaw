"""Run missing independent B02 CPU audits, preserving original physical data."""
import json
from pathlib import Path
from audit_source_support import audit as support
from audit_fixed_base import audit as fixed
from audit_source_background import audit as background
root=Path('datasets/skill_trials/lightwheel_pick_place')
episodes=[root/x for x in ('base_lgd_b02_20261024T072827Z','human_lgd_b02_gate_20261025T041003Z','property_lgd_b02_gate_20261025T051400Z','robot_lgd_b02_gate_20261024T103739Z')]
for ep in episodes:
    jobs=[('source_support_audit.json',lambda ep=ep:support(ep))]
    if ep!=episodes[0]: jobs += [('fixed_base_hazard_audit.json',lambda ep=ep:fixed(ep)),('source_background_audit.json',lambda ep=ep:background(episodes[0],ep))]
    for name,func in jobs:
        target=ep/name
        if target.exists(): print('EXISTS',target,flush=True);continue
        try: result=func()
        except Exception as exc: result={'passed':False,'error':f'{type(exc).__name__}: {exc}'}
        with target.open('x') as f:json.dump(result,f,indent=2)
        print('AUDIT',ep.name,name,{k:v for k,v in result.items() if k in ('passed','checks','error')},flush=True)
