"""Reproducibly author anchored floor pedestals for path-driven trials."""
import json
from pathlib import Path

ROOT=Path(__file__).resolve().parents[3]
specs=json.loads((ROOT/'configs/extensions/doors/open_fridge_variants.json').read_text())
for variant,index,report in [('property',26,'property_refined_search'),('robot',5,'robot_pedestal_search')]:
    candidate=json.loads((ROOT/f'reports/avoidance_trials/doors/{report}.json').read_text())['candidates'][index]
    spec=candidate['spec'];lo=candidate['pedestal']['bottom'];hi=candidate['pedestal']['top'];size=[hi[i]-lo[i] for i in range(3)];height=size[2]
    asset=Path(f'assets/extensions/doors/{variant}_floor_pedestal.usda');(ROOT/asset).parent.mkdir(parents=True,exist_ok=True)
    (ROOT/asset).write_text('''#usda 1.0
(
    defaultPrim = "Pedestal"
    metersPerUnit = 1
    upAxis = "Z"
)
def Xform "Pedestal" {
    custom string provenance = "Locally authored anchored floor display pedestal; dimensions from CPU counterfactual search. No source kitchen geometry changed."
    def Cube "SolidColumn" (prepend apiSchemas = ["PhysicsCollisionAPI"]) {
        double size = 1
        color3f[] primvars:displayColor = [(0.19, 0.23, 0.28)]
        bool physics:collisionEnabled = true
        double3 xformOp:translate = (0, 0, HEIGHT_HALF)
        double3 xformOp:scale = (WIDTH, DEPTH, HEIGHT)
        uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:scale"]
    }
}
'''.replace('HEIGHT_HALF',str(height/2)).replace('WIDTH',str(size[0])).replace('DEPTH',str(size[1])).replace('HEIGHT',str(height)))
    spec['support']={'name':variant+'_pedestal','asset':str(asset),'position':[spec['position'][0],spec['position'][1],lo[2]],'static':True,'dimensions_m':size}
    spec['semantic_anchor']='anchored floor pedestal in the original refrigerator approach corridor; original baseline violates the category clearance during free pregrasp'
    spec['candidate_evidence']=f'reports/avoidance_trials/doors/{report}.json candidate {index}'
    specs[variant]=spec
if (ROOT/'configs/extensions/doors/fixed_source_layout.json').exists():specs['_fixed_source_layout']='configs/extensions/doors/fixed_source_layout.json'
(ROOT/'configs/extensions/doors/avoidance_variants.json').write_text(json.dumps(specs,indent=2)+'\n')
print('Authored 2 physical support assets and independent avoidance config; human remains pending.')
