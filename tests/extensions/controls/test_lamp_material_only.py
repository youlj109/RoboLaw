"""Asset edit-boundary checks, not visual or physical acceptance."""
from pathlib import Path
from pxr import Usd


ROOT=Path(__file__).resolve().parents[3]


def test_glass_revision_does_not_change_any_nonmaterial_property():
    directory=ROOT/'assets/extensions/controls/visual_20261027'
    original=Usd.Stage.Open(str(directory/'reading_lamp_r01.usda'))
    revised=Usd.Stage.Open(str(directory/'reading_lamp_glass_r02.usda'))
    def properties(stage):
        return {str(p.GetPath()):dict(type=p.GetTypeName(),schemas=list(p.GetAppliedSchemas()),
            attrs={a.GetName():str(a.Get()) for a in p.GetAttributes()},
            rels={r.GetName():list(map(str,r.GetTargets())) for r in p.GetRelationships()})
            for p in stage.Traverse() if '/Materials' not in str(p.GetPath())}
    assert properties(original)==properties(revised)
    shader=revised.GetPrimAtPath('/Lamp/Materials/FrostedGlass/Shader')
    assert abs(shader.GetAttribute('inputs:opacity').Get()-.18)<1e-6
