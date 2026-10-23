"""Exercise the actual source predicate method without starting Isaac/Kit."""
import ast
import math
from types import SimpleNamespace

import torch

from kitchen_safety.extensions.controls.task_contract import ROOT


def source_level_function():
    path=ROOT/'third_party/IsaacLab-Arena/isaaclab_arena/affordances/turnable.py'
    tree=ast.parse(path.read_text())
    cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='Turnable')
    method=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='get_turning_level')
    module=ast.Module(body=[ast.ImportFrom(module='__future__',names=[ast.alias(name='annotations')],level=0),method],type_ignores=[])
    namespace={'torch':torch, 'SceneEntityCfg':lambda name:SimpleNamespace(name=name),
        'get_unnormalized_joint_position':lambda env,cfg:env,
        'normalize_value':lambda value,low,high:(value-low)/(high-low)}
    exec(compile(ast.fix_missing_locations(module),str(path),'exec'),namespace)
    return namespace['get_turning_level']


def test_original_temperature_level_three_boundaries():
    obj=SimpleNamespace(name='oven_temperature_knob',min_level_angle_rad=math.pi/6,
        max_level_angle_rad=5*math.pi/6,num_levels=5,
        _add_joint_name_to_scene_entity_cfg=lambda cfg:cfg)
    degrees=[0.,29.999,30.001,59.999,60.001,90.001,119.999,120.001,135.,149.999,150.001]
    actual=source_level_function()(obj,torch.deg2rad(torch.tensor(degrees,dtype=torch.float64)))
    assert actual.tolist()==[-1,-1,0,0,1,2,2,3,3,3,4]


def test_exact_mathematical_boundary_is_not_assumed_rounded():
    obj=SimpleNamespace(name='oven_temperature_knob',min_level_angle_rad=math.pi/6,
        max_level_angle_rad=5*math.pi/6,num_levels=5,
        _add_joint_name_to_scene_entity_cfg=lambda cfg:cfg)
    actual=source_level_function()(obj,torch.deg2rad(torch.tensor([60.],dtype=torch.float64)))
    assert actual.item()==0  # Source floor preserves the small negative roundoff.
