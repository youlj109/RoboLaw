"""Synthetic CPU provenance checks; no scene or physical evidence is fabricated."""
import json
from types import SimpleNamespace as N

import pytest

from kitchen_safety.extensions.replicator.collection_evidence import CollectionProvenance
from kitchen_safety.extensions.replicator.mustard_family_preparation import sha
from kitchen_safety.extensions.replicator.mustard_family_evidence import MustardFamilyProvenance
from kitchen_safety.extensions.replicator.task_collection_evidence import audit_provenance


def fixture(tmp_path, monkeypatch, housing_only=True):
    import kitchen_safety.extensions.replicator.mustard_family_evidence as family
    import kitchen_safety.extensions.replicator.task_collection_evidence as shared
    root=tmp_path/'repo';root.mkdir()
    monkeypatch.setattr(family,'ROOT',root)
    base='kitchen_safety/extensions/replicator/background_physics.py'
    leaf='kitchen_safety/extensions/replicator/mustard_family_background.py'
    monkeypatch.setattr(shared,'TASK_CODE',(base,leaf))
    monkeypatch.setattr(family,'FAMILY_CODE',(base,leaf))
    for name,text in {base:'base=1\n',leaf:'leaf=1\n','source.usda':'kitchen',
            'overlay.usda':'overlay','task.yaml':'task', 'bottle.usd':'bottle','bowl.usd':'bowl'}.items():
        path=root/name;path.parent.mkdir(parents=True,exist_ok=True);path.write_text(text)
    runtime=tmp_path/'runtime';runtime.mkdir();assembly=runtime/'assembled_droid.usd';assembly.write_text('stand')
    alias=root/'runtime/home/cache/assembled_droid.usd';alias.parent.mkdir(parents=True);alias.symlink_to(assembly)
    declared=str(alias.relative_to(root))
    assets={name:sha(root/name) for name in ['source.usda','bottle.usd','bowl.usd',declared]}
    layout=dict(task_id='synthetic_family_task')
    layout_path=root/'layout.json';layout_path.write_text(json.dumps(layout))
    profile=dict(task_id=layout['task_id'],source_layout_sha256=sha(layout_path),
        source_kitchen_sha256=assets['source.usda'],source_object_sha256=dict(mustard_bottle=assets['bottle.usd'],bowl=assets['bowl.usd']))
    profile_path=root/'profile.json';profile_path.write_text(json.dumps(profile))
    recorder=MustardFamilyProvenance.__new__(MustardFamilyProvenance)
    CollectionProvenance.__init__(recorder,root,code_paths=(base,leaf))
    recorder.layout=layout;recorder.task_id=layout['task_id'];recorder.scene_bound=False
    recorder.controller_sha256=sha(profile_path);recorder.relation_asset_aliases={}
    recorder.capture(layout_path,'source_layout_input',snapshot=True)
    recorder.capture(profile_path,'controller_input',snapshot=True)
    source=dict(source_yaml=str(root/'task.yaml'),source_yaml_sha256=sha(root/'task.yaml'),assets_sha256=assets)
    recipe=leaf if housing_only else base
    physical=dict(recipe='additional-mustard-fixed-refrigerator-housing-only-v1' if housing_only else 'replicator-source-background-v2',
        source=str(root/'source.usda'),source_sha256=assets['source.usda'],recipe_source_path=recipe,
        recipe_sha256=sha(root/recipe),overlay_sha256=sha(root/'overlay.usda'))
    definition=N(resolved_task_layout=layout,background_physics_provenance=physical,
        scene=N(assets={'kitchen':N(usd_path=str(root/'overlay.usda'))}),source_relation_audit=source,
        embodiment=N(scene_config=N(robot=N(spawn=N(usd_path=str(assembly))))))
    metadata=dict(task_id=layout['task_id'],source_layout=layout,background_physics=physical,
        source_relations=source,task_object_names=['mustard_bottle','bowl'],source_hashes=profile['source_object_sha256'])
    return recorder,definition,metadata,declared,assembly


@pytest.mark.parametrize('housing_only',[True,False])
def test_explicit_source_alias_and_actual_assembly_survive_independent_audit(tmp_path,monkeypatch,housing_only):
    recorder,definition,metadata,declared,assembly=fixture(tmp_path,monkeypatch,housing_only)
    recorder.bind_environment(definition)
    assert recorder.entries[declared]['declared_path_alias']==declared
    assert recorder.entries[declared]['canonical_capture_key']==str(assembly)
    for key,digest in definition.source_relation_audit['assets_sha256'].items():
        assert recorder.entries[key]['initial_sha256']==recorder.entries[key]['expected_sha256']==digest
    report=recorder.finish_task(tmp_path,22)
    assert report['passed'] and report['checked_physics_step']==22
    assert report['actual_robot_assembly']==dict(runtime_spawn_path=str(assembly),canonical_capture_key=str(assembly),
        source_relation_asset_path=declared,expected_sha256=sha(assembly))
    # The auditor's default ROOT differs in this synthetic fixture. Use literal
    # source identities, as the real container's shared modules agree on ROOT.
    metadata['source_relations']['source_yaml']='task.yaml'
    result=audit_provenance(tmp_path/'task_collection_provenance.json',metadata,22)
    assert result['passed'] and result['accepted_input_aliases']==[]


def test_changed_runtime_assembly_rejected_before_use(tmp_path,monkeypatch):
    recorder,definition,_,_,assembly=fixture(tmp_path,monkeypatch)
    other=tmp_path/'different';other.mkdir();bad=other/assembly.name;bad.write_text('different stand')
    definition.embodiment.scene_config.robot.spawn.usd_path=str(bad)
    with pytest.raises(ValueError,match='runtime assembled DROID'):
        recorder.bind_environment(definition)
    assert not recorder.scene_bound


def test_declared_asset_change_rejected_before_use(tmp_path,monkeypatch):
    recorder,definition,_,_,assembly=fixture(tmp_path,monkeypatch)
    assembly.write_text('changed declared stand')
    with pytest.raises(ValueError,match='original source input differs'):
        recorder.bind_environment(definition)
    assert not recorder.scene_bound


@pytest.mark.parametrize('change',['missing_path','wrong_path','changed_recipe'])
def test_uncaptured_or_misidentified_recipe_rejected_before_use(tmp_path,monkeypatch,change):
    recorder,definition,_,_,_=fixture(tmp_path,monkeypatch)
    info=definition.background_physics_provenance
    if change=='missing_path':info.pop('recipe_source_path')
    elif change=='wrong_path':info['recipe_source_path']='wrong.py'
    else:(recorder.root/info['recipe_source_path']).write_text('later code')
    with pytest.raises(ValueError,match='recipe'):
        recorder.bind_environment(definition)
    assert not recorder.scene_bound
