import json
from copy import deepcopy

import pytest

from kitchen_safety.extensions.replicator.bowl_paired_query import ROOT
from kitchen_safety.extensions.replicator.bowl_safety_provenance_review import projection


@pytest.fixture
def inputs():
    p=ROOT/'datasets/replicator/g_shape_bowl_sink/property/property75_02'
    return [json.loads((p/name).read_text()) for name in
        ('task_collection_provenance.json','physics_metadata.json','result.json')]


def test_exact_original_and_additive_role_mapping(inputs):
    raw,meta,result=inputs
    before=deepcopy(raw)
    derived,mapped,evidence=projection(raw,meta,result)
    assert raw==before
    assert 'obstacle' not in derived['source_layout'] and derived['source_layout']==mapped['source_layout']
    assert evidence['canonical_original_task_verified']
    for key in raw['entries']:
        a,b=deepcopy(raw['entries'][key]),deepcopy(derived['entries'][key])
        a.pop('kind');b.pop('kind')
        assert a==b


@pytest.mark.parametrize('changed',['canonical_bytes','augmented_bytes','profile','asset_hash','unknown_role','missing_canonical','extra_source_field'])
def test_role_projection_cannot_excuse_real_provenance_failure(inputs,changed):
    raw,meta,result=inputs
    canonical='configs/extensions/replicator/g_shape_bowl_sink_source_layout.json'
    actual=next(k for k,v in raw['entries'].items() if v['kind']=='source_layout_input')
    if changed=='canonical_bytes':raw['entries'][canonical]['source_text']+=' '
    if changed=='augmented_bytes':raw['entries'][actual]['source_text']+=' '
    if changed=='profile':result['controller_profile']['source_layout_sha256']='0'*64
    if changed=='asset_hash':result['safety_policy']['asset_inputs']['assets/obstacle_library/property/vase.usda']='0'*64
    if changed=='unknown_role':raw['entries'][canonical]['kind']='other'
    if changed=='missing_canonical':del raw['entries'][canonical]
    if changed=='extra_source_field':
        for value in (raw['source_layout'],meta['source_layout'],result['layout']):value['robot_yaw_deg']=5.
    with pytest.raises(ValueError):projection(raw,meta,result)
