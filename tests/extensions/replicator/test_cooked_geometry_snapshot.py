"""CPU-only data/identity contracts; synthetic hulls are not physics evidence."""
from copy import deepcopy
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from kitchen_safety.extensions.replicator.cooked_geometry_snapshot import (
    native_inventory, read_declaration, representation, transform_points, validate_convex,
)

ROOT = Path(__file__).resolve().parents[3]


def tetra():
    return dict(vertices_local=[[0,0,0],[1,0,0],[0,1,0],[0,0,1]],
        indices=[0,2,1,0,1,3,0,3,2,1,2,3], polygons=[
            dict(index_base=0, num_vertices=3, plane_local=[0,0,-1,0]),
            dict(index_base=3, num_vertices=3, plane_local=[0,-1,0,0]),
            dict(index_base=6, num_vertices=3, plane_local=[-1,0,0,0]),
            dict(index_base=9, num_vertices=3, plane_local=[1,1,1,-1])])


def callback_hull(raw):
    return SimpleNamespace(vertices=raw['vertices_local'], indices=raw['indices'], polygons=[
        SimpleNamespace(index_base=p['index_base'], num_vertices=p['num_vertices'], plane=p['plane_local']) for p in raw['polygons']])


def test_full_local_scale_rotation_translation_applies_once():
    matrix = np.array([[0,-3,0,10],[2,0,0,20],[0,0,4,30],[0,0,0,1.]])
    out = validate_convex(tetra(), matrix)
    assert np.array_equal(out['vertices_world'], [[10,20,30],[10,22,30],[7,20,30],[10,20,34]])
    assert out['plane_vertex_max_residual_local_units'] == 0
    assert len(out['triangles']) == 4


@pytest.mark.parametrize('bad', ['nan','bad_index','fractional_index','overlap','gap','short_face','zero_normal','nan_plane','duplicate_vertex','singular_matrix','projective_matrix'])
def test_malformed_cooking_result_refused(bad):
    raw, matrix = deepcopy(tetra()), np.eye(4)
    if bad == 'nan': raw['vertices_local'][0][0] = float('nan')
    if bad == 'bad_index': raw['indices'][0] = 99
    if bad == 'fractional_index': raw['indices'][0] = .5
    if bad == 'overlap': raw['polygons'][1]['index_base'] = 0
    if bad == 'gap': raw['indices'].append(0)
    if bad == 'short_face': raw['polygons'][0]['num_vertices'] = 2
    if bad == 'zero_normal': raw['polygons'][0]['plane_local'] = [0,0,0,0]
    if bad == 'nan_plane': raw['polygons'][0]['plane_local'][0] = float('nan')
    if bad == 'duplicate_vertex': raw['indices'][0] = raw['indices'][1]
    if bad == 'singular_matrix': matrix[0] = 0
    if bad == 'projective_matrix': matrix[3,0] = .1
    with pytest.raises(ValueError): validate_convex(raw, matrix)


def test_unsupported_floor_not_replaced_with_a_convenient_convex():
    report = representation('RESULT_ERROR_UNSUPPORTED_APPROXIMATION', [], np.eye(4))
    assert report['available'] is False
    assert report['runtime_shape_equivalence_verified'] is False
    with pytest.raises(ValueError): representation('RESULT_VALID', [], np.eye(4))
    with pytest.raises(ValueError): representation('RESULT_ERROR_COOKING_FAILED', [callback_hull(tetra())], np.eye(4))


def test_callback_order_not_native_slot_mapping():
    report = representation('RESULT_VALID', [callback_hull(tetra())], np.eye(4))
    assert report['hull_count'] == 1
    assert report['hulls'][0]['callback_index'] == 0
    assert report['runtime_shape_equivalence_verified'] is False


def test_one_body_max_shapes_count_does_not_prove_geometry_or_slot_identity():
    offsets = dict(rest_offsets=[[0,0]], contact_offsets=[[.001,.001]])
    report = native_inventory(['/bowl'], 1, 2, offsets, ['/bowl/mesh'], 2)
    assert report['actual_shape_count'] == 2
    assert report['count_matches_cooking'] is True
    assert report['native_shape_slot_to_hull'] is None
    assert report['runtime_shape_equivalence_verified'] is False
    assert report['native_shape_slot_mapping_verified'] is False


def test_multiple_bodies_capacity_has_no_per_body_valid_mask():
    offsets = dict(rest_offsets=[[0,0],[0,0]], contact_offsets=[[.001,.001],[.001,0]])
    report = native_inventory(['/a','/b'], 2, 2, offsets, ['/a/mesh','/b/mesh'], 2)
    assert report['actual_shape_count'] is None
    assert report['count_matches_cooking'] is None


@pytest.mark.parametrize('bad', ['missing_body','wrong_shape','nan','zero_count'])
def test_native_inventory_refuses_ambiguous_or_invalid_arrays(bad):
    paths, count, maximum = ['/bowl'], 1, 2
    offsets = dict(rest_offsets=[[0,0]], contact_offsets=[[.001,.001]])
    if bad == 'missing_body': paths=[]
    if bad == 'wrong_shape': offsets['rest_offsets'] = [[0]]
    if bad == 'nan': offsets['contact_offsets'][0][0] = float('nan')
    if bad == 'zero_count': count=0
    with pytest.raises(ValueError): native_inventory(paths,count,maximum,offsets,['/bowl/mesh'],2)


def test_real_source_declaration_bound_to_assets_without_kit():
    declaration = read_declaration(ROOT/'configs/extensions/replicator/g_bowl_cooked_snapshot.json', ROOT)
    assert declaration['maximum_control_actions'] == 0
    assert declaration['explicit_simulation_steps'] == 0


def test_declaration_cannot_gain_control_actions_or_change_source(tmp_path):
    import json
    original = json.loads((ROOT/'configs/extensions/replicator/g_bowl_cooked_snapshot.json').read_text())
    for mutate in ('actions','hash','floor'):
        value = deepcopy(original)
        if mutate == 'actions': value['maximum_control_actions'] = 1
        if mutate == 'hash': value['source_sha256'][next(iter(value['source_sha256']))] = '0'*64
        if mutate == 'floor': value['targets']['floor'] += '/another'
        path = tmp_path/(mutate+'.json'); path.write_text(json.dumps(value))
        with pytest.raises(ValueError): read_declaration(path, ROOT)
