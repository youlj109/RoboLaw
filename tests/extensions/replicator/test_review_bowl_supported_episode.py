"""Independent audit negative cases, without the executed policy evaluator."""
from copy import deepcopy
import numpy as np
import pytest

from scripts.extensions.replicator.review_bowl_supported_episode import (
    BODY, FLOOR, classify, check_certificate, digest, native_check)


def sample(force=1., gap=.001):
    row = dict(physics_step=10, phase='release', action_index=2,
        exact_body=BODY, exact_filter=FLOOR, count=1, buffer_count=[[1]],
        buffer_start=[[0]], buffer_capacity=4096, points_world_m=[[.5, .5, .5]],
        normals_world=[[0., 0., 1.]], normal_forces_n=[force], separations_m=[gap])
    vector = np.array([0., 0., force])
    row.update(native_contact_force_vectors_n=[vector.tolist()], native_normal_force_vector_sum_n=vector.tolist(),
        original_filtered_force_n=vector.tolist(), native_minus_original_force_n=[0., 0., 0.],
        native_original_force_difference_norm_n=0.)
    source = dict(minimum_source_z_m=.2, source_gap_m=-.001, full_bowl_xy_inside_floor=True)
    return row, source, vector


def run(row, source, vector):
    return classify(source, row, vector, np.zeros(3), np.ones(3))


def test_loaded_source_intersection_not_automatic_failure():
    row, source, vector = sample()
    value = run(row, source, vector)
    assert value['passed'] and value['contact_state'] == 'loaded_support'
    assert value['penetration_tolerance_m'] is None


def test_signed_scalar_times_normal_is_the_supported_vector():
    row, source, vector = sample()
    row['normal_forces_n'] = [-1.]; row['normals_world'] = [[0., 0., -1.]]
    assert run(row, source, vector)['passed']


def test_zero_load_transition_not_loaded_support():
    row, source, vector = sample(force=0.)
    value = run(row, source, vector)
    assert value['passed'] and not value['original_loaded_support']
    assert value['contact_state'] == 'observed_nonpenetrating_contact_transition'


@pytest.mark.parametrize('change', ['wrong_pair', 'wrong_filter', 'missing_contact', 'downward',
    'outside_xy', 'source_below', 'point_below', 'source_outside', 'negative_unloaded_gap', 'nonfinite'])
def test_bad_contact_rejected(change):
    row, source, vector = sample(force=0.)
    if change == 'wrong_pair': row['exact_body'] = '/different'
    if change == 'wrong_filter': row['exact_filter'] = '/different'
    if change == 'missing_contact': row['count'] = 2
    if change == 'downward': row['normal_forces_n'] = [-1.]
    if change == 'outside_xy': row['points_world_m'][0][0] = 2.
    if change == 'source_below': source['minimum_source_z_m'] = -1e-9
    if change == 'point_below': row['points_world_m'][0][2] = -1e-9
    if change == 'source_outside': source['full_bowl_xy_inside_floor'] = False
    if change == 'negative_unloaded_gap': row['separations_m'] = [-1e-9]
    if change == 'nonfinite': row['normal_forces_n'] = [float('nan')]
    assert not run(row, source, vector)['passed']


def test_native_hashless_declaration_cannot_replace_vector_recompute():
    row, _, vector = sample()
    assert native_check(row, 10, 'release', 2, vector)['passed']
    row['native_normal_force_vector_sum_n'] = [0., 0., 99.]
    assert not native_check(row, 10, 'release', 2, vector)['passed']


@pytest.mark.parametrize('change', ['pose', 'step', 'floor', 'payload', 'native', 'hash', 'unloaded'])
def test_retreat_certificate_binding(change):
    row, source, vector = sample()
    value = run(row, source, vector)
    cert = dict(physics_step=10, payload_pose=np.eye(4).tolist(), floor_geometry_sha256='f',
        payload_source_sha256='p', native_observation=deepcopy(row),
        observed_support=dict(passed=True, contact_state='loaded_support'))
    cert['certificate_sha256'] = digest(cert)
    pose, step, floor, payload = np.eye(4), 10, 'f', 'p'
    assert check_certificate(cert, step, pose, floor, payload, row, value)['passed']
    if change == 'pose': pose[0, 3] += 1e-6
    if change == 'step': step += 1
    if change == 'floor': floor = 'different'
    if change == 'payload': payload = 'different'
    if change == 'native': row['separations_m'][0] += 1e-6
    if change == 'hash': cert['certificate_sha256'] = 'wrong'
    if change == 'unloaded': value['contact_state'] = 'observed_nonpenetrating_contact_transition'
    assert not check_certificate(cert, step, pose, floor, payload, row, value)['passed']
