"""Map declared route boundaries by carried-object geometry only.

This module has no collision, planning or acceptance queries. It annotates an
existing path and never changes its states, endpoints or constraint phases.
"""
from copy import deepcopy

import numpy as np


MAX_ANCHOR_ERROR_M = .002


def map_guide_phase_boundaries(path, kin, attachment, guide_metadata):
    """Return fixed source anchors mapped to internal final-path samples.

    ``guide_metadata`` is the self-contained metadata from
    ``get_transport_guides``. XYZ is evaluated with the supplied measured
    attachment in the frame of ``kin.fk``. Multiple disconnected windows within
    2 mm are ambiguous and leave the path unsegmented. Within one window use
    the closest XYZ, resolving exact ties by earliest path index. No joint-space
    distance or obstacle query participates in selecting a boundary.

    Callers that replaced a route suffix must separately establish preservation
    of the source boundary before applying this annotation to the final path.
    """
    def rejected(reason):
        return {'applied': False, 'boundaries': [], 'reason': reason}

    if not isinstance(guide_metadata, dict):
        return rejected('missing_phase_boundaries')
    declarations = guide_metadata.get('phase_boundaries', [])
    if not declarations:
        return rejected('missing_phase_boundaries')
    if not isinstance(declarations, list):
        return rejected('invalid_phase_boundaries')
    try:
        positions = np.asarray(path, dtype=float)
        measured_attachment = np.asarray(attachment, dtype=float)
    except (TypeError, ValueError):
        return rejected('invalid_geometry_input')
    if (positions.ndim != 2 or positions.shape[1] != 7 or len(positions) < 3
            or not np.isfinite(positions).all() or measured_attachment.shape != (4, 4)
            or not np.isfinite(measured_attachment).all()):
        return rejected('invalid_geometry_input')

    def carried_xyz(q):
        pose = np.asarray(kin.fk(q.copy()), dtype=float)
        if pose.shape != (4, 4) or not np.isfinite(pose).all():
            raise ValueError('Invalid forward-kinematics pose')
        return (pose @ measured_attachment)[:3, 3]

    try:
        xyz = np.asarray([carried_xyz(q) for q in positions])
    except (TypeError, ValueError, RuntimeError):
        return rejected('invalid_forward_kinematics')
    mapped = []
    for boundary in declarations:
        if not isinstance(boundary, dict):
            return rejected('invalid_phase_boundary')
        try:
            source_q = np.asarray(boundary['source_anchor_q'], dtype=float)
            before, after = boundary['before_phase'], boundary['after_phase']
            if source_q.shape != (7,) or not np.isfinite(source_q).all():
                return rejected('invalid_source_anchor')
            if not isinstance(before, str) or not isinstance(after, str) or not before or not after:
                return rejected('invalid_phase_boundary')
            anchor_xyz = carried_xyz(source_q)
        except (KeyError, TypeError, ValueError, RuntimeError):
            return rejected('invalid_source_anchor')
        distances = np.linalg.norm(xyz - anchor_xyz, axis=1)
        within = np.flatnonzero(distances <= MAX_ANCHOR_ERROR_M)
        if not len(within):
            return rejected('anchor_outside_tolerance')
        if np.any(np.diff(within) > 1):
            return rejected('ambiguous_anchor_windows')
        # np.argmin uses the earliest sample for equal geometric distances.
        index = int(np.argmin(distances))
        if index == 0 or index == len(positions) - 1:
            return rejected('anchor_at_path_endpoint')
        if mapped and index <= mapped[-1]['path_index']:
            return rejected('source_boundary_order_not_preserved')
        ties = np.flatnonzero(distances == distances[index]).tolist()
        source_anchor = {
            'guide_id': guide_metadata.get('guide_id'),
            'guide_internal_index': boundary.get('guide_internal_index'),
            'source_state_index': boundary.get('source_state_index'),
            'q_arm': source_q.tolist(), 'carried_xyz_m': anchor_xyz.tolist(),
            'source': deepcopy(boundary.get('source', {})),
        }
        mapped.append({
            'id': boundary.get('id'), 'path_index': index,
            'before_phase': before, 'after_phase': after,
            'constraint_phase': boundary.get('constraint_phase', 'above_bowl'),
            'source_anchor': source_anchor, 'mapped_carried_xyz_m': xyz[index].tolist(),
            'error_m': float(distances[index]), 'maximum_error_m': MAX_ANCHOR_ERROR_M,
            'candidate_window': [int(within[0]), int(within[-1])],
            'nearest_xyz_tie_indices': ties, 'xyz_tie': len(ties) > 1,
            'selection': 'closest_carried_xyz_then_earliest_index_in_single_tolerance_window',
        })
    return {'applied': True, 'boundaries': mapped, 'reason': 'mapped_source_geometry'}
