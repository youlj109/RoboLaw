"""Control-side contact provenance audit, separate from risk feature rules."""
from collections import Counter


def native_contact_trace661(context, native_report, point_count):
    """Stamp a real callback header with the coordinator's pending step.

    context must be bound immediately before World.step, after recording starts.
    A global step includes bootstrap physics; local step is the collector's
    next frame index. No context means setup/warmup, and produces no trace.
    """
    if context is None:
        return None
    return {**context, 'source': 'PhysX native contact-report callback',
            'source_contact_count': int(point_count),
            **{key: native_report.get(key) for key in ('actor0', 'actor1', 'collider0', 'collider1')}}


def audit_contact_trace661(rows, *, environment_root, environment_id, episode_id,
                           shared_roots=(), require_nonempty=True):
    root = str(environment_root).rstrip('/')
    roots = (root, *(str(p).rstrip('/') for p in shared_roots))
    def under(path, parent):
        return path == parent or path.startswith(parent + '/')
    errors, step_map, previous_global = [], {}, -1
    pairs = Counter()
    for index, row in enumerate(rows):
        prefix = {'row': index}
        if str(row.get('environment_id')) != str(environment_id) or str(row.get('episode_id')) != str(episode_id):
            errors.append({**prefix, 'reason': 'episode_or_environment_mismatch'})
        global_step, local_step = row.get('global_physics_step'), row.get('local_physics_step')
        if not isinstance(global_step, int) or not isinstance(local_step, int) or global_step < 0 or local_step < 0:
            errors.append({**prefix, 'reason': 'missing_or_invalid_step'})
        else:
            if global_step < previous_global:
                errors.append({**prefix, 'reason': 'global_step_reversed'})
            if local_step in step_map and step_map[local_step] != global_step:
                errors.append({**prefix, 'reason': 'local_step_maps_to_multiple_global_steps'})
            step_map[local_step] = global_step
            previous_global = global_step
        paths = [row.get(key) for key in ('actor0', 'actor1', 'collider0', 'collider1')]
        if any(not isinstance(p, str) or not p.startswith('/') for p in paths):
            errors.append({**prefix, 'reason': 'missing_native_contact_source'})
        elif not all(any(under(p, allowed) for allowed in roots) for p in paths) or not any(under(p, root) for p in paths):
            errors.append({**prefix, 'reason': 'foreign_environment_contact'})
        else:
            pairs[(paths[0], paths[1], paths[2], paths[3])] += 1
        if row.get('source') != 'PhysX native contact-report callback' or row.get('source_contact_count', 0) <= 0:
            errors.append({**prefix, 'reason': 'no_native_contact_points'})
    if require_nonempty and not rows:
        errors.append({'reason': 'empty_contact_evidence'})
    ordered = sorted(step_map.items())
    if any(g1 >= g2 for (_, g1), (_, g2) in zip(ordered, ordered[1:])):
        errors.append({'reason': 'local_global_step_order_mismatch'})
    return {'version': 'contact-source-audit661-v1', 'passed': not errors,
            'environment_id': str(environment_id), 'episode_id': str(episode_id),
            'nonempty_native_reports': len(rows), 'contact_physics_steps': len(step_map),
            'source_pairs': [{'actor0': key[0], 'actor1': key[1], 'collider0': key[2],
                              'collider1': key[3], 'report_count': count} for key, count in sorted(pairs.items())],
            'first_global_step': min(step_map.values()) if step_map else None,
            'last_global_step': max(step_map.values()) if step_map else None,
            'errors': errors}
