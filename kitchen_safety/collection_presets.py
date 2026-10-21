"""Resolve collection parameters without importing Arena or starting simulation.

The existing reproduction JSON is the only preset source. Only obstacle IDs
listed in ``nonhuman10.ids`` inherit its ``collector`` settings, followed by
optional ``collector_by_id[obstacle_id]`` settings. Other IDs receive no preset.

Runtime defaults remain owned by the caller. Resolution order is caller
defaults, obstacle preset, then explicit CLI overrides. Values are merged by
key presence: zero, False, and None are all meaningful explicit overrides.

For CLI parsing, use ``default=argparse.SUPPRESS`` on overridable arguments,
then call ``explicit_collector_overrides(args)`` before filling defaults. A
BooleanOptionalAction with the same suppressed default supports explicit
``--supported-release`` and ``--no-supported-release``. This module does not
change an argument parser or infer a scene variant.
"""

import json
from pathlib import Path
from typing import Mapping


PRESETS_PATH = Path(__file__).resolve().parents[1] / 'configs/reproduction_presets.json'

COLLECTOR_OPTION_NAMES = (
    'placement_strategy',
    'grasp_tilt_deg',
    'grasp_yaw_deg',
    'grasp_offset_z_m',
    'grasp_offset_y_m',
    'transport_bottle_height_m',
    'supported_release',
    'transport_tilt_cap_deg',
    'placement_tcp_height_m',
)


def _collector_mapping(values, source):
    if not isinstance(values, Mapping):
        raise ValueError(f'{source} must be an object of collector options')
    unknown = set(values) - set(COLLECTOR_OPTION_NAMES)
    if unknown:
        raise ValueError(f'Unknown collector options in {source}: {sorted(unknown)}')
    return dict(values)


def explicit_collector_overrides(parsed):
    """Extract present collector options from an explicit-only parse result.

    ``parsed`` may be a mapping or argparse Namespace. Its collector arguments
    must have suppressed parser defaults; otherwise a parser-injected value
    cannot be distinguished from a user's explicit value. Noncollector CLI
    options such as seed, output, or variant are ignored here.
    """
    values = parsed if isinstance(parsed, Mapping) else vars(parsed)
    return {name: values[name] for name in COLLECTOR_OPTION_NAMES if name in values}


def obstacle_collector_defaults(obstacle_id='', *, presets=None):
    """Return configured options for one listed nonhuman ID, or an empty dict.

    ``presets`` can be the already-loaded reproduction JSON. Passing it avoids
    rereading the file when constructing several jobs and makes CPU inspection
    straightforward. Preset status fields are descriptive metadata, not gates.
    """
    if not obstacle_id:
        return {}
    if presets is None:
        presets = json.loads(PRESETS_PATH.read_text())
    group = presets.get('nonhuman10', {})
    if obstacle_id not in group.get('ids', []):
        return {}
    common = _collector_mapping(group.get('collector', {}), 'nonhuman10.collector')
    per_id = group.get('collector_by_id', {})
    if not isinstance(per_id, Mapping):
        raise ValueError('nonhuman10.collector_by_id must be an object keyed by obstacle ID')
    specific = _collector_mapping(
        per_id.get(obstacle_id, {}), f'nonhuman10.collector_by_id.{obstacle_id}')
    return {**common, **specific}


def resolve_collector_options(obstacle_id='', *, base_defaults=None,
                              cli_overrides=None, presets=None):
    """Merge caller defaults, ID settings, and explicit CLI values in that order.

    A caller can supply its existing defaults unchanged. This helper does not
    invent defaults for humans or base scenes, inspect old successes, select
    a variant, or start any simulation. Runtime type/range and coupled-option
    validation should run on the resolved options in the collector.
    """
    defaults = _collector_mapping(
        {} if base_defaults is None else base_defaults, 'caller defaults')
    overrides = _collector_mapping(
        {} if cli_overrides is None else cli_overrides, 'explicit CLI overrides')
    return {**defaults, **obstacle_collector_defaults(obstacle_id, presets=presets),
            **overrides}
