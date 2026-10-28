"""Declare one fixed-layout axial contact candidate for the next review round."""
import argparse
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, required=True)
    parser.add_argument('--round', type=int, required=True)
    parser.add_argument('--angle-deg', type=float, required=True)
    parser.add_argument('--tilt-deg', type=float, default=45.)
    parser.add_argument('--tool-roll-deg', type=float, default=-90.)
    parser.add_argument('--contact-offset-m', type=float)
    parser.add_argument('--phase-deg', type=float)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError(args.output)
    if not np.isfinite(args.angle_deg) or abs(args.angle_deg) > 180:
        raise ValueError('Expected an axial angle within [-180, 180] degrees')
    if not np.isfinite(args.tilt_deg) or abs(args.tilt_deg) >= 90:
        raise ValueError('Tool must retain a positive component along the knob axis')
    if not np.isfinite(args.tool_roll_deg) or abs(args.tool_roll_deg) > 180:
        raise ValueError('Expected a finite tool roll within [-180, 180] degrees')
    config = json.loads(args.source.read_text())
    if args.phase_deg is not None:
        if not np.isfinite(args.phase_deg) or abs(args.phase_deg) > 180:
            raise ValueError('Expected a finite contact phase within [-180, 180] degrees')
        config['friction_start_angle_deg'] = args.phase_deg
    if args.contact_offset_m is not None:
        if not np.isfinite(args.contact_offset_m) or not -.02 <= args.contact_offset_m <= 0:
            raise ValueError('Contact start must stay on the robot side of the original anchor')
        config['friction_start_x_offset_m'] = args.contact_offset_m
    base = Rotation.from_euler('y', args.tilt_deg, degrees=True) * Rotation.from_euler('x', args.tool_roll_deg, degrees=True)
    config['grasp_rotation_world'] = (
        Rotation.from_euler('x', args.angle_deg, degrees=True) * base
    ).as_matrix().tolist()
    config['benchmark_execution_round'] = dict(
        round=args.round, source=str(args.source), axial_rotation_deg=args.angle_deg,
        tilt_deg=args.tilt_deg,
        tool_roll_deg=args.tool_roll_deg,
        contact_phase_deg=config.get('friction_start_angle_deg', 0.),
        contact_start_offset_m=config.get('friction_start_x_offset_m', -.02),
        scope='One declared contact-family candidate; unchanged frozen layout, '
              'auxiliary connection references, original contact anchor, knob axis, '
              'target level and margins. All contact changes are explicit above. '
              'CPU endpoint/connection diagnosis first; no same-round retry or acceptance claim.')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(config, indent=2) + '\n')
    print(args.output)


if __name__ == '__main__':
    main()
