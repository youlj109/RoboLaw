"""Independent new-review gates for neutral mechanisms and contact-before-motion."""
import argparse
import hashlib
import json
from pathlib import Path
from kitchen_safety.extensions.doors.evidence import mechanism_quality, contact_causality


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('episode', type=Path)
    p.add_argument('--neutral', type=Path, help='Same-variant 40-second neutral control')
    a = p.parse_args()
    root = a.episode
    output = root / 'review_audit.json'
    if output.exists():
        p.error('Refusing to overwrite review evidence')
    metadata = json.loads((root / 'metadata.json').read_text())
    result = json.loads((root / 'result.json').read_text())
    samples = json.loads((root / 'physics.json').read_text())['samples']
    limits = metadata['review_gates']
    neutral_mode = metadata['mode'] == 'neutral'
    mechanisms = mechanism_quality(samples, metadata, limits, neutral=neutral_mode)
    errors = list(mechanisms['errors'])
    report = {'passed': False, 'mode': metadata['mode'], 'mechanisms': mechanisms,
              'physics_sha256': hashlib.sha256((root / 'physics.json').read_bytes()).hexdigest(),
              'review_config_sha256': metadata['review_config_sha256'], 'errors': errors}
    if neutral_mode:
        observed = samples[-1]['time_s'] - samples[0]['time_s'] if samples else 0.
        if not result.get('neutral_completed') or observed < limits['neutral_seconds']:
            errors.append('Neutral control does not cover the predeclared 40 seconds')
        if any(r['contacts']['normal_force_abs_sum_n'] > limits['contact_force_noise_n'] for r in samples):
            errors.append('Robot contacted target in neutral control')
    else:
        report['contact_causality'] = contact_causality(samples, metadata, limits)
        errors.extend(report['contact_causality']['errors'])
        if not result['accepted']:
            errors.append('Physical collector rejected operation')
        if a.neutral is None:
            errors.append('Missing matching neutral control')
        else:
            nm = json.loads((a.neutral / 'metadata.json').read_text())
            nr = json.loads((a.neutral / 'review_audit.json').read_text())
            duration = samples[-1]['time_s'] - samples[0]['time_s']
            same = all(metadata[k] == nm[k] for k in (
                'task', 'variant', 'variant_config', 'controller_parameters',
                'source_yaml_sha256', 'review_config_sha256', 'robot_base', 'robot_yaw_deg'))
            coverage = nr['mechanisms'].get('duration_s', 0.) >= duration
            report['matching_neutral'] = {'path': str(a.neutral), 'same_parameters': same,
                                         'covers_operation_duration': coverage, 'passed': nr['passed']}
            if not (same and coverage and nr['passed']):
                errors.append('Neutral control differs, fails, or is shorter than operation')
    quality = json.loads((root / 'rgb_quality.json').read_text())
    if not quality['passed']:
        errors.append('Camera image quality failed')
    report['passed'] = not errors
    output.write_text(json.dumps(report, indent=2))
    print(json.dumps(report, indent=2))
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
