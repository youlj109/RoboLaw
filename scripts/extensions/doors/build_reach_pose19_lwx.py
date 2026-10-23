"""Build one longer, lower reach using the same MPFB rig and unchanged bone lengths."""
import hashlib
import json
from pathlib import Path
import subprocess

asset = Path('assets/extensions/doors/human_reach_lean_low_lwx_19')
if asset.exists():
    raise FileExistsError(asset)
command = ['runtime/tools/blender-4.0.2-linux-x64/blender', '--background',
           'assets/people/mpfb_corridor/posed.blend', '--python',
           'scripts/extensions/doors/repose_human_both_lwx.py', '--', '--asset-dir', str(asset),
           '--wrist', '-.90', '.15', '1.25', '--auto-reachable-target', '--elbow-up',
           '--clavicle-y-deg', '45', '--torso-y-deg', '-18']
subprocess.run(command, check=True)
subprocess.run(['docker', 'exec', 'robolaw-wenxuanli', 'env', 'PYTHONPATH=/workspace',
                '/workspace/.venv-arena/bin/python', 'scripts/extensions/doors/wrap_reposed_human.py',
                '--asset-dir', str(asset)], check=True)
pose = json.loads((asset / 'pose.json').read_text())
if not (pose['wrist_error_m'] < 1e-5 and pose['max_unchanged_bone_error_m'] < 1e-5
        and pose['max_bone_length_change_m'] < 1e-5):
    raise RuntimeError('Pose geometry validation failed')
files = {str(p): {'bytes': p.stat().st_size, 'sha256': hashlib.sha256(p.read_bytes()).hexdigest()}
         for p in asset.rglob('*') if p.is_file()}
(asset / 'artifact_manifest_lwx.json').write_text(json.dumps({
    'build_command': command, 'files': files,
    'scope': 'Numerical rig and export checks only. No scene, visual pose, obstruction, or physical task acceptance.',
    'requested_wrist': pose['requested_wrist_world_asset_m'],
    'reachable_wrist': pose['actual_wrist_world_asset_m'],
    'target_clamp': 'Requested wrist is clamped to original arm reach, with a 12mm reach reserve.'
}, indent=2) + '\n')
print('POSE19_BUILT', len(files), pose['actual_wrist_world_asset_m'], flush=True)
