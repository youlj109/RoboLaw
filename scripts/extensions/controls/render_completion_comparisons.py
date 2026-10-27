"""Original-speed, same-camera baseline/variant display copies; raw data untouched."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess

import imageio_ffmpeg
from kitchen_safety.dataset import decode_video


def sha(path):
    digest=hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda:stream.read(1024*1024),b''):digest.update(chunk)
    return digest.hexdigest()


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--summary',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    args=p.parse_args()
    report=json.loads(args.summary.read_text())
    rows=report['scenes']
    if not report['all_nine_physical_tasks_complete'] or len(rows)!=9:
        raise ValueError('Nine independently accepted recordings required')
    args.output.mkdir(parents=True,exist_ok=False)
    manifest=dict(scope='Display-only comparisons: LEFT baseline, RIGHT variant. Original 15 fps and start time retained; '
        'shorter video holds its final frame, which is not additional simulated data. '
        'Not obstacle-necessity/counterfactual evidence.',summary_sha256=sha(args.summary),scenes=[])
    for row in rows:
        if not row['physical_task_accepted']:raise ValueError(row['scene'])
        base,variant=Path(row['baseline']),Path(row['episode'])
        paths=[directory/'external_camera.mp4' for directory in (base,variant)]
        cameras=[json.loads((directory/'cameras.json').read_text())['external_camera'] for directory in (base,variant)]
        if cameras[0]!=cameras[1]:raise ValueError('Camera configurations differ')
        metas=[json.loads((directory/'result.json').read_text()) for directory in (base,variant)]
        if any(abs(meta['dt']-1/15)>1e-9 for meta in metas):raise ValueError('Expected source 15 fps')
        counts=[json.loads((directory/'validation.json').read_text())['record_count'] for directory in (base,variant)]
        destination=args.output/(row['scene']+'.mp4')
        # Captions live in the webpage/manifest; the bundled FFmpeg has no drawtext.
        filters='[0:v]scale=640:360[a];[1:v]scale=640:360[b];[a][b]hstack=inputs=2:shortest=0[v]'
        subprocess.run([imageio_ffmpeg.get_ffmpeg_exe(),'-n','-loglevel','error',
            '-i',str(paths[0]),'-i',str(paths[1]),'-filter_complex',filters,'-map','[v]',
            '-an','-c:v','libx264','-preset','fast','-crf','20','-pix_fmt','yuv420p',
            '-movflags','+faststart',str(destination)],check=True)
        decoded=decode_video(destination,max(counts),expected_dt=1/15)
        manifest['scenes'].append(dict(scene=row['scene'],video=str(destination),
            baseline=str(paths[0]),variant=str(paths[1]),input_frames=counts,
            baseline_sha256=sha(paths[0]),variant_sha256=sha(paths[1]),
            video_sha256=sha(destination),decoded=decoded))
        (args.output/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
        print('COMPARISON',row['scene'],max(counts),flush=True)


if __name__=='__main__':main()
