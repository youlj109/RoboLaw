"""Export original three views plus montage as default-MPFB successes arrive."""
import argparse
import json
from pathlib import Path
import time

from export_collection_previews import export_episode
from summarize_human_collection import ROOT


def write_json(path, value):
    temporary = path.with_suffix(path.suffix + '.tmp')
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2) + '\n')
    temporary.replace(path)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=ROOT / 'datasets/augmentation/human_default20_n4_20261019')
    args = parser.parse_args()
    root = args.output.resolve()
    exported = {}
    while True:
        selection = json.loads((root / 'successful_episodes.json').read_text())
        for row in selection['episodes']:
            index = row['episode_index']
            if index in exported:
                if exported[index]['path'] != row['source_episode']:
                    raise ValueError(f'Selected source changed for episode {index}')
                continue
            result = export_episode(row['source_episode'], render_root=row['rgb_directory'])
            result['episode_index'] = index
            write_json(root / f'preview_episode_{index:04d}.json', [result])
            exported[index] = result
            print(json.dumps({k:result.get(k) for k in ('episode_index', 'passed', 'frames', 'skipped_existing', 'elapsed_seconds')}, ensure_ascii=False), flush=True)
            write_json(root / 'preview_summary.json', dict(
                target_episodes=selection['target_count'], exported_episodes=len(exported),
                videos_per_episode=4, video_count=4*len(exported),
                passed=selection['complete'] and len(exported)==selection['target_count']
                    and all(item['passed'] for item in exported.values()),
                episodes=list(exported.values())))
        if selection['complete'] and len(exported) == selection['target_count']:
            print('ALL_DEFAULT_HUMAN_PREVIEWS_COMPLETE', flush=True)
            return 0
        time.sleep(15)


if __name__ == '__main__':
    raise SystemExit(main())
