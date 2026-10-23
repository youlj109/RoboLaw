"""Read the supported appearance inputs of one frozen raw reconstruction."""
import argparse
import hashlib
import json
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))


def main():
    from pxr import Usd
    from kitchen_safety.task_contract import digest
    from kitchen_safety.task_appearance import catalog
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--episode',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    scene=args.episode/'reconstruction.usda'
    contract=json.loads((args.episode/'task_contract.json').read_text())
    summary=json.loads((args.episode/'raw_capture_summary.json').read_text())
    stage=Usd.Stage.Open(str(scene))
    if stage is None:raise ValueError('Cannot open frozen reconstruction')
    result=dict(schema_version='robolaw-native-appearance-catalog-1',task_sha256=digest(contract),
        raw_sha256=summary['raw_writer']['sha256'],reconstruction_sha256=hashlib.sha256(scene.read_bytes()).hexdigest(),
        source_episode=str(args.episode.resolve()),catalog=catalog(stage),
        scope='discovery only; exact target selection and visual-only ranges still required')
    with args.output.open('x') as file:json.dump(result,file,ensure_ascii=False,indent=2);file.write('\n')
    print(json.dumps(dict(lights=len(result['catalog']['lights']),shaders=len(result['catalog']['shaders']),
        unsupported_shaders=result['catalog']['unsupported_shaders'])))


if __name__=='__main__':main()
