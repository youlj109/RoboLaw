#!/usr/bin/env python3
"""Bind extracted G reference hashes to the actual collection-start asset hashes."""
import argparse
import json
from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[3];sys.path.insert(0,str(ROOT))
from kitchen_safety.planning import ROBOT_USD
from kitchen_safety.extensions.replicator.mustard_family_preparation import sha
from scripts.extensions.replicator.variant_geometry import code_evidence


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--reference',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    args=p.parse_args()
    if args.output.exists():raise FileExistsError(args.output)
    ref=json.loads(args.reference.read_text());episode=ROOT/ref['reference_episode']
    for name,digest in ref['reference_input_sha256'].items():
        if sha(episode/name)!=digest:raise ValueError('Reference data changed '+name)
    provenance=json.loads((episode/'collection_provenance.json').read_text());rows=[]
    for name,path in (('robot',ROBOT_USD),('bottle',ROOT/'assets/robolab/objects/hope/mustard_bottle.usd')):
        path=Path(path).resolve();key=str(path.relative_to(ROOT));entry=provenance['entries'][key]
        hashes=[sha(path),ref['source_'+name+'_sha256'],entry['initial_sha256'],entry['checked_sha256']]
        rows.append(dict(name=name,path=key,current_sha256=hashes[0],extracted_reference_sha256=hashes[1],
            actual_collection_start_sha256=hashes[2],actual_collection_end_sha256=hashes[3],
            passed=bool(len(set(hashes))==1 and entry['unchanged'] and entry['matches_declared_source'])))
    output=dict(passed=bool(provenance['passed'] and all(row['passed'] for row in rows)),assets=rows,
        reference=str(args.reference),reference_sha256=sha(args.reference),collection_provenance_sha256=sha(episode/'collection_provenance.json'),
        scope='Exact current source root assets match hashes captured at actual G collection start/end. This is asset identity, not transferred grasp/contact validation.',
        **code_evidence([Path(__file__)]))
    args.output.parent.mkdir(parents=True,exist_ok=True);args.output.write_text(json.dumps(output,indent=2)+'\n')
    print(json.dumps(dict(passed=output['passed'],output=str(args.output))))
    return int(not output['passed'])


if __name__=='__main__':raise SystemExit(main())
