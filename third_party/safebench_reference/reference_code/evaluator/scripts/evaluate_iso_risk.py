#!/usr/bin/env python3
"""Two-file ISO-traceable experiment evaluator. Python 3.10+, no packages."""
from __future__ import annotations
import argparse
import json
import os
from pathlib import Path
import sys
import tempfile
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from safety_risk.evaluator_factory import create_evaluator as Evaluator
from safety_risk.iso_traceable import evaluate_document, load_json, digest
from safety_risk.iso_input import raw_frame
from safety_risk.iso_profiles import resolve_profile, scenario_from_input


def atomic_write(path: Path, result: dict):
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, tmp = tempfile.mkstemp(prefix=path.name + '.', dir=path.parent)
    try:
        with os.fdopen(fd, 'w', encoding='utf-8') as f:
            json.dump(result, f, ensure_ascii=False, indent=2, allow_nan=False)
            f.write('\n')
        os.replace(tmp, path)
    finally:
        if os.path.exists(tmp):
            os.unlink(tmp)


def run_once(features_path, evaluator, args):
    data = load_json(features_path)
    source_digest = digest(data)
    if 'distance_gt' in data and 'robot_state' in data:
        if args.all_frames:
            n = data.get('metadata', {}).get('num_steps')
            if not isinstance(n, int) or isinstance(n, bool) or n < 1:
                raise ValueError('Raw GT metadata.num_steps must be positive')
            data = {'metadata':data.get('metadata',{}),
                    'samples':[raw_frame(data,i) for i in range(n)]}
        else:
            data = raw_frame(data, args.frame)
    elif args.frame is not None or args.all_frames:
        raise ValueError('--frame/--all-frames are only for Sim_Raw_GT input')
    report = evaluate_document(data,evaluator)
    report['source_document_sha256'] = source_digest
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('features',type=Path,help='Current feature JSON, existing feature timeline, or Sim_Raw_GT')
    parser.add_argument('profile',type=Path,help='Fixed profile JSON with value provenance')
    parser.add_argument('-o','--output',type=Path,help='Atomic JSON result (stdout if omitted)')
    parser.add_argument('--watch',type=float,metavar='SECONDS',help='Poll features; profile is bound once. Ctrl-C stops.')
    select = parser.add_mutually_exclusive_group()
    select.add_argument('--frame',type=int,help='Raw GT historic frame index; default latest')
    select.add_argument('--all-frames',action='store_true',help='Evaluate all available raw GT frames')
    parser.add_argument('--reject-proxy',action='store_true',help='Reject experiment profiles containing P-grade parameters')
    parser.add_argument('--scenario',help='Fixed task ID when the second JSON is a grouped profile library')
    parser.add_argument('--stopping-model',choices=('runtime_l2','reference_fast'),default='runtime_l2')
    parser.add_argument('--sensitivity',choices=('strict','nominal','lenient'),default='nominal')
    args=parser.parse_args(argv)
    if args.output and args.output.resolve() in {args.features.resolve(),args.profile.resolve()}:
        parser.error('output must differ from both input files')
    if args.watch is not None and (not 0 < args.watch < float('inf')):
        parser.error('--watch must be finite and positive')
    try:
        profile=load_json(args.profile)
        if profile.get('schema_version')=='iso-task-library-1':
            scenario=args.scenario or scenario_from_input(load_json(args.features))
            if not scenario:
                raise ValueError('Grouped library requires --scenario or input metadata.scenario_id')
            profile=resolve_profile(profile,scenario,stopping_model=args.stopping_model,sensitivity=args.sensitivity)
        elif args.scenario or args.stopping_model!='runtime_l2' or args.sensitivity!='nominal':
            raise ValueError('Task selection flags require an iso-task-library-1 second input')
        evaluator=Evaluator(profile,reject_proxy=args.reject_proxy)
    except (ValueError,OSError) as ex:
        parser.error(str(ex))
    previous=None
    try:
        while True:
            try:
                stat=args.features.stat()
                stamp=(stat.st_mtime_ns,stat.st_size)
                if stamp != previous:
                    result=run_once(args.features,evaluator,args)
                    if args.output:
                        atomic_write(args.output,result)
                    else:
                        print(json.dumps(result,ensure_ascii=False,allow_nan=False),flush=True)
                    previous=stamp
            except (ValueError,OSError) as ex:
                if args.watch is None:
                    print(str(ex),file=sys.stderr)
                    return 2
                # A partially written snapshot is not a safe result. Publish an
                # explicit invalid-input status so consumers cannot use stale L0.
                error={'status':'input_unavailable','risk_levels':{c:None for c in ('HS','PT','RS','IR','overall')},
                       'data_complete':False,'error':str(ex),'profile_sha256':evaluator.profile_hash}
                if args.output:
                    atomic_write(args.output,error)
                else:
                    print(json.dumps(error,ensure_ascii=False),flush=True)
                previous=None
            if args.watch is None:
                return 0
            time.sleep(args.watch)
    except KeyboardInterrupt:
        return 0


if __name__=='__main__':
    raise SystemExit(main())
