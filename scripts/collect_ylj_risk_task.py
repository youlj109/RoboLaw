"""Replay one exact ylj scene/trajectory into the current raw-only risk recorder."""
from pathlib import Path
import json,os,sys
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))


def main():
    from isaaclab_arena.cli.isaaclab_arena_cli import get_isaaclab_arena_cli_parser
    parser=get_isaaclab_arena_cli_parser()
    parser.add_argument('--task-contract',type=Path,required=True)
    parser.add_argument('--scene',required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--asset-cache',type=Path,required=True)
    parser.add_argument('--dry-run',action='store_true')
    parser.add_argument('--physics-threads',type=int,default=8)
    parser.add_argument('--max-actions',type=int,default=3000)
    parser.add_argument('--episode-seconds',type=float,default=120.)
    parser.add_argument('--initial-settle-seconds',type=float,default=5.)
    parser.add_argument('--close-seconds',type=float,default=2.)
    parser.add_argument('--release-seconds',type=float,default=2.)
    parser.add_argument('--final-settle-seconds',type=float,default=3.)
    args=parser.parse_args()
    from kitchen_safety.reviewed_scope_lock import load_lock
    from kitchen_safety.ylj_risk_capture import input_preflight,capture
    contract=json.loads(args.task_contract.read_text());lock=load_lock(ROOT,contract)
    index=json.loads((ROOT/'configs/extensions/replicator/final_154_scenes_20261028.json').read_text())
    preflight=input_preflight(contract,index,args.scene,lock)
    if args.dry_run or not preflight['ready']:
        print(json.dumps(preflight,ensure_ascii=False,indent=2),flush=True)
        return 0 if preflight['ready'] else 2
    for variable,suffix in [('WARP_CACHE_PATH','warp_kernels'),('CUDA_CACHE_PATH','cuda_cache'),('TMPDIR','tmp')]:
        folder=args.asset_cache/suffix;folder.mkdir(parents=True,exist_ok=True);os.environ[variable]=str(folder)
    from isaaclab_arena.utils.isaaclab_utils.simulation_app import SimulationAppContext
    with SimulationAppContext(args):capture(args,contract,preflight['entry'],lock)
    return 0

if __name__=='__main__':raise SystemExit(main())
