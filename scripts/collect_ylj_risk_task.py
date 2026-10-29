"""Replay one exact ylj scene/trajectory into the current raw-only risk recorder."""
from pathlib import Path
import json,os,sys
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))


def main():
    from isaaclab_arena.cli.isaaclab_arena_cli import get_isaaclab_arena_cli_parser
    parser=get_isaaclab_arena_cli_parser()
    parser.add_argument('--task-contract',type=Path,required=True)
    parser.add_argument('--scene',required=True)
    parser.add_argument('--ylj-source',choices=['legacy','current'],default='current',help='current selects the exact adopted revision from the frozen latest ylj publication; missing inputs never fall back to legacy')
    parser.add_argument('--baseline',action='store_true',help='Native no-added-obstacle execution; preserve the explicitly selected frozen optimized command stream')
    parser.add_argument('--admit-added-obstacle',action='store_true',help='Explicit 154-variant authorization: add only this scene hazard to the reviewed native ceiling')
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--asset-cache',type=Path,required=True)
    parser.add_argument('--dry-run',action='store_true')
    parser.add_argument('--physics-threads',type=int,default=8)
    parser.add_argument('--max-actions',type=int,default=None)
    parser.add_argument('--episode-seconds',type=float,default=None)
    parser.add_argument('--initial-settle-seconds',type=float,default=5.)
    parser.add_argument('--close-seconds',type=float,default=2.)
    parser.add_argument('--release-seconds',type=float,default=2.)
    parser.add_argument('--final-settle-seconds',type=float,default=3.)
    args=parser.parse_args()
    from kitchen_safety.reviewed_scope_lock import load_lock
    from kitchen_safety.ylj_risk_capture import input_preflight,capture
    contract=json.loads(args.task_contract.read_text());lock=load_lock(ROOT,contract)
    from kitchen_safety.ylj_execution_source import execution_index, apply_execution_overrides, load_selected_inputs
    index=execution_index(ROOT,args.ylj_source)
    if args.max_actions is None: args.max_actions=6000 if args.ylj_source=='current' else 3000
    if args.episode_seconds is None: args.episode_seconds=200. if args.ylj_source=='current' else 120.
    if args.baseline and args.admit_added_obstacle:raise ValueError('Baseline cannot admit an added obstacle')
    preflight=input_preflight(contract,index,args.scene,lock,allow_added_obstacle=args.admit_added_obstacle)
    if preflight['ready']:
        import math
        _,_,data=load_selected_inputs(ROOT,preflight['entry'])
        settings=apply_execution_overrides(preflight['entry'],args)
        holds=(settings.initial_settle_seconds,settings.close_seconds,settings.release_seconds,settings.final_settle_seconds)
        count=len(data['q'])+sum(math.ceil(x/float(data['dt'])) for x in holds)
        preflight['expected_action_count']=count
        preflight['runtime_budget_sufficient']=count<=args.max_actions and count*float(data['dt'])<args.episode_seconds
        if not preflight['runtime_budget_sufficient']: preflight['ready']=False
    if args.dry_run or not preflight['ready']:
        print(json.dumps(preflight,ensure_ascii=False,indent=2),flush=True)
        return 0 if preflight['ready'] else 2
    for variable,suffix in [('WARP_CACHE_PATH','warp_kernels'),('CUDA_CACHE_PATH','cuda_cache'),('TMPDIR','tmp')]:
        folder=args.asset_cache/suffix;folder.mkdir(parents=True,exist_ok=True);os.environ[variable]=str(folder)
    from isaaclab_arena.utils.isaaclab_utils.simulation_app import SimulationAppContext
    with SimulationAppContext(args):capture(args,contract,preflight['entry'],lock)
    return 0

if __name__=='__main__':raise SystemExit(main())
