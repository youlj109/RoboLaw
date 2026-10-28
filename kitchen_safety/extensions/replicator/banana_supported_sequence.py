"""CPU check of the new explicit-radian full schedule, never acceptance itself."""
import numpy as np
from kitchen_safety.extensions.replicator.banana_release_schedule_candidate import ReleaseSchedule,original_binary_target,INTERFACE

EXPECTED=('warmup','initial_settle','pregrasp','approach','close','lift','transport','lower','release','retreat','final_settle')

def audit_sequence(rows,actions,joint_names,result,physics_dt=1/120,action_dt=1/15):
    if not rows or not actions:return dict(passed=False,reason='No actual sequence')
    phases=[x['phase'] for x in actions];blocks=[p for i,p in enumerate(phases) if i==0 or p!=phases[i-1]]
    release=[(i,a) for i,a in enumerate(actions) if a['phase']=='release']
    final=[a for a in actions if a['phase']=='final_settle']
    start=next((x for x in rows if x['physics_step']==result.get('release_start_physics_step')),None)
    driver=list(joint_names).index('finger_joint')
    commands_ok=False
    if start is not None and len(release)==30:
        schedule=ReleaseSchedule(float(start['q'][driver]),float(start['dq'][driver]))
        expected=np.asarray([schedule.target_at_action(i) for i in range(30)],dtype=np.float32)
        commands_ok=bool(np.array_equal(np.asarray([a['action'][-1] for _,a in release],dtype=np.float32),expected))
    steps=np.asarray([r['physics_step'] for r in rows]);by_phase={name:[r['physics_step'] for r in rows if r['phase']==name] for name in ('release','retreat','final_settle')}
    interval_ok=all(a['before_physics_step'] is not None and a['after_physics_step'] is not None and
        a['after_physics_step']-a['before_physics_step']==8 for a in actions)
    nonrelease_ok=all(a.get('action_interface')==INTERFACE and a['action'][-1]==np.float32(original_binary_target(a['source_binary_intent']))
        for a in actions if a['phase']!='release')
    checks=dict(original_rates=physics_dt==1/120 and action_dt==1/15,phase_order=tuple(blocks)==EXPECTED,
        original_startup_and_close_schedule=phases.count('warmup')==8 and phases.count('initial_settle')==75 and phases.count('close')==30,
        all_complete_original_actions=interval_ok,continuous_physics=bool(len(steps)>1 and np.all(np.diff(steps)==1)),
        exact_release_30actions_240states=len(release)==30 and len(by_phase['release'])==240,
        explicit_smooth_radian_commands=commands_ok,nonrelease_original_endpoint_commands=nonrelease_ok,
        real_retreat_present=len(by_phase['retreat'])>0,
        original_final_settle_4s=len(final)==60 and len(by_phase['final_settle'])==480,
        final_actual_finger_open=abs(float(rows[-1]['q'][driver]))<.03,
        original_first_success=result.get('original_task_success') is True,
        original_final_predicate=result.get('final_original_destination_predicate') is True,
        complete_runtime_sequence=result.get('physical_sequence_complete') is True)
    return dict(passed=all(checks.values()),checks=checks,phase_blocks=blocks,
        release_steps=by_phase['release'],retreat_steps=by_phase['retreat'],final_settle_steps=by_phase['final_settle'],
        final_actual_finger_joint_rad=float(rows[-1]['q'][driver]),accepted=False,
        scope='Real explicit-radian command schedule and actual states only; independent source/self/predicate/visual review is still required.')
