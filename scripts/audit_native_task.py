"""Audit native causal raw evidence without geometry, physics or rule evaluation."""
import argparse,hashlib,json,math
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from kitchen_safety.hs_data_policy import omitted_contact

def main():
    p=argparse.ArgumentParser();p.add_argument('--episode',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();summary=json.loads((a.episode/'raw_capture_summary.json').read_text());errors=[];h=hashlib.sha256()
    schema=json.loads((a.episode/'raw_capture_schema.json').read_text())
    scope=json.loads((a.episode/'relation_scope.json').read_text())
    omissions=scope.get('hs_acquisition_omissions',{})
    if schema.get('hs_acquisition_omissions',{})!=omissions:errors.append('HS acquisition scope differs from frozen relation scope')
    if schema.get('physics_hz')!=120 or schema.get('action_hz')!=30:errors.append('unsupported source clocks')
    def time_matches(value,expected):
        return isinstance(value,(int,float)) and math.isfinite(value) and abs(value-expected)<1e-8
    steps=frames=command_writes=terminals=0;latest=None;initial_count=0;terminal_outcome=None
    for line in (a.episode/'raw_evidence.jsonl').open('rb'):
        h.update(line);r=json.loads(line)
        if terminals:errors.append('record after terminal observation')
        if r['kind']=='initial_state':initial_count+=1;latest=r['state']
        elif r['kind']=='physics':
            if r['physics_step']!=steps or r['action_index']!=frames-1:errors.append('physics sequence differs at '+str(steps))
            if not time_matches(r.get('start_time_s'),steps/120) or not time_matches(r.get('end_time_s'),(steps+1)/120):
                errors.append('physical timestamps differ at '+str(steps))
            if r['state'].get('complete') is not True:errors.append('measured state incomplete at '+str(steps))
            if not r.get('contact_coverage'):errors.append('contact stream incomplete at '+str(steps))
            if any(omitted_contact(c,omissions) for c in r.get('contacts',[])):
                errors.append('omitted HS contact persisted at '+str(steps))
            command_writes+=len(r.get('commands',[]));steps+=1;latest=r['state']
        elif r['kind']=='observation':
            if r['action_index']!=frames or r['source_physics_step']!=steps-1 or steps!=frames*4:errors.append('noncausal observation '+str(frames))
            if not time_matches(r.get('time_s'),steps/120):errors.append('observation timestamp differs')
            if r['state']!=latest:errors.append('observation is not the immediately preceding measured state')
            frames+=1
        elif r['kind']=='terminal_observation':
            terminals+=1;terminal_outcome=r.get('native_outcome')
            if r['action_index']!=frames or r['source_physics_step']!=steps-1 or r['state']!=latest:
                errors.append('terminal observation is not the measured final action state')
            if not time_matches(r.get('time_s'),steps/120):errors.append('terminal timestamp differs')
        elif r['kind']!='fault_event':errors.append('unrecognized raw record')
    if initial_count!=1 or not frames or steps!=4*frames:errors.append('incomplete initial/action windows')
    if schema.get('terminal_sample_required') and terminals!=1:errors.append('missing terminal safety sample')
    if schema.get('episode_boundary_required'):
        if schema.get('native_episode_limit')!=1 or summary.get('native_episode_limit')!=1:
            errors.append('native episode replacement was not disabled')
        if not isinstance(terminal_outcome,dict) or terminal_outcome!=summary.get('native_outcome'):
            errors.append('terminal outcome missing or differs from summary')
        else:
            if (terminal_outcome.get('action_index')!=frames-1 or terminal_outcome.get('source_physics_step')!=steps-1 or
                terminal_outcome.get('native_episode_index')!=schema.get('native_episode_index') or
                summary.get('native_episode_index')!=schema.get('native_episode_index')):
                errors.append('terminal outcome refers to another episode or source step')
            if any(type(terminal_outcome.get(k)) is not bool for k in ('terminated','truncated')):
                errors.append('native termination flags are not known booleans')
            if summary.get('task_success') is not None and summary['task_success']!=terminal_outcome.get('success'):
                errors.append('task success differs from native terminal outcome')
    if not command_writes:errors.append('no actual command submissions captured')
    if h.hexdigest()!=summary['raw_writer']['sha256']:errors.append('raw digest mismatch')
    if summary.get('execution_error'):errors.append('execution aborted')
    variation=json.loads((a.episode/'variation_snapshot.json').read_text())
    placement=variation.get('placement_validation',dict(available=False,passed=None))
    if 'placement_validation' in summary and summary['placement_validation']!=placement:
        errors.append('placement validation differs from frozen reset evidence')
    report=dict(passed=not errors,errors=errors,physics_steps=steps,observations=frames,terminal_samples=terminals,command_writes=command_writes,
        raw_sha256=h.hexdigest(),hs_acquisition_scope_checked=bool(omissions),geometry_computed=False,risk_evaluated=False,task_success=summary.get('task_success'),
        placement_validation=placement,placement_accepted=placement.get('passed'),
        native_episode_boundary_checked=bool(schema.get('episode_boundary_required')),native_outcome=terminal_outcome,
        scope='raw chronology, measured state/contact coverage and actual command presence; not full scene/rule/material acceptance')
    if a.output.exists():raise ValueError('Audit output already exists')
    a.output.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n');print(json.dumps(report,ensure_ascii=False))
    return 0 if report['passed'] else 1

if __name__=='__main__':raise SystemExit(main())
