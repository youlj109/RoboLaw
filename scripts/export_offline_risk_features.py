"""Compact reviewed features; PT nearest relation follows actual exemptions."""
import argparse
import csv
import json
from pathlib import Path
import time

FIELDS = ['action_index', 'time_s', 'source_physics_step', 'phase', 'HS', 'PT', 'RS',
    'hs_robot_distance_m', 'hs_ee_distance_m', 'hs_payload_distance_m',
    'hs_closing_speed_mps', 'hs_ttc_s', 'hs_effective_contact',
    'pt_min_included_distance_m', 'pt_nearest_actor', 'pt_nearest_asset',
    'pt_max_applicable_force_n', 'pt_max_applicable_impulse_ns',
    'pt_held', 'pt_drop', 'pt_drop_height_m', 'pt_support_margin_m', 'pt_damage_flag',
    'rs_joint_limit_margin_rad', 'rs_joint_torque_ratio', 'rs_self_collision',
    'rs_fault_state', 'rs_motion_after_fault', 'HS_coverage_complete',
    'PT_coverage_complete', 'RS_coverage_complete']


def project(record):
    f, report = record['input']['features'], record['report']
    h, p, r = (f[x] for x in ('hs', 'pt', 'rs'))
    nearest = report.get('arena_pt_distance_summary', {}).get('nearest_pair') or {}
    def measured_max(rule_id):
        values = [q['evidence'].get('measured') for q in report['rules']
                  if q['rule_id'] == rule_id and q.get('counted_in_grade')]
        values = [v for v in values if isinstance(v, (int, float))]
        return max(values) if values else None
    return dict(action_index=record['action_index'], time_s=record['decision_time_s'],
        source_physics_step=record['source_physics_step'], phase=record['input']['metadata']['phase'],
        **{c: report['risk_levels'][c] for c in ('HS', 'PT', 'RS')},
        hs_robot_distance_m=h.get('d_robot_h_min_gt_m'), hs_ee_distance_m=h.get('d_ee_h_min_gt_m'),
        hs_payload_distance_m=h.get('d_obj_h_min_gt_m'), hs_closing_speed_mps=h.get('v_rel_h_gt_mps'),
        hs_ttc_s=h.get('TTC_h_min_gt_s'), hs_effective_contact=h.get('human_contact_flag_gt'),
        pt_min_included_distance_m=nearest.get('distance_m'),
        pt_nearest_actor=nearest.get('actor'), pt_nearest_asset=nearest.get('object_id'),
        pt_max_applicable_force_n=measured_max('PT-FORCE-002'),
        pt_max_applicable_impulse_ns=measured_max('PT-IMP-002'),
        pt_held=p['grasp_evidence'].get('held'), pt_drop=p.get('drop_flag_gt'),
        pt_drop_height_m=p.get('h_drop_gt_m'), pt_support_margin_m=p.get('support_margin_gt_m'),
        pt_damage_flag=p.get('damage_flag_gt'),
        rs_joint_limit_margin_rad=r.get('joint_limit_margin_gt_rad'),
        rs_joint_torque_ratio=r.get('joint_torque_ratio_gt'), rs_self_collision=r.get('self_collision_flag_gt'),
        rs_fault_state=r.get('fault_state_gt'), rs_motion_after_fault=r.get('motion_after_fault_gt'),
        **{c+'_coverage_complete':report['categories'][c].get('data_complete') for c in ('HS','PT','RS')})


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--input', type=Path, required=True); p.add_argument('--output', type=Path, required=True)
    args = p.parse_args(); start = time.perf_counter(); count = 0
    with args.input.open() as source, args.output.open('x', newline='') as output:
        writer = csv.DictWriter(output, fieldnames=FIELDS); writer.writeheader()
        for line in source:
            writer.writerow(project(json.loads(line))); count += 1
    print(json.dumps(dict(records=count, export_wall_s=time.perf_counter()-start)), flush=True)

if __name__ == '__main__': main()
