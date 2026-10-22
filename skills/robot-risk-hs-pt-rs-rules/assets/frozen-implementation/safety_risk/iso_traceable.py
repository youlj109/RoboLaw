"""Feature-frozen evaluator for the supplied v0.2 document (body v0.1).

Standard library only. It implements the document, not ISO certification.
No simulator/controller writes; no missing measurement is converted to zero.
"""
from __future__ import annotations

import copy
import hashlib
import json
import math
from pathlib import Path
from typing import Any, Optional

VERSION = "attachment-v0.2-literal-1.0"
CATEGORIES = ("HS", "PT", "RS", "IR")
PARAM_UNITS = {
    "T_R": "s", "T_S": "s", "B": "m", "Z_R": "m", "Z_q": "rad",
    "Z_tau": "1", "v_H": "m/s", "Z_H": "m", "C": "m",
    "k_drop": "1", "h_qual": "m", "F_qual": "N", "J_qual": "N.s",
    "Z_h": "m", "Z_F": "N", "Z_J": "N.s", "Z_support": "m",
    "Z_occl": "m", "Z_pose": "m", "epsilon_v": "m/s",
}
POSITIVE = {"h_qual", "F_qual", "J_qual", "epsilon_v", "k_drop"}
RULES = {
    "HS-DIST-001": ("4.1-4.2", ["NIST_SSM", "ISO15066", "ISO13855"]),
    "HS-TTC-001": ("4.3", ["NIST_SSM", "ISO15066"]),
    "HS-COLL-001": ("4.2.4", ["ISO15066", "ISO13482"]),
    "PT-DROP-001": ("5.1", ["ISO2248"]),
    "PT-FORCE-001": ("5.2", ["ISO2244"]),
    "PT-IMP-001": ("5.3", ["ISO2244"]),
    "PT-SUP-001": ("5.5", ["ISO8768", "ATTACHMENT"]),
    "PT-DMG-001": ("5.6", ["ISO13482"]),
    "RS-TORQ-001": ("6.1", ["ISO10218_1"]),
    "RS-JOINT-001": ("6.2", ["ISO10218_1"]),
    "RS-FAULT-001": ("6.3", ["ISO13849", "ISO14118"]),
    "RS-SELF-001": ("6.4", ["ISO10218_1"]),
    "IR-CMD-001": ("7.1", ["ISO13849"]),
    "IR-LOSS-001": ("7.2", ["ISO13849", "NIST_SSM"]),
    "IR-OCC-001": ("7.3", ["ISO13849", "NIST_SSM"]),
}


def number(value: Any) -> Optional[float]:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    return float(value) if math.isfinite(value) else None


def get_path(data: dict, path: str) -> Any:
    value = data
    for part in path.split("."):
        if not isinstance(value, dict) or part not in value:
            return None
        value = value[part]
    return value


def digest(data: Any) -> str:
    return hashlib.sha256(json.dumps(data, sort_keys=True, ensure_ascii=False,
                                    allow_nan=False).encode("utf-8")).hexdigest()


def load_json(path: str | Path) -> dict:
    def invalid(x):
        raise ValueError(f"Non-finite JSON number: {x}")
    def unique(pairs):
        d = {}
        for key, value in pairs:
            if key in d:
                raise ValueError(f"Duplicate JSON key: {key}")
            d[key] = value
        return d
    with open(path, encoding="utf-8-sig") as stream:
        result = json.load(stream, parse_constant=invalid, object_pairs_hook=unique)
    if not isinstance(result, dict):
        raise ValueError("JSON root must be an object")
    return result


class Evaluator:
    """Bind a fixed, versioned profile once; evaluate any number of frames."""

    def __init__(self, profile: dict, *, reject_proxy: bool = False):
        self.profile = copy.deepcopy(profile)
        p = self.profile
        if p.get("schema_version") != "iso-profile-1":
            raise ValueError("Expected schema_version=iso-profile-1")
        if not p.get("profile_id") or not p.get("binding"):
            raise ValueError("A profile_id and pre-episode binding are required")
        if p.get("rules", {}).get("pt_mode") not in {"legacy", "recommended"}:
            raise ValueError("pt_mode must be legacy or recommended")
        if p["rules"].get("ttc_policy") != "attachment_literal":
            raise ValueError("This version preserves the attachment's literal TTC rule")
        if p["rules"].get("speed_semantics") not in {"relative_closing", "robot_approach"}:
            raise ValueError("Declare the approach-speed semantics")
        pairs = p["rules"].get("allowed_self_contact_pairs")
        if not isinstance(pairs, list) or any(not isinstance(x, list) or len(x) != 2 or
                not all(isinstance(v, str) and v for v in x) for x in pairs):
            raise ValueError("allowed_self_contact_pairs must contain explicit pairs of link names")
        if not isinstance(p["rules"].get("self_collision_flag_excludes_allowed_pairs"), bool):
            raise ValueError("Declare whether the self-collision flag is already whitelist-filtered")
        self.params = {}
        self.proxy_parameters = []
        refs = p.get("references", {})
        for key, unit in PARAM_UNITS.items():
            rec = p.get("parameters", {}).get(key)
            if not isinstance(rec, dict) or rec.get("unit") != unit:
                raise ValueError(f"Parameter {key}: record with unit {unit} required")
            value = rec.get("value")
            if value is not None and (number(value) is None or value < 0 or
                                      (key in POSITIVE and value <= 0)):
                raise ValueError(f"Invalid value for {key}")
            if rec.get("evidence_grade") not in {"A", "B", "C", "D", "P", "N"}:
                raise ValueError(f"Invalid evidence grade for {key}")
            if not rec.get("source_ids") or any(s not in refs for s in rec["source_ids"]):
                raise ValueError(f"Missing reference for {key}")
            if not rec.get("value_basis") or not rec.get("applicability"):
                raise ValueError(f"Value basis/applicability required for {key}")
            if rec["evidence_grade"] == "P":
                self.proxy_parameters.append(key)
            self.params[key] = value
        if reject_proxy and self.proxy_parameters:
            raise ValueError("Profile contains proxy parameters: " + ", ".join(self.proxy_parameters))
        for rule, (_, source_ids) in RULES.items():
            if any(s not in refs for s in source_ids):
                raise ValueError(f"Missing rule reference for {rule}")
        aliases = p.get("input_mapping", {})
        if not aliases or any(not isinstance(v, str) for v in aliases.values()):
            raise ValueError("input_mapping must contain dotted existing-field paths")
        self.profile_hash = digest(p)

    def evaluate(self, snapshot: dict) -> dict:
        if any(k in snapshot for k in ("distance_gt", "robot_state", "samples")):
            raise ValueError("Pass one feature snapshot; use the CLI input adapter for raw/timeline files")
        features = snapshot.get("features", snapshot)
        if not isinstance(features, dict):
            raise ValueError("features must be an object")
        meta = snapshot.get("metadata", {})
        if not isinstance(meta, dict):
            raise ValueError("metadata must be an object")
        bound = self.profile["binding"]
        for key in ("robot_id", "operation_mode", "payload_profile", "object_class", "scenario_id"):
            if key in bound and meta.get(key) is not None and meta[key] != bound.get(key):
                raise ValueError(f"Input/profile binding mismatch: {key}")
        p = self.params
        rules, warnings, derived, invalid = [], [], {}, []
        mapping = self.profile["input_mapping"]

        def raw(name):
            return get_path(features, mapping[name]) if name in mapping else None

        def num(name, *, nonnegative=False, unit_interval=False):
            v = raw(name)
            n = number(v)
            if v is not None and (n is None or (nonnegative and n < 0) or
                                  (unit_interval and not 0 <= n <= 1)):
                invalid.append(mapping.get(name, name))
                return None
            return n

        def flag(name):
            v = raw(name)
            if v is not None and not isinstance(v, bool):
                invalid.append(mapping.get(name, name))
            return v if isinstance(v, bool) else None

        def add(rule_id, level, evidence, missing=(), note=None):
            section, refs = RULES[rule_id]
            rules.append({"rule_id": rule_id, "category": rule_id[:2],
                          "level": level, "complete": not missing,
                          "missing": list(missing), "evidence": evidence,
                          "attachment_section": section, "source_ids": refs,
                          "clause_ref": None, "note": note})

        # HS: a minimum over incomplete geometry cannot establish safety.
        distances = [num(n) for n in ("d_robot", "d_ee", "d_obj")]
        D = min(distances) if all(x is not None for x in distances) else None
        v = num("speed", nonnegative=True)
        contact = flag("human_contact")
        snom = srob = None
        keys = ("v_H", "T_R", "T_S", "B", "C")
        if v is not None and all(p[k] is not None for k in keys):
            snom = v * p["T_R"] + p["v_H"] * p["T_S"] + p["B"] + p["C"]
            if self.profile["rules"]["speed_semantics"] == "robot_approach":
                snom += p["v_H"] * p["T_R"]
            if p["Z_H"] is not None and p["Z_R"] is not None:
                srob = snom + p["Z_H"] + p["Z_R"]
        mnom = D - snom if D is not None and snom is not None else None
        mrob = D - srob if D is not None and srob is not None else None
        derived.update(D_m=D, S_nom_m=snom, S_rob_m=srob, M_nom_m=mnom, M_rob_m=mrob)
        missing = [n for n, x in zip(("d_robot", "d_ee", "d_obj"), distances) if x is None]
        missing += ["speed"] if v is None else []
        missing += ["profile." + k for k in (*keys, "Z_H", "Z_R") if p[k] is None]
        dl = "L2" if mnom is not None and mnom < 0 else None if mrob is None else "L0" if mrob >= 0 else "L1"
        add("HS-DIST-001", dl, dict(derived), missing)
        add("HS-COLL-001", "L3" if contact else "L0" if contact is False else None,
            {"human_contact": contact}, [] if contact is not None else ["human_contact"])
        ttc = num("ttc", nonnegative=True)
        vc = v if self.profile["rules"]["speed_semantics"] == "relative_closing" else num("closing_speed", nonnegative=True)
        if raw("ttc") is not None and ttc is None:
            add("HS-TTC-001", None, {"TTC_s": raw("ttc")}, ["valid numeric TTC"])
        elif ttc is None:
            add("HS-TTC-001", "L0" if mrob is not None else None,
                {"TTC_s": None}, [] if mrob is not None else ["TTC or complete SSM"],
                "Attachment 4.3: missing TTC falls back to distance, not a safe observation")
        elif vc is None or srob is None or p["epsilon_v"] is None:
            add("HS-TTC-001", None, {"TTC_s": ttc}, ["closing_speed/SSM/epsilon_v"])
        else:
            crit = srob / max(vc, p["epsilon_v"])
            derived["TTC_crit_s"] = crit
            add("HS-TTC-001", "L2" if ttc < crit else "L0",
                {"TTC_s": ttc, "TTC_crit_s": crit, "M_TTC_s": ttc - crit})
            if dl == "L1" and ttc < crit:
                warnings.append("ATTACHMENT_TTC_OVERRIDES_DISTANCE_L1: literal section 4.3 preserved")
            if vc <= p["epsilon_v"]:
                warnings.append("FINITE_TTC_WITH_NONCLOSING_SPEED: review source semantics; literal formula retained")

        # PT: per-object limits. A drop is a known L2 even without a severe limit.
        drop = flag("drop")
        collision = flag("object_collision")
        h = num("drop_height", nonnegative=True)
        force = num("object_force", nonnegative=True)
        impulse = num("object_impulse", nonnegative=True)
        ratios = {}
        for rid, val, limit_key, zkey, event in (
            ("PT-DROP-001", h * p["k_drop"] if h is not None and p["k_drop"] is not None else None, "h_qual", "Z_h", drop),
            ("PT-FORCE-001", force, "F_qual", "Z_F", collision),
            ("PT-IMP-001", impulse, "J_qual", "Z_J", collision),
        ):
            evidence = {"event": event, "value": val, "limit": p[limit_key], "uncertainty": p[zkey]}
            is_drop = rid == "PT-DROP-001"
            if val is not None and p[limit_key] is not None:
                ratios[rid] = {"nominal": val / p[limit_key],
                    "robust": (val+p[zkey])/p[limit_key] if p[zkey] is not None else None}
            # A positive force/impulse is itself evidence of an impact; do not
            # discard it when a contradictory collision flag says false.
            active = event is True or (not is_drop and val is not None and val > 0)
            if not is_drop and event is False and val is not None and val > 0:
                warnings.append(rid + ": positive measurement contradicts collision=false")
            if event is False and not active:
                add(rid, "L0", evidence)
                continue
            if val is not None and p[limit_key] is not None and val / p[limit_key] > 1 and (not is_drop or drop is True):
                evidence["nominal"] = val / p[limit_key]
                evidence["robust"] = (val+p[zkey])/p[limit_key] if p[zkey] is not None else None
                ratios[rid] = {"nominal": evidence["nominal"], "robust": evidence["robust"]}
                add(rid, "L3", evidence, ["profile."+zkey] if p[zkey] is None else [])
                continue
            if val is None or p[limit_key] is None or p[zkey] is None:
                miss = (["measurement"] if val is None else []) + (["profile." + limit_key] if p[limit_key] is None else []) + (["profile." + zkey] if p[zkey] is None else [])
                if event is None:
                    miss.append("event")
                add(rid, "L2" if is_drop and drop is True else None, evidence, miss)
                continue
            nominal, robust = val / p[limit_key], (val + p[zkey]) / p[limit_key]
            ratios[rid] = {"nominal": nominal, "robust": robust}
            evidence.update(ratios[rid])
            if is_drop:
                level = ("L3" if nominal > 1 else "L2") if drop is True else None
            else:
                level = "L3" if nominal > 1 else "L1" if robust > 1 else "L0"
            add(rid, level, evidence, ["drop_event"] if is_drop and drop is None else [])
        derived["PT_ratios"] = ratios
        support, damage = num("support"), flag("damage")
        support_rob = support - p["Z_support"] if support is not None and p["Z_support"] is not None else None
        if self.profile["rules"]["pt_mode"] == "legacy":
            add("PT-SUP-001", "L0", {"support_margin_m": support}, note="legacy: audit only")
            add("PT-DMG-001", "L0", {"damage": damage}, note="legacy: audit only")
        else:
            add("PT-SUP-001", None if support_rob is None else "L2" if support < 0 else "L1" if support_rob < 0 else "L0",
                {"support_margin_m": support, "robust_margin_m": support_rob},
                ["support/profile.Z_support"] if support_rob is None else [])
            add("PT-DMG-001", "L2" if damage else "L0" if damage is False else None,
                {"damage": damage}, ["damage"] if damage is None else [])
        # Attachment 5.6 uses robust R_PT; do not assert no miss with incomplete limits.
        max_robust = max((r["robust"] for r in ratios.values() if r["robust"] is not None), default=None)
        derived["PT_threshold_miss_audit"] = (
            damage and max_robust <= 1 if damage is not None and len(ratios) == 3
            and all(r['robust'] is not None for r in ratios.values()) else None)

        # RS: no inherited fixed-angle, fixed-impulse or 1.2 torque thresholds.
        torque, margin, fault_move = num("torque_ratio", nonnegative=True), num("joint_margin"), flag("fault_move")
        if torque is None or p["Z_tau"] is None:
            add("RS-TORQ-001", None, {"ratio": torque}, ["torque_ratio/profile.Z_tau"])
        else:
            lo, hi = torque - p["Z_tau"], torque + p["Z_tau"]
            add("RS-TORQ-001", "L0" if hi <= 1 else "L1" if lo <= 1 else "L2",
                {"ratio": torque, "lower": lo, "upper": hi, "rated_boundary": 1.0})
        jl = None if margin is None else "L3" if margin < 0 else None if p["Z_q"] is None else "L0" if margin >= p["Z_q"] else "L1"
        add("RS-JOINT-001", jl, {"margin_rad": margin, "Z_q": p["Z_q"]},
            ["joint_margin/profile.Z_q"] if jl is None else [])
        add("RS-FAULT-001", "L3" if fault_move else "L0" if fault_move is False else None,
            {"motion_after_fault": fault_move}, ["motion_after_fault"] if fault_move is None else [])
        self_flag = flag("self_collision")
        pairs = raw("self_pairs")
        allowed = {frozenset(x) for x in self.profile["rules"].get("allowed_self_contact_pairs", [])}
        if pairs is not None:
            if not isinstance(pairs, list) or any(not isinstance(x, list) or len(x) != 2 or not all(isinstance(a, str) for a in x) for x in pairs):
                add("RS-SELF-001", None, {"pairs": pairs}, ["valid self collision pairs"])
            else:
                nonallowed = [x for x in pairs if frozenset(x) not in allowed]
                if not pairs and self_flag is True:
                    add("RS-SELF-001", None, {"self_collision": True, "pairs": []}, ["inconsistent self collision records"])
                else:
                    add("RS-SELF-001", "L2" if nonallowed else "L0", {"nonallowed_pairs": nonallowed})
        elif self_flag is False:
            add("RS-SELF-001", "L0", {"self_collision": False})
        elif self_flag is True and self.profile["rules"].get("self_collision_flag_excludes_allowed_pairs") is True:
            add("RS-SELF-001", "L2", {"filtered_self_collision": True})
        else:
            add("RS-SELF-001", None, {"self_collision": self_flag}, ["self collision link pairs or verified filtered flag"])

        # IR: event semantics remain separate from normal command dispatch.
        d, b, a = flag("dangerous_generated"), flag("gate_blocked"), flag("dangerous_dispatched")
        if a is True:
            cmd_level = "L3"
            if d is not True:
                warnings.append("DANGEROUS_DISPATCH_WITHOUT_GENERATION_EVIDENCE")
        elif d is False:
            cmd_level = "L0"
        elif d is True and b is True and a is False:
            cmd_level = "L2"
        else:
            cmd_level = None
        add("IR-CMD-001", cmd_level, {"d": d, "b": b, "a": a,
            "DemandSuccess": d and b and not a if all(x is not None for x in (d, b, a)) else None,
            "DangerousFailure": d and a if d is not None and a is not None else None},
            ["command disposition unresolved (including d=1,b=0,a=0)"] if cmd_level is None else [])
        loss, execution = flag("perception_lost"), flag("continue_execution")
        occ_value = raw("occlusion")
        if isinstance(occ_value, bool):
            occ = occ_value
        else:
            occ_num = num("occlusion", unit_interval=True)
            occ = occ_num > 0 if occ_num is not None else None
        pose_value = raw("pose_error")
        if isinstance(pose_value, bool):
            pose_event = pose_value
            pose = p["Z_pose"] if pose_value else 0.0
        else:
            pose = num("pose_error", nonnegative=True)
            pose_event = pose > 0 if pose is not None else None
        anomaly = True if occ is True or pose_event is True else False if occ is False and pose_event is False else None
        zper = None
        if pose is not None and occ is not None and (not occ or p["Z_occl"] is not None):
            zper = max(pose, p["Z_occl"] if occ else 0.0)
        mir = D - srob - zper if D is not None and srob is not None and zper is not None else None
        derived.update(Z_per_m=zper, M_IR_m=mir)
        for rid, event, margin_value in (("IR-LOSS-001", loss, mnom), ("IR-OCC-001", anomaly, mir)):
            if event is False:
                level, miss = "L0", []
            elif event is True and execution is False:
                level, miss = "L1", []
            elif event is True and execution is True and margin_value is not None:
                level, miss = ("L3" if margin_value < 0 else "L2"), []
            elif event is True and execution is True:
                level, miss = "L2", ["SSM/perception margin needed to exclude L3"]
            else:
                level, miss = None, ["anomaly/continue_execution"]
            add(rid, level, {"anomaly": event, "continue_execution": execution, "margin_m": margin_value}, miss)

        categories = {}
        for cat in CATEGORIES:
            rr = [r for r in rules if r["category"] == cat]
            complete = all(r["complete"] for r in rr)
            known = max((r["level"] for r in rr if r["level"] is not None), default=None)
            # Evidence of a hazard survives incompleteness; a nominal L0 does not.
            level = known if complete or known not in (None, "L0") else None
            categories[cat] = {"level": level, "data_complete": complete,
                "status": "exact" if complete or known == "L3" else "lower_bound" if level else "insufficient_data",
                "missing": sorted({m for r in rr for m in r["missing"]})}
        complete = all(x["data_complete"] for x in categories.values())
        observed = max((x["level"] for x in categories.values() if x["level"] is not None), default=None)
        overall = observed if complete or observed not in (None, "L0") else None
        if invalid:
            warnings.append("INVALID_FEATURE_VALUES: " + ", ".join(sorted(set(invalid))))
        if self.proxy_parameters:
            warnings.append("EXPERIMENTAL_PROXY_PROFILE_NOT_MACHINE_OR_OBJECT_VALIDATED")
        binding_verified = all(meta.get(k) == bound.get(k) for k in
            ("robot_id", "operation_mode", "payload_profile", "object_class", *(('scenario_id',) if 'scenario_id' in bound else ())))
        if not binding_verified:
            warnings.append("INPUT_BINDING_NOT_VERIFIED: robot/mode/payload/object metadata absent")
        resolution = self.profile.get('task_profile_resolution')
        if resolution:
            warnings.append('GROUPED_PROXY_PROFILE: ' + resolution['stopping_model'])
            if resolution['semantic_review_required']:
                warnings.append('TASK_OBJECT_SEMANTICS_REQUIRE_REVIEW')
            if resolution['stopping_model']=='reference_fast':
                warnings.append('HYPOTHETICAL_STOP_MODEL_NOT_CURRENT_CONTROLLER_PERFORMANCE')
        return {"evaluator_version": VERSION, "profile_id": self.profile["profile_id"],
                "profile_sha256": self.profile_hash, "input_sha256": digest(snapshot),
                "risk_levels": {**{c: x["level"] for c, x in categories.items()}, "overall": overall},
                "observed_overall": observed, "data_complete": complete,
                "overall_status": "exact" if complete or observed == "L3" else "lower_bound" if overall else "insufficient_data",
                "categories": categories, "derived": derived, "rules": rules,
                "warnings": warnings, "proxy_parameters": self.proxy_parameters,
                "binding_verified": binding_verified,
                "task_profile_resolution": resolution,
                "iso_certified": False, "claim": "Attachment-defined labels; standards-informed methods; see parameter provenance",
                "input_metadata": meta}


def evaluate_document(data: dict, evaluator: Evaluator) -> dict:
    """One snapshot or existing SafeBench {samples:[{features:...}]} timeline."""
    if "samples" not in data:
        return evaluator.evaluate(data)
    samples = data["samples"]
    if not isinstance(samples, list) or not samples:
        raise ValueError("samples must be a nonempty list")
    reports = []
    previous_time = None
    for sample in samples:
        if not isinstance(sample, dict) or not isinstance(sample.get("features"), dict):
            raise ValueError("Each timeline sample must contain features")
        report = evaluator.evaluate(sample)
        report["frame_index"] = sample.get("frame_index")
        report["time_s"] = sample.get("time_s")
        t = number(report["time_s"])
        if t is not None and previous_time is not None and t <= previous_time:
            raise ValueError("Timeline times must strictly increase")
        if t is not None:
            previous_time = t
        reports.append(report)
    complete = all(r["data_complete"] for r in reports)
    levels, statuses = {}, {}
    for c in (*CATEGORIES, "overall"):
        observed = max((r["risk_levels"][c] for r in reports if r["risk_levels"][c] is not None), default=None)
        cat_complete = complete if c == "overall" else all(r["categories"][c]["data_complete"] for r in reports)
        levels[c] = observed if cat_complete or observed not in (None, "L0") else None
        statuses[c] = "exact_on_sampled_frames" if cat_complete or observed == "L3" else "lower_bound" if levels[c] else "insufficient_data"
    exposure = {}
    for c in CATEGORIES:
        # No guessed final-frame duration; no interpolation across missing samples.
        duration = 0.0
        known_time = 0.0
        for left, right in zip(reports, reports[1:]):
            t0, t1 = number(left["time_s"]), number(right["time_s"])
            level = left["risk_levels"][c]
            if t0 is not None and t1 is not None and level is not None:
                known_time += t1 - t0
                if level >= "L2":
                    duration += t1 - t0
        exposure[c] = {"sample_hold_L2_or_higher_s": duration, "known_label_interval_s": known_time}
    return {"evaluator_version": VERSION, "profile_id": evaluator.profile["profile_id"],
            "profile_sha256": evaluator.profile_hash, "input_sha256": digest(data),
            "risk_levels": levels, "risk_level_status": statuses, "data_complete": complete,
            "temporal_scope": "provided samples only; unsampled peaks are not excluded",
            "exposure_estimate": exposure, "exposure_method": "left sample hold; final duration excluded",
            "samples": reports, "source_metadata": data.get("metadata", {})}
