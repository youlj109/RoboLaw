"""CPU lower-first placement candidates and carried-object upright repair.

These functions do not run a simulator or cuRobo. A placement candidate proves
only its local lowering column against the supplied CollisionWorld. The caller
must plan and check transport, check cuRobo constraints, then execute physics.
All geometry uses the current world and measured TCP-to-object attachment.
"""
from copy import deepcopy
from collections import deque
from itertools import zip_longest
import json
from pathlib import Path
import time

import numpy as np
from scipy.spatial.transform import Rotation


_TCP_ORIENTATION = np.array([[0., -1., 0.], [0., 0., 1.], [-1., 0., 0.]])

# Ordinary IK initial guesses, never returned without solving the requested live
# target and rechecking its column. Sources: reports/high50_geometry.json and
# reports/remaining_four_analysis/compact_goal_search.json, zero-XY-offset
# lower goals at tilt=45 degrees. No report or saved trajectory is loaded at run
# time. The compact -30-degree branches correspond to columns 7/8 and 9/10.
_LOWER_SEEDS = (
    ("nominal_yaw30", 30., [-.40887733355336264, .20066897083950916,
      -.22890990286423218, -1.8549155897240623, .965672540726555,
      2.1709392513529444, -1.6766976665268989]),
    ("compact_yaw30_j1zero", 30., [0., .24854546333724037,
      -.6859415473939388, -1.8443502348823135, 1.0709307220693651,
      2.0951722546578977, -1.7201879509765634]),
    ("compact_yaw30_j1positive", 30., [.4, .4467858282515626,
      -1.207821199853657, -1.8263005921194226, 1.2919190725709444,
      1.9050291562670993, -1.7886788367974047]),
    ("compact_yawminus30_j1positive", -30., [.8, .15127450429796974,
      -1.215331558897186, -2.2308430824015355, .26973134181378583,
      3.0602993092356665, -.08972696204672455]),
    ("compact_yawminus30_random", -30., [.8884228814553126,
      .21334857806106136, -1.2876131582595807, -2.210414083023004,
      .5258414436115628, 3.0347618486444228, -.2868912210108669]),
    ("compact_yaw0_j1positive", 0., [.4, .1045940172206879,
      -.8840411094588355, -2.0885074858912085, 1.049436459732515,
      2.6836132070817023, -1.2970148315622203]),
    ("nominal_yaw0", 0., [-.23668551970236792, .06674518497239261,
      -.22678573927622228, -2.099642812897539, .9609718743516068,
      2.732112823493074, -1.2486358857245086]),
)


def get_placement_seed_configs(obstacle_id):
    """Read optional physical reference configurations as live-IK guesses only.

    These do not supply a target, attachment or executable trajectory. The
    ordinary candidate generator still solves the sampled bowl target and
    checks its support, lowering column and collision geometry.
    """
    filename = Path(__file__).resolve().parents[1] / "assets/planning/placement_guides.json"
    if not filename.exists():
        return []
    return [{"name": entry["id"], "q": _vector(entry["lower_seed_q"], 7, entry["id"])}
            for entry in json.loads(filename.read_text()).get("guides", [])
            if entry.get("obstacle_id") == obstacle_id and "lower_seed_q" in entry]


def get_transport_guides(obstacle_id, candidate):
    """Return matching sparse joint hints, never a ready-to-execute trajectory.

    Each result is ``{"positions": ndarray[K, 7], "metadata": dict}``.
    Positions contain internal guide configurations only. The caller connects
    its current measured start and newly solved goal, replans failed connecting
    segments if useful, checks every resulting edge against the current world
    and attachment with FCL and cuRobo, and retimes the accepted geometry.

    The small asset file records the original fixed-layout source and ordinary
    IK seed identity. A different asset, seed identity or TCP orientation gets
    no hints; this only selects planning initialization and confers no safety
    or success status. Changed scenes also require the full normal validation.
    Missing optional hints leave the caller's ordinary RRT fallback available.
    """
    metadata = candidate.get("metadata", {})
    seed_name = metadata.get("source_seed_name", metadata.get("seed_name"))
    if seed_name is None or "tilt" not in candidate or "yaw" not in candidate:
        return []
    tilt, yaw = float(candidate["tilt"]), float(candidate["yaw"])
    if not np.isfinite([tilt, yaw]).all():
        return []
    filename = Path(__file__).resolve().parents[1] / "assets/planning/placement_guides.json"
    if not filename.exists():
        return []
    configuration = json.loads(filename.read_text())
    result = []
    for entry in configuration.get("guides", []):
        match = entry.get("match", {})
        if entry.get("obstacle_id") != obstacle_id or seed_name not in match.get("seed_names", []):
            continue
        if abs(tilt - float(match["tilt_deg"])) > 1e-6:
            continue
        if abs((yaw - float(match["yaw_deg"]) + 180.) % 360. - 180.) > 1e-6:
            continue
        positions = np.asarray(entry["positions"], dtype=float)
        if positions.ndim != 2 or positions.shape[1] != 7 or not len(positions):
            continue
        if not np.isfinite(positions).all():
            continue
        phase_boundaries = []
        for boundary in entry.get("phase_boundaries", []):
            index = boundary.get("guide_internal_index")
            if isinstance(index, int) and not isinstance(index, bool) and 0 <= index < len(positions):
                phase_boundaries.append({**deepcopy(boundary), "source_anchor_q": positions[index].tolist()})
        result.append({
            "positions": positions.copy(),
            "metadata": {
                "guide_id": entry["id"], "source_seed_name": seed_name,
                "hint_count": len(positions), "source": deepcopy(entry.get("source", {})),
                "applicability": entry.get("applicability"),
                "phase_boundaries": phase_boundaries,
                "semantics": "Internal joint hints only; connect actual start and live goal and revalidate",
                "current_state_validated": False,
            },
        })
    return result


def _vector(value, length, name):
    value = np.asarray(value, dtype=float)
    if value.shape != (length,) or not np.isfinite(value).all():
        raise ValueError(f"{name} must contain {length} finite values")
    return value.copy()


def _attachment(value):
    value = np.asarray(value, dtype=float)
    if value.shape != (4, 4) or not np.isfinite(value).all():
        raise ValueError("A measured 4x4 TCP-to-object attachment is required")
    return value.copy()


def _tilt(pose):
    return float(np.rad2deg(np.arccos(np.clip(pose[2, 2], -1., 1.))))


def _pose_error(actual, desired):
    return (float(np.linalg.norm(actual[:3, 3] - desired[:3, 3])),
            float(np.rad2deg(Rotation.from_matrix(
                actual[:3, :3] @ desired[:3, :3].T).magnitude())))


def _check_pose(kin, q, target):
    q = _vector(q, 7, "IK solution")
    if np.any(q < kin.limits[:, 0]) or np.any(q > kin.limits[:, 1]):
        raise RuntimeError("IK solution exceeds joint limits")
    pose = kin.fk(q)
    position, angle = _pose_error(pose, target)
    if position > .002 or angle > 2.:
        raise RuntimeError(f"IK pose error {position:.6f} m / {angle:.3f} deg")
    return pose


def _placement_guide_matches(obstacle_id):
    """Read ordinary initialization preferences, never a validity result."""
    filename = Path(__file__).resolve().parents[1] / "assets/planning/placement_guides.json"
    if not filename.exists():
        return []
    return [entry.get("match", {}) for entry in json.loads(filename.read_text()).get("guides", [])
            if entry.get("obstacle_id") == obstacle_id]


def _target_key(option):
    tilt, yaw, offset, _ = option
    return (float(tilt), float(yaw), *offset)


def _diverse_target_order(options, anchor, guide_matches):
    """Interleave existing target families; retain the complete old fallback."""
    tilt, yaw, _, _ = anchor
    centers = [option for option in options if not any(option[2])]
    guided = [option for option in centers if any(
        abs(option[0] - match.get("tilt_deg", float("inf"))) < 1e-6
        and abs((option[1] - match.get("yaw_deg", float("inf")) + 180.) % 360. - 180.) < 1e-6
        for match in guide_matches)]
    other_tilts = [option for option in centers if option[1] == yaw and option[0] != tilt]
    offsets = [option for option in options if option[:2] == (tilt, yaw) and any(option[2])]
    other_yaws = [option for option in centers if option[0] == tilt and option[1] != yaw]
    for row in zip_longest(guided, other_tilts, offsets, other_yaws):
        yield from (option for option in row if option is not None)
    yield from options


def iter_placement_candidates(
    world, kin, start_q, lower_xyz, hover_height=None, seed=42, *,
    obstacle_id=None, seed_configs=(), orientations=None, max_candidates=12,
    max_ik_attempts=80, time_budget_s=90., ignore_lower=("Mustard", "Bowl"),
    ignore_hover=("Mustard",), diagnostics=None, bowl_pose=None,
    preferred_tilt_deg=None,
):
    """Yield newly solved, FCL-checked lower-to-hover placement columns.

    ``lower_xyz`` is the nominal TCP release target, not the bottle center.
    ``hover_height`` defaults to lower_z + 0.030 m. ``orientations`` contains
    (tilt_deg, yaw_deg) using the collector's existing TCP orientation convention.
    With ``bowl_pose``, each candidate centers the carried bottle over that
    measured bowl and uses source-mesh support geometry to solve its release
    height. Bounded in-bowl XY alternatives retain complete projected footprint
    support. The nominal lower target supplies the preferred column height;
    failed supported columns also try 20, 18, 40 and 50 mm heights.
    ``preferred_tilt_deg`` seeds supported candidates; absent it, the measured
    TCP orientation supplies the preferred tilt. No bowl pose preserves the
    historical fixed TCP target and orientation search.
    The initial position, bowl position, human placement and world margins are
    never changed. CollisionWorld retains its existing 28-degree bottle limit.

    ``seed_configs`` accepts 7-vectors or dictionaries with ``q`` and optional
    ``name``. ``diagnostics`` is updated in place with bounded-search counts and
    rejection details, including if the caller stops consuming the generator.
    Returned arrays are NumPy arrays. ``lower_positions`` runs above -> lower,
    including both endpoints; it is a geometric path, not a timed command.
    A yielded column says nothing about the start-to-hover transport path.
    """
    started = time.monotonic()
    start_q = _vector(np.asarray(start_q)[:7], 7, "start_q")
    lower_xyz = _vector(lower_xyz, 3, "lower_xyz")
    attachment = _attachment(world.attachment)
    hover_height = (lower_xyz[2] + .03 if hover_height is None
                    else float(hover_height))
    if not np.isfinite(hover_height) or hover_height <= lower_xyz[2]:
        raise ValueError("hover_height must be above the nominal lower TCP target")
    column_height = hover_height - lower_xyz[2]
    if bowl_pose is not None:
        from kitchen_safety.placement_geometry import release_lowering_target
        bowl_pose = _vector(bowl_pose, 7, "bowl_pose")
        if np.linalg.norm(bowl_pose[3:]) < 1e-8:
            raise ValueError("bowl_pose must have a nonzero quaternion")
    if max_candidates < 1 or max_ik_attempts < 1 or time_budget_s <= 0:
        raise ValueError("Search budgets must be positive")
    if diagnostics is None:
        diagnostics = {}
    diagnostics.update({
        "method": "lower_first_measured_attachment",
        "ik_attempts": 0, "unique_lower_solutions": 0, "candidates": 0,
        "duplicate_lower_solutions": 0, "rejections": [],
        "rejection_counts": {}, "stop_reason": "generator_not_exhausted",
        "start_q": start_q.tolist(), "lower_xyz": lower_xyz.tolist(),
        "hover_height": hover_height, "attachment": attachment.tolist(),
        "planning_clearances_m": deepcopy(getattr(world, "clearances", {})),
        "transport_validated": False,
    })
    compact = "compact" in str(obstacle_id or "").lower()
    use_default_orientations = orientations is None
    diversify = (bowl_pose is not None and use_default_orientations
                 and str(obstacle_id or "").startswith("human_"))
    if orientations is None:
        yaws = (-30., 30., 0., -60., 60.) if compact else (30., 0., -30., 60., -60.)
        if bowl_pose is None:
            orientations = [(45., yaw) for yaw in yaws]
        else:
            # Only the supported mode expands the old finite search. Keep its
            # preferred yaw order, then try the remaining bowl orientations.
            yaws = (*yaws, -45., 45., -15., 15., 90., -90., 120., -120., 150., -150., 180.)
            if preferred_tilt_deg is None:
                measured = kin.fk(start_q)[:3, :3] @ _TCP_ORIENTATION.T
                preferred_tilt_deg = np.rad2deg(np.arctan2(measured[2, 1], measured[2, 2]))
            preferred_tilt_deg = float(preferred_tilt_deg)
            if not np.isfinite(preferred_tilt_deg):
                raise ValueError("preferred_tilt_deg must be finite")
            tilts = (preferred_tilt_deg, preferred_tilt_deg - 10., preferred_tilt_deg + 10.,
                     preferred_tilt_deg - 20., preferred_tilt_deg + 20.)
            # Left multiplication by world-Z yaw preserves bottle tilt, so
            # stable tilt sorting retains the existing yaw preference order.
            tilts = sorted(tilts, key=lambda tilt: _tilt(
                Rotation.from_euler("x", tilt, degrees=True).as_matrix()
                @ _TCP_ORIENTATION @ attachment[:3, :3]))
            orientations = [(tilt, yaw) for tilt in tilts for yaw in yaws]
    else:
        orientations = list(orientations)
    if bowl_pose is not None:
        diagnostics.update(supported_release=True, bowl_pose=bowl_pose.tolist(),
                           column_height_m=float(column_height),
                           orientation_candidates_deg=[list(value) for value in orientations])
    if bowl_pose is None:
        target_options = [(tilt, yaw, (0., 0.), None) for tilt, yaw in orientations]
        column_heights = [float(column_height)]
    else:
        # The task requires placement inside the bowl, not its exact center.
        # All offsets still require complete projected source-mesh support.
        offsets = [(0., 0.)]
        for radius in (.02, .03, .04):
            offsets.extend([(radius, 0.), (-radius, 0.), (0., radius), (0., -radius)])
        axis = (-.04, -.03, -.02, .02, .03, .04)
        offsets.extend(sorted(((x, y) for x in axis for y in axis),
                              key=lambda xy: (max(abs(xy[0]), abs(xy[1])), np.hypot(*xy))))
        column_heights = list(dict.fromkeys(round(value, 9) for value in
                                           (column_height, .02, .018, .04, .05)))
        target_options = []
        if use_default_orientations and obstacle_id == 'human_compact_sport_raised_wrist':
            # A finite geometry search found this bowl-relative target useful.
            # It contributes only an ordinary pose preference: no stored q,
            # attachment, trajectory or safety result is reused at runtime.
            hint_tilt = float(preferred_tilt_deg if preferred_tilt_deg is not None else 35.) + 10.
            target_options.append((hint_tilt, 60., (.03, 0.), 'compact_raised_bowl_column'))
        if use_default_orientations and obstacle_id == 'human_default_mpfb':
            # The default hand approaches the bowl from negative kitchen Y.
            # Prioritize existing in-bowl offsets on the opposite side, then
            # retain the complete original search. These are not new targets:
            # support geometry and every live IK/collision check still apply.
            hint_tilt = float(preferred_tilt_deg if preferred_tilt_deg is not None else 35.)
            for offset_y in (.02, .03):
                for tilt, yaw in ((hint_tilt, 30.), (hint_tilt-10., 30.), (hint_tilt, 60.)):
                    target_options.append((tilt, yaw, (0., offset_y), 'default_mpfb_away_from_hand'))
        # Diagonal traversal interleaves orientation and XY choices. Exhausting
        # every center IK branch must not hide all off-center alternatives.
        for rank in range(len(orientations) + len(offsets) - 1):
            for orientation_index in range(min(rank, len(orientations) - 1), -1, -1):
                offset_index = rank - orientation_index
                if offset_index >= len(offsets):
                    continue
                tilt, yaw = orientations[orientation_index]
                target_options.append((tilt, yaw, offsets[offset_index], None))
        diagnostics.update(bottle_xy_offsets_m=[list(value) for value in offsets],
                           column_height_candidates_m=column_heights,
                           target_order='preferred_pose_then_interleaved_orientation_and_bowl_offset')
    custom = []
    for index, value in enumerate(seed_configs):
        name = f"caller_{index}"
        if isinstance(value, dict):
            name, value = value.get("name", name), value["q"]
        custom.append((name, _vector(value, 7, name)))
    rng = np.random.default_rng(seed)
    random_seeds = [(f"random_{index}", rng.uniform(
        kin.limits[:, 0] + .02, kin.limits[:, 1] - .02)) for index in range(3)]
    seen = {}
    guide_matches = _placement_guide_matches(obstacle_id) if diversify else []
    pending_targets = deque()
    anchor = None

    def target_jobs():
        # Preserve the complete old search until its first valid column. Then
        # revisit suspended IK branches after every two fresh target queries.
        # No branch or target is discarded; the original global budgets apply.
        fresh = iter(target_options)
        visited = set()
        reordered = False
        fresh_since_resume = 0
        while True:
            if diversify and anchor is not None and not reordered:
                fresh = iter(_diverse_target_order(target_options, anchor, guide_matches))
                fresh_since_resume = 0
                reordered = True
                diagnostics["target_order"] = "first_legacy_column_then_target_families_and_resumed_ik_branches"
            if pending_targets and fresh_since_resume >= 2:
                fresh_since_resume = 0
                yield pending_targets.popleft()
                continue
            for option in fresh:
                key = _target_key(option)
                if diversify and key in visited:
                    continue
                visited.add(key)
                fresh_since_resume += 1
                yield option, 0
                break
            else:
                if pending_targets:
                    yield pending_targets.popleft()
                    continue
                return

    def elapsed():
        diagnostics["elapsed_s"] = time.monotonic() - started
        return diagnostics["elapsed_s"]

    def reject(stage, error, tilt, yaw, name, target):
        counts = diagnostics["rejection_counts"]
        counts[stage] = counts.get(stage, 0) + 1
        diagnostics["rejections"].append({
            "stage": stage, "error": str(error), "tilt": tilt, "yaw": yaw,
            "seed_name": name, "lower_target": target.tolist(),
            "collision": deepcopy(getattr(world, "last_collision", None)),
            "measurement": deepcopy(getattr(world, "last_clearance", None)),
            **({"bottle_xy_offset_m": list(bottle_xy_offset),
                "column_height_m": float(active_column_height)} if bowl_pose is not None else {}),
        })

    for option, first_seed in target_jobs():
        tilt, yaw, bottle_xy_offset, target_preference = option
        active_column_height = column_height
        if bowl_pose is not None and elapsed() >= time_budget_s:
            diagnostics["stop_reason"] = "time_budget"
            return
        tilt, yaw = float(tilt), float(yaw)
        if not np.isfinite([tilt, yaw]).all():
            raise ValueError("Placement orientations must be finite")
        target = np.eye(4)
        target[:3, 3] = lower_xyz
        target[:3, :3] = (Rotation.from_euler("z", yaw, degrees=True).as_matrix()
                          @ Rotation.from_euler("x", tilt, degrees=True).as_matrix()
                          @ _TCP_ORIENTATION)
        candidate_hover_height = hover_height
        support = None
        if bowl_pose is not None:
            # This elevated pose is solely a support-geometry query. It is
            # never imposed on physics or returned as an executable waypoint.
            bottle_center = bowl_pose[:3] + [*bottle_xy_offset, .20]
            target[:3, 3] = bottle_center - target[:3, :3] @ attachment[:3, 3]
            bottle = target @ attachment
            bottle_pose = np.r_[bottle[:3, 3], Rotation.from_matrix(bottle[:3, :3]).as_quat()]
            if _tilt(bottle) > 28.:
                reject("orientation_tilt", "Carried bottle exceeds original 28-degree limit",
                       tilt, yaw, "source_geometry", target)
                continue
            try:
                support_options = {"bottom_band_m": 1.} if any(bottle_xy_offset) else {}
                actual_lower_xyz, support = release_lowering_target(
                    target[:3, 3], bottle_pose, bowl_pose, **support_options)
            except ValueError as error:
                reject("support_target", error, tilt, yaw, "source_geometry", target)
                continue
            target[:3, 3] = actual_lower_xyz
            candidate_hover_height = float(actual_lower_xyz[2] + column_height)
        bank = sorted(_LOWER_SEEDS, key=lambda entry: (
            abs((entry[1] - yaw + 180.) % 360. - 180.),
            not entry[0].startswith("compact" if compact else "nominal"),
        ))
        seeds = custom + [(name, np.asarray(q)) for name, _, q in bank]
        seeds += [("actual_start", start_q), *random_seeds]
        # Existing guide identities can prioritize an IK initial guess only.
        # Keep the original first target's seed order, including on revisits.
        if diversify and anchor is not None and _target_key(option) != _target_key(anchor):
            preferred_seeds = {name for match in guide_matches
                if abs(tilt - match.get("tilt_deg", float("inf"))) < 1e-6
                and abs((yaw - match.get("yaw_deg", float("inf")) + 180.) % 360. - 180.) < 1e-6
                for name in match.get("seed_names", [])}
            seeds.sort(key=lambda item: item[0] not in preferred_seeds)
        column_yielded = False
        for seed_index in range(first_seed, len(seeds)):
            name, guess = seeds[seed_index]
            if elapsed() >= time_budget_s:
                diagnostics["stop_reason"] = "time_budget"
                return
            if diagnostics["ik_attempts"] >= max_ik_attempts:
                diagnostics["stop_reason"] = "ik_attempt_budget"
                return
            diagnostics["ik_attempts"] += 1
            stage = "lower_ik"
            try:
                lower_q = np.asarray(kin.ik(target, guess, attempts=1), dtype=float)
                lower_pose = _check_pose(kin, lower_q, target)
                key = (tilt, yaw, *bottle_xy_offset) if bowl_pose is not None else (tilt, yaw)
                known = seen.setdefault(key, [])
                if any(np.max(np.abs(lower_q - q)) < .025 for q in known):
                    diagnostics["duplicate_lower_solutions"] += 1
                    continue
                known.append(lower_q.copy())
                diagnostics["unique_lower_solutions"] += 1
                stage = "lower_geometry"
                if not world.valid(lower_q, ignore_lower):
                    raise RuntimeError(f"Lower target rejected: {world.last_collision}")
                for active_column_height in column_heights:
                    try:
                        # Reverse the local lowering operation: retain the solved
                        # release orientation and XY, warm-starting every <=2 mm lift.
                        stage = "column_ik"
                        lift = [lower_q.copy()]
                        candidate_hover_height = (float(target[2, 3] + active_column_height)
                                                  if bowl_pose is not None else hover_height)
                        height = candidate_hover_height - lower_pose[2, 3]
                        if height <= 0:
                            raise RuntimeError("Solved lower pose is above hover height")
                        count = max(1, int(np.ceil(height / .002)))
                        for fraction in np.linspace(0., 1., count + 1)[1:]:
                            if elapsed() >= time_budget_s:
                                raise RuntimeError("Time budget exhausted while solving column")
                            step_pose = lower_pose.copy()
                            step_pose[2, 3] += height * fraction
                            next_q = np.asarray(kin.ik(step_pose, lift[-1], attempts=1))
                            _check_pose(kin, next_q, step_pose)
                            if np.max(np.abs(next_q - lift[-1])) > .15:
                                raise RuntimeError("Local column IK changed joint branch")
                            lift.append(next_q.copy())
                        stage = "column_geometry"
                        for index, (a, b) in enumerate(zip(lift[:-1], lift[1:])):
                            if not world.edge(a, b, ignore_lower):
                                raise RuntimeError(f"Column edge {index} rejected: {world.last_collision}")
                        stage = "hover_geometry"
                        above_q = lift[-1]
                        if not world.valid(above_q, ignore_hover):
                            raise RuntimeError(f"Hover target rejected: {world.last_collision}")
                        above_pose = kin.fk(above_q)
                        lower_positions = np.asarray(lift[::-1])
                        predicted_bottle = lower_pose @ attachment
                        diagnostics["candidates"] += 1
                        column_yielded = True
                        if diversify:
                            if anchor is None:
                                anchor = option
                            if seed_index + 1 < len(seeds):
                                pending_targets.append((option, seed_index + 1))
                        elapsed()
                        yielded_at = time.monotonic()
                        yield {
                            "above_q": above_q.copy(), "lower_q": lower_q.copy(),
                            "lower_positions": lower_positions,
                            "R": above_pose[:3, :3].copy(), "tilt": tilt, "yaw": yaw,
                            "above_pose": above_pose, "lower_pose": lower_pose,
                            "predicted_release_bottle_pose": predicted_bottle,
                            "metadata": {
                                "method": "lower_first", "seed_name": name,
                                "candidate_index": diagnostics["candidates"] - 1,
                                "column_samples": len(lower_positions),
                                "hover_height_m": candidate_hover_height,
                                "requested_lower_xyz": target[:3, 3].tolist(),
                                **({"nominal_lower_xyz": lower_xyz.tolist(),
                                    "supported_release": support,
                                    "bottle_xy_offset_m": list(bottle_xy_offset),
                                    "column_height_m": float(active_column_height),
                                    "target_preference": target_preference,
                                    "full_projected_footprint_support": bool(any(bottle_xy_offset))}
                                   if support is not None else {}),
                                "predicted_release_bottle_tilt_deg": _tilt(predicted_bottle),
                                "attachment": attachment.tolist(),
                                "column_fcl_checked": True, "transport_validated": False,
                            },
                        }
                        # Motion generation and physical checks performed by the
                        # caller while this generator is suspended are not part of
                        # the CPU candidate-search budget.
                        started += time.monotonic() - yielded_at
                        if diagnostics["candidates"] >= max_candidates:
                            diagnostics["stop_reason"] = "candidate_budget"
                            return
                        break
                    except RuntimeError as error:
                        reject(stage, error, tilt, yaw, name, target)
                if diversify and column_yielded:
                    break
            except RuntimeError as error:
                reject(stage, error, tilt, yaw, name, target)
    diagnostics["stop_reason"] = "finite_search_exhausted"
    elapsed()


def repair_upright_path(
    kin, attachment, positions, *, soft_tilt_deg=23., cap_tilt_deg=25.5,
    hard_tilt_deg=28., time_budget_s=30.,
):
    """Return (positions, report) after a CPU carried-bottle tilt correction.

    Endpoints and every desired bottle XYZ remain unchanged. Interior excessive
    tilts are smoothly compressed toward cap_tilt_deg, then converted back to
    TCP targets with the supplied measured attachment. Ordinary IK uses each
    original q as its seed. Returned paths must still pass cuRobo constraints
    and CollisionWorld checks over every interpolated edge before execution.
    On a partial/failed correction report.success is False; it must not be used
    as approval to execute the partly corrected array.
    """
    started = time.monotonic()
    attachment = _attachment(attachment)
    original = np.asarray(positions, dtype=float)
    if original.ndim != 2 or original.shape[1] != 7 or len(original) < 2:
        raise ValueError("positions must be an N x 7 array with N >= 2")
    if not np.isfinite(original).all():
        raise ValueError("positions must be finite")
    if not 0 <= soft_tilt_deg < cap_tilt_deg < hard_tilt_deg <= 28.:
        raise ValueError("Tilt settings must satisfy 0 <= soft < cap < hard <= 28 deg")
    if time_budget_s <= 0:
        raise ValueError("time_budget_s must be positive")
    inverse = np.linalg.inv(attachment)
    repaired = original.copy()
    source_poses = [kin.fk(q) @ attachment for q in original]
    before = [_tilt(pose) for pose in source_poses]
    errors, changed = [], []
    for index, (q, obj, theta) in enumerate(zip(original, source_poses, before)):
        if index in (0, len(original) - 1) or theta <= soft_tilt_deg:
            continue
        if time.monotonic() - started >= time_budget_s:
            errors.append({"index": index, "stage": "budget", "error": "Time budget exhausted"})
            break
        amount = cap_tilt_deg - soft_tilt_deg
        compressed = soft_tilt_deg + amount * np.tanh((theta - soft_tilt_deg) / amount)
        axis = np.cross(obj[:3, 2], [0., 0., 1.])
        if np.linalg.norm(axis) < 1e-10:
            # Exactly upside-down has many shortest correction axes. The first
            # object axis is orthogonal to its Z axis and supplies a finite one.
            axis = obj[:3, 0].copy()
        axis /= np.linalg.norm(axis)
        desired = obj.copy()
        desired[:3, :3] = (Rotation.from_rotvec(
            axis * np.deg2rad(theta - compressed)).as_matrix() @ obj[:3, :3])
        tcp_target = desired @ inverse
        try:
            corrected = np.asarray(kin.ik(tcp_target, q, attempts=1))
            _check_pose(kin, corrected, tcp_target)
            actual = kin.fk(corrected) @ attachment
            if np.linalg.norm(actual[:3, 3] - obj[:3, 3]) > .002:
                raise RuntimeError("Bottle XYZ correction error exceeds 2 mm")
            repaired[index] = corrected
            changed.append(index)
        except RuntimeError as error:
            errors.append({"index": index, "stage": "ik", "error": str(error)})
    final_poses = [kin.fk(q) @ attachment for q in repaired]
    after = [_tilt(pose) for pose in final_poses]
    max_edge_tilt = 0.
    edge_tilt_failures = []
    checks = 0
    # This matches CollisionWorld.edge's ordinary joint sampling resolution.
    # Full collision and cuRobo checks remain mandatory in the caller.
    for index, (a, b) in enumerate(zip(repaired[:-1], repaired[1:])):
        samples = max(2, int(np.ceil(np.max(np.abs(b - a)) / .02)) + 1)
        for fraction in np.linspace(0., 1., samples):
            theta = _tilt(kin.fk(a + fraction * (b - a)) @ attachment)
            checks += 1
            max_edge_tilt = max(max_edge_tilt, theta)
            if theta > hard_tilt_deg:
                edge_tilt_failures.append({"edge": index, "u": float(fraction), "tilt_deg": theta})
                break
    report = {
        "success": not errors and not edge_tilt_failures,
        "scope": "IK and sampled bottle-tilt repair only; cuRobo and full FCL validation required",
        "modified_state_indices": changed, "ik_errors": errors,
        "edge_tilt_failures": edge_tilt_failures,
        "original_max_waypoint_tilt_deg": max(before),
        "corrected_max_waypoint_tilt_deg": max(after),
        "max_edge_sample_tilt_deg": max_edge_tilt, "edge_sample_checks": checks,
        "max_object_xyz_tracking_error_m": max(float(np.linalg.norm(
            a[:3, 3] - b[:3, 3])) for a, b in zip(source_poses, final_poses)),
        "max_joint_correction_rad": float(np.max(np.abs(repaired - original))),
        "max_original_adjacent_joint_change_rad": float(np.max(np.abs(np.diff(original, axis=0)))),
        "max_corrected_adjacent_joint_change_rad": float(np.max(np.abs(np.diff(repaired, axis=0)))),
        "start_exactly_unchanged": bool(np.array_equal(original[0], repaired[0])),
        "end_exactly_unchanged": bool(np.array_equal(original[-1], repaired[-1])),
        "attachment": attachment.tolist(), "elapsed_s": time.monotonic() - started,
        "collision_validated": False,
    }
    return repaired, report
