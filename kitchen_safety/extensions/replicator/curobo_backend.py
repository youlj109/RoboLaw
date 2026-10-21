"""Obstacle-aware internal hover/lift waypoints; original grasp/task goals fixed.

The same rule is available in no-hazard controls: try the original waypoint
first. All alternative paths are fully planned/checked before any action.
"""
import numpy as np
from kitchen_safety.curobo_backend import CuroboPlanner as OriginalCuroboPlanner
from kitchen_safety.planning_checks import check_planned_path

class CuroboPlanner(OriginalCuroboPlanner):
    def _free_route(self,q_arm7,target,ignore_names,grip_binary,phase):
        attempts=[]
        for index in range(4):
            try:
                route=super().plan_pose(q_arm7,target,ignore_names,grip_binary,phase=phase)
                if check_planned_path(self.world,route['positions'],ignore_names):
                    route['metadata']['independent_geometry_retries']=attempts
                    route['metadata']['same_seed_stream']=self.seed
                    return route
                attempts.append(dict(attempt=index,error='Independent geometry rejected path',pair=self.world.last_collision))
            except RuntimeError as error:
                attempts.append(dict(attempt=index,error=str(error)))
        raise RuntimeError(f'Four cuRobo proposals failed unchanged independent geometry: {attempts}')

    def _pregrasp(self,q_arm7,goal,ignore_names,grip_binary):
        failures=[]
        for dz in [0.,-.04,-.075,.04,.08]:
            target=np.asarray(goal).copy();target[2,3]+=dz
            try:
                # Lower alternative hovers must not touch the free bottle on
                # their global arrival path; contact is reserved for approach.
                route_ignore=ignore_names if dz==0 else tuple(n for n in ignore_names if n!='mustard_bottle')
                route=self._free_route(q_arm7,target,route_ignore,grip_binary,'pregrasp')
                if dz:
                    grasp=np.asarray(goal).copy();grasp[2,3]-=.125
                    approach=super().plan_pose(route['positions'][-1],grasp,ignore_names,grip_binary,phase='approach')
                    if not check_planned_path(self.world,approach['positions'],ignore_names):
                        raise RuntimeError(f'Approach lookahead violates geometry: {self.world.last_collision}')
                route['metadata']['obstacle_aware_pregrasp']=dict(nominal_target=np.asarray(goal).tolist(),selected_target=target.tolist(),height_delta_m=dz,actual_grasp_target_unchanged=True,rejected_candidates=failures,approach_lookahead_passed=bool(dz),rejected_path_executed_actions=0)
                route['effective_goal_pose']=target;self.last_plan=route['metadata'];return route
            except RuntimeError as error:failures.append(dict(height_delta_m=dz,error=str(error),executed_actions=0))
        raise RuntimeError(f'No safe hover and unchanged approach target: {failures}')

    def plan_pose(self,q_arm7,T_W_TCP,ignore_names=(),grip_binary=0,phase=None):
        try:
            if phase=='pregrasp':return self._pregrasp(q_arm7,T_W_TCP,ignore_names,grip_binary)
            if phase=='above_bowl':return self._free_route(q_arm7,T_W_TCP,ignore_names,grip_binary,phase)
            route=super().plan_pose(q_arm7,T_W_TCP,ignore_names,grip_binary,phase=phase)
            if phase=='lift' and not check_planned_path(self.world,route['positions'],ignore_names):
                raise RuntimeError(f'Nominal lift violates independent geometry: {self.world.last_collision}')
            return route
        except RuntimeError as original_error:
            if phase!='lift' or self.world.attachment is None:raise
            start=self.kin.fk(np.asarray(q_arm7));goal=np.asarray(T_W_TCP)
            if goal[2,3]-start[2,3] <= .035:
                raise RuntimeError('Rejected lift has insufficient vertical room for safe routing prefix') from original_error
            prefix_target=start.copy();prefix_target[2,3]+=.025
            prefix=super().plan_pose(q_arm7,prefix_target,ignore_names,grip_binary,phase='lift')
            if not check_planned_path(self.world,prefix['positions'],ignore_names):
                raise RuntimeError(f'Vertical separation prefix violates geometry: {self.world.last_collision}')
            failures=[]
            # Deterministic internal-waypoint shell for every variant/control.
            # Task grasp and placement targets are never changed.
            offsets=[(0.,0.,z) for z in [0.,.04,.08]]+[(x,y,z) for z in [0.,.04] for x,y in [(-.08,0.),(.08,0.),(0.,-.08),(0.,.08),(-.12,0.),(.12,0.),(-.08,-.08),(-.08,.08),(.08,-.08),(.08,.08)]]
            for offset in offsets:
                target=goal.copy();target[:3,3]+=offset
                try:
                    route=self._free_route(prefix['positions'][-1],target,ignore_names,grip_binary,'lift_avoidance')
                    positions=np.concatenate([prefix['positions'],route['positions'][1:]])
                    object_positions=np.array([(self.kin.fk(q)@self.world.attachment)[:3,3] for q in route['positions']]);original_object=(start@self.world.attachment)[:3,3]
                    if object_positions[:,2].min() < original_object[2]+.02:
                        raise RuntimeError('Lift route would dip the carried object toward its original support')
                    validation=self.validate_path(positions,ignore_names,grip_binary,phase='lift')
                    if not validation['valid']:raise RuntimeError(f'Complete prefixed lift failed original constraints: {validation}')
                    metadata=dict(route['metadata'],method='vertical_separation_then_motion_gen',original_vertical_rejection=str(original_error),nominal_lift_target=goal.tolist(),selected_lift_target=target.tolist(),offset_world_m=list(offset),rejected_lift_candidates=failures,vertical_prefix_height_m=.025,vertical_prefix_samples=len(prefix['positions']),free_route_samples=len(route['positions']),minimum_object_height_gain_in_free_route_m=float(object_positions[:,2].min()-original_object[2]),complete_path_validation=validation,samples=len(positions),rejected_path_executed_actions=0)
                    self.last_plan=metadata
                    return dict(positions=positions,dt=self.dt,metadata=metadata,effective_goal_pose=target)
                except RuntimeError as error:failures.append(dict(offset_world_m=list(offset),error=str(error),executed_actions=0))
            raise RuntimeError(f'No safe prefixed lift route: {failures}') from original_error
