"""CPU checks for continuous-domain and conservative reach certificate helpers."""
import json
import unittest
import numpy as np
import trimesh
from pxr import Usd,UsdGeom
from scipy.spatial import ConvexHull
from kitchen_safety.planning import DroidKinematics
from kitchen_safety.extensions.replicator.collision_planner import CollisionWorld
from scripts.extensions.replicator.variant_geometry import SourceRobotGeometry
from scripts.extensions.replicator.search_source_layout_candidates import CandidateSearch
from scripts.extensions.replicator.search_l_island_bowl_domain import source_position_domain
from scripts.extensions.replicator.source_domain_rounding import float32_outer_domain
from scripts.extensions.replicator.certify_l_island_bowl_reach import (
    ROOT,INSET_M,convex_source_prism,all_gripper_reach_bound,all_robot_reach_bound,
)


class SourceDomainReachTests(unittest.TestCase):
    def test_float32_outer_domain_contains_real_rounded_boundary_counterexamples(self):
        import torch
        layout=json.loads((ROOT/'configs/extensions/replicator/l_island_bowl_sink_source_layout.json').read_text())
        scene=CandidateSearch(layout)
        examples={'droid':[-2.3626777456832886,1.907289630174637,.7999999523162842],
                  'bowl':[-.38407073765993116,2.107334315614605,.9442080386430025]}
        for subject,example in examples.items():
            domain=float32_outer_domain(scene,subject)
            lo,hi=np.array(domain['origin_xyz_lower']),np.array(domain['origin_xyz_upper'])
            old=domain['analytic_real_arithmetic_domain']
            self.assertTrue(np.any(np.array(example)<old['origin_xyz_lower']))
            self.assertTrue(np.all(example>=lo) and np.all(example<=hi))
            origin=(np.asarray(layout['robot_position'])+[0,0,layout['stand_height']] if subject=='droid'
                    else np.array(layout['objects'][subject]['position']))
            poses=scene.positions(subject,np.array(example)-origin)
            self.assertTrue(all(v._validate(poses,scene.bounds) for v in scene.validators))
            bbox=scene.bounds[scene.assets[subject]]
            self.assertEqual(bbox.translated(tuple(example)).min_point.dtype,torch.float32)
            # Validate source predicates immediately outside the *outer* domain,
            # including scalar64 Z. Accepted points there would invalidate proof.
            center=(lo+hi)/2
            for axis in range(3):
                for endpoint,sign in ((lo[axis],-1),(hi[axis],1)):
                    position=center.copy();position[axis]=endpoint+sign*1e-10
                    poses=scene.positions(subject,position-origin)
                    self.assertFalse(all(v._validate(poses,scene.bounds) for v in scene.validators),(subject,axis,sign))

    def test_all_robot_bound_includes_arm_and_primitive_inventory(self):
        world=CollisionWorld(Usd.Stage.CreateInMemory(),DroidKinematics([0,0,.8],0.))
        robot=SourceRobotGeometry(world);report=all_robot_reach_bound(world,robot)
        self.assertEqual({p['path'] for p in report['parts']},{p[0] for p in robot.parts})
        self.assertTrue(any('primitive' in p['representation'] and p['body']=='/panda/panda_link8' for p in report['parts']))
        self.assertTrue(any(p['body']=='/panda/panda_link0' for p in report['parts']))
        self.assertAlmostEqual(report['radius_upper_bound_m'],1.2086786787259252,places=12)

    def test_inner_prism_is_inside_real_convex_mesh(self):
        cube=trimesh.creation.box(extents=[2,2,2])
        polygon,info=convex_source_prism('unit_box',np.asarray(cube.vertices),np.asarray(cube.faces),-.5,.5)
        self.assertLess(polygon.area,4.)
        self.assertLessEqual(info['interior_plane_max_signed_m'],-INSET_M+1e-12)
        self.assertTrue(info['source_watertight'])

    def test_open_source_surface_cannot_certify_solid_overlap(self):
        cube=trimesh.creation.box(extents=[2,2,2])
        with self.assertRaises(ValueError):
            convex_source_prism('open_box',np.asarray(cube.vertices),np.asarray(cube.faces)[:-1],-.5,.5)

    def test_continuous_domain_boundaries_match_original_validators(self):
        layout=json.loads((ROOT/'configs/extensions/replicator/l_island_bowl_sink_source_layout.json').read_text())
        scene=CandidateSearch(layout)
        original=np.asarray(layout['robot_position'])+[0,0,layout['stand_height']]
        for subject in ('droid','bowl'):
            domain=source_position_domain(scene,subject)
            lo,hi=np.array(domain['origin_xyz_lower']),np.array(domain['origin_xyz_upper'])
            origin=original if subject=='droid' else np.array(layout['objects'][subject]['position'])
            def valid(position):
                poses=scene.positions(subject,position-origin)
                return all(v._validate(poses,scene.bounds) for v in scene.validators)
            center=(lo+hi)/2
            self.assertTrue(valid(center))
            for axis in range(3):
                for edge,outward in ((lo[axis],-1),(hi[axis],1)):
                    outside=center.copy();outside[axis]=edge+outward*.0001
                    inside=center.copy();inside[axis]=edge-outward*.0001
                    self.assertFalse(valid(outside),(subject,axis,'outside'))
                    self.assertTrue(valid(inside),(subject,axis,'inside'))

    def test_reach_upper_bound_contains_unrestricted_joint_stress_samples(self):
        world=CollisionWorld(Usd.Stage.CreateInMemory(),DroidKinematics([0,0,.8],0.))
        robot=SourceRobotGeometry(world);bound=all_gripper_reach_bound(world,robot)['radius_upper_bound_m']
        cache=UsdGeom.XformCache();local=[]
        for name,body,_,inverse,*_ in robot.parts:
            if not body.startswith('/panda/Gripper/'):continue
            prim=robot.stage.GetPrimAtPath(name)
            matrix=inverse@np.asarray(cache.GetLocalToWorldTransform(prim)).T
            vertices=np.asarray(UsdGeom.Mesh(prim).GetPointsAttr().Get())
            local.append((body,vertices@matrix[:3,:3].T+matrix[:3,3]))
        shoulder=(world.kin.base@world.kin.chain[0][0])[:3,3]
        rng=np.random.default_rng(20261023)
        for _ in range(16):
            arm=rng.uniform(-np.pi,np.pi,7)
            fingers=dict(zip(world.GRIPPER_JOINT_NAMES,rng.uniform(-np.pi,np.pi,6)))
            links=world.kin.links(arm);root=links[world.gripper_root]
            links.update({name:root@matrix for name,matrix in world._gripper_links(fingers).items()})
            for body,points in local:
                t=links[body];actual=points@t[:3,:3].T+t[:3,3]
                self.assertLessEqual(float(np.linalg.norm(actual-shoulder,axis=1).max()),bound+1e-12)


if __name__=='__main__':unittest.main()
