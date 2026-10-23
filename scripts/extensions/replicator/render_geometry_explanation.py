"""Calibrated source-geometry overlays on original-rate measured RGB recordings.

This CPU-only explanation never modifies raw videos, simulation, task state or
safety thresholds. Projection is not collision evidence or visual acceptance.
"""
import argparse
import io
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))
import fcl
import h5py
import numpy as np
from PIL import Image, ImageChops, ImageDraw, ImageFilter
from pxr import Usd, UsdGeom, UsdPhysics
from scipy.spatial.transform import Rotation

from kitchen_safety.dataset import close_encoders, decode_video, video_encoder
from kitchen_safety.geometry_audit import minimum_distance
from kitchen_safety.extensions.replicator.collection_evidence import audit_saved_collection_evidence
from kitchen_safety.extensions.replicator.video_projection import camera_points, project_points, project_polygon
from scripts.extensions.replicator.render_comparison import CAMERAS, font, recording, resolve_recorded_path, sample_index
from scripts.extensions.replicator.variant_geometry import (
    MeasuredHazardShapes, SourceRobotGeometry, assert_source_layout, make_hazard, sha, variant_settings,
)

BOX_FACES = np.asarray([[0,1,3,2],[4,6,7,5],[0,4,5,1],[2,3,7,6],[0,2,6,4],[1,5,7,3]])


def require(condition, message):
    if not condition:
        raise ValueError(message)


def corners(low, high):
    return np.asarray([[x,y,z] for x in (low[0],high[0]) for y in (low[1],high[1]) for z in (low[2],high[2])])


def source_visual_meshes(stage, root):
    """Raw authored meshes; exact cubes; 64-sided cylinder visualization only."""
    cache, rows = UsdGeom.XformCache(), []
    for p in Usd.PrimRange(stage.GetPrimAtPath(root), Usd.TraverseInstanceProxies()):
        if not p.HasAPI(UsdPhysics.CollisionAPI) or not UsdPhysics.CollisionAPI(p).GetCollisionEnabledAttr().Get():
            continue
        matrix = np.asarray(cache.GetLocalToWorldTransform(p)).T
        if p.IsA(UsdGeom.Mesh):
            mesh = UsdGeom.Mesh(p); vertices = np.asarray(mesh.GetPointsAttr().Get())
            indices = np.asarray(mesh.GetFaceVertexIndicesAttr().Get()); faces, offset = [], 0
            for count in mesh.GetFaceVertexCountsAttr().Get():
                poly = indices[offset:offset+count]; offset += count
                faces.extend([[poly[0],poly[j],poly[j+1]] for j in range(1,count-1)])
            faces = np.asarray(faces)
        elif p.IsA(UsdGeom.Cube):
            half = float(UsdGeom.Cube(p).GetSizeAttr().Get())/2
            vertices, faces = corners([-half]*3,[half]*3), BOX_FACES
        elif p.IsA(UsdGeom.Cylinder):
            import trimesh
            cylinder = UsdGeom.Cylinder(p)
            mesh = trimesh.creation.cylinder(radius=float(cylinder.GetRadiusAttr().Get()),
                                             height=float(cylinder.GetHeightAttr().Get()), sections=64)
            vertices, faces = np.asarray(mesh.vertices), np.asarray(mesh.faces)
            axis = str(cylinder.GetAxisAttr().Get())
            if axis != 'Z':
                vertices = Rotation.from_euler('y' if axis == 'X' else 'x',90 if axis == 'X' else -90,degrees=True).apply(vertices)
        else:
            raise ValueError('Unsupported explanatory source primitive: '+str(p.GetPath()))
        rows.append((str(p.GetPath()), vertices@matrix[:3,:3].T+matrix[:3,3], faces))
    require(bool(rows), 'Missing source hazard geometry')
    return rows


def bind_report(path, record, layout_path, expected, require_complete=True):
    report = json.loads(Path(path).read_text())
    require(report.get('passed') is True and report['expected'] == expected, 'Source audit failed or wrong expectation')
    require(resolve_recorded_path(report['episode']).resolve() == record['episode'].resolve(), 'Source audit belongs to another episode')
    for key, source in [('result_sha256',record['episode']/'result.json'),
                        ('physics_sha256',record['episode']/'physics_states.npz'), ('layout_sha256',layout_path)]:
        require(report[key] == sha(source), 'Source audit input hash mismatch: '+str(source))
    for source, digest in report['code_sha256'].items():
        require(sha(ROOT/source) == digest, 'Current geometry implementation differs from exact source audit: '+source)
    for item in report['source']['hazard_layers']:
        require(sha(resolve_recorded_path(item['path'])) == item['sha256'], 'Hazard USD layer differs from audited source')
    for item in [report['source']['bottle_mesh'], report['source']['robot_source_mesh']]:
        require(sha(resolve_recorded_path(item['source'])) == item['sha256'], 'Robot/bottle geometry differs from audit')
    if require_complete:
        from scripts.extensions.replicator.build_delivery_manifest import Manifest, verify_source_geometry_coverage
        verify_source_geometry_coverage(Manifest(), report, record['episode'], expected)
    with np.load(record['episode']/'physics_states.npz',allow_pickle=False) as file:
        physics = {key:file[key] for key in file.files}
    require(len(report['rows']) == len(physics['q']) == report['samples'], 'Incomplete source audit rows')
    groups = {}
    for i,row in enumerate(report['rows']):
        require(row['physics_index'] == i and row['action_index'] == int(physics['action_index'][i])
                and row['phase'] == str(physics['phase'][i]), 'Source rows do not align to real physics states')
        groups.setdefault(row['action_index'],[]).append(row)
    return report, physics, groups


class Overlay:
    def __init__(self, layout, report):
        from kitchen_safety.planning import DroidKinematics
        from kitchen_safety.extensions.replicator.collision_planner import CollisionWorld
        from kitchen_safety.extensions.replicator.source_geometry import SourceBottleGeometry
        self.variant = layout['obstacle']['category']
        _,self.required,_,_ = variant_settings(layout,self.variant)
        self.reference = np.asarray(report['hazard_pose_evidence']['pose_xyz_xyzw'])
        stage,root = make_hazard(layout,self.variant,self.reference)
        kin = DroidKinematics(np.asarray(layout['robot_position'])+[0,0,layout['stand_height']],layout['robot_yaw_deg'])
        self.world = CollisionWorld(stage,kin)
        self.robot,self.bottle = SourceRobotGeometry(self.world),SourceBottleGeometry()
        self.hazard = MeasuredHazardShapes(stage,root,self.reference)
        self.meshes = source_visual_meshes(stage,root)
        self.mask_cache = None

    def current(self,q,bottle,pose,names,ids):
        robot = self.robot.shapes(q[ids],dict(zip(names,q)))+self.bottle.shapes(bottle[:7])
        hazard = self.hazard.at(pose)
        value = minimum_distance(robot,hazard,self.required)
        points = None
        if not value['intersecting']:
            a = next(row[1] for row in robot if row[0] == value['robot_part'])
            b = next(row[1] for row in hazard if row[0] == value['obstacle'])
            result = fcl.DistanceResult()
            distance = fcl.distance(a,b,fcl.DistanceRequest(enable_nearest_points=True),result)
            points = np.asarray(result.nearest_points)
            require(points.shape == (2,3) and np.isfinite(points).all(), 'Invalid source nearest points')
            require(abs(float(distance)-value['distance_m']) < 1e-7, 'Nearest-point query disagrees with source distance')
        return value,points,hazard

    def masks(self,pose,camera,k,size,hazard):
        key = (np.asarray(pose).tobytes(),np.asarray(camera).tobytes(),np.asarray(k).tobytes(),size)
        if self.mask_cache and self.mask_cache[0] == key:
            return self.mask_cache[1:]
        raw,envelope = Image.new('L',size),Image.new('L',size)
        raw_draw,box_draw = ImageDraw.Draw(raw),ImageDraw.Draw(envelope)
        r = Rotation.from_quat(pose[3:]).as_matrix()@Rotation.from_quat(self.reference[3:]).as_matrix().T
        t = pose[:3]-r@self.reference[:3]
        for _,vertices,faces in self.meshes:
            values = camera_points(vertices@r.T+t,camera)
            for face in faces:
                polygon = project_polygon(values[face],k,size)
                if len(polygon):
                    raw_draw.polygon([tuple(x) for x in polygon],fill=255)
        for _,_,low,high in hazard:
            values = camera_points(corners(low-self.required,high+self.required),camera)
            for face in BOX_FACES:
                polygon = project_polygon(values[face],k,size)
                if len(polygon):
                    box_draw.polygon([tuple(x) for x in polygon],fill=255)
        edge = lambda mask: ImageChops.difference(mask.filter(ImageFilter.MaxFilter(5)),mask.filter(ImageFilter.MinFilter(5)))
        self.mask_cache = (key,raw,edge(raw),edge(envelope))
        return self.mask_cache[1:]


def require_matched_controls(baseline, episode):
    """Profile is a declared variant label; every effective numeric option must match."""
    require(baseline['variant'] == 'base', 'Matched control must be a no-hazard baseline')
    variant = episode['variant']
    require(variant in ['human', 'property', 'robot'], 'Unknown variant controls')
    left, right = baseline['effective_controls'], episode['effective_controls']
    require(left.get('profile') == ('human' if variant == 'human' else 'base') and
            right.get('profile') == variant, 'Wrong baseline/variant control profile identities')
    require({key: value for key, value in left.items() if key != 'profile'} ==
            {key: value for key, value in right.items() if key != 'profile'},
            'Controls differ; not a matched pair')


def render(args):
    output = args.output; validation = output.with_suffix('.validation.json')
    if output.exists() or validation.exists():
        raise FileExistsError('Preserve existing output/evidence')
    layout = json.loads(args.layout.read_text())
    records = [recording(args.baseline.resolve(),args.camera),recording(args.episode.resolve(),args.camera)]
    left,right = records
    require(left['result']['variant']=='base' and right['result']['variant']==layout['obstacle']['category'],'Wrong baseline/variant pair')
    require(left['camera_meta']==right['camera_meta'] and left['dt']==right['dt'],'Calibration/rate mismatch')
    require_matched_controls(left['result'], right['result'])
    for record in records:
        assert_source_layout(layout,record['result'])
        if not args.diagnostic:
            require(json.loads((record['episode']/'startup_audit.json').read_text())['passed'],'Startup audit failed')
            require(audit_saved_collection_evidence(record['episode'])['passed'],'Missing complete physical timeline/source provenance')
    bound = [bind_report(path,r,args.layout,expected, require_complete=not args.diagnostic) for path,r,expected in
             [(args.risk_report,left,'risk'),(args.safe_report,right,'safe')]]
    require(bound[0][0]['required_m']==bound[1][0]['required_m'],'Different physical safety margins')
    sources = [args.layout,args.risk_report,args.safe_report,Path(__file__).resolve(),
               ROOT/'kitchen_safety/extensions/replicator/video_projection.py']
    for record in records:
        sources += record['input_paths']+[record['episode']/'physics_states.npz']
        if not args.diagnostic:
            sources += [record['episode']/name for name in ['startup_physics_states.npz', 'startup_audit.json',
                'motion_physics_index.npz', 'motion_physics_alignment.json', 'collection_provenance.json']]
    for audit,_,_ in bound:
        sources += [resolve_recorded_path(row['path']) for row in audit['source']['hazard_layers']]
        sources += [ROOT/key for key in audit['code_sha256']]
    hashes = {str(path):sha(path) for path in sorted(set(sources))}
    report = dict(schema_version=1,passed=False,diagnostic_only=args.diagnostic,frames=[],input_sha256=hashes,
                  baseline=str(left['episode']), episode=str(right['episode']), camera=args.camera,
                  layout=str(args.layout), output=str(output), complete_source_physics_bound=not args.diagnostic,
                  visual_review='Pending separate sampled inspection; decode alone is not visual acceptance.',
                  source_reports=[str(args.risk_report),str(args.safe_report)],
                  semantics='RGB is original images.h5 JPEG at obs/action index i, before that action. Nearest-source markers are recomputed from the SAME obs state and measured ROS camera pose. Numeric action minimum is explicitly the ensuing action physics interval from the bound source audit; its future pose is NEVER projected onto the current image. Independent timelines, original speed, no event/phase synchronization; final frame held explicitly.',
                  visualization='Magenta: source collision silhouette (64-sided cylinder tessellation only for visualization). Orange: union of source-collider world AABBs expanded by the unchanged required margin, a conservative 3-D visual envelope. Both are x-ray projections without reconstructed depth occlusion. 2-D overlap is not a safety test; exact source audit distances are separate. Near plane 0.01m and viewport clipping applied before drawing.',
                  scope='G-shape mustard representative human/property/robot explanations only; neither cooked PhysX shape nor contact/breakage proof. Original raw videos/data preserved.')
    output.parent.mkdir(parents=True,exist_ok=True)
    with output.open('xb'):
        pass
    encoder,files = None,[]
    try:
        visualizers = [Overlay(layout,item[0]) for item in bound]
        for record in records:
            camera_root = record['episode']/'overview' if args.camera=='overview_camera' else record['episode']
            files.append((h5py.File(camera_root/'images.h5','r'),h5py.File(record['episode']/'trajectory.h5','r')))
        total,dt = max(r['count'] for r in records),left['dt']
        encoder = video_encoder(output,1280,632,dt)
        last_panels = [None,None]
        for frame in range(total):
            panel = Image.new('RGB',(1280,632),'#101d30');draw = ImageDraw.Draw(panel)
            banner = 'DIAGNOSTIC OLD SAMPLE - NOT DELIVERY EVIDENCE' if args.diagnostic else 'SOURCE-GEOMETRY EXPLANATION - RAW RECORDINGS PRESERVED'
            draw.text((14,7),banner,font=font(18),fill='#ffcf70')
            frame_evidence = []
            for side,(record,(audit,physics,groups),viz,(images,trajectory)) in enumerate(zip(records,bound,visualizers,files)):
                index,held = sample_index(frame,record['count'])
                if not held:
                    stream = images[args.camera]
                    image = Image.open(io.BytesIO(stream['jpeg'][index].tobytes())).convert('RGB')
                    camera = np.r_[stream['position_world'][index],stream['quaternion_world_ros_xyzw'][index]]
                    k = np.asarray(record['camera_meta']['intrinsics'])
                    q,bottle = trajectory['obs/q'][index],trajectory['obs/mustard'][index]
                    pose = trajectory['obs/hazard_pose'][index] if side==1 and not layout['obstacle']['static'] else viz.reference
                    value,points,hazard = viz.current(q,bottle,pose,physics['joint_names'].tolist(),physics['arm_ids'])
                    preceding = groups.get(index-1,[])
                    matched = False
                    if preceding:
                        row=preceding[-1];i=row['physics_index']
                        require(np.allclose(q,physics['q'][i],atol=1e-7,rtol=0) and np.allclose(bottle[:7],physics['mustard'][i],atol=1e-7,rtol=0),'Image obs is not the preceding measured physics state')
                        require(abs(value['distance_m']-row['source_mesh']['distance_m'])<2e-6,'Frame-state source distance disagrees with bound exact source audit')
                        matched=True
                    raw,edge,envelope = viz.masks(pose,camera,k,record['size'],hazard)
                    if side==0:
                        image.paste(Image.new('RGB',image.size,'#d169ff'),(0,0),raw.point(lambda x:x//7))
                    image.paste(Image.new('RGB',image.size,'#f074ff'),(0,0),edge)
                    image.paste(Image.new('RGB',image.size,'#ffa936'),(0,0),envelope)
                    visible=False
                    if points is not None:
                        pixels,valid=project_points(points,camera,k,record['size'])
                        if valid.all():
                            marks=ImageDraw.Draw(image);color='#56f0ed' if value['valid'] else '#ff4d4d'
                            marks.line([tuple(x) for x in pixels],fill=color,width=5)
                            for x,y in pixels: marks.ellipse((x-8,y-8,x+8,y+8),outline=color,width=4)
                            visible=True
                    image.thumbnail((640,360),Image.Resampling.LANCZOS)
                    interval=min(groups.get(index,[]),key=lambda x:x['source_mesh']['distance_m'],default=None)
                    last_panels[side]=(image,dict(image_obs_index=index,held=False,obs_source_distance=value,
                        nearest_points_world=None if points is None else points.tolist(),nearest_points_visible=bool(visible),
                        matched_preceding_source_physics_state=matched,ensuing_action_source_minimum=interval))
                image,evidence=last_panels[side];evidence=dict(evidence,held=held)
                frame_evidence.append(evidence)
                x=side*640
                panel.paste(image,(x+(640-image.width)//2,65+(360-image.height)//2))
                draw.text((x+12,35),'BASELINE + OFFLINE HYPOTHETICAL HAZARD' if side==0 else 'ACTUAL HAZARD: REAL ROBOT EXECUTION',font=font(17),fill='white')
                draw.text((x+12,432),f"obs/action {index} | t={record['times'][index]:.2f}s | {record['phases'][index]}",font=font(15),fill='white')
                d=evidence['obs_source_distance'];draw.text((x+12,455),f"At image obs: source clearance {d['distance_m']*1000:.1f} mm / required {viz.required*1000:.0f} mm",font=font(15),fill='#56f0ed' if d['valid'] else '#ff7777')
                interval=evidence['ensuing_action_source_minimum']
                label=(f"Ensuing action min: {interval['source_mesh']['distance_m']*1000:.1f} mm (physics {interval['physics_index']})" if interval else 'No bound main-source audit interval for this startup frame')
                draw.text((x+12,478),label,font=font(14),fill='#c7d6e6')
                draw.text((x+12,501),'FINISHED: final frame held' if held else ('Source nearest-point markers' if evidence['nearest_points_visible'] else 'Nearest source points offscreen / no marker'),font=font(14),fill='#ffcf70')
            draw.text((14,531),'Magenta: source silhouette. Orange: conservative AABB + margin envelope, NOT exact safety surface.',font=font(15),fill='#f3d195')
            draw.text((14,555),'X-RAY overlay: depth occlusion NOT reconstructed. 2-D projection is NOT collision/safety evidence.',font=font(15),fill='#f3d195')
            draw.text((14,579),'Markers use current image obs only; the ensuing-action minimum is a separate physics-interval number.',font=font(15),fill='#c7d6e6')
            draw.text((14,603),'Original speed; independent elapsed timelines; NO event alignment. Baseline hazard was never physically added.',font=font(15),fill='#c7d6e6')
            encoder.stdin.write(panel.tobytes());report['frames'].append(dict(output_frame=frame,panels=frame_evidence))
            if frame%100==0:print('EXPLANATION_FRAME',frame,'/',total,flush=True)
        process,encoder=encoder,None;close_encoders({'geometry_explanation':process})
        decoded=decode_video(output,total,expected_size=(1280,632),expected_dt=dt)
        require(all(sha(path)==digest for path,digest in hashes.items()),'Inputs changed during explanation rendering')
        report.update(passed=True,output_sha256=sha(output),decoded_frames=decoded,dt=dt,duration_s=total*dt)
    except Exception as error:
        report['error']=repr(error)
    finally:
        for pair in files:
            for file in pair:file.close()
        if encoder is not None:
            try:close_encoders({'geometry_explanation':encoder})
            except Exception as error:report['encoder_shutdown_error']=repr(error)
        validation.write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    return report


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    for flag in ('baseline','episode','layout','risk-report','safe-report','output'):
        parser.add_argument('--'+flag,type=Path,required=True)
    parser.add_argument('--camera',choices=CAMERAS,default='overview_camera')
    parser.add_argument('--diagnostic',action='store_true',help='Explicit old/incomplete sample explanation; conspicuous diagnostic banner')
    args=parser.parse_args();report=render(args)
    print(json.dumps(dict(passed=report['passed'],error=report.get('error'),output=str(args.output))))
    return 0 if report['passed'] else 1


if __name__=='__main__':
    raise SystemExit(main())
