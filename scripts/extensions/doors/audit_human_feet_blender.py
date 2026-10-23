"""Blender CPU: compare original posed source foot geometry to exported new pose."""
import bpy,json,hashlib,argparse,sys
from pathlib import Path
from mathutils.kdtree import KDTree
p=argparse.ArgumentParser();p.add_argument('--asset-dir',required=True);p.add_argument('--report-name',default='feet_geometry_audit.json');a=p.parse_args(sys.argv[sys.argv.index('--')+1:]);root=Path.cwd();out=root/a.asset_dir;deps=bpy.context.evaluated_depsgraph_get()
if Path(a.report_name).name!=a.report_name or (out/a.report_name).exists():raise FileExistsError('Use a new report filename; original evidence is retained')
def points(objects):
 result=[]
 for obj in objects:
  if obj.type!='MESH':continue
  evaluated=obj.evaluated_get(deps);mesh=evaluated.to_mesh()
  result.extend([obj.matrix_world@v.co for v in mesh.vertices if (obj.matrix_world@v.co).z<=.2]);evaluated.to_mesh_clear()
 return result
old_objects=set(bpy.data.objects);original=points(old_objects);bpy.ops.wm.usd_import(filepath=str(out/'person.usdc'));deps=bpy.context.evaluated_depsgraph_get();new=points(set(bpy.data.objects)-old_objects)
def maximum_distance(src,dst):
 tree=KDTree(len(dst))
 for i,p in enumerate(dst):tree.insert(p,i)
 tree.balance();return max(tree.find(p)[2] for p in src)
d1=maximum_distance(original,new);d2=maximum_distance(new,original);r={'passed':max(d1,d2)<1e-5,'source_vertices_below_20cm':len(original),'exported_vertices_below_20cm':len(new),'source_to_export_max_m':d1,'export_to_source_max_m':d2,'source_sha256':hashlib.sha256((root/'assets/people/mpfb_corridor/posed.blend').read_bytes()).hexdigest(),'export_sha256':hashlib.sha256((out/'person.usdc').read_bytes()).hexdigest(),'scope':'Bidirectional nearest original world-space evaluated mesh vertices below 0.2m, before any new-pose operations, compared to independently imported exported USDC. Feet/root were not moved by the arm pose generator.'};(out/a.report_name).write_text(json.dumps(r,indent=2));print(json.dumps(r,indent=2));assert r['passed']
